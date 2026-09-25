use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::mpsc::channel;
use std::sync::Arc;
use serde::{de::DeserializeOwned, Serialize};
use tracing::info;

use crate::blocking::async_result::AsyncResult;
use crate::error::{Result, SimError};
use crate::protocol::frame::TopicFrame;
use crate::protocol::request::RequestEnvelope;
use crate::protocol::response::ResponseDecoder;
use crate::transport::nng_transport::NngTransport;

pub const DEFAULT_PORT_TOPICS: u16 = 8989;
pub const DEFAULT_PORT_SERVICES: u16 = 8990;

/// Synchronous blocking client for ProjectAirSim simulation server.
#[derive(Clone)]
pub struct Client {
    transport: Arc<NngTransport>,
    request_id: Arc<AtomicI32>,
}

impl Client {
    /// Connects to a ProjectAirSim simulation server synchronously using default ports.
    pub fn connect(address: &str) -> Result<Self> {
        Self::connect_with_ports(address, DEFAULT_PORT_TOPICS, DEFAULT_PORT_SERVICES)
    }

    /// Connects to a ProjectAirSim simulation server synchronously with specified ports.
    pub fn connect_with_ports(address: &str, port_topics: u16, port_services: u16) -> Result<Self> {
        info!("Connecting (blocking) to ProjectAirSim at {address} (topics: {port_topics}, services: {port_services})");
        let transport = Arc::new(NngTransport::connect(address, port_topics, port_services)?);
        Ok(Self {
            transport,
            request_id: Arc::new(AtomicI32::new(1)),
        })
    }

    /// Dispatches an asynchronous RPC request, immediately returning an `AsyncResult` handle.
    pub fn request_async<P: Serialize, R: DeserializeOwned + Send + 'static>(
        &self,
        method: &str,
        params: &P,
    ) -> AsyncResult<R> {
        let (tx, rx) = channel();
        let id = self.request_id.fetch_add(1, Ordering::Relaxed);

        let req_bytes = match RequestEnvelope::new(id, method, params).and_then(|r| r.to_bytes()) {
            Ok(b) => b,
            Err(e) => {
                let _ = tx.send(Err(e));
                return AsyncResult::new(rx);
            }
        };

        let transport = Arc::clone(&self.transport);
        std::thread::spawn(move || {
            let res = transport
                .send_request_sync(&req_bytes)
                .and_then(|resp_bytes| ResponseDecoder::decode_typed::<R>(&resp_bytes));
            let _ = tx.send(res);
        });

        AsyncResult::new(rx)
    }

    /// Dispatches a synchronous RPC request and blocks until the reply is received.
    pub fn request<P: Serialize, R: DeserializeOwned>(
        &self,
        method: &str,
        params: &P,
    ) -> Result<R> {
        let id = self.request_id.fetch_add(1, Ordering::Relaxed);
        let req = RequestEnvelope::new(id, method, params)?;
        let req_bytes = req.to_bytes()?;
        let resp_bytes = self.transport.send_request_sync(&req_bytes)?;
        ResponseDecoder::decode_typed::<R>(&resp_bytes)
    }

    /// Subscribes to a simulation topic with a callback function invoked on a background thread.
    pub fn subscribe<F>(&self, topic: impl Into<String>, callback: F) -> Result<()>
    where
        F: Fn(TopicFrame) + Send + 'static,
    {
        let topic_str = topic.into();
        let sub_frame = TopicFrame::subscribe(&topic_str).to_bytes()?;
        self.transport.send_topic_frame_sync(&sub_frame)?;

        let transport = Arc::clone(&self.transport);
        std::thread::spawn(move || loop {
            match transport.recv_topic_frame_sync(200) {
                Ok(Some(raw_bytes)) => {
                    if let Ok(frame) = TopicFrame::from_bytes(&raw_bytes) {
                        if frame.topic() == topic_str {
                            callback(frame);
                        }
                    }
                }
                Ok(None) | Err(SimError::TimedOut) => continue,
                Err(_) => break,
            }
        });

        Ok(())
    }

    /// Publishes a typed MessagePack payload to a simulation topic.
    pub fn publish<T: Serialize>(&self, topic: &str, message: &T) -> Result<()> {
        let body = rmp_serde::to_vec(message)
            .map_err(|e| SimError::SerializationError(format!("Failed to serialize publish payload: {e}")))?;
        let frame = TopicFrame::message(topic, body).to_bytes()?;
        self.transport.send_topic_frame_sync(&frame)
    }

    /// Tests connection liveness with the simulation server.
    pub fn ping(&self) -> Result<bool> {
        #[derive(Serialize)]
        struct EmptyParams {}
        let res: Result<serde_json::Value> = self.request("/Sim/Ping", &EmptyParams {});
        match res {
            Ok(_) => Ok(true),
            Err(SimError::ServerRejected { code: 404, .. }) => Ok(true),
            Err(e) => Err(e),
        }
    }

    /// Retrieves the simulation server's git commit hash.
    pub fn get_build_commit_hash(&self) -> Result<String> {
        #[derive(Serialize)]
        struct EmptyParams {}
        self.request("/Sim/GetBuildCommitHash", &EmptyParams {})
    }
}
