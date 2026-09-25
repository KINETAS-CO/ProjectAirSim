use std::sync::Arc;
use std::sync::atomic::{AtomicI32, Ordering};
use serde::{de::DeserializeOwned, Serialize};
use tokio::sync::broadcast;
use tracing::info;

use crate::error::{Result, SimError};
use crate::protocol::frame::{TopicFrame, TopicInfo};
use crate::protocol::request::RequestEnvelope;
use crate::protocol::response::ResponseDecoder;
use crate::transport::nng_actor::NngActor;

pub const DEFAULT_PORT_TOPICS: u16 = 8989;
pub const DEFAULT_PORT_SERVICES: u16 = 8990;

/// A subscription handle for receiving messages on a specific simulation topic.
pub struct TopicSubscription {
    topic: String,
    receiver: broadcast::Receiver<TopicFrame>,
}

impl TopicSubscription {
    /// Returns the topic path being monitored.
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Asynchronously awaits the next message body on this topic.
    pub async fn recv(&mut self) -> Result<Vec<u8>> {
        loop {
            match self.receiver.recv().await {
                Ok(frame) => {
                    if frame.topic() == self.topic {
                        return Ok(frame.into_body());
                    }
                }
                Err(broadcast::error::RecvError::Lagged(_)) => continue,
                Err(broadcast::error::RecvError::Closed) => return Err(SimError::ConnectionClosed),
            }
        }
    }

    /// Asynchronously awaits the next typed MessagePack message on this topic.
    pub async fn recv_typed<T: DeserializeOwned>(&mut self) -> Result<T> {
        let bytes = self.recv().await?;
        rmp_serde::from_slice(&bytes)
            .map_err(|e| SimError::SerializationError(format!("Failed to deserialize topic message: {e}")))
    }
}

/// Asynchronous client for ProjectAirSim simulation server.
#[derive(Clone)]
pub struct Client {
    actor: Arc<NngActor>,
    request_id: Arc<AtomicI32>,
}

impl Client {
    /// Connects to a ProjectAirSim simulation server using default ports (8989 Topics, 8990 Services).
    pub async fn connect(address: &str) -> Result<Self> {
        Self::connect_with_ports(address, DEFAULT_PORT_TOPICS, DEFAULT_PORT_SERVICES).await
    }

    /// Connects to a ProjectAirSim simulation server with specified ports.
    pub async fn connect_with_ports(address: &str, port_topics: u16, port_services: u16) -> Result<Self> {
        info!("Connecting to ProjectAirSim at {address} (topics: {port_topics}, services: {port_services})");
        let actor = NngActor::start(address, port_topics, port_services)?;
        Ok(Self {
            actor: Arc::new(actor),
            request_id: Arc::new(AtomicI32::new(1)),
        })
    }

    /// Creates a client wrapping an existing NngActor instance (useful for testing or custom configuration).
    pub fn from_actor(actor: Arc<NngActor>) -> Self {
        Self {
            actor,
            request_id: Arc::new(AtomicI32::new(1)),
        }
    }

    /// Dispatches a typed RPC method call to the simulation server and deserializes the typed response.
    pub async fn request<P: Serialize, R: DeserializeOwned>(&self, method: &str, params: &P) -> Result<R> {
        let id = self.request_id.fetch_add(1, Ordering::Relaxed);
        let req = RequestEnvelope::new(id, method, params)?;
        let req_bytes = req.to_bytes()?;

        let resp_bytes = self.actor.send_rpc(req_bytes).await?;
        ResponseDecoder::decode_typed::<R>(&resp_bytes)
    }

    /// Dispatches an RPC method call with raw MessagePack parameter bytes.
    pub async fn request_raw(&self, method: &str, raw_param_bytes: &[u8]) -> Result<Vec<u8>> {
        let id = self.request_id.fetch_add(1, Ordering::Relaxed);
        let req = RequestEnvelope {
            id,
            method,
            params: crate::protocol::request::RawDataPayload {
                data: raw_param_bytes.to_vec(),
            },
            version: 1.0,
        };
        let req_bytes = req.to_bytes()?;
        let resp_bytes = self.actor.send_rpc(req_bytes).await?;
        ResponseDecoder::decode(&resp_bytes)
    }

    /// Subscribes to a simulation topic and returns a `TopicSubscription` receiver stream.
    pub async fn subscribe(&self, topic: impl Into<String>) -> Result<TopicSubscription> {
        let topic_str = topic.into();
        let frame = TopicFrame::subscribe(&topic_str);
        self.actor.send_topic_frame(frame).await?;

        let receiver = self.actor.subscribe_broadcast();
        Ok(TopicSubscription {
            topic: topic_str,
            receiver,
        })
    }

    /// Subscribes to all simulation topic frames without filtering.
    pub fn subscribe_all(&self) -> broadcast::Receiver<TopicFrame> {
        self.actor.subscribe_broadcast()
    }

    /// Unsubscribes from a simulation topic.
    pub async fn unsubscribe(&self, topic: impl Into<String>) -> Result<()> {
        let topic_str = topic.into();
        let frame = TopicFrame::unsubscribe(&topic_str);
        self.actor.send_topic_frame(frame).await
    }

    /// Publishes a typed MessagePack payload to a simulation topic.
    pub async fn publish<T: Serialize>(&self, topic: &str, message: &T) -> Result<()> {
        let body = rmp_serde::to_vec(message)
            .map_err(|e| SimError::SerializationError(format!("Failed to serialize publish payload: {e}")))?;
        self.publish_raw(topic, body).await
    }

    /// Publishes raw bytes to a simulation topic.
    pub async fn publish_raw(&self, topic: &str, payload: Vec<u8>) -> Result<()> {
        let frame = TopicFrame::message(topic, payload);
        self.actor.send_topic_frame(frame).await
    }

    /// Retrieves the list of active simulation topics discovered via `/$topics`.
    pub async fn get_topic_info(&self) -> Result<Vec<TopicInfo>> {
        Ok(self.actor.get_topic_info().await)
    }

    /// Tests connection liveness with the simulation server.
    pub async fn ping(&self) -> Result<bool> {
        #[derive(Serialize)]
        struct EmptyParams {}
        let res: Result<serde_json::Value> = self.request("/Sim/Ping", &EmptyParams {}).await;
        match res {
            Ok(_) => Ok(true),
            Err(SimError::ServerRejected { code: 404, .. }) => Ok(true), // Method found or server replied
            Err(e) => Err(e),
        }
    }

    /// Retrieves the simulation server's git commit hash.
    pub async fn get_build_commit_hash(&self) -> Result<String> {
        #[derive(Serialize)]
        struct EmptyParams {}
        self.request("/Sim/GetBuildCommitHash", &EmptyParams {}).await
    }
}
