use serde::{de::DeserializeOwned, Serialize};
use std::collections::HashSet;
use std::sync::atomic::{AtomicI32, Ordering};
use std::sync::Arc;
use tokio::sync::{broadcast, RwLock};
use tracing::info;

use crate::error::{Result, SimError};
use crate::protocol::frame::{TopicFrame, TopicInfo};
use crate::protocol::params::{EmptyParams, FeatureParams, LoadSceneParams};
use crate::protocol::request::RequestEnvelope;
use crate::protocol::response::ResponseDecoder;
use crate::transport::nng_actor::NngActor;

pub const DEFAULT_PORT_TOPICS: u16 = 8989;
pub const DEFAULT_PORT_SERVICES: u16 = 8990;

/// A subscription handle for receiving messages on a specific simulation topic.
pub struct TopicSubscription {
    topic: String,
    receiver: tokio::sync::mpsc::Receiver<TopicFrame>,
}

impl TopicSubscription {
    pub fn new(topic: String, receiver: tokio::sync::mpsc::Receiver<TopicFrame>) -> Self {
        Self { topic, receiver }
    }

    /// Returns the topic path being monitored.
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Asynchronously awaits the next message body on this topic.
    pub async fn recv(&mut self) -> Result<Vec<u8>> {
        match self.receiver.recv().await {
            Some(frame) => Ok(frame.into_body()),
            None => Err(SimError::ConnectionClosed),
        }
    }

    /// Asynchronously awaits the next raw `TopicFrame` on this topic.
    pub async fn recv_frame(&mut self) -> Result<TopicFrame> {
        match self.receiver.recv().await {
            Some(frame) => Ok(frame),
            None => Err(SimError::ConnectionClosed),
        }
    }

    /// Asynchronously awaits the next typed MessagePack message on this topic.
    pub async fn recv_typed<T: DeserializeOwned>(&mut self) -> Result<T> {
        let bytes = self.recv().await?;
        rmp_serde::from_slice(&bytes).map_err(|e| {
            SimError::SerializationError(format!("Failed to deserialize topic message: {e}"))
        })
    }

    /// Asynchronously awaits the next JSON-deserializable message on this topic.
    pub async fn recv_json<T: DeserializeOwned>(&mut self) -> Result<T> {
        let bytes = self.recv().await?;
        serde_json::from_slice(&bytes).map_err(|e| {
            SimError::SerializationError(format!("Failed to deserialize JSON topic message: {e}"))
        })
    }
}

impl futures_util::Stream for TopicSubscription {
    type Item = Result<TopicFrame>;

    fn poll_next(
        mut self: std::pin::Pin<&mut Self>,
        cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        match self.receiver.poll_recv(cx) {
            std::task::Poll::Ready(Some(frame)) => std::task::Poll::Ready(Some(Ok(frame))),
            std::task::Poll::Ready(None) => std::task::Poll::Ready(None),
            std::task::Poll::Pending => std::task::Poll::Pending,
        }
    }
}

/// Asynchronous client for ProjectAirSim simulation server.
#[derive(Clone)]
pub struct Client {
    actor: Arc<NngActor>,
    request_id: Arc<AtomicI32>,
    subscriptions: Arc<RwLock<HashSet<String>>>,
}

impl Client {
    /// Returns the client library version string.
    pub fn get_version() -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    /// Returns the underlying NNG messaging library version string.
    pub fn get_nng_version() -> &'static str {
        "1.0"
    }

    /// Connects to a ProjectAirSim simulation server using default ports (8989 Topics, 8990 Services).
    pub async fn connect(address: &str) -> Result<Self> {
        Self::connect_with_ports(address, DEFAULT_PORT_TOPICS, DEFAULT_PORT_SERVICES).await
    }

    /// Connects to a ProjectAirSim simulation server with specified ports.
    pub async fn connect_with_ports(
        address: &str,
        port_topics: u16,
        port_services: u16,
    ) -> Result<Self> {
        info!("Connecting to ProjectAirSim at {address} (topics: {port_topics}, services: {port_services})");
        let actor = NngActor::start(address, port_topics, port_services)?;
        Ok(Self {
            actor: Arc::new(actor),
            request_id: Arc::new(AtomicI32::new(1)),
            subscriptions: Arc::new(RwLock::new(HashSet::new())),
        })
    }

    /// Creates a client wrapping an existing NngActor instance (useful for testing or custom configuration).
    pub fn from_actor(actor: Arc<NngActor>) -> Self {
        Self {
            actor,
            request_id: Arc::new(AtomicI32::new(1)),
            subscriptions: Arc::new(RwLock::new(HashSet::new())),
        }
    }

    /// Dispatches a typed RPC method call to the simulation server and deserializes the typed response.
    pub async fn request<P: Serialize, R: DeserializeOwned>(
        &self,
        method: &str,
        params: &P,
    ) -> Result<R> {
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

    /// Dispatches a high-priority RPC method call that jumps ahead of queued normal requests.
    pub async fn request_priority<P: Serialize, R: DeserializeOwned>(
        &self,
        method: &str,
        params: &P,
    ) -> Result<R> {
        let id = self.request_id.fetch_add(1, Ordering::Relaxed);
        let req = RequestEnvelope::new(id, method, params)?;
        let req_bytes = req.to_bytes()?;

        let resp_bytes = self.actor.send_rpc_priority(req_bytes).await?;
        ResponseDecoder::decode_typed::<R>(&resp_bytes)
    }

    /// Cancels all pending queued requests that have not yet been sent to the server.
    pub fn cancel_all_requests(&self) {
        self.actor.cancel_all_requests();
    }

    /// Subscribes to a simulation topic and returns a `TopicSubscription` receiver stream.
    pub async fn subscribe(&self, topic: impl Into<String>) -> Result<TopicSubscription> {
        let topic_str = topic.into();
        let frame = TopicFrame::subscribe(&topic_str);
        self.actor.send_topic_frame(frame).await?;
        self.subscriptions.write().await.insert(topic_str.clone());

        let mut bcast = self.actor.subscribe_broadcast();
        let (tx, rx) = tokio::sync::mpsc::channel(256);
        let filter_topic = topic_str.clone();

        tokio::spawn(async move {
            loop {
                match bcast.recv().await {
                    Ok(frame) => {
                        if frame.topic() == filter_topic && tx.send(frame).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
        });

        Ok(TopicSubscription::new(topic_str, rx))
    }

    /// Subscribes to all simulation topic frames without filtering.
    pub fn subscribe_all(&self) -> broadcast::Receiver<TopicFrame> {
        self.actor.subscribe_broadcast()
    }

    /// Returns a list of currently active subscribed topic paths.
    pub async fn get_active_subscriptions(&self) -> Vec<String> {
        self.subscriptions.read().await.iter().cloned().collect()
    }

    /// Unsubscribes from a simulation topic by path.
    pub async fn unsubscribe(&self, topic: impl AsRef<str>) -> Result<()> {
        let topic_str = topic.as_ref();
        self.subscriptions.write().await.remove(topic_str);
        let frame = TopicFrame::unsubscribe(topic_str);
        let _ = self.actor.send_topic_frame(frame).await;

        let paths = [topic_str];
        let res: Result<serde_json::Value> = self
            .request(
                "/Sim/Unsubscribe",
                &serde_json::json!({ "topic_paths": paths }),
            )
            .await;
        match res {
            Ok(_) => Ok(()),
            Err(SimError::ServerRejected { .. }) => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Unsubscribes from multiple simulation topics at once.
    pub async fn unsubscribe_topics(&self, topics: &[impl AsRef<str>]) -> Result<()> {
        let paths: Vec<String> = topics.iter().map(|t| t.as_ref().to_string()).collect();
        {
            let mut subs = self.subscriptions.write().await;
            for path in &paths {
                subs.remove(path);
            }
        }
        for path in &paths {
            let frame = TopicFrame::unsubscribe(path);
            let _ = self.actor.send_topic_frame(frame).await;
        }

        let res: Result<serde_json::Value> = self
            .request(
                "/Sim/Unsubscribe",
                &serde_json::json!({ "topic_paths": paths }),
            )
            .await;
        match res {
            Ok(_) => Ok(()),
            Err(SimError::ServerRejected { .. }) => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Unsubscribes from all currently tracked simulation topics.
    pub async fn unsubscribe_all(&self) -> Result<()> {
        let active_topics: Vec<String> = {
            let mut subs = self.subscriptions.write().await;
            subs.drain().collect()
        };
        if active_topics.is_empty() {
            return Ok(());
        }
        for topic in &active_topics {
            let frame = TopicFrame::unsubscribe(topic);
            let _ = self.actor.send_topic_frame(frame).await;
        }

        let res: Result<serde_json::Value> = self
            .request(
                "/Sim/Unsubscribe",
                &serde_json::json!({ "topic_paths": active_topics }),
            )
            .await;
        match res {
            Ok(_) => Ok(()),
            Err(SimError::ServerRejected { .. }) => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Publishes a typed MessagePack payload to a simulation topic.
    pub async fn publish<T: Serialize>(&self, topic: &str, message: &T) -> Result<()> {
        let body = rmp_serde::to_vec(message).map_err(|e| {
            SimError::SerializationError(format!("Failed to serialize publish payload: {e}"))
        })?;
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

    /// Retrieves the list of active simulation topic paths discovered via `/$topics`.
    pub async fn get_topic_paths(&self) -> Result<Vec<String>> {
        let infos = self.get_topic_info().await?;
        Ok(infos.into_iter().map(|info| info.path).collect())
    }

    /// Tests connection liveness with the simulation server.
    pub async fn ping(&self) -> Result<bool> {
        let res: Result<serde_json::Value> = self.request("/Sim/Ping", &EmptyParams {}).await;
        match res {
            Ok(_) => Ok(true),
            Err(SimError::ServerRejected { code: 404, .. }) => Ok(true),
            Err(e) => Err(e),
        }
    }

    /// Retrieves the simulation server's git commit hash.
    pub async fn get_build_commit_hash(&self) -> Result<String> {
        self.request("/Sim/GetBuildCommitHash", &EmptyParams {})
            .await
    }

    /// Enables or disables an interactive feature on the simulation server (e.g. weather, physics).
    pub async fn set_interactive_feature(&self, feature_id: &str, enable: bool) -> Result<bool> {
        self.request(
            "/Sim/SetInteractiveFeature",
            &FeatureParams {
                feature_id,
                enable,
            },
        )
        .await
    }

    /// Requests the server to load or reload a scene from a JSONC configuration string.
    pub async fn request_load_scene(&self, scene_config: &str) -> Result<String> {
        self.cancel_all_requests();
        let _ = self.unsubscribe_all().await;
        let res: serde_json::Value = self
            .request(
                "/Sim/LoadScene",
                &LoadSceneParams { scene_config },
            )
            .await?;
        if let Some(s) = res.as_str() {
            Ok(s.to_string())
        } else {
            Ok(res.to_string())
        }
    }
}
