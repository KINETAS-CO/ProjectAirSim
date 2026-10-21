use serde::{de::DeserializeOwned, Serialize};
use std::sync::Arc;
use tokio::runtime::{Builder, Runtime};

use crate::blocking::async_result::{async_result_channel, AsyncResult};
use crate::error::{Result, SimError};
use crate::protocol::frame::{TopicFrame, TopicInfo};
use crate::transport::nng_actor::NngActor;
use crate::transport::nng_transport::NngTransport;

pub const DEFAULT_PORT_TOPICS: u16 = 8989;
pub const DEFAULT_PORT_SERVICES: u16 = 8990;

/// Synchronous iterator yielding incoming topic frames.
pub struct TopicIterator {
    topic: String,
    receiver: std::sync::mpsc::Receiver<TopicFrame>,
}

impl TopicIterator {
    pub fn new(topic: impl Into<String>, receiver: std::sync::mpsc::Receiver<TopicFrame>) -> Self {
        Self {
            topic: topic.into(),
            receiver,
        }
    }

    /// Returns the topic path being monitored.
    pub fn topic(&self) -> &str {
        &self.topic
    }

    /// Receives the next frame blocking indefinitely, returning None if disconnected.
    pub fn recv(&self) -> Option<TopicFrame> {
        self.receiver.recv().ok()
    }

    /// Receives the next frame waiting up to `timeout`.
    pub fn recv_timeout(&self, timeout: std::time::Duration) -> Result<TopicFrame> {
        self.receiver.recv_timeout(timeout).map_err(|e| match e {
            std::sync::mpsc::RecvTimeoutError::Timeout => SimError::TimedOut,
            std::sync::mpsc::RecvTimeoutError::Disconnected => SimError::ConnectionClosed,
        })
    }
}

impl Iterator for TopicIterator {
    type Item = TopicFrame;

    fn next(&mut self) -> Option<Self::Item> {
        self.receiver.recv().ok()
    }
}

fn create_runtime() -> Result<Runtime> {
    Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()
        .map_err(|e| SimError::TransportError(format!("Failed to build Tokio runtime: {e}")))
}

/// Synchronous blocking client for ProjectAirSim simulation server.
///
/// Wraps the asynchronous client with an internal Tokio runtime, providing
/// synchronous functions and `AsyncResult` handles matching C++ client semantics.
#[derive(Clone)]
pub struct Client {
    inner: crate::async_api::Client,
    runtime: Arc<Runtime>,
}

impl Client {
    /// Returns the client library version string.
    pub fn get_version() -> &'static str {
        crate::async_api::Client::get_version()
    }

    /// Returns the underlying NNG messaging library version string.
    pub fn get_nng_version() -> &'static str {
        crate::async_api::Client::get_nng_version()
    }

    /// Connects to a ProjectAirSim simulation server using default ports (8989 Topics, 8990 Services).
    pub fn connect(address: &str) -> Result<Self> {
        Self::connect_with_ports(address, DEFAULT_PORT_TOPICS, DEFAULT_PORT_SERVICES)
    }

    /// Connects to a ProjectAirSim simulation server with specified ports.
    pub fn connect_with_ports(
        address: &str,
        port_topics: u16,
        port_services: u16,
    ) -> Result<Self> {
        let runtime = Arc::new(create_runtime()?);
        let inner = runtime.block_on(crate::async_api::Client::connect_with_ports(
            address,
            port_topics,
            port_services,
        ))?;
        Ok(Self { inner, runtime })
    }

    /// Creates a synchronous client wrapping an existing NngTransport instance.
    pub fn from_transport(transport: Arc<NngTransport>) -> Result<Self> {
        let runtime = Arc::new(create_runtime()?);
        let actor = Arc::new(NngActor::from_transport(transport)?);
        let inner = crate::async_api::Client::from_actor(actor);
        Ok(Self { inner, runtime })
    }

    /// Access the underlying Tokio runtime handle.
    pub fn runtime(&self) -> &Runtime {
        &self.runtime
    }

    /// Access the underlying async client.
    pub fn inner(&self) -> &crate::async_api::Client {
        &self.inner
    }

    /// Spawns an asynchronous future on the Tokio runtime and returns an `AsyncResult` handle.
    pub fn spawn_async<T: Send + 'static>(
        &self,
        fut: impl std::future::Future<Output = Result<T>> + Send + 'static,
    ) -> AsyncResult<T> {
        let (sender, ar) = async_result_channel();
        self.runtime.spawn(async move {
            let res = fut.await;
            sender.send(res);
        });
        ar
    }

    /// Dispatches an asynchronous RPC request, returning an `AsyncResult` handle.
    pub fn request_async<P: Serialize + Send + 'static, R: DeserializeOwned + Send + 'static>(
        &self,
        method: &str,
        params: &P,
    ) -> AsyncResult<R> {
        let client = self.inner.clone();
        let method = method.to_string();
        let params_val = serde_json::to_value(params).unwrap_or_default();
        self.spawn_async(async move { client.request(&method, &params_val).await })
    }

    /// Dispatches a high-priority asynchronous RPC request.
    pub fn request_priority_async<P: Serialize + Send + 'static, R: DeserializeOwned + Send + 'static>(
        &self,
        method: &str,
        params: &P,
    ) -> AsyncResult<R> {
        let client = self.inner.clone();
        let method = method.to_string();
        let params_val = serde_json::to_value(params).unwrap_or_default();
        self.spawn_async(async move { client.request_priority(&method, &params_val).await })
    }

    /// Dispatches a synchronous RPC request and blocks until the reply is received.
    pub fn request<P: Serialize, R: DeserializeOwned>(
        &self,
        method: &str,
        params: &P,
    ) -> Result<R> {
        self.runtime.block_on(self.inner.request(method, params))
    }

    /// Dispatches a high-priority synchronous RPC request.
    pub fn request_priority<P: Serialize, R: DeserializeOwned>(
        &self,
        method: &str,
        params: &P,
    ) -> Result<R> {
        self.runtime.block_on(self.inner.request_priority(method, params))
    }

    /// Cancels all pending queued requests.
    pub fn cancel_all_requests(&self) {
        self.inner.cancel_all_requests();
    }

    /// Subscribes to a simulation topic with a callback function invoked on a background thread.
    pub fn subscribe<F>(&self, topic: impl Into<String>, callback: F) -> Result<()>
    where
        F: Fn(TopicFrame) + Send + Sync + 'static,
    {
        let mut sub = self.runtime.block_on(self.inner.subscribe(topic))?;
        let cb = Arc::new(callback);
        self.runtime.spawn(async move {
            while let Ok(frame) = sub.recv_frame().await {
                cb(frame);
            }
        });
        Ok(())
    }

    /// Subscribes to a simulation topic and returns a synchronous iterator yielding incoming frames.
    pub fn subscribe_iterator(&self, topic: impl Into<String>) -> Result<TopicIterator> {
        let topic_str = topic.into();
        let mut sub = self.runtime.block_on(self.inner.subscribe(&topic_str))?;
        let (tx, rx) = std::sync::mpsc::channel();
        self.runtime.spawn(async move {
            while let Ok(frame) = sub.recv_frame().await {
                if tx.send(frame).is_err() {
                    break;
                }
            }
        });
        Ok(TopicIterator::new(topic_str, rx))
    }

    /// Unsubscribes from a simulation topic.
    pub fn unsubscribe(&self, topic: impl AsRef<str>) -> Result<()> {
        self.runtime.block_on(self.inner.unsubscribe(topic))
    }

    /// Unsubscribes from multiple simulation topics in a single request.
    pub fn unsubscribe_topics(&self, topics: &[impl AsRef<str>]) -> Result<()> {
        self.runtime.block_on(self.inner.unsubscribe_topics(topics))
    }

    /// Unsubscribes from all active topics.
    pub fn unsubscribe_all(&self) -> Result<()> {
        self.runtime.block_on(self.inner.unsubscribe_all())
    }

    /// Retrieves simulation topic descriptors published by the server.
    pub fn get_topic_info(&self) -> Result<Vec<TopicInfo>> {
        self.runtime.block_on(self.inner.get_topic_info())
    }

    /// Retrieves the list of active simulation topic paths discovered via `/$topics`.
    pub fn get_topic_paths(&self) -> Result<Vec<String>> {
        self.runtime.block_on(self.inner.get_topic_paths())
    }

    /// Returns the set of currently subscribed topic names.
    pub fn get_active_subscriptions(&self) -> Vec<String> {
        self.runtime.block_on(self.inner.get_active_subscriptions())
    }

    /// Tests connection liveness with the simulation server.
    pub fn ping(&self) -> Result<bool> {
        self.runtime.block_on(self.inner.ping())
    }

    /// Retrieves the simulation server's git commit hash.
    pub fn get_build_commit_hash(&self) -> Result<String> {
        self.runtime.block_on(self.inner.get_build_commit_hash())
    }

    /// Enables or disables an interactive feature on the simulation server.
    pub fn set_interactive_feature(&self, feature_id: &str, enable: bool) -> Result<bool> {
        self.runtime.block_on(self.inner.set_interactive_feature(feature_id, enable))
    }

    /// Loads a new scene configuration into the simulation.
    pub fn request_load_scene(&self, scene_config: &str) -> Result<String> {
        self.runtime.block_on(self.inner.request_load_scene(scene_config))
    }
}
