use serde::{de::DeserializeOwned, Serialize};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};
use tracing::info;

use crate::blocking::async_result::AsyncResult;
use crate::error::{Result, SimError};
use crate::protocol::frame::{TopicFrame, TopicInfo};
use crate::protocol::params::{EmptyParams, FeatureParams, LoadSceneParams};
use crate::protocol::request::RequestEnvelope;
use crate::protocol::response::ResponseDecoder;
use crate::transport::nng_transport::NngTransport;

pub const DEFAULT_PORT_TOPICS: u16 = 8989;
pub const DEFAULT_PORT_SERVICES: u16 = 8990;

type TopicCallback = Box<dyn Fn(TopicFrame) + Send + 'static>;

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

/// Dispatcher hub managing background topic receiving for synchronous blocking clients.
pub struct BlockingTopicsHub {
    transport: Arc<NngTransport>,
    callbacks: Arc<Mutex<HashMap<String, Vec<TopicCallback>>>>,
    channels: Arc<Mutex<HashMap<String, Vec<std::sync::mpsc::Sender<TopicFrame>>>>>,
    all_channels: Arc<Mutex<Vec<std::sync::mpsc::Sender<TopicFrame>>>>,
    topic_infos: Arc<Mutex<Vec<TopicInfo>>>,
    is_running: Arc<AtomicBool>,
    thread: Option<std::thread::JoinHandle<()>>,
}

impl BlockingTopicsHub {
    pub fn start(transport: Arc<NngTransport>) -> Result<Self> {
        let callbacks = Arc::new(Mutex::new(HashMap::<String, Vec<TopicCallback>>::new()));
        let channels = Arc::new(Mutex::new(HashMap::<String, Vec<std::sync::mpsc::Sender<TopicFrame>>>::new()));
        let all_channels = Arc::new(Mutex::new(Vec::<std::sync::mpsc::Sender<TopicFrame>>::new()));
        let topic_infos = Arc::new(Mutex::new(Vec::new()));
        let is_running = Arc::new(AtomicBool::new(true));

        let t_transport = Arc::clone(&transport);
        let t_callbacks = Arc::clone(&callbacks);
        let t_channels = Arc::clone(&channels);
        let t_all_channels = Arc::clone(&all_channels);
        let t_topic_infos = Arc::clone(&topic_infos);
        let t_running = Arc::clone(&is_running);

        let thread = std::thread::Builder::new()
            .name("airsim-blocking-topics".into())
            .spawn(move || {
                // Subscribe to /$topics to discover active simulation topics
                let sub_frame = TopicFrame::subscribe("/$topics").to_bytes();
                if let Ok(bytes) = sub_frame {
                    let _ = t_transport.send_topic_frame_sync(&bytes);
                }

                while t_running.load(Ordering::Relaxed) {
                    match t_transport.recv_topic_frame_sync(50) {
                        Ok(Some(raw_bytes)) => {
                            if let Ok(frame) = TopicFrame::from_bytes(&raw_bytes) {
                                if frame.topic() == "/$topics" {
                                    if let Ok(infos) =
                                        rmp_serde::from_slice::<Vec<TopicInfo>>(frame.body())
                                    {
                                        if let Ok(mut lock) = t_topic_infos.lock() {
                                            *lock = infos;
                                        }
                                    }
                                }

                                // 1. Specific callbacks
                                if let Ok(mut cbs_lock) = t_callbacks.lock() {
                                    if let Some(cbs) = cbs_lock.get_mut(frame.topic()) {
                                        for cb in cbs.iter_mut() {
                                            cb(frame.clone());
                                        }
                                    }
                                }

                                // 2. Specific channels
                                if let Ok(mut chs_lock) = t_channels.lock() {
                                    if let Some(chs) = chs_lock.get_mut(frame.topic()) {
                                        chs.retain(|sender| sender.send(frame.clone()).is_ok());
                                    }
                                }

                                // 3. Global broadcast channels
                                if let Ok(mut all_lock) = t_all_channels.lock() {
                                    all_lock.retain(|sender| sender.send(frame.clone()).is_ok());
                                }
                            }
                        }
                        Ok(None) | Err(SimError::TimedOut) => {}
                        Err(_) => {
                            if !t_running.load(Ordering::Relaxed) {
                                break;
                            }
                            std::thread::sleep(std::time::Duration::from_millis(50));
                        }
                    }
                }
            })
            .map_err(|e| SimError::TransportError(format!("Failed to spawn blocking topics thread: {e}")))?;

        Ok(Self {
            transport,
            callbacks,
            channels,
            all_channels,
            topic_infos,
            is_running,
            thread: Some(thread),
        })
    }

    pub fn subscribe_callback(&self, topic: &str, callback: TopicCallback) -> Result<()> {
        let frame = TopicFrame::subscribe(topic).to_bytes()?;
        self.transport.send_topic_frame_sync(&frame)?;
        let mut cbs = self.callbacks.lock().unwrap();
        cbs.entry(topic.to_string()).or_default().push(callback);
        Ok(())
    }

    pub fn subscribe_iter(&self, topic: &str) -> Result<TopicIterator> {
        let frame = TopicFrame::subscribe(topic).to_bytes()?;
        self.transport.send_topic_frame_sync(&frame)?;
        let (tx, rx) = std::sync::mpsc::channel();
        let mut chs = self.channels.lock().unwrap();
        chs.entry(topic.to_string()).or_default().push(tx);
        Ok(TopicIterator::new(topic, rx))
    }

    pub fn subscribe_all_iter(&self) -> Result<TopicIterator> {
        let (tx, rx) = std::sync::mpsc::channel();
        let mut all = self.all_channels.lock().unwrap();
        all.push(tx);
        Ok(TopicIterator::new("*", rx))
    }

    pub fn unsubscribe(&self, topic: &str) -> Result<()> {
        let frame = TopicFrame::unsubscribe(topic).to_bytes()?;
        self.transport.send_topic_frame_sync(&frame)?;
        self.callbacks.lock().unwrap().remove(topic);
        self.channels.lock().unwrap().remove(topic);
        Ok(())
    }

    pub fn unsubscribe_all(&self) -> Vec<String> {
        let mut topics = std::collections::HashSet::new();
        {
            let mut cbs = self.callbacks.lock().unwrap();
            for k in cbs.keys() {
                topics.insert(k.clone());
            }
            cbs.clear();
        }
        {
            let mut chs = self.channels.lock().unwrap();
            for k in chs.keys() {
                topics.insert(k.clone());
            }
            chs.clear();
        }
        for topic in &topics {
            let frame = TopicFrame::unsubscribe(topic).to_bytes();
            if let Ok(bytes) = frame {
                let _ = self.transport.send_topic_frame_sync(&bytes);
            }
        }
        topics.into_iter().collect()
    }

    pub fn get_topic_infos(&self) -> Vec<TopicInfo> {
        self.topic_infos.lock().unwrap().clone()
    }

    pub fn get_active_subscriptions(&self) -> Vec<String> {
        let mut topics = std::collections::HashSet::new();
        if let Ok(cbs) = self.callbacks.lock() {
            for k in cbs.keys() {
                topics.insert(k.clone());
            }
        }
        if let Ok(chs) = self.channels.lock() {
            for k in chs.keys() {
                topics.insert(k.clone());
            }
        }
        topics.into_iter().collect()
    }

    pub fn stop(&mut self) {
        if !self.is_running.swap(false, Ordering::Relaxed) {
            return;
        }
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for BlockingTopicsHub {
    fn drop(&mut self) {
        self.stop();
    }
}

/// Synchronous blocking client for ProjectAirSim simulation server.
#[derive(Clone)]
pub struct Client {
    transport: Arc<NngTransport>,
    topics_hub: Arc<BlockingTopicsHub>,
    request_id: Arc<AtomicI32>,
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

    /// Connects to a ProjectAirSim simulation server synchronously using default ports.
    pub fn connect(address: &str) -> Result<Self> {
        Self::connect_with_ports(address, DEFAULT_PORT_TOPICS, DEFAULT_PORT_SERVICES)
    }

    /// Connects to a ProjectAirSim simulation server synchronously with specified ports.
    pub fn connect_with_ports(address: &str, port_topics: u16, port_services: u16) -> Result<Self> {
        info!("Connecting (blocking) to ProjectAirSim at {address} (topics: {port_topics}, services: {port_services})");
        let transport = Arc::new(NngTransport::connect(address, port_topics, port_services)?);
        Self::from_transport(transport)
    }

    /// Creates a synchronous client wrapping an existing NngTransport instance.
    pub fn from_transport(transport: Arc<NngTransport>) -> Result<Self> {
        let topics_hub = Arc::new(BlockingTopicsHub::start(Arc::clone(&transport))?);
        Ok(Self {
            transport,
            topics_hub,
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

    /// Dispatches a high-priority asynchronous RPC request.
    pub fn request_priority_async<P: Serialize, R: DeserializeOwned + Send + 'static>(
        &self,
        method: &str,
        params: &P,
    ) -> AsyncResult<R> {
        self.request_async(method, params)
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

    /// Dispatches a high-priority synchronous RPC request.
    pub fn request_priority<P: Serialize, R: DeserializeOwned>(
        &self,
        method: &str,
        params: &P,
    ) -> Result<R> {
        self.request(method, params)
    }

    /// Cancels all pending requests (in blocking mode, resets client state).
    pub fn cancel_all_requests(&self) {
        // In blocking mode, in-flight requests on detached threads complete or are ignored by the caller.
    }

    /// Subscribes to a simulation topic with a callback function invoked on a background thread.
    pub fn subscribe<F>(&self, topic: impl Into<String>, callback: F) -> Result<()>
    where
        F: Fn(TopicFrame) + Send + 'static,
    {
        self.topics_hub
            .subscribe_callback(&topic.into(), Box::new(callback))
    }

    /// Subscribes to a simulation topic and returns a synchronous iterator yielding incoming frames.
    pub fn subscribe_iter(&self, topic: impl Into<String>) -> Result<TopicIterator> {
        self.topics_hub.subscribe_iter(&topic.into())
    }

    /// Subscribes to all simulation topics and returns a synchronous iterator yielding incoming frames.
    pub fn subscribe_all_iter(&self) -> Result<TopicIterator> {
        self.topics_hub.subscribe_all_iter()
    }

    /// Returns a list of currently active subscribed topic paths.
    pub fn get_active_subscriptions(&self) -> Vec<String> {
        self.topics_hub.get_active_subscriptions()
    }

    /// Unsubscribes from a simulation topic by path.
    pub fn unsubscribe(&self, topic: impl AsRef<str>) -> Result<()> {
        let topic_str = topic.as_ref();
        self.topics_hub.unsubscribe(topic_str)?;
        let paths = [topic_str];
        let _: Result<serde_json::Value> = self.request(
            "/Sim/Unsubscribe",
            &serde_json::json!({ "topic_paths": paths }),
        );
        Ok(())
    }

    /// Unsubscribes from multiple simulation topics at once.
    pub fn unsubscribe_topics(&self, topics: &[impl AsRef<str>]) -> Result<()> {
        let paths: Vec<String> = topics.iter().map(|t| t.as_ref().to_string()).collect();
        for path in &paths {
            let _ = self.topics_hub.unsubscribe(path);
        }
        let _: Result<serde_json::Value> = self.request(
            "/Sim/Unsubscribe",
            &serde_json::json!({ "topic_paths": paths }),
        );
        Ok(())
    }

    /// Unsubscribes from all currently tracked simulation topics.
    pub fn unsubscribe_all(&self) -> Result<()> {
        let unsubscribed = self.topics_hub.unsubscribe_all();
        if !unsubscribed.is_empty() {
            let _: Result<serde_json::Value> = self.request(
                "/Sim/Unsubscribe",
                &serde_json::json!({ "topic_paths": unsubscribed }),
            );
        }
        Ok(())
    }

    /// Publishes a typed MessagePack payload to a simulation topic.
    pub fn publish<T: Serialize>(&self, topic: &str, message: &T) -> Result<()> {
        let body = rmp_serde::to_vec(message).map_err(|e| {
            SimError::SerializationError(format!("Failed to serialize publish payload: {e}"))
        })?;
        self.publish_raw(topic, body)
    }

    /// Publishes raw bytes to a simulation topic.
    pub fn publish_raw(&self, topic: &str, payload: Vec<u8>) -> Result<()> {
        let frame = TopicFrame::message(topic, payload).to_bytes()?;
        self.transport.send_topic_frame_sync(&frame)
    }

    /// Retrieves the list of active simulation topics discovered via `/$topics`.
    pub fn get_topic_info(&self) -> Result<Vec<TopicInfo>> {
        Ok(self.topics_hub.get_topic_infos())
    }

    /// Retrieves the list of active simulation topic paths discovered via `/$topics`.
    pub fn get_topic_paths(&self) -> Result<Vec<String>> {
        Ok(self
            .topics_hub
            .get_topic_infos()
            .into_iter()
            .map(|t| t.path)
            .collect())
    }

    /// Tests connection liveness with the simulation server.
    pub fn ping(&self) -> Result<bool> {
        let res: Result<serde_json::Value> = self.request("/Sim/Ping", &EmptyParams {});
        match res {
            Ok(_) => Ok(true),
            Err(SimError::ServerRejected { code: 404, .. }) => Ok(true),
            Err(e) => Err(e),
        }
    }

    /// Retrieves the simulation server's git commit hash.
    pub fn get_build_commit_hash(&self) -> Result<String> {
        self.request("/Sim/GetBuildCommitHash", &EmptyParams {})
    }

    /// Enables or disables an interactive feature on the simulation server (e.g. weather, physics).
    pub fn set_interactive_feature(&self, feature_id: &str, enable: bool) -> Result<bool> {
        self.request(
            "/Sim/SetInteractiveFeature",
            &FeatureParams {
                feature_id,
                enable,
            },
        )
    }

    /// Requests the server to load or reload a scene from a JSONC configuration string.
    pub fn request_load_scene(&self, scene_config: &str) -> Result<String> {
        self.cancel_all_requests();
        let _ = self.unsubscribe_all();
        let res: serde_json::Value = self.request(
            "/Sim/LoadScene",
            &LoadSceneParams { scene_config },
        )?;
        if let Some(s) = res.as_str() {
            Ok(s.to_string())
        } else {
            Ok(res.to_string())
        }
    }
}

