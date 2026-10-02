use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::JoinHandle;
use tokio::sync::{broadcast, mpsc, oneshot, RwLock};
use tracing::{debug, warn};

use crate::error::{Result, SimError};
use crate::protocol::frame::{TopicFrame, TopicInfo};
use crate::transport::nng_transport::NngTransport;

struct RpcRequest {
    payload: Vec<u8>,
    reply: oneshot::Sender<Result<Vec<u8>>>,
}

struct RpcQueueState {
    priority: VecDeque<RpcRequest>,
    normal: VecDeque<RpcRequest>,
    is_running: bool,
}

/// Actor managing NNG sockets over dedicated worker threads,
/// exposing asynchronous Tokio channels for RPC and Topics.
pub struct NngActor {
    transport: Arc<NngTransport>,
    rpc_queue: Arc<(Mutex<RpcQueueState>, Condvar)>,
    topic_tx: mpsc::Sender<Vec<u8>>,
    topic_broadcast: broadcast::Sender<TopicFrame>,
    topic_infos: Arc<RwLock<Vec<TopicInfo>>>,
    is_running: Arc<AtomicBool>,
    rpc_thread: Option<JoinHandle<()>>,
    topics_thread: Option<JoinHandle<()>>,
}

impl NngActor {
    /// Starts the NNG actor, connecting to the simulation server.
    pub fn start(address: &str, port_topics: u16, port_services: u16) -> Result<Self> {
        let transport = Arc::new(NngTransport::connect(address, port_topics, port_services)?);
        Self::from_transport(transport)
    }

    /// Creates an NNG actor wrapping an existing NngTransport instance.
    pub fn from_transport(transport: Arc<NngTransport>) -> Result<Self> {
        let is_running = Arc::new(AtomicBool::new(true));
        let rpc_queue = Arc::new((
            Mutex::new(RpcQueueState {
                priority: VecDeque::new(),
                normal: VecDeque::new(),
                is_running: true,
            }),
            Condvar::new(),
        ));
        let (topic_tx, mut topic_out_rx) = mpsc::channel::<Vec<u8>>(128);
        let (topic_broadcast, _) = broadcast::channel::<TopicFrame>(2048);
        let topic_infos = Arc::new(RwLock::new(Vec::new()));

        // 1. RPC Worker Thread
        let rpc_transport = Arc::clone(&transport);
        let rpc_running = Arc::clone(&is_running);
        let rpc_queue_clone = Arc::clone(&rpc_queue);
        let rpc_thread = std::thread::Builder::new()
            .name("airsim-rpc-actor".into())
            .spawn(move || {
                debug!("RPC actor thread started");
                let (lock, cvar) = &*rpc_queue_clone;
                while rpc_running.load(Ordering::Relaxed) {
                    let req = {
                        let mut state = lock.lock().unwrap();
                        while state.is_running && state.priority.is_empty() && state.normal.is_empty() {
                            state = cvar.wait(state).unwrap();
                        }
                        if !state.is_running && state.priority.is_empty() && state.normal.is_empty() {
                            break;
                        }
                        if let Some(req) = state.priority.pop_front() {
                            req
                        } else if let Some(req) = state.normal.pop_front() {
                            req
                        } else {
                            continue;
                        }
                    };

                    if req.payload.is_empty() {
                        break;
                    }
                    let res = rpc_transport.send_request_sync(&req.payload);
                    let _ = req.reply.send(res);
                }
                debug!("RPC actor thread exiting");
            })
            .map_err(|e| SimError::TransportError(format!("Failed to spawn RPC thread: {e}")))?;

        // 2. Topics Worker Thread (receives incoming frames and sends outgoing frames)
        let topic_transport = Arc::clone(&transport);
        let topics_running = Arc::clone(&is_running);
        let topic_bcast = topic_broadcast.clone();
        let topic_info_cache = Arc::clone(&topic_infos);

        let topics_thread = std::thread::Builder::new()
            .name("airsim-topics-actor".into())
            .spawn(move || {
                debug!("Topics actor thread started");

                // Automatically subscribe to /$topics to discover active simulation topics
                let sub_frame = TopicFrame::subscribe("/$topics").to_bytes();
                if let Ok(bytes) = sub_frame {
                    let _ = topic_transport.send_topic_frame_sync(&bytes);
                }

                while topics_running.load(Ordering::Relaxed) {
                    // Check for outgoing topic frames
                    while let Ok(frame_bytes) = topic_out_rx.try_recv() {
                        if let Err(e) = topic_transport.send_topic_frame_sync(&frame_bytes) {
                            warn!("Failed to send outgoing topic frame: {e}");
                        }
                    }

                    // Poll for incoming frames with 50ms timeout
                    match topic_transport.recv_topic_frame_sync(50) {
                        Ok(Some(raw_bytes)) => {
                            match TopicFrame::from_bytes(&raw_bytes) {
                                Ok(frame) => {
                                    // If this is the topics directory, update the topic list cache
                                    if frame.topic() == "/$topics" {
                                        if let Ok(infos) =
                                            rmp_serde::from_slice::<Vec<TopicInfo>>(frame.body())
                                        {
                                            if let Ok(mut lock) = topic_info_cache.try_write() {
                                                *lock = infos;
                                            }
                                        }
                                    }

                                    // Broadcast frame to all subscribers
                                    let _ = topic_bcast.send(frame);
                                }
                                Err(e) => {
                                    warn!("Failed to decode received topic frame: {e}");
                                }
                            }
                        }
                        Ok(None) => {
                            // Timeout, continue loop
                        }
                        Err(SimError::TimedOut) => {
                            // Timeout, continue loop
                        }
                        Err(e) => {
                            if topics_running.load(Ordering::Relaxed) {
                                warn!("Error receiving topic frame: {e}");
                                std::thread::sleep(std::time::Duration::from_millis(50));
                            } else {
                                break;
                            }
                        }
                    }
                }
                debug!("Topics actor thread exiting");
            })
            .map_err(|e| SimError::TransportError(format!("Failed to spawn Topics thread: {e}")))?;

        Ok(Self {
            transport,
            rpc_queue,
            topic_tx,
            topic_broadcast,
            topic_infos,
            is_running,
            rpc_thread: Some(rpc_thread),
            topics_thread: Some(topics_thread),
        })
    }

    /// Asynchronously dispatches an RPC request buffer and awaits the response.
    pub async fn send_rpc(&self, payload: Vec<u8>) -> Result<Vec<u8>> {
        if payload.is_empty() {
            return Err(SimError::ProtocolError(
                "Cannot send empty RPC payload".into(),
            ));
        }
        if !self.is_running.load(Ordering::Relaxed) {
            return Err(SimError::ConnectionClosed);
        }

        let (reply_tx, reply_rx) = oneshot::channel();
        {
            let (lock, cvar) = &*self.rpc_queue;
            let mut state = lock.lock().unwrap();
            if !state.is_running {
                return Err(SimError::ConnectionClosed);
            }
            state.normal.push_back(RpcRequest {
                payload,
                reply: reply_tx,
            });
            cvar.notify_one();
        }

        reply_rx.await.map_err(|_| SimError::Cancelled)?
    }

    /// Asynchronously dispatches a high-priority RPC request, jumping ahead of queued normal requests.
    pub async fn send_rpc_priority(&self, payload: Vec<u8>) -> Result<Vec<u8>> {
        if payload.is_empty() {
            return Err(SimError::ProtocolError(
                "Cannot send empty RPC payload".into(),
            ));
        }
        if !self.is_running.load(Ordering::Relaxed) {
            return Err(SimError::ConnectionClosed);
        }

        let (reply_tx, reply_rx) = oneshot::channel();
        {
            let (lock, cvar) = &*self.rpc_queue;
            let mut state = lock.lock().unwrap();
            if !state.is_running {
                return Err(SimError::ConnectionClosed);
            }
            state.priority.push_back(RpcRequest {
                payload,
                reply: reply_tx,
            });
            cvar.notify_one();
        }

        reply_rx.await.map_err(|_| SimError::Cancelled)?
    }

    /// Cancels all pending queued requests that have not yet been sent to the server.
    pub fn cancel_all_requests(&self) {
        let (lock, _) = &*self.rpc_queue;
        let mut state = lock.lock().unwrap();
        for req in state.priority.drain(..) {
            let _ = req.reply.send(Err(SimError::Cancelled));
        }
        for req in state.normal.drain(..) {
            let _ = req.reply.send(Err(SimError::Cancelled));
        }
    }

    /// Asynchronously sends an outgoing topic frame (e.g. Subscribe, Unsubscribe, Publish).
    pub async fn send_topic_frame(&self, frame: TopicFrame) -> Result<()> {
        if !self.is_running.load(Ordering::Relaxed) {
            return Err(SimError::ConnectionClosed);
        }

        let bytes = frame.to_bytes()?;
        self.topic_tx
            .send(bytes)
            .await
            .map_err(|_| SimError::ConnectionClosed)?;
        Ok(())
    }

    /// Returns a broadcast receiver for incoming topic frames.
    pub fn subscribe_broadcast(&self) -> broadcast::Receiver<TopicFrame> {
        self.topic_broadcast.subscribe()
    }

    /// Returns cached topic info entries.
    pub async fn get_topic_info(&self) -> Vec<TopicInfo> {
        self.topic_infos.read().await.clone()
    }

    /// Shuts down the actor and its worker threads.
    pub fn shutdown(&mut self) {
        if !self.is_running.swap(false, Ordering::Relaxed) {
            return;
        }

        // Unblock RPC thread and cancel any pending requests
        {
            let (lock, cvar) = &*self.rpc_queue;
            let mut state = lock.lock().unwrap();
            state.is_running = false;
            for req in state.priority.drain(..) {
                let _ = req.reply.send(Err(SimError::Cancelled));
            }
            for req in state.normal.drain(..) {
                let _ = req.reply.send(Err(SimError::Cancelled));
            }
            cvar.notify_all();
        }

        // Close sockets to immediately unblock any thread waiting on recv
        self.transport.close();

        if let Some(handle) = self.rpc_thread.take() {
            let _ = handle.join();
        }
        if let Some(handle) = self.topics_thread.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for NngActor {
    fn drop(&mut self) {
        self.shutdown();
    }
}
