use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
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

/// Actor managing NNG sockets over dedicated worker threads,
/// exposing asynchronous Tokio channels for RPC and Topics.
pub struct NngActor {
    transport: Arc<NngTransport>,
    rpc_tx: mpsc::Sender<RpcRequest>,
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

        let is_running = Arc::new(AtomicBool::new(true));
        let (rpc_tx, mut rpc_rx) = mpsc::channel::<RpcRequest>(128);
        let (topic_tx, mut topic_out_rx) = mpsc::channel::<Vec<u8>>(128);
        let (topic_broadcast, _) = broadcast::channel::<TopicFrame>(2048);
        let topic_infos = Arc::new(RwLock::new(Vec::new()));

        // 1. RPC Worker Thread
        let rpc_transport = Arc::clone(&transport);
        let rpc_running = Arc::clone(&is_running);
        let rpc_thread = std::thread::Builder::new()
            .name("airsim-rpc-actor".into())
            .spawn(move || {
                debug!("RPC actor thread started");
                while rpc_running.load(Ordering::Relaxed) {
                    match rpc_rx.blocking_recv() {
                        Some(RpcRequest { payload, reply }) => {
                            if payload.is_empty() {
                                break; // Shutdown sentinel received
                            }
                            let res = rpc_transport.send_request_sync(&payload);
                            let _ = reply.send(res);
                        }
                        None => break, // Channel closed
                    }
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
                                        if let Ok(infos) = rmp_serde::from_slice::<Vec<TopicInfo>>(frame.body()) {
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
            rpc_tx,
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
            return Err(SimError::ProtocolError("Cannot send empty RPC payload".into()));
        }
        if !self.is_running.load(Ordering::Relaxed) {
            return Err(SimError::ConnectionClosed);
        }

        let (reply_tx, reply_rx) = oneshot::channel();
        self.rpc_tx
            .send(RpcRequest {
                payload,
                reply: reply_tx,
            })
            .await
            .map_err(|_| SimError::ConnectionClosed)?;

        reply_rx
            .await
            .map_err(|_| SimError::Cancelled)?
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

        // Send sentinel to unblock RPC thread
        let (dummy_tx, _) = oneshot::channel();
        let _ = self.rpc_tx.try_send(RpcRequest {
            payload: Vec::new(),
            reply: dummy_tx,
        });

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
