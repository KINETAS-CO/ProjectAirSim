use super::Transport;
#[cfg(any(feature = "async", feature = "sync"))]
use crate::error::{Result, SimError};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

#[derive(Debug, Default)]
struct MockTransportInner {
    service_responses: Mutex<VecDeque<Vec<u8>>>,
    topic_messages: Mutex<VecDeque<Vec<u8>>>,
    sent_requests: Mutex<Vec<Vec<u8>>>,
    sent_topic_frames: Mutex<Vec<Vec<u8>>>,
}

/// In-memory mock transport mirroring C++ FakeNNGI.
///
/// Enables offline unit testing of request serialization, response parsing,
/// topic dispatching, and error handling without running the simulator.
/// Internally backed by an `Arc`, allowing cheap cloning and thread-safe sharing.
#[derive(Clone, Debug, Default)]
pub struct MockTransport {
    inner: Arc<MockTransportInner>,
}

impl MockTransport {
    pub fn new() -> Self {
        Self::default()
    }

    /// Clears all recorded and queued messages.
    pub fn reset(&self) {
        self.inner
            .service_responses
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
        self.inner
            .topic_messages
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
        self.inner
            .sent_requests
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
        self.inner
            .sent_topic_frames
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clear();
    }

    /// Queues a raw response buffer to be returned by the next `send_request()`.
    pub fn push_service_response(&self, resp: Vec<u8>) {
        self.inner
            .service_responses
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push_back(resp);
    }

    /// Queues a raw topic message buffer to be returned by `recv_topic_frame()`.
    pub fn push_topic_message(&self, msg: Vec<u8>) {
        self.inner
            .topic_messages
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push_back(msg);
    }

    /// Returns a snapshot of all RPC requests sent through this transport.
    pub fn sent_requests(&self) -> Vec<Vec<u8>> {
        self.inner
            .sent_requests
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }

    /// Returns a snapshot of all topic frames sent through this transport.
    pub fn sent_topic_frames(&self) -> Vec<Vec<u8>> {
        self.inner
            .sent_topic_frames
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .clone()
    }
}

#[cfg_attr(feature = "async", async_trait::async_trait)]
impl Transport for MockTransport {
    #[cfg(feature = "async")]
    async fn send_request(&self, req_bytes: &[u8]) -> Result<Vec<u8>> {
        self.inner
            .sent_requests
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(req_bytes.to_vec());
        let mut queue = self
            .inner
            .service_responses
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        queue.pop_front().ok_or_else(|| {
            SimError::TransportError("MockTransport: No queued service response available".into())
        })
    }

    #[cfg(feature = "async")]
    async fn send_topic_frame(&self, frame_bytes: &[u8]) -> Result<()> {
        self.inner
            .sent_topic_frames
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(frame_bytes.to_vec());
        Ok(())
    }

    #[cfg(feature = "async")]
    async fn recv_topic_frame(&self, _timeout_ms: u32) -> Result<Option<Vec<u8>>> {
        let mut queue = self
            .inner
            .topic_messages
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        Ok(queue.pop_front())
    }

    #[cfg(feature = "sync")]
    fn send_request_sync(&self, req_bytes: &[u8]) -> Result<Vec<u8>> {
        self.inner
            .sent_requests
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(req_bytes.to_vec());
        let mut queue = self
            .inner
            .service_responses
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        queue.pop_front().ok_or_else(|| {
            SimError::TransportError("MockTransport: No queued service response available".into())
        })
    }

    #[cfg(feature = "sync")]
    fn send_topic_frame_sync(&self, frame_bytes: &[u8]) -> Result<()> {
        self.inner
            .sent_topic_frames
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .push(frame_bytes.to_vec());
        Ok(())
    }

    #[cfg(feature = "sync")]
    fn recv_topic_frame_sync(&self, _timeout_ms: u32) -> Result<Option<Vec<u8>>> {
        let mut queue = self
            .inner
            .topic_messages
            .lock()
            .unwrap_or_else(|e| e.into_inner());
        Ok(queue.pop_front())
    }
}
