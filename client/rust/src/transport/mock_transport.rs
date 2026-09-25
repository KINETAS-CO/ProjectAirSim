use std::collections::VecDeque;
use std::sync::Mutex;
use crate::error::{Result, SimError};
use super::Transport;

/// In-memory mock transport mirroring C++ FakeNNGI.
///
/// Enables offline unit testing of request serialization, response parsing,
/// topic dispatching, and error handling without running the simulator.
#[derive(Debug, Default)]
pub struct MockTransport {
    service_responses: Mutex<VecDeque<Vec<u8>>>,
    topic_messages: Mutex<VecDeque<Vec<u8>>>,
    sent_requests: Mutex<Vec<Vec<u8>>>,
    sent_topic_frames: Mutex<Vec<Vec<u8>>>,
}

impl MockTransport {
    pub fn new() -> Self {
        Self::default()
    }

    /// Clears all recorded and queued messages.
    pub fn reset(&self) {
        self.service_responses.lock().unwrap().clear();
        self.topic_messages.lock().unwrap().clear();
        self.sent_requests.lock().unwrap().clear();
        self.sent_topic_frames.lock().unwrap().clear();
    }

    /// Queues a raw response buffer to be returned by the next `send_request()`.
    pub fn push_service_response(&self, resp: Vec<u8>) {
        self.service_responses.lock().unwrap().push_back(resp);
    }

    /// Queues a raw topic message buffer to be returned by `recv_topic_frame()`.
    pub fn push_topic_message(&self, msg: Vec<u8>) {
        self.topic_messages.lock().unwrap().push_back(msg);
    }

    /// Returns a snapshot of all RPC requests sent through this transport.
    pub fn sent_requests(&self) -> Vec<Vec<u8>> {
        self.sent_requests.lock().unwrap().clone()
    }

    /// Returns a snapshot of all topic frames sent through this transport.
    pub fn sent_topic_frames(&self) -> Vec<Vec<u8>> {
        self.sent_topic_frames.lock().unwrap().clone()
    }
}

#[cfg_attr(feature = "async", async_trait::async_trait)]
impl Transport for MockTransport {
    #[cfg(feature = "async")]
    async fn send_request(&self, req_bytes: &[u8]) -> Result<Vec<u8>> {
        self.sent_requests.lock().unwrap().push(req_bytes.to_vec());
        let mut queue = self.service_responses.lock().unwrap();
        queue.pop_front().ok_or_else(|| {
            SimError::TransportError("MockTransport: No queued service response available".into())
        })
    }

    #[cfg(feature = "async")]
    async fn send_topic_frame(&self, frame_bytes: &[u8]) -> Result<()> {
        self.sent_topic_frames.lock().unwrap().push(frame_bytes.to_vec());
        Ok(())
    }

    #[cfg(feature = "async")]
    async fn recv_topic_frame(&self, _timeout_ms: u32) -> Result<Option<Vec<u8>>> {
        let mut queue = self.topic_messages.lock().unwrap();
        Ok(queue.pop_front())
    }

    #[cfg(feature = "sync")]
    fn send_request_sync(&self, req_bytes: &[u8]) -> Result<Vec<u8>> {
        self.sent_requests.lock().unwrap().push(req_bytes.to_vec());
        let mut queue = self.service_responses.lock().unwrap();
        queue.pop_front().ok_or_else(|| {
            SimError::TransportError("MockTransport: No queued service response available".into())
        })
    }

    #[cfg(feature = "sync")]
    fn send_topic_frame_sync(&self, frame_bytes: &[u8]) -> Result<()> {
        self.sent_topic_frames.lock().unwrap().push(frame_bytes.to_vec());
        Ok(())
    }

    #[cfg(feature = "sync")]
    fn recv_topic_frame_sync(&self, _timeout_ms: u32) -> Result<Option<Vec<u8>>> {
        let mut queue = self.topic_messages.lock().unwrap();
        Ok(queue.pop_front())
    }
}
