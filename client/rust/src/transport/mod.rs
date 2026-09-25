pub mod mock_transport;
pub use mock_transport::MockTransport;

pub mod nng_transport;
pub use nng_transport::NngTransport;

#[cfg(feature = "async")]
pub mod nng_actor;
#[cfg(feature = "async")]
pub use nng_actor::NngActor;

use crate::error::Result;

#[cfg_attr(feature = "async", async_trait::async_trait)]
pub trait Transport: Send + Sync {
    /// Sends an RPC request buffer and returns the raw response buffer.
    #[cfg(feature = "async")]
    async fn send_request(&self, req_bytes: &[u8]) -> Result<Vec<u8>>;

    /// Sends a topic frame (e.g. subscribe, unsubscribe, message).
    #[cfg(feature = "async")]
    async fn send_topic_frame(&self, frame_bytes: &[u8]) -> Result<()>;

    /// Receives the next available topic frame, waiting up to `timeout_ms`.
    #[cfg(feature = "async")]
    async fn recv_topic_frame(&self, timeout_ms: u32) -> Result<Option<Vec<u8>>>;

    /// Blocking RPC request (for sync mode).
    #[cfg(feature = "sync")]
    fn send_request_sync(&self, req_bytes: &[u8]) -> Result<Vec<u8>>;

    /// Blocking topic frame send (for sync mode).
    #[cfg(feature = "sync")]
    fn send_topic_frame_sync(&self, frame_bytes: &[u8]) -> Result<()>;

    /// Blocking topic frame receive (for sync mode).
    #[cfg(feature = "sync")]
    fn recv_topic_frame_sync(&self, timeout_ms: u32) -> Result<Option<Vec<u8>>>;
}
