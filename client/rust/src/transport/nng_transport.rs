use std::time::Duration;
use nng::{Protocol, Socket, options::{Options, RecvTimeout, SendTimeout}};
use crate::error::{Result, SimError};

/// Low-level NNG transport wrapping Req0 (RPC services) and Pair0 (Topics).
pub struct NngTransport {
    req_socket: Socket,
    pair_socket: Socket,
}

impl NngTransport {
    /// Creates and dials a new NNG connection to the ProjectAirSim server.
    pub fn connect(address: &str, port_topics: u16, port_services: u16) -> Result<Self> {
        let req_url = format!("tcp://{}:{}", address, port_services);
        let pair_url = format!("tcp://{}:{}", address, port_topics);

        // Open Req0 socket for RPC
        let req_socket = Socket::new(Protocol::Req0)
            .map_err(|e| SimError::TransportError(format!("Failed to open Req0 socket: {e}")))?;

        // Open Pair0 socket for Topics
        let pair_socket = Socket::new(Protocol::Pair0)
            .map_err(|e| SimError::TransportError(format!("Failed to open Pair0 socket: {e}")))?;

        // Set default timeouts (60s recv, 10s send)
        let _ = req_socket.set_opt::<RecvTimeout>(Some(Duration::from_secs(60)));
        let _ = req_socket.set_opt::<SendTimeout>(Some(Duration::from_secs(10)));
        let _ = pair_socket.set_opt::<SendTimeout>(Some(Duration::from_secs(10)));
        let _ = pair_socket.set_opt::<RecvTimeout>(Some(Duration::from_millis(500)));

        // Dial RPC services
        req_socket.dial(&req_url)
            .map_err(|e| SimError::TransportError(format!("Failed to dial RPC services at {req_url}: {e}")))?;

        // Dial Topics
        pair_socket.dial(&pair_url)
            .map_err(|e| SimError::TransportError(format!("Failed to dial Topics at {pair_url}: {e}")))?;

        Ok(Self {
            req_socket,
            pair_socket,
        })
    }

    /// Sends a raw request over Req0 and blocks for the reply.
    pub fn send_request_sync(&self, req_bytes: &[u8]) -> Result<Vec<u8>> {
        self.req_socket.send(req_bytes)
            .map_err(|(_, e)| SimError::TransportError(format!("Failed to send RPC request: {e}")))?;

        let msg = self.req_socket.recv()
            .map_err(|e| SimError::TransportError(format!("Failed to receive RPC response: {e}")))?;

        Ok(msg.as_slice().to_vec())
    }

    /// Sends a topic frame over Pair0.
    pub fn send_topic_frame_sync(&self, frame_bytes: &[u8]) -> Result<()> {
        self.pair_socket.send(frame_bytes)
            .map_err(|(_, e)| SimError::TransportError(format!("Failed to send topic frame: {e}")))?;
        Ok(())
    }

    /// Receives a topic frame over Pair0, waiting up to `timeout_ms`.
    pub fn recv_topic_frame_sync(&self, timeout_ms: u32) -> Result<Option<Vec<u8>>> {
        let _ = self.pair_socket.set_opt::<RecvTimeout>(Some(Duration::from_millis(timeout_ms as u64)));
        match self.pair_socket.recv() {
            Ok(msg) => Ok(Some(msg.as_slice().to_vec())),
            Err(nng::Error::TimedOut) => Ok(None),
            Err(e) => Err(SimError::TransportError(format!("Failed to receive topic frame: {e}"))),
        }
    }

    /// Closes the sockets.
    pub fn close(&self) {
        let _ = self.req_socket.close();
        let _ = self.pair_socket.close();
    }
}
