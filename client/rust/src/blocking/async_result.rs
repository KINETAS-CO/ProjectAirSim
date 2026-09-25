use crate::error::{Result, SimError};
use std::sync::mpsc::{Receiver, RecvTimeoutError};
use std::time::Duration;

/// Synchronization handle for asynchronous operations initiated in blocking mode.
///
/// Mirrors the C++ ProjectAirSim `AsyncResult` API, allowing callers to wait
/// synchronously, query completion status, or block with a timeout.
pub struct AsyncResult<T> {
    receiver: Receiver<Result<T>>,
    cached: Option<Result<T>>,
}

impl<T> AsyncResult<T> {
    pub fn new(receiver: Receiver<Result<T>>) -> Self {
        Self {
            receiver,
            cached: None,
        }
    }

    /// Blocks the current thread until the operation finishes.
    pub fn wait(&mut self) -> Result<()> {
        if self.cached.is_none() {
            let res = self.receiver.recv().map_err(|_| SimError::Cancelled)?;
            self.cached = Some(res);
        }
        match self.cached.as_ref().unwrap() {
            Ok(_) => Ok(()),
            Err(e) => Err(e.clone()),
        }
    }

    /// Blocks the current thread for up to `timeout` duration.
    pub fn wait_timeout(&mut self, timeout: Duration) -> Result<()> {
        if self.cached.is_none() {
            let res = self.receiver.recv_timeout(timeout).map_err(|e| match e {
                RecvTimeoutError::Timeout => SimError::TimedOut,
                RecvTimeoutError::Disconnected => SimError::Cancelled,
            })?;
            self.cached = Some(res);
        }
        match self.cached.as_ref().unwrap() {
            Ok(_) => Ok(()),
            Err(e) => Err(e.clone()),
        }
    }

    /// Blocks until completion and returns the unpacked result value.
    pub fn get_result(mut self) -> Result<T> {
        self.wait()?;
        self.cached.unwrap()
    }

    /// Returns `true` if the operation has completed.
    pub fn is_done(&self) -> bool {
        self.cached.is_some()
    }
}
