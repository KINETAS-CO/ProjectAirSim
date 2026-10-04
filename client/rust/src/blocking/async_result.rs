use crate::error::{Result, SimError};
use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::sync::Mutex;
use std::time::Duration;

/// Synchronization handle for asynchronous operations initiated in blocking mode.
///
/// Mirrors the C++ ProjectAirSim `AsyncResult` API, allowing callers to wait
/// synchronously, query completion status, or block with a timeout.
pub struct AsyncResult<T> {
    receiver: Receiver<Result<T>>,
    cached: Mutex<Option<Result<T>>>,
}

impl<T> AsyncResult<T> {
    pub fn new(receiver: Receiver<Result<T>>) -> Self {
        Self {
            receiver,
            cached: Mutex::new(None),
        }
    }

    /// Blocks the current thread until the operation finishes.
    pub fn wait(&mut self) -> Result<()> {
        let mut guard = match self.cached.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        if guard.is_none() {
            let res = self.receiver.recv().map_err(|_| SimError::Cancelled)?;
            *guard = Some(res);
        }
        match guard.as_ref().unwrap() {
            Ok(_) => Ok(()),
            Err(e) => Err(e.clone()),
        }
    }

    /// Blocks the current thread for up to `timeout` duration.
    pub fn wait_timeout(&mut self, timeout: Duration) -> Result<()> {
        let mut guard = match self.cached.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        if guard.is_none() {
            let res = self.receiver.recv_timeout(timeout).map_err(|e| match e {
                RecvTimeoutError::Timeout => SimError::TimedOut,
                RecvTimeoutError::Disconnected => SimError::Cancelled,
            })?;
            *guard = Some(res);
        }
        match guard.as_ref().unwrap() {
            Ok(_) => Ok(()),
            Err(e) => Err(e.clone()),
        }
    }

    /// Blocks until completion and returns the unpacked result value.
    pub fn get_result(mut self) -> Result<T> {
        self.wait()?;
        match self.cached.into_inner() {
            Ok(Some(res)) => res,
            Ok(None) => Err(SimError::Cancelled),
            Err(poisoned) => poisoned.into_inner().unwrap_or(Err(SimError::Cancelled)),
        }
    }

    /// Returns `true` if the operation has completed.
    ///
    /// Non-blocking check that inspects cached completion or polls the underlying receiver.
    pub fn is_done(&self) -> bool {
        let mut guard = match self.cached.lock() {
            Ok(g) => g,
            Err(poisoned) => poisoned.into_inner(),
        };
        if guard.is_some() {
            return true;
        }
        match self.receiver.try_recv() {
            Ok(res) => {
                *guard = Some(res);
                true
            }
            Err(TryRecvError::Disconnected) => {
                *guard = Some(Err(SimError::Cancelled));
                true
            }
            Err(TryRecvError::Empty) => false,
        }
    }
}
