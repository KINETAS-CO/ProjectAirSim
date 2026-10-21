use crate::error::{Result, SimError};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

struct Inner<T> {
    state: Mutex<Option<Result<T>>>,
    receiver: Mutex<Option<Receiver<Result<T>>>>,
    cvar: Condvar,
    is_done: AtomicBool,
}

/// Sender handle used to deliver results to an [`AsyncResult`].
///
/// If dropped before sending, automatically marks the [`AsyncResult`] as cancelled.
pub struct AsyncResultSender<T> {
    inner: Arc<Inner<T>>,
}

impl<T> AsyncResultSender<T> {
    pub fn send(self, res: Result<T>) {
        let mut guard = self.inner.state.lock().unwrap();
        *guard = Some(res);
        self.inner.is_done.store(true, Ordering::Release);
        self.inner.cvar.notify_all();
    }
}

impl<T> Drop for AsyncResultSender<T> {
    fn drop(&mut self) {
        let mut guard = self.inner.state.lock().unwrap();
        if guard.is_none() {
            *guard = Some(Err(SimError::Cancelled));
            self.inner.is_done.store(true, Ordering::Release);
            self.inner.cvar.notify_all();
        }
    }
}

/// Creates a new paired sender and asynchronous result synchronization handle.
pub fn async_result_channel<T>() -> (AsyncResultSender<T>, AsyncResult<T>) {
    let inner = Arc::new(Inner {
        state: Mutex::new(None),
        receiver: Mutex::new(None),
        cvar: Condvar::new(),
        is_done: AtomicBool::new(false),
    });
    (
        AsyncResultSender {
            inner: Arc::clone(&inner),
        },
        AsyncResult { inner },
    )
}

/// Synchronization handle for asynchronous operations initiated in blocking mode.
///
/// Mirrors the C++ ProjectAirSim `AsyncResult` API, allowing callers to wait
/// synchronously, query completion status, or block with a timeout.
/// Internally backed by an `Arc`, allowing cheap cloning and thread-safe inspection.
#[derive(Clone)]
pub struct AsyncResult<T> {
    inner: Arc<Inner<T>>,
}

impl<T> AsyncResult<T> {
    /// Creates an `AsyncResult` wrapping an existing channel receiver.
    pub fn new(receiver: Receiver<Result<T>>) -> Self {
        let inner = Arc::new(Inner {
            state: Mutex::new(None),
            receiver: Mutex::new(Some(receiver)),
            cvar: Condvar::new(),
            is_done: AtomicBool::new(false),
        });
        AsyncResult { inner }
    }

    /// Blocks the current thread until the operation finishes.
    pub fn wait(&self) -> Result<()> {
        if self.is_done() {
            let guard = self.inner.state.lock().unwrap();
            return match guard.as_ref().unwrap() {
                Ok(_) => Ok(()),
                Err(e) => Err(e.clone()),
            };
        }

        // If an underlying channel receiver exists, take and block on it directly
        let rx_opt = {
            if let Ok(mut rx_guard) = self.inner.receiver.lock() {
                rx_guard.take()
            } else {
                None
            }
        };

        if let Some(rx) = rx_opt {
            let res = rx.recv().unwrap_or(Err(SimError::Cancelled));
            let mut guard = self.inner.state.lock().unwrap();
            *guard = Some(res);
            self.inner.is_done.store(true, Ordering::Release);
            self.inner.cvar.notify_all();
            return match guard.as_ref().unwrap() {
                Ok(_) => Ok(()),
                Err(e) => Err(e.clone()),
            };
        }

        // Otherwise wait on condvar
        let mut guard = self.inner.state.lock().unwrap();
        while guard.is_none() {
            guard = self.inner.cvar.wait(guard).unwrap();
        }
        match guard.as_ref().unwrap() {
            Ok(_) => Ok(()),
            Err(e) => Err(e.clone()),
        }
    }

    /// Blocks the current thread for up to `timeout` duration.
    pub fn wait_timeout(&self, timeout: Duration) -> Result<()> {
        if self.is_done() {
            let guard = self.inner.state.lock().unwrap();
            return match guard.as_ref().unwrap() {
                Ok(_) => Ok(()),
                Err(e) => Err(e.clone()),
            };
        }

        let rx_opt = {
            if let Ok(mut rx_guard) = self.inner.receiver.lock() {
                rx_guard.take()
            } else {
                None
            }
        };

        if let Some(rx) = rx_opt {
            let res = match rx.recv_timeout(timeout) {
                Ok(r) => r,
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    if let Ok(mut rx_guard) = self.inner.receiver.lock() {
                        *rx_guard = Some(rx);
                    }
                    return Err(SimError::TimedOut);
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(SimError::Cancelled),
            };
            let mut guard = self.inner.state.lock().unwrap();
            *guard = Some(res);
            self.inner.is_done.store(true, Ordering::Release);
            self.inner.cvar.notify_all();
            return match guard.as_ref().unwrap() {
                Ok(_) => Ok(()),
                Err(e) => Err(e.clone()),
            };
        }

        let mut guard = self.inner.state.lock().unwrap();
        let start = std::time::Instant::now();
        while guard.is_none() {
            let elapsed = start.elapsed();
            if elapsed >= timeout {
                return Err(SimError::TimedOut);
            }
            let remaining = timeout - elapsed;
            let (new_guard, timeout_res) = self.inner.cvar.wait_timeout(guard, remaining).unwrap();
            guard = new_guard;
            if timeout_res.timed_out() && guard.is_none() {
                return Err(SimError::TimedOut);
            }
        }
        match guard.as_ref().unwrap() {
            Ok(_) => Ok(()),
            Err(e) => Err(e.clone()),
        }
    }

    /// Returns `true` if the operation has completed.
    ///
    /// Non-blocking check that inspects completion status.
    pub fn is_done(&self) -> bool {
        if self.inner.is_done.load(Ordering::Acquire) {
            return true;
        }

        if let Ok(mut rx_guard) = self.inner.receiver.try_lock() {
            if let Some(ref rx) = *rx_guard {
                match rx.try_recv() {
                    Ok(res) => {
                        *self.inner.state.lock().unwrap() = Some(res);
                        self.inner.is_done.store(true, Ordering::Release);
                        self.inner.cvar.notify_all();
                        *rx_guard = None;
                        return true;
                    }
                    Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                        *self.inner.state.lock().unwrap() = Some(Err(SimError::Cancelled));
                        self.inner.is_done.store(true, Ordering::Release);
                        self.inner.cvar.notify_all();
                        *rx_guard = None;
                        return true;
                    }
                    Err(std::sync::mpsc::TryRecvError::Empty) => {}
                }
            }
        }

        self.inner.is_done.load(Ordering::Acquire)
    }
}

impl<T: Clone> AsyncResult<T> {
    /// Blocks until completion and returns a clone of the unpacked result value.
    pub fn get_result(&self) -> Result<T> {
        self.wait()?;
        let guard = self.inner.state.lock().unwrap();
        guard.as_ref().unwrap().clone()
    }
}
