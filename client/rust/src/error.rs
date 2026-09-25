use thiserror::Error;

/// Status codes matching ProjectAirSim C++ client Status enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u32)]
pub enum Status {
    Ok = 0,
    Failed = 1,
    Canceled = 2,
    Closed = 3,
    InProgress = 4,
    TimedOut = 5,
    NotConnected = 6,
    NotFound = 7,
    RejectedByServer = 8,
    NoScene = 9,
}

#[derive(Error, Debug, Clone, PartialEq)]
pub enum SimError {
    #[error("Serialization error: {0}")]
    SerializationError(String),

    #[error("Protocol error: {0}")]
    ProtocolError(String),

    #[error("Server rejected request (code {code}): {message}")]
    ServerRejected { code: i32, message: String },

    #[error("Transport error: {0}")]
    TransportError(String),

    #[error("Operation timed out")]
    TimedOut,

    #[error("Client is not connected to simulation server")]
    NotConnected,

    #[error("Operation was cancelled")]
    Cancelled,

    #[error("No scene is currently loaded on the simulation server")]
    NoScene,

    #[error("Target entity or topic not found: {0}")]
    NotFound(String),

    #[error("Connection closed")]
    ConnectionClosed,
}

impl SimError {
    pub fn to_status(&self) -> Status {
        match self {
            SimError::TimedOut => Status::TimedOut,
            SimError::NotConnected => Status::NotConnected,
            SimError::Cancelled => Status::Canceled,
            SimError::NoScene => Status::NoScene,
            SimError::NotFound(_) => Status::NotFound,
            SimError::ServerRejected { .. } => Status::RejectedByServer,
            SimError::ConnectionClosed => Status::Closed,
            _ => Status::Failed,
        }
    }
}

pub type Result<T> = std::result::Result<T, SimError>;
