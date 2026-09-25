use crate::error::{Result, SimError};
use serde::{Deserialize, Serialize};

/// Encapsulates binary-packed parameters inside the `{"data": bytes}` map
/// required by ProjectAirSim's MSGPACK_JSON protocol.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct RawDataPayload {
    #[serde(with = "serde_bytes")]
    pub data: Vec<u8>,
}

/// ProjectAirSim RPC Request envelope (Port 8990).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RequestEnvelope<'a> {
    pub id: i32,
    pub method: &'a str,
    pub params: RawDataPayload,
    pub version: f32,
}

impl<'a> RequestEnvelope<'a> {
    /// Creates a new request envelope with version 1.0 and MessagePack-encoded parameters.
    pub fn new<P: Serialize>(id: i32, method: &'a str, params: &P) -> Result<Self> {
        let packed_params = rmp_serde::to_vec_named(params).map_err(|e| {
            SimError::SerializationError(format!("Failed to pack request params: {e}"))
        })?;

        Ok(Self {
            id,
            method,
            params: RawDataPayload {
                data: packed_params,
            },
            version: 1.0,
        })
    }

    /// Creates an envelope with already packed binary parameter bytes.
    pub fn with_raw_bytes(id: i32, method: &'a str, raw_params: Vec<u8>) -> Self {
        Self {
            id,
            method,
            params: RawDataPayload { data: raw_params },
            version: 1.0,
        }
    }

    /// Serializes the entire envelope into MSGPACK_JSON wire bytes.
    pub fn to_bytes(&self) -> Result<Vec<u8>> {
        rmp_serde::to_vec_named(self).map_err(|e| {
            SimError::SerializationError(format!("Failed to serialize RequestEnvelope: {e}"))
        })
    }
}
