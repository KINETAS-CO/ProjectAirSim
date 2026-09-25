use crate::error::{Result, SimError};
use crate::protocol::request::RawDataPayload;
use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Raw deserialization envelope for ProjectAirSim RPC responses.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RawResponseEnvelope {
    pub id: Option<i32>,
    pub version: Option<f32>,
    pub result: Option<RawDataPayload>,
    pub error: Option<Value>,
}

/// Helper for decoding and validating response payloads from the simulation server.
pub struct ResponseDecoder;

impl ResponseDecoder {
    /// Decodes a response buffer into the raw result bytes.
    ///
    /// Returns `Ok(Vec<u8>)` containing the inner result payload,
    /// or `Err(SimError::ServerRejected { .. })` if the server returned an error.
    pub fn decode(bytes: &[u8]) -> Result<Vec<u8>> {
        let envelope: RawResponseEnvelope = rmp_serde::from_slice(bytes).map_err(|e| {
            SimError::SerializationError(format!("Failed to unpack response envelope: {e}"))
        })?;

        if let Some(err_val) = envelope.error {
            let (code, message) = Self::extract_error(&err_val);
            return Err(SimError::ServerRejected { code, message });
        }

        if let Some(result_payload) = envelope.result {
            return Ok(result_payload.data);
        }

        Err(SimError::ProtocolError(
            "Response envelope contained neither 'result' nor 'error'".to_string(),
        ))
    }

    /// Decodes a response buffer and deserializes the inner result into a typed struct.
    pub fn decode_typed<T: for<'de> Deserialize<'de>>(bytes: &[u8]) -> Result<T> {
        let raw_result_bytes = Self::decode(bytes)?;
        rmp_serde::from_slice(&raw_result_bytes).map_err(|e| {
            SimError::SerializationError(format!("Failed to deserialize typed response: {e}"))
        })
    }

    fn extract_error(val: &Value) -> (i32, String) {
        // Case 1: error is wrapped in {"data": <bytes>} (JsonMsgpack)
        if let Some(data) = val.get("data") {
            if let Some(bytes) = data.as_str().map(|s| s.as_bytes().to_vec()) {
                if let Ok(nested_val) = rmp_serde::from_slice::<Value>(&bytes) {
                    return Self::extract_error(&nested_val);
                }
            } else if let Some(arr) = data.as_array() {
                let bytes: Vec<u8> = arr
                    .iter()
                    .filter_map(|v| v.as_u64().map(|n| n as u8))
                    .collect();
                if let Ok(nested_val) = rmp_serde::from_slice::<Value>(&bytes) {
                    return Self::extract_error(&nested_val);
                }
            }
        }

        // Case 2: error is directly {"code": ..., "message": ...}
        let code = val
            .get("code")
            .and_then(|c| c.as_i64().or_else(|| c.as_f64().map(|f| f as i64)))
            .unwrap_or(1) as i32;

        let message = val
            .get("message")
            .and_then(|m| m.as_str())
            .unwrap_or("Unknown server error")
            .to_string();

        (code, message)
    }
}
