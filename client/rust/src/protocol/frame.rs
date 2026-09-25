use serde::{Deserialize, Serialize};
use crate::error::{Result, SimError};

/// Message frame type for ProjectAirSim topic streaming (Port 8989).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[repr(i32)]
pub enum FrameType {
    Subscribe = 0,
    Unsubscribe = 1,
    Message = 2,
}

/// A 3-element MessagePack array representing a topic frame:
/// `[frame_type, topic_path, body_bytes]`
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopicFrame(
    pub FrameType,
    pub String,
    #[serde(with = "serde_bytes")] pub Vec<u8>,
);

impl TopicFrame {
    pub fn subscribe(topic: impl Into<String>) -> Self {
        Self(FrameType::Subscribe, topic.into(), Vec::new())
    }

    pub fn unsubscribe(topic: impl Into<String>) -> Self {
        Self(FrameType::Unsubscribe, topic.into(), Vec::new())
    }

    pub fn message(topic: impl Into<String>, body: Vec<u8>) -> Self {
        Self(FrameType::Message, topic.into(), body)
    }

    pub fn frame_type(&self) -> FrameType {
        self.0
    }

    pub fn topic(&self) -> &str {
        &self.1
    }

    pub fn body(&self) -> &[u8] {
        &self.2
    }

    pub fn into_body(self) -> Vec<u8> {
        self.2
    }

    pub fn to_bytes(&self) -> Result<Vec<u8>> {
        rmp_serde::to_vec(self)
            .map_err(|e| SimError::SerializationError(format!("Failed to serialize TopicFrame: {e}")))
    }

    pub fn from_bytes(bytes: &[u8]) -> Result<Self> {
        rmp_serde::from_slice(bytes)
            .map_err(|e| SimError::SerializationError(format!("Failed to deserialize TopicFrame: {e}")))
    }
}

/// Metadata about a registered simulation topic returned by the `/$topics` stream.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct TopicInfo {
    pub path: String,
    #[serde(rename = "type")]
    pub topic_type: String,
    pub message_type: String,
    pub frequency: i32,
}
