use crate::blocking::client::Client;
use crate::error::Result;
use crate::types::{ImageResponse, ImageType};

/// Synchronous blocking control handle for a stationary sensor platform or camera tower.
#[derive(Clone)]
pub struct StaticSensorActor {
    inner: crate::async_api::StaticSensorActor,
    client: Client,
}

impl StaticSensorActor {
    /// Creates a new blocking StaticSensorActor handle.
    pub fn new(
        client: Client,
        actor_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        let inner = crate::async_api::StaticSensorActor::new(
            client.inner().clone(),
            actor_name,
            parent_topic,
        );
        Self { inner, client }
    }

    /// Creates a blocking StaticSensorActor handle wrapping an existing async handle.
    pub fn from_inner(inner: crate::async_api::StaticSensorActor, client: Client) -> Self {
        Self { inner, client }
    }

    /// Returns the actor's unique name.
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// Retrieves raw image payload JSON from a camera attached to this sensor platform.
    pub fn get_images(&self, camera_id: &str, image_type_ids: &[i32]) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_images(camera_id, image_type_ids))
    }

    /// Retrieves strongly-typed ImageResponse instances for specified ImageType variants.
    pub fn get_camera_images(
        &self,
        camera_id: &str,
        image_types: &[ImageType],
    ) -> Result<Vec<ImageResponse>> {
        self.client
            .runtime()
            .block_on(self.inner.get_camera_images(camera_id, image_types))
    }
}
