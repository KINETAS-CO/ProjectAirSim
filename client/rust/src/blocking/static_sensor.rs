use tracing::info;

use crate::blocking::client::Client;
use crate::error::Result;
use crate::protocol::params::StaticGetImagesParams as GetImagesParams;
use crate::types::{ImageResponse, ImageType};

/// Synchronous blocking control handle for a stationary sensor platform or camera tower.
#[derive(Clone)]
pub struct StaticSensorActor {
    client: Client,
    actor_name: String,
    parent_topic: String,
}

impl StaticSensorActor {
    /// Creates a new blocking StaticSensorActor handle.
    pub fn new(
        client: Client,
        actor_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        Self {
            client,
            actor_name: actor_name.into(),
            parent_topic: parent_topic.into(),
        }
    }

    /// Returns the actor's unique name.
    pub fn name(&self) -> &str {
        &self.actor_name
    }

    /// Builds the base sensor topic path (e.g. `/Sim/robots/Static1/sensors`).
    fn sensor_topic(&self) -> String {
        let base = if self
            .parent_topic
            .ends_with(&format!("/robots/{}", self.actor_name))
        {
            self.parent_topic.clone()
        } else if self.parent_topic.ends_with("/robots") {
            format!("{}/{}", self.parent_topic, self.actor_name)
        } else if self.parent_topic.contains("/robots/") {
            self.parent_topic.clone()
        } else {
            format!("{}/robots/{}", self.parent_topic, self.actor_name)
        };
        format!("{base}/sensors")
    }

    /// Retrieves raw image payload JSON from a camera attached to this sensor platform.
    pub fn get_images(&self, camera_id: &str, image_type_ids: &[i32]) -> Result<serde_json::Value> {
        info!(
            "Requesting images from static sensor '{}' camera '{}'",
            self.actor_name, camera_id
        );
        let path = format!("{}/{}/GetImages", self.sensor_topic(), camera_id);
        self.client
            .request(&path, &GetImagesParams { image_type_ids })
    }

    /// Retrieves strongly-typed ImageResponse instances for specified ImageType variants.
    pub fn get_camera_images(
        &self,
        camera_id: &str,
        image_types: &[ImageType],
    ) -> Result<Vec<ImageResponse>> {
        let ids: Vec<i32> = image_types.iter().map(|t| *t as i32).collect();
        let val = self.get_images(camera_id, &ids)?;
        serde_json::from_value(val).map_err(Into::into)
    }
}
