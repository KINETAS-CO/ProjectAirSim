use serde::Serialize;
use tracing::info;

use crate::blocking::client::Client;
use crate::error::Result;

/// Synchronous blocking control handle for an Unreal AWheeledVehiclePawn.
#[derive(Clone)]
pub struct WheeledVehicle {
    client: Client,
    vehicle_name: String,
    parent_topic: String,
}

#[derive(Serialize)]
struct SingleValueParams {
    value: f32,
}

#[derive(Serialize)]
struct EmptyParams {}

impl WheeledVehicle {
    /// Creates a new blocking WheeledVehicle handle.
    pub fn new(
        client: Client,
        vehicle_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        Self {
            client,
            vehicle_name: vehicle_name.into(),
            parent_topic: parent_topic.into(),
        }
    }

    /// Returns the vehicle's unique name.
    pub fn name(&self) -> &str {
        &self.vehicle_name
    }

    /// Builds the full RPC method path (e.g. `/Sim/robots/Car1/SetThrottle`).
    fn method_path(&self, method: &str) -> String {
        let base = if self
            .parent_topic
            .ends_with(&format!("/robots/{}", self.vehicle_name))
        {
            self.parent_topic.clone()
        } else if self.parent_topic.ends_with("/robots") {
            format!("{}/{}", self.parent_topic, self.vehicle_name)
        } else if self.parent_topic.contains("/robots/") {
            self.parent_topic.clone()
        } else {
            format!("{}/robots/{}", self.parent_topic, self.vehicle_name)
        };
        format!("{base}/{method}")
    }

    /// Sets the engine throttle in range [-1.0, 1.0].
    pub fn set_throttle(&self, value: f32) -> Result<bool> {
        info!("Setting throttle on '{}' to {}", self.vehicle_name, value);
        self.client.request(
            &self.method_path("SetThrottle"),
            &SingleValueParams { value },
        )
    }

    /// Sets the steering angle command in range [-1.0, 1.0].
    pub fn set_steering(&self, value: f32) -> Result<bool> {
        info!("Setting steering on '{}' to {}", self.vehicle_name, value);
        self.client.request(
            &self.method_path("SetSteering"),
            &SingleValueParams { value },
        )
    }

    /// Sets the wheel brakes command in range [0.0, 1.0].
    pub fn set_brakes(&self, value: f32) -> Result<bool> {
        info!("Setting brakes on '{}' to {}", self.vehicle_name, value);
        self.client
            .request(&self.method_path("SetBrakes"), &SingleValueParams { value })
    }

    /// Sets throttle, steering, and brakes simultaneously.
    pub fn set_controls(&self, throttle: f32, steering: f32, brake: f32) -> Result<bool> {
        let t = self.set_throttle(throttle)?;
        let s = self.set_steering(steering)?;
        let b = self.set_brakes(brake)?;
        Ok(t && s && b)
    }

    /// Retrieves current estimated vehicle kinematics.
    pub fn get_kinematics(&self) -> Result<serde_json::Value> {
        self.get_ground_truth_kinematics()
    }

    /// Retrieves ground truth vehicle kinematics directly from Unreal physics.
    pub fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client.request(
            &self.method_path("GetGroundTruthKinematics"),
            &EmptyParams {},
        )
    }
}
