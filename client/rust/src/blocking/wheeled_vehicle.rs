use crate::blocking::client::Client;
use crate::error::Result;

/// Synchronous blocking control handle for an Unreal AWheeledVehiclePawn.
#[derive(Clone)]
pub struct WheeledVehicle {
    inner: crate::async_api::WheeledVehicle,
    client: Client,
}

impl WheeledVehicle {
    /// Creates a new blocking WheeledVehicle handle.
    pub fn new(
        client: Client,
        vehicle_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        let inner = crate::async_api::WheeledVehicle::new(
            client.inner().clone(),
            vehicle_name,
            parent_topic,
        );
        Self { inner, client }
    }

    /// Creates a blocking WheeledVehicle handle wrapping an existing async handle.
    pub fn from_inner(inner: crate::async_api::WheeledVehicle, client: Client) -> Self {
        Self { inner, client }
    }

    /// Returns the vehicle's unique name.
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// Sets the engine throttle in range [-1.0, 1.0].
    pub fn set_throttle(&self, value: f32) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_throttle(value))
    }

    /// Sets the steering angle command in range [-1.0, 1.0].
    pub fn set_steering(&self, value: f32) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_steering(value))
    }

    /// Sets the wheel brakes command in range [0.0, 1.0].
    pub fn set_brakes(&self, value: f32) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_brakes(value))
    }

    /// Sets throttle, steering, and brakes simultaneously.
    pub fn set_controls(&self, throttle: f32, steering: f32, brake: f32) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_controls(throttle, steering, brake))
    }

    /// Retrieves current estimated vehicle kinematics.
    pub fn get_kinematics(&self) -> Result<serde_json::Value> {
        self.client.runtime().block_on(self.inner.get_kinematics())
    }

    /// Retrieves ground truth vehicle kinematics directly from Unreal physics.
    pub fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client.runtime().block_on(self.inner.get_ground_truth_kinematics())
    }
}
