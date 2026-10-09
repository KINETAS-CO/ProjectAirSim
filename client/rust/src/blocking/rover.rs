use crate::blocking::async_result::AsyncResult;
use crate::blocking::client::Client;
use crate::error::Result;
use crate::types::Pose;

/// Synchronous blocking control handle for a ground rover vehicle.
#[derive(Clone)]
pub struct Rover {
    inner: crate::async_api::Rover,
    client: Client,
}

impl Rover {
    /// Creates a new blocking Rover handle.
    pub fn new(
        client: Client,
        rover_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        let inner =
            crate::async_api::Rover::new(client.inner().clone(), rover_name, parent_topic);
        Self { inner, client }
    }

    /// Creates a blocking Rover handle wrapping an existing async handle.
    pub fn from_inner(inner: crate::async_api::Rover, client: Client) -> Self {
        Self { inner, client }
    }

    /// Returns the rover's unique name.
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    // --- API Control ---

    /// Requests API control over the rover.
    pub fn enable_api_control(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.enable_api_control())
    }

    /// Releases API control back to manual or onboard autonomy.
    pub fn disable_api_control(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.disable_api_control())
    }

    /// Checks if API control is currently granted.
    pub fn is_api_control_enabled(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.is_api_control_enabled())
    }

    /// Cancels the currently executing movement task.
    pub fn cancel_last_task(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.cancel_last_task())
    }

    // --- Arming ---

    /// Arms the rover drivetrain.
    pub fn arm(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.arm())
    }

    /// Disarms the rover drivetrain.
    pub fn disarm(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.disarm())
    }

    /// Checks if the rover passes pre-checks and can arm.
    pub fn can_arm(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.can_arm())
    }

    // --- State ---

    /// Retrieves ground truth kinematics JSON from the simulation.
    pub fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_ground_truth_kinematics())
    }

    /// Sets rover pose directly in the simulation world.
    pub fn set_pose(&self, pose: Pose, reset_kinematics: bool) -> Result<()> {
        self.client
            .runtime()
            .block_on(self.inner.set_pose(pose, reset_kinematics))
    }

    // --- Movement ---

    /// Navigates the rover to target North and East coordinates (blocking).
    #[allow(clippy::too_many_arguments)]
    pub fn move_to_position(
        &self,
        north: f32,
        east: f32,
        velocity: f32,
        timeout_sec: Option<f32>,
        yaw_rate_max: Option<f32>,
        lookahead: Option<f32>,
        adaptive_lookahead: Option<f32>,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.move_to_position(
            north,
            east,
            velocity,
            timeout_sec,
            yaw_rate_max,
            lookahead,
            adaptive_lookahead,
        ))
    }

    /// Starts navigating to target coordinates returning an AsyncResult handle.
    #[allow(clippy::too_many_arguments)]
    pub fn move_to_position_async(
        &self,
        north: f32,
        east: f32,
        velocity: f32,
        timeout_sec: Option<f32>,
        yaw_rate_max: Option<f32>,
        lookahead: Option<f32>,
        adaptive_lookahead: Option<f32>,
    ) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_to_position(
                    north,
                    east,
                    velocity,
                    timeout_sec,
                    yaw_rate_max,
                    lookahead,
                    adaptive_lookahead,
                )
                .await
        })
    }

    /// Drives the rover along a specified compass heading (blocking).
    pub fn move_by_heading(
        &self,
        heading: f32,
        speed: f32,
        duration_sec: Option<f32>,
        heading_margin: Option<f32>,
        yaw_rate: Option<f32>,
        timeout_sec: Option<f32>,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.move_by_heading(
            heading,
            speed,
            duration_sec,
            heading_margin,
            yaw_rate,
            timeout_sec,
        ))
    }

    /// Starts driving along a specified compass heading returning an AsyncResult handle.
    pub fn move_by_heading_async(
        &self,
        heading: f32,
        speed: f32,
        duration_sec: Option<f32>,
        heading_margin: Option<f32>,
        yaw_rate: Option<f32>,
        timeout_sec: Option<f32>,
    ) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_by_heading(
                    heading,
                    speed,
                    duration_sec,
                    heading_margin,
                    yaw_rate,
                    timeout_sec,
                )
                .await
        })
    }

    /// Directly sets throttle engine, steering angle, and brake forces.
    pub fn set_rover_controls(&self, engine: f32, steering_angle: f32, brake: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_rover_controls(engine, steering_angle, brake))
    }
}
