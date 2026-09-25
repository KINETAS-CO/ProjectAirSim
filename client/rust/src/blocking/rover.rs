use serde::Serialize;
use tracing::info;

use crate::blocking::async_result::AsyncResult;
use crate::blocking::client::Client;
use crate::error::Result;
use crate::types::Pose;

/// Synchronous blocking control handle for a ground rover vehicle.
#[derive(Clone)]
pub struct Rover {
    client: Client,
    rover_name: String,
    parent_topic: String,
}

#[derive(Serialize)]
struct SetPoseParams {
    pose: Pose,
    reset_kinematics: bool,
}

#[derive(Serialize)]
struct MoveToPositionParams {
    x: f32,
    y: f32,
    velocity: f32,
    timeout_sec: f32,
    yaw_rate_max: f32,
    lookahead: f32,
    adaptive_lookahead: f32,
}

#[derive(Serialize)]
struct MoveByHeadingParams {
    heading: f32,
    speed: f32,
    duration: f32,
    heading_margin: f32,
    yaw_rate: f32,
    timeout_sec: f32,
}

#[derive(Serialize)]
struct SetRoverControlsParams {
    engine: f32,
    steering_angle: f32,
    brake: f32,
}

#[derive(Serialize)]
struct EmptyParams {}

impl Rover {
    /// Creates a new blocking Rover handle.
    pub fn new(
        client: Client,
        rover_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        Self {
            client,
            rover_name: rover_name.into(),
            parent_topic: parent_topic.into(),
        }
    }

    /// Returns the rover's unique name.
    pub fn name(&self) -> &str {
        &self.rover_name
    }

    /// Builds the full RPC method path (e.g. `/Sim/robots/Rover1/SetRoverControls`).
    fn method_path(&self, method: &str) -> String {
        let base = if self
            .parent_topic
            .ends_with(&format!("/robots/{}", self.rover_name))
        {
            self.parent_topic.clone()
        } else if self.parent_topic.ends_with("/robots") {
            format!("{}/{}", self.parent_topic, self.rover_name)
        } else if self.parent_topic.contains("/robots/") {
            self.parent_topic.clone()
        } else {
            format!("{}/robots/{}", self.parent_topic, self.rover_name)
        };
        format!("{base}/{method}")
    }

    // --- API Control ---

    /// Requests API control over the rover.
    pub fn enable_api_control(&self) -> Result<bool> {
        info!("Enabling API control for rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("EnableApiControl"), &EmptyParams {})
    }

    /// Releases API control back to manual or onboard autonomy.
    pub fn disable_api_control(&self) -> Result<bool> {
        info!("Disabling API control for rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("DisableApiControl"), &EmptyParams {})
    }

    /// Checks if API control is currently granted.
    pub fn is_api_control_enabled(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("IsApiControlEnabled"), &EmptyParams {})
    }

    /// Cancels the currently executing movement task.
    pub fn cancel_last_task(&self) -> Result<bool> {
        info!("Canceling last task for rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("CancelLastTask"), &EmptyParams {})
    }

    // --- Arming ---

    /// Arms the rover drivetrain.
    pub fn arm(&self) -> Result<bool> {
        info!("Arming rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("Arm"), &EmptyParams {})
    }

    /// Disarms the rover drivetrain.
    pub fn disarm(&self) -> Result<bool> {
        info!("Disarming rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("Disarm"), &EmptyParams {})
    }

    /// Checks if the rover passes pre-checks and can arm.
    pub fn can_arm(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("CanArm"), &EmptyParams {})
    }

    // --- State ---

    /// Retrieves ground truth kinematics JSON from the simulation.
    pub fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client.request(
            &self.method_path("GetGroundTruthKinematics"),
            &EmptyParams {},
        )
    }

    /// Sets rover pose directly in the simulation world.
    pub fn set_pose(&self, pose: Pose, reset_kinematics: bool) -> Result<()> {
        let _: serde_json::Value = self.client.request(
            &self.method_path("SetPose"),
            &SetPoseParams {
                pose,
                reset_kinematics,
            },
        )?;
        Ok(())
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
        self.move_to_position_async(
            north,
            east,
            velocity,
            timeout_sec,
            yaw_rate_max,
            lookahead,
            adaptive_lookahead,
        )
        .get_result()
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
        self.client.request_async(
            &self.method_path("MoveToPosition"),
            &MoveToPositionParams {
                x: north,
                y: east,
                velocity,
                timeout_sec: timeout_sec.unwrap_or(f32::MAX),
                yaw_rate_max: yaw_rate_max.unwrap_or(-1.0),
                lookahead: lookahead.unwrap_or(-1.0),
                adaptive_lookahead: adaptive_lookahead.unwrap_or(1.0),
            },
        )
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
        self.move_by_heading_async(
            heading,
            speed,
            duration_sec,
            heading_margin,
            yaw_rate,
            timeout_sec,
        )
        .get_result()
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
        self.client.request_async(
            &self.method_path("MoveByHeading"),
            &MoveByHeadingParams {
                heading,
                speed,
                duration: duration_sec.unwrap_or(3.0),
                heading_margin: heading_margin.unwrap_or(0.08726646),
                yaw_rate: yaw_rate.unwrap_or(5.0),
                timeout_sec: timeout_sec.unwrap_or(f32::MAX),
            },
        )
    }

    /// Directly sets throttle engine, steering angle, and brake forces.
    pub fn set_rover_controls(&self, engine: f32, steering_angle: f32, brake: f32) -> Result<bool> {
        self.client.request(
            &self.method_path("SetRoverControls"),
            &SetRoverControlsParams {
                engine,
                steering_angle,
                brake,
            },
        )
    }
}
