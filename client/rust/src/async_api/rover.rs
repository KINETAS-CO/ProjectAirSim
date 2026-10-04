use tracing::info;

use crate::async_api::client::Client;
use crate::error::Result;
use crate::protocol::params::{
    EmptyParams, RoverMoveByHeadingParams as MoveByHeadingParams,
    RoverMoveToPositionParams as MoveToPositionParams, RoverSetPoseParams as SetPoseParams,
    SetRoverControlsParams,
};
use crate::types::Pose;

/// Asynchronous control handle for a ground rover vehicle.
#[derive(Clone)]
pub struct Rover {
    client: Client,
    rover_name: String,
    parent_topic: String,
}

impl Rover {
    /// Creates a new Rover handle.
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
    pub async fn enable_api_control(&self) -> Result<bool> {
        info!("Enabling API control for rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("EnableApiControl"), &EmptyParams {})
            .await
    }

    /// Releases API control back to manual or onboard autonomy.
    pub async fn disable_api_control(&self) -> Result<bool> {
        info!("Disabling API control for rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("DisableApiControl"), &EmptyParams {})
            .await
    }

    /// Checks if API control is currently granted.
    pub async fn is_api_control_enabled(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("IsApiControlEnabled"), &EmptyParams {})
            .await
    }

    /// Cancels the currently executing movement task.
    pub async fn cancel_last_task(&self) -> Result<bool> {
        info!("Canceling last task for rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("CancelLastTask"), &EmptyParams {})
            .await
    }

    // --- Arming ---

    /// Arms the rover drivetrain.
    pub async fn arm(&self) -> Result<bool> {
        info!("Arming rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("Arm"), &EmptyParams {})
            .await
    }

    /// Disarms the rover drivetrain.
    pub async fn disarm(&self) -> Result<bool> {
        info!("Disarming rover '{}'", self.rover_name);
        self.client
            .request(&self.method_path("Disarm"), &EmptyParams {})
            .await
    }

    /// Checks if the rover passes pre-checks and can arm.
    pub async fn can_arm(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("CanArm"), &EmptyParams {})
            .await
    }

    // --- State ---

    /// Retrieves ground truth kinematics JSON from the simulation.
    pub async fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.method_path("GetGroundTruthKinematics"),
                &EmptyParams {},
            )
            .await
    }

    /// Sets rover pose directly in the simulation world.
    pub async fn set_pose(&self, pose: Pose, reset_kinematics: bool) -> Result<()> {
        let _: serde_json::Value = self
            .client
            .request(
                &self.method_path("SetPose"),
                &SetPoseParams {
                    pose,
                    reset_kinematics,
                },
            )
            .await?;
        Ok(())
    }

    // --- Movement ---

    /// Navigates the rover to target North and East coordinates.
    #[allow(clippy::too_many_arguments)]
    pub async fn move_to_position(
        &self,
        north: f32,
        east: f32,
        velocity: f32,
        timeout_sec: Option<f32>,
        yaw_rate_max: Option<f32>,
        lookahead: Option<f32>,
        adaptive_lookahead: Option<f32>,
    ) -> Result<bool> {
        self.client
            .request(
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
            .await
    }

    /// Drives the rover along a specified compass heading.
    pub async fn move_by_heading(
        &self,
        heading: f32,
        speed: f32,
        duration_sec: Option<f32>,
        heading_margin: Option<f32>,
        yaw_rate: Option<f32>,
        timeout_sec: Option<f32>,
    ) -> Result<bool> {
        self.client
            .request(
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
            .await
    }

    /// Directly sets throttle engine, steering angle, and brake forces.
    pub async fn set_rover_controls(
        &self,
        engine: f32,
        steering_angle: f32,
        brake: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("SetRoverControls"),
                &SetRoverControlsParams {
                    engine,
                    steering_angle,
                    brake,
                },
            )
            .await
    }
}
