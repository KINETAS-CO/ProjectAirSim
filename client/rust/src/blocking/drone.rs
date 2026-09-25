use serde::Serialize;
use tracing::info;

use crate::blocking::async_result::AsyncResult;
use crate::blocking::client::Client;
use crate::error::Result;
use crate::types::{ImageResponse, ImageType, LandedState, Pose, ReadyState};

/// Synchronous blocking control handle for a drone/multirotor vehicle.
#[derive(Clone)]
pub struct Drone {
    client: Client,
    drone_name: String,
    parent_topic: String,
}

#[derive(Serialize)]
struct TimeoutParams {
    timeout_sec: f32,
}

#[derive(Serialize)]
struct MoveVelocityParams {
    vx: f64,
    vy: f64,
    vz: f64,
    duration: f64,
    drivetrain: i32,
    yaw_is_rate: bool,
    yaw: f64,
}

#[derive(Serialize)]
struct MoveToPositionParams {
    north: f64,
    east: f64,
    down: f64,
    velocity: f64,
    timeout_sec: f32,
    drivetrain: i32,
    yaw_is_rate: bool,
    yaw: f64,
    lookahead: f64,
    adaptive_lookahead: f64,
}

#[derive(Serialize)]
struct GetImagesParams<'a> {
    camera_id: &'a str,
    image_types: Vec<i32>,
}

#[derive(Serialize)]
struct EmptyParams {}

impl Drone {
    /// Creates a new blocking Drone handle.
    pub fn new(
        client: Client,
        drone_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        Self {
            client,
            drone_name: drone_name.into(),
            parent_topic: parent_topic.into(),
        }
    }

    /// Returns the vehicle's unique name.
    pub fn name(&self) -> &str {
        &self.drone_name
    }

    /// Builds the full RPC method path (e.g. `/Sim/Drone1/Takeoff`).
    fn method_path(&self, method: &str) -> String {
        format!("{}/{}/{}", self.parent_topic, self.drone_name, method)
    }

    // --- API Control & Readiness ---

    /// Requests API control over the vehicle.
    pub fn enable_api_control(&self) -> Result<bool> {
        info!("Enabling API control for drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("EnableApiControl"), &EmptyParams {})
    }

    /// Releases API control back to RC/manual or onboard autonomy.
    pub fn disable_api_control(&self) -> Result<bool> {
        info!("Disabling API control for drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("DisableApiControl"), &EmptyParams {})
    }

    /// Checks if API control is currently granted.
    pub fn is_api_control_enabled(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("IsApiControlEnabled"), &EmptyParams {})
    }

    /// Arms the drone's motors.
    pub fn arm(&self) -> Result<bool> {
        info!("Arming drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("Arm"), &EmptyParams {})
    }

    /// Disarms the drone's motors.
    pub fn disarm(&self) -> Result<bool> {
        info!("Disarming drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("Disarm"), &EmptyParams {})
    }

    /// Checks if the drone passes all pre-flight checks and can arm.
    pub fn can_arm(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("CanArm"), &EmptyParams {})
    }

    /// Retrieves readiness state.
    pub fn get_ready_state(&self) -> Result<ReadyState> {
        self.client
            .request(&self.method_path("GetReadyState"), &EmptyParams {})
    }

    /// Retrieves current landed state (Landed, Airborne, Unknown).
    pub fn get_landed_state(&self) -> Result<LandedState> {
        self.client
            .request(&self.method_path("GetLandedState"), &EmptyParams {})
    }

    // --- Flight Operations ---

    /// Dispatches takeoff asynchronously, returning an `AsyncResult` handle.
    pub fn takeoff_async(&self, timeout_sec: f32) -> AsyncResult<bool> {
        info!(
            "Commanding takeoff (async) for drone '{}' (timeout: {}s)",
            self.drone_name, timeout_sec
        );
        self.client
            .request_async(&self.method_path("Takeoff"), &TimeoutParams { timeout_sec })
    }

    /// Commands the drone to take off and blocks until complete.
    pub fn takeoff(&self, timeout_sec: f32) -> Result<bool> {
        self.takeoff_async(timeout_sec).get_result()
    }

    /// Dispatches landing asynchronously, returning an `AsyncResult` handle.
    pub fn land_async(&self, timeout_sec: f32) -> AsyncResult<bool> {
        info!("Commanding landing (async) for drone '{}'", self.drone_name);
        self.client
            .request_async(&self.method_path("Land"), &TimeoutParams { timeout_sec })
    }

    /// Commands the drone to land and blocks until complete.
    pub fn land(&self, timeout_sec: f32) -> Result<bool> {
        self.land_async(timeout_sec).get_result()
    }

    /// Commands the drone to hold position and hover in place.
    pub fn hover(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("Hover"), &EmptyParams {})
    }

    /// Commands velocity in the world frame (North, East, Down in m/s).
    pub fn move_by_velocity(&self, vx: f64, vy: f64, vz: f64, duration_sec: f64) -> Result<bool> {
        self.client.request(
            &self.method_path("MoveByVelocity"),
            &MoveVelocityParams {
                vx,
                vy,
                vz,
                duration: duration_sec,
                drivetrain: 0,
                yaw_is_rate: true,
                yaw: 0.0,
            },
        )
    }

    /// Moves the drone to target NED coordinates asynchronously.
    pub fn move_to_position_async(
        &self,
        north: f64,
        east: f64,
        down: f64,
        velocity: f64,
        timeout_sec: f32,
    ) -> AsyncResult<bool> {
        self.client.request_async(
            &self.method_path("MoveToPosition"),
            &MoveToPositionParams {
                north,
                east,
                down,
                velocity,
                timeout_sec,
                drivetrain: 0,
                yaw_is_rate: true,
                yaw: 0.0,
                lookahead: -1.0,
                adaptive_lookahead: 1.0,
            },
        )
    }

    /// Moves the drone to target NED coordinates and blocks until complete.
    pub fn move_to_position(
        &self,
        north: f64,
        east: f64,
        down: f64,
        velocity: f64,
        timeout_sec: f32,
    ) -> Result<bool> {
        self.move_to_position_async(north, east, down, velocity, timeout_sec)
            .get_result()
    }

    // --- State & Sensors ---

    /// Queries the ground-truth 6-DoF pose of the drone.
    pub fn get_ground_truth_pose(&self) -> Result<Pose> {
        self.client
            .request(&self.method_path("GetGroundTruthPose"), &EmptyParams {})
    }

    /// Captures images from a named onboard camera.
    pub fn get_images(
        &self,
        camera_id: &str,
        image_types: &[ImageType],
    ) -> Result<Vec<ImageResponse>> {
        let type_ids: Vec<i32> = image_types.iter().map(|t| *t as i32).collect();
        self.client.request(
            &self.method_path("GetImages"),
            &GetImagesParams {
                camera_id,
                image_types: type_ids,
            },
        )
    }
}
