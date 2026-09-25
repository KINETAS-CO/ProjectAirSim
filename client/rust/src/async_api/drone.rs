use serde::Serialize;
use tracing::info;

use crate::async_api::client::{Client, TopicSubscription};
use crate::error::Result;
use crate::types::{ImageResponse, ImageType, LandedState, Pose, ReadyState};

/// Asynchronous control handle for a drone/multirotor vehicle.
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
    /// Creates a new Drone handle.
    pub fn new(client: Client, drone_name: impl Into<String>, parent_topic: impl Into<String>) -> Self {
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
    pub async fn enable_api_control(&self) -> Result<bool> {
        info!("Enabling API control for drone '{}'", self.drone_name);
        self.client.request(&self.method_path("EnableApiControl"), &EmptyParams {}).await
    }

    /// Releases API control back to RC/manual or onboard autonomy.
    pub async fn disable_api_control(&self) -> Result<bool> {
        info!("Disabling API control for drone '{}'", self.drone_name);
        self.client.request(&self.method_path("DisableApiControl"), &EmptyParams {}).await
    }

    /// Checks if API control is currently granted.
    pub async fn is_api_control_enabled(&self) -> Result<bool> {
        self.client.request(&self.method_path("IsApiControlEnabled"), &EmptyParams {}).await
    }

    /// Arms the drone's motors.
    pub async fn arm(&self) -> Result<bool> {
        info!("Arming drone '{}'", self.drone_name);
        self.client.request(&self.method_path("Arm"), &EmptyParams {}).await
    }

    /// Disarms the drone's motors.
    pub async fn disarm(&self) -> Result<bool> {
        info!("Disarming drone '{}'", self.drone_name);
        self.client.request(&self.method_path("Disarm"), &EmptyParams {}).await
    }

    /// Checks if the drone passes all pre-flight checks and can arm.
    pub async fn can_arm(&self) -> Result<bool> {
        self.client.request(&self.method_path("CanArm"), &EmptyParams {}).await
    }

    /// Retrieves readiness state.
    pub async fn get_ready_state(&self) -> Result<ReadyState> {
        self.client.request(&self.method_path("GetReadyState"), &EmptyParams {}).await
    }

    /// Retrieves current landed state (Landed, Airborne, Unknown).
    pub async fn get_landed_state(&self) -> Result<LandedState> {
        self.client.request(&self.method_path("GetLandedState"), &EmptyParams {}).await
    }

    // --- Flight Operations ---

    /// Commands the drone to take off and hover at default takeoff altitude.
    pub async fn takeoff(&self, timeout_sec: f32) -> Result<bool> {
        info!("Commanding takeoff for drone '{}' (timeout: {}s)", self.drone_name, timeout_sec);
        self.client
            .request(&self.method_path("Takeoff"), &TimeoutParams { timeout_sec })
            .await
    }

    /// Commands the drone to land at its current position.
    pub async fn land(&self, timeout_sec: f32) -> Result<bool> {
        info!("Commanding landing for drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("Land"), &TimeoutParams { timeout_sec })
            .await
    }

    /// Commands the drone to hold position and hover in place.
    pub async fn hover(&self) -> Result<bool> {
        self.client.request(&self.method_path("Hover"), &EmptyParams {}).await
    }

    /// Commands the drone to return to home/launch coordinates.
    pub async fn go_home(&self, timeout_sec: f32) -> Result<bool> {
        self.client
            .request(&self.method_path("GoHome"), &TimeoutParams { timeout_sec })
            .await
    }

    /// Commands velocity in the world frame (North, East, Down in m/s).
    pub async fn move_by_velocity(&self, vx: f64, vy: f64, vz: f64, duration_sec: f64) -> Result<bool> {
        self.client
            .request(
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
            .await
    }

    /// Moves the drone to target NED coordinates with specified cruise velocity.
    pub async fn move_to_position(
        &self,
        north: f64,
        east: f64,
        down: f64,
        velocity: f64,
        timeout_sec: f32,
    ) -> Result<bool> {
        self.client
            .request(
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
            .await
    }

    // --- State & Sensors ---

    /// Queries the ground-truth 6-DoF pose of the drone.
    pub async fn get_ground_truth_pose(&self) -> Result<Pose> {
        self.client.request(&self.method_path("GetGroundTruthPose"), &EmptyParams {}).await
    }

    /// Captures images from a named onboard camera.
    pub async fn get_images(&self, camera_id: &str, image_types: &[ImageType]) -> Result<Vec<ImageResponse>> {
        let type_ids: Vec<i32> = image_types.iter().map(|t| *t as i32).collect();
        self.client
            .request(
                &self.method_path("GetImages"),
                &GetImagesParams {
                    camera_id,
                    image_types: type_ids,
                },
            )
            .await
    }

    /// Subscribes to a real-time telemetry topic streamed for this vehicle (e.g. `robot_info/ground_truth_pose`).
    pub async fn subscribe_telemetry(&self, subtopic: &str) -> Result<TopicSubscription> {
        let topic = format!("{}/{}/{}", self.parent_topic, self.drone_name, subtopic);
        self.client.subscribe(topic).await
    }
}
