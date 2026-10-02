use std::collections::HashMap;

use serde::Serialize;
use tracing::info;

use crate::async_api::client::{Client, TopicSubscription};
use crate::error::Result;
use crate::types::{
    GeoPosition, ImageResponse, ImageType, LandedState, Pose, ReadyState, Transform, VTOLMode,
    Vector3, YawControlMode,
};

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
struct GoHomeParams {
    timeout_sec: f32,
    velocity: f32,
}

#[derive(Serialize)]
struct SetVTOLModeParams {
    vtol_mode: i32,
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
struct MoveVelocityZParams {
    vx: f64,
    vy: f64,
    z: f64,
    duration: f64,
    drivetrain: i32,
    yaw_is_rate: bool,
    yaw: f64,
}

#[derive(Serialize)]
struct MoveByHeadingParams {
    heading: f32,
    speed: f32,
    vz: f32,
    duration: f32,
    heading_margin: f32,
    yaw_rate: f32,
    timeout_sec: f32,
}

#[derive(Serialize)]
struct MoveOnPathParams<'a> {
    path: &'a [Vector3],
    velocity: f32,
    timeout_sec: f32,
    drivetrain: i32,
    yaw_is_rate: bool,
    yaw: f32,
    lookahead: f32,
    adaptive_lookahead: f32,
}

#[derive(Serialize)]
struct MoveToPositionParams {
    x: f64,
    y: f64,
    z: f64,
    velocity: f64,
    timeout_sec: f32,
    drivetrain: i32,
    yaw_is_rate: bool,
    yaw: f64,
    lookahead: f64,
    adaptive_lookahead: f64,
}

#[derive(Serialize)]
struct RotateToYawParams {
    yaw: f32,
    timeout_sec: f32,
    margin: f32,
    yaw_rate: f32,
}

#[derive(Serialize)]
struct RotateByYawRateParams {
    yaw_rate: f32,
    duration: f32,
}

#[derive(Serialize)]
struct SetBatteryRemainingParams {
    desired_battery_remaining: f32,
}

#[derive(Serialize)]
struct SetBatteryDrainRateParams {
    desired_drain_rate: f32,
}

#[derive(Serialize)]
struct SetBatteryHealthStatusParams {
    battery_health_indicator: bool,
}

#[derive(Serialize)]
struct GetCameraRayParams<'a> {
    camera_id: &'a str,
    image_type: i32,
    x: i32,
    y: i32,
}

#[derive(Serialize)]
struct CameraLookAtObjectParams<'a> {
    object_name: &'a str,
    wait_for_pose_update: bool,
}

#[derive(Serialize)]
struct CameraDrawFrustumParams {
    image_type: i32,
    to_enable: bool,
}

#[derive(Serialize)]
struct SetCameraPoseParams<'a> {
    pose: &'a Pose,
    wait_for_pose_update: bool,
}

#[derive(Serialize)]
struct ResetCameraPoseParams {
    wait_for_pose_update: bool,
}

#[derive(Serialize)]
struct CameraOpticsFocalLengthParams {
    image_type_id: i32,
    focal_length: f32,
}

#[derive(Serialize)]
struct CameraOpticsTransitionParams {
    image_type_id: i32,
    transition_threshold: f32,
}

#[derive(Serialize)]
struct CameraOpticsIntensityParams {
    image_type_id: i32,
    intensity: f32,
}

#[derive(Serialize)]
struct CameraOpticsFOVParams {
    image_type_id: i32,
    field_of_view: f32,
}

#[derive(Serialize)]
struct UpdateActuatorFaultParams {
    enable: bool,
}

#[derive(Serialize)]
struct SetExternalForceParams<'a> {
    ext_force: &'a [f32],
}

#[derive(Serialize)]
struct SetControlSignalsParams<'a> {
    control_signal_map: &'a HashMap<String, f32>,
}

#[derive(Serialize)]
struct SetKinematicsParams<'a> {
    kinematics: &'a serde_json::Value,
}

#[derive(Serialize)]
struct SetPoseParams<'a> {
    pose: &'a Transform,
    reset_kinematics: bool,
}

#[derive(Serialize)]
struct GetImagesParams {
    image_type_ids: Vec<i32>,
}

#[derive(Serialize)]
struct EmptyParams {}

impl Drone {
    /// Creates a new Drone handle.
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

    /// Computes the base robot topic root path.
    fn base_topic(&self) -> String {
        if self
            .parent_topic
            .ends_with(&format!("/robots/{}", self.drone_name))
        {
            self.parent_topic.clone()
        } else if self.parent_topic.ends_with("/robots") {
            format!("{}/{}", self.parent_topic, self.drone_name)
        } else if self.parent_topic.contains("/robots/") {
            self.parent_topic.clone()
        } else if self.parent_topic == "/Sim" {
            format!("{}/{}", self.parent_topic, self.drone_name)
        } else {
            format!("{}/robots/{}", self.parent_topic, self.drone_name)
        }
    }

    /// Builds the full RPC method path (e.g. `/Sim/Scene/robots/Drone1/Takeoff`).
    fn method_path(&self, method: &str) -> String {
        format!("{}/{}", self.base_topic(), method)
    }

    /// Builds the full RPC sensor method path (e.g. `/Sim/Scene/robots/Drone1/sensors/Camera1/GetImages`).
    fn sensor_path(&self, sensor_name: &str, method: &str) -> String {
        format!("{}/sensors/{sensor_name}/{method}", self.base_topic())
    }

    /// Builds the full published topic path for a sensor (e.g. `/Sim/robots/Drone1/sensors/lidar1/lidar`).
    pub fn get_sensor_topic(&self, sensor_name: &str, topic: &str) -> String {
        format!("{}/sensors/{sensor_name}/{topic}", self.base_topic())
    }

    /// Builds the full RPC actuator method path (e.g. `/Sim/Scene/robots/Drone1/actuators/0/ToggleFault`).
    fn actuator_path(&self, actuator_id: &str, method: &str) -> String {
        format!("{}/actuators/{actuator_id}/{method}", self.base_topic())
    }

    // --- API Control & Readiness ---

    /// Requests API control over the vehicle.
    pub async fn enable_api_control(&self) -> Result<bool> {
        info!("Enabling API control for drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("EnableApiControl"), &EmptyParams {})
            .await
    }

    /// Releases API control back to RC/manual or onboard autonomy.
    pub async fn disable_api_control(&self) -> Result<bool> {
        info!("Disabling API control for drone '{}'", self.drone_name);
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

    /// Cancels the currently executing asynchronous movement task.
    pub async fn cancel_last_task(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("CancelLastTask"), &EmptyParams {})
            .await
    }

    /// Arms the drone's motors.
    pub async fn arm(&self) -> Result<bool> {
        info!("Arming drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("Arm"), &EmptyParams {})
            .await
    }

    /// Disarms the drone's motors.
    pub async fn disarm(&self) -> Result<bool> {
        info!("Disarming drone '{}'", self.drone_name);
        self.client
            .request(&self.method_path("Disarm"), &EmptyParams {})
            .await
    }

    /// Checks if the drone passes all pre-flight checks and can arm.
    pub async fn can_arm(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("CanArm"), &EmptyParams {})
            .await
    }

    /// Retrieves readiness state.
    pub async fn get_ready_state(&self) -> Result<ReadyState> {
        self.client
            .request(&self.method_path("GetReadyState"), &EmptyParams {})
            .await
    }

    /// Retrieves current landed state (Landed, Airborne, Unknown).
    pub async fn get_landed_state(&self) -> Result<LandedState> {
        let code: i32 = self
            .client
            .request(&self.method_path("GetLandedState"), &EmptyParams {})
            .await?;
        match code {
            0 => Ok(LandedState::Landed),
            1 => Ok(LandedState::Airborne),
            _ => Ok(LandedState::Unknown),
        }
    }

    // --- Flight Operations & Maneuvers ---

    /// Commands the drone to take off and hover at default takeoff altitude.
    pub async fn takeoff(&self, timeout_sec: f32) -> Result<bool> {
        info!(
            "Commanding takeoff for drone '{}' (timeout: {}s)",
            self.drone_name, timeout_sec
        );
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
        self.client
            .request(&self.method_path("Hover"), &EmptyParams {})
            .await
    }

    /// Commands the drone to return to home/launch coordinates.
    pub async fn go_home(&self, timeout_sec: f32, velocity: f32) -> Result<bool> {
        self.client
            .request(
                &self.method_path("GoHome"),
                &GoHomeParams {
                    timeout_sec,
                    velocity,
                },
            )
            .await
    }

    /// Requests drone exit automatic mode and enable manual mode.
    pub async fn request_control(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("RequestControl"), &EmptyParams {})
            .await
    }

    /// Sets drone to execute a previously loaded mission profile.
    pub async fn set_mission_mode(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("SetMissionMode"), &EmptyParams {})
            .await
    }

    /// Sets drone flight mode on VTOL convertible vehicles (Multirotor or FixedWing).
    pub async fn set_vtol_mode(&self, vtol_mode: VTOLMode) -> Result<bool> {
        self.client
            .request(
                &self.method_path("SetVTOLMode"),
                &SetVTOLModeParams {
                    vtol_mode: vtol_mode as i32,
                },
            )
            .await
    }

    /// Commands velocity in the world frame (North, East, Down in m/s).
    pub async fn move_by_velocity(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
    ) -> Result<bool> {
        self.move_by_velocity_with_yaw(
            vx,
            vy,
            vz,
            duration_sec,
            YawControlMode::MaxDegreeOfFreedom,
            true,
            0.0,
        )
        .await
    }

    /// Commands velocity in the world frame with full yaw control specification.
    pub async fn move_by_velocity_with_yaw(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("MoveByVelocity"),
                &MoveVelocityParams {
                    vx,
                    vy,
                    vz,
                    duration: duration_sec,
                    drivetrain: drivetrain as i32,
                    yaw_is_rate,
                    yaw,
                },
            )
            .await
    }

    /// Commands horizontal world velocities (North, East) while holding fixed target altitude Z (Down in meters).
    pub async fn move_by_velocity_z(
        &self,
        vx: f64,
        vy: f64,
        z: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("MoveByVelocityZ"),
                &MoveVelocityZParams {
                    vx,
                    vy,
                    z,
                    duration: duration_sec,
                    drivetrain: drivetrain as i32,
                    yaw_is_rate,
                    yaw,
                },
            )
            .await
    }

    /// Commands velocity relative to the drone's body frame (Forward, Right, Down in m/s).
    pub async fn move_by_velocity_body_frame(
        &self,
        v_forward: f64,
        v_right: f64,
        v_down: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("MoveByVelocityBodyFrame"),
                &MoveVelocityParams {
                    vx: v_forward,
                    vy: v_right,
                    vz: v_down,
                    duration: duration_sec,
                    drivetrain: drivetrain as i32,
                    yaw_is_rate,
                    yaw,
                },
            )
            .await
    }

    /// Commands velocity in the body frame with fixed target altitude Z (Down in meters).
    pub async fn move_by_velocity_body_frame_z(
        &self,
        v_forward: f64,
        v_right: f64,
        z: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("MoveByVelocityBodyFrameZ"),
                &MoveVelocityZParams {
                    vx: v_forward,
                    vy: v_right,
                    z,
                    duration: duration_sec,
                    drivetrain: drivetrain as i32,
                    yaw_is_rate,
                    yaw,
                },
            )
            .await
    }

    /// Commands flight along a heading angle at a specified horizontal speed and vertical rate.
    pub async fn move_by_heading(
        &self,
        heading: f32,
        speed: f32,
        v_down: f32,
        duration_sec: f32,
        heading_margin: f32,
        yaw_rate: f32,
        timeout_sec: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("MoveByHeading"),
                &MoveByHeadingParams {
                    heading,
                    speed,
                    vz: v_down,
                    duration: duration_sec,
                    heading_margin,
                    yaw_rate,
                    timeout_sec,
                },
            )
            .await
    }

    /// Traverses a sequential 3D waypoint path using carrot-following guidance.
    pub async fn move_on_path(
        &self,
        path: &[Vector3],
        velocity: f32,
        timeout_sec: f32,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f32,
        lookahead: f32,
        adaptive_lookahead: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("MoveOnPath"),
                &MoveOnPathParams {
                    path,
                    velocity,
                    timeout_sec,
                    drivetrain: drivetrain as i32,
                    yaw_is_rate,
                    yaw,
                    lookahead,
                    adaptive_lookahead,
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
        self.move_to_position_with_yaw(
            north,
            east,
            down,
            velocity,
            timeout_sec,
            YawControlMode::MaxDegreeOfFreedom,
            true,
            0.0,
            -1.0,
            1.0,
        )
        .await
    }

    /// Moves the drone to target NED coordinates with full yaw and lookahead parameters.
    pub async fn move_to_position_with_yaw(
        &self,
        north: f64,
        east: f64,
        down: f64,
        velocity: f64,
        timeout_sec: f32,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
        lookahead: f64,
        adaptive_lookahead: f64,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("MoveToPosition"),
                &MoveToPositionParams {
                    x: north,
                    y: east,
                    z: down,
                    velocity,
                    timeout_sec,
                    drivetrain: drivetrain as i32,
                    yaw_is_rate,
                    yaw,
                    lookahead,
                    adaptive_lookahead,
                },
            )
            .await
    }

    /// Rotates the aircraft to an absolute target yaw heading in radians.
    pub async fn rotate_to_yaw(
        &self,
        yaw: f32,
        timeout_sec: f32,
        margin: f32,
        yaw_rate: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("RotateToYaw"),
                &RotateToYawParams {
                    yaw,
                    timeout_sec,
                    margin,
                    yaw_rate,
                },
            )
            .await
    }

    /// Rotates the aircraft at a fixed yaw rate (radians/second) for a specified duration.
    pub async fn rotate_by_yaw_rate(&self, yaw_rate: f32, duration_sec: f32) -> Result<bool> {
        self.client
            .request(
                &self.method_path("RotateByYawRate"),
                &RotateByYawRateParams {
                    yaw_rate,
                    duration: duration_sec,
                },
            )
            .await
    }

    // --- Direct Sensor Reads & Battery ---

    /// Queries raw telemetry data from a named onboard IMU sensor.
    pub async fn get_imu_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.sensor_path(sensor_name, "imu_kinematics"),
                &EmptyParams {},
            )
            .await
    }

    /// Queries raw telemetry data from a named onboard GPS sensor.
    pub async fn get_gps_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "gps"), &EmptyParams {})
            .await
    }

    /// Queries raw telemetry data from a named onboard airspeed sensor.
    pub async fn get_airspeed_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "airspeed"), &EmptyParams {})
            .await
    }

    /// Queries raw telemetry data from a named onboard barometer.
    pub async fn get_barometer_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "barometer"), &EmptyParams {})
            .await
    }

    /// Queries raw telemetry data from a named onboard magnetometer.
    pub async fn get_magnetometer_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.sensor_path(sensor_name, "magnetometer"),
                &EmptyParams {},
            )
            .await
    }

    /// Queries raw point cloud payload from a named onboard lidar sensor.
    pub async fn get_lidar_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "lidar"), &EmptyParams {})
            .await
    }

    /// Queries raw detections from a named onboard radar sensor.
    pub async fn get_radar_detections(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.sensor_path(sensor_name, "radar_detections"),
                &EmptyParams {},
            )
            .await
    }

    /// Queries clustered target tracks from a named onboard radar sensor.
    pub async fn get_radar_tracks(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.sensor_path(sensor_name, "radar_tracks"),
                &EmptyParams {},
            )
            .await
    }

    /// Queries the vehicle's battery state.
    pub async fn get_battery_state(&self) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.sensor_path("Battery", "GetBatteryStatus"),
                &EmptyParams {},
            )
            .await
    }

    /// Sets the remaining battery capacity fraction (0.0 to 1.0).
    pub async fn set_battery_remaining(&self, desired_battery_remaining: f32) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path("Battery", "SetBatteryRemaining"),
                &SetBatteryRemainingParams {
                    desired_battery_remaining,
                },
            )
            .await
    }

    /// Queries the current battery drain rate.
    pub async fn get_battery_drain_rate(&self) -> Result<f32> {
        self.client
            .request(
                &self.sensor_path("Battery", "GetBatteryDrainRate"),
                &EmptyParams {},
            )
            .await
    }

    /// Sets the simulation battery drain rate.
    pub async fn set_battery_drain_rate(&self, desired_drain_rate: f32) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path("Battery", "SetBatteryDrainRate"),
                &SetBatteryDrainRateParams { desired_drain_rate },
            )
            .await
    }

    /// Sets the health indicator state for the onboard battery.
    pub async fn set_battery_health_status(&self, is_desired_state_healthy: bool) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path("Battery", "SetBatteryHealthStatus"),
                &SetBatteryHealthStatusParams {
                    battery_health_indicator: is_desired_state_healthy,
                },
            )
            .await
    }

    // --- Camera Optics & Rendering Controls ---

    /// Captures images from a named onboard camera.
    pub async fn get_images(
        &self,
        camera_id: &str,
        image_types: &[ImageType],
    ) -> Result<Vec<ImageResponse>> {
        let type_ids: Vec<i32> = image_types.iter().map(|t| *t as i32).collect();
        self.client
            .request(
                &self.sensor_path(camera_id, "GetImages"),
                &GetImagesParams {
                    image_type_ids: type_ids,
                },
            )
            .await
    }

    /// Computes a 3D ray through a specified pixel in an onboard camera sensor.
    pub async fn get_camera_ray(
        &self,
        camera_id: &str,
        image_type: ImageType,
        x: i32,
        y: i32,
    ) -> Result<Pose> {
        self.client
            .request(
                &self.method_path("GetCameraRay"),
                &GetCameraRayParams {
                    camera_id,
                    image_type: image_type as i32,
                    x,
                    y,
                },
            )
            .await
    }

    /// Orients an onboard camera sensor to track a named scene actor.
    pub async fn camera_look_at_object(
        &self,
        camera_id: &str,
        object_name: &str,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "LookAtObject"),
                &CameraLookAtObjectParams {
                    object_name,
                    wait_for_pose_update,
                },
            )
            .await
    }

    /// Toggles frustum wireframe visualization in the viewport for a camera sensor.
    pub async fn camera_draw_frustum(
        &self,
        camera_id: &str,
        to_enable: bool,
        image_type: ImageType,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "DrawFrustum"),
                &CameraDrawFrustumParams {
                    image_type: image_type as i32,
                    to_enable,
                },
            )
            .await
    }

    /// Overrides the 6-DoF mounting pose of an onboard camera sensor.
    pub async fn set_camera_pose(
        &self,
        camera_id: &str,
        pose: &Pose,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "SetPose"),
                &SetCameraPoseParams {
                    pose,
                    wait_for_pose_update,
                },
            )
            .await
    }

    /// Resets an onboard camera sensor to its default mounting pose.
    pub async fn reset_camera_pose(
        &self,
        camera_id: &str,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "ResetCameraPose"),
                &ResetCameraPoseParams {
                    wait_for_pose_update,
                },
            )
            .await
    }

    /// Configures the focal length of an onboard camera sensor lens.
    pub async fn set_focal_length(
        &self,
        camera_id: &str,
        image_type_id: i32,
        focal_length: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "SetFocalLength"),
                &CameraOpticsFocalLengthParams {
                    image_type_id,
                    focal_length,
                },
            )
            .await
    }

    /// Configures depth of field transition threshold.
    pub async fn set_depth_of_field_transition_threshold(
        &self,
        camera_id: &str,
        image_type_id: i32,
        transition_threshold: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "SetDepthOfFieldTransitionRegion"),
                &CameraOpticsTransitionParams {
                    image_type_id,
                    transition_threshold,
                },
            )
            .await
    }

    /// Configures depth of field focal region.
    pub async fn set_depth_of_field_focal_region(
        &self,
        camera_id: &str,
        image_type_id: i32,
        max_focal_distance: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "SetDepthOfFieldFocalRegion"),
                &CameraOpticsFocalLengthParams {
                    image_type_id,
                    focal_length: max_focal_distance,
                },
            )
            .await
    }

    /// Configures chromatic aberration intensity on captured camera frames.
    pub async fn set_chromatic_aberration_intensity(
        &self,
        camera_id: &str,
        image_type_id: i32,
        intensity: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "SetChromaticAberrationIntensity"),
                &CameraOpticsIntensityParams {
                    image_type_id,
                    intensity,
                },
            )
            .await
    }

    /// Configures the horizontal field of view (FOV) in degrees.
    pub async fn set_field_of_view(
        &self,
        camera_id: &str,
        image_type_id: i32,
        field_of_view: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.sensor_path(camera_id, "SetFieldOfView"),
                &CameraOpticsFOVParams {
                    image_type_id,
                    field_of_view,
                },
            )
            .await
    }

    // --- Fault Injection, Disturbances & Kinematics ---

    /// Toggles a simulated actuator fault (e.g. motor failure) on a named rotor actuator.
    pub async fn update_actuator_fault_state(
        &self,
        actuator_id: &str,
        fault_configured: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.actuator_path(actuator_id, "ToggleFault"),
                &UpdateActuatorFaultParams {
                    enable: fault_configured,
                },
            )
            .await
    }

    /// Injects an external force disturbance vector [fx, fy, fz] onto the aircraft body.
    pub async fn set_external_force(&self, ext_force: &[f32]) -> Result<bool> {
        self.client
            .request(
                &self.method_path("SetExternalForce"),
                &SetExternalForceParams { ext_force },
            )
            .await
    }

    /// Overrides low-level actuator control signals (e.g. PWM/thrust commands).
    pub async fn set_control_signals(
        &self,
        control_signal_map: &HashMap<String, f32>,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("SetControlSignals"),
                &SetControlSignalsParams { control_signal_map },
            )
            .await
    }

    /// Queries ground-truth 6-DoF pose of the drone.
    pub async fn get_ground_truth_pose(&self) -> Result<Pose> {
        self.client
            .request(&self.method_path("GetGroundTruthPose"), &EmptyParams {})
            .await
    }

    /// Queries ground-truth geographic location (lat, lon, alt).
    pub async fn get_ground_truth_geo_location(&self) -> Result<GeoPosition> {
        self.client
            .request(
                &self.method_path("GetGroundTruthGeoLocation"),
                &EmptyParams {},
            )
            .await
    }

    /// Queries estimated geographic location as computed by vehicle state estimators.
    pub async fn get_estimated_geo_location(&self) -> Result<GeoPosition> {
        self.client
            .request(
                &self.method_path("GetEstimatedGeoLocation"),
                &EmptyParams {},
            )
            .await
    }

    /// Queries full ground-truth kinematic state dictionary.
    pub async fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.method_path("GetGroundTruthKinematics"),
                &EmptyParams {},
            )
            .await
    }

    /// Queries estimated kinematic state dictionary.
    pub async fn get_estimated_kinematics(&self) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.method_path("GetEstimatedKinematics"),
                &EmptyParams {},
            )
            .await
    }

    /// Overrides vehicle ground-truth kinematics directly in the simulation engine.
    pub async fn set_ground_truth_kinematics(
        &self,
        kinematics: &serde_json::Value,
    ) -> Result<bool> {
        self.client
            .request(
                &self.method_path("SetGroundTruthKinematics"),
                &SetKinematicsParams { kinematics },
            )
            .await
    }

    /// Teleports or relocates the vehicle to a target spatial transform.
    pub async fn set_pose(&self, transform: &Transform, reset_kinematics: bool) -> Result<bool> {
        self.client
            .request(
                &self.method_path("SetPose"),
                &SetPoseParams {
                    pose: transform,
                    reset_kinematics,
                },
            )
            .await
    }

    /// Subscribes to a real-time telemetry topic streamed for this vehicle (e.g. `robot_info/ground_truth_pose`).
    pub async fn subscribe_telemetry(&self, subtopic: &str) -> Result<TopicSubscription> {
        let topic = format!("{}/{}", self.base_topic(), subtopic);
        self.client.subscribe(topic).await
    }
}
