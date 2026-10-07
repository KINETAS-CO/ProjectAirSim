use std::collections::HashMap;

use tracing::info;

use crate::blocking::async_result::AsyncResult;
use crate::blocking::client::Client;
use crate::error::Result;
use crate::protocol::frame::TopicFrame;
use crate::protocol::params::{
    CameraDrawFrustumParams, CameraLookAtObjectParams, CameraOpticsFOVParams,
    CameraOpticsFocalLengthParams, CameraOpticsIntensityParams, CameraOpticsTransitionParams,
    DroneGetImagesParams as GetImagesParams, EmptyParams, GetCameraRayParams, GoHomeParams,
    MoveByHeadingParams, MoveOnPathParams, MoveToPositionParams, MoveVelocityParams,
    MoveVelocityZParams, ResetCameraPoseParams, RotateByYawRateParams, RotateToYawParams,
    SetBatteryDrainRateParams, SetBatteryHealthStatusParams, SetBatteryRemainingParams,
    SetCameraPoseParams, SetControlSignalsParams, SetExternalForceParams, SetKinematicsParams,
    SetPoseParams, SetVTOLModeParams, TimeoutParams, UpdateActuatorFaultParams,
};
use crate::types::{
    GeoPosition, ImageResponse, ImageType, LandedState, Pose, ReadyState, Transform, VTOLMode,
    Vector3, YawControlMode,
};

/// Synchronous blocking control handle for a drone/multirotor vehicle.
#[derive(Clone)]
pub struct Drone {
    client: Client,
    drone_name: String,
    parent_topic: String,
}

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

    /// Cancels the currently executing asynchronous movement command or mission.
    pub fn cancel_last_task(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("CancelLastTask"), &EmptyParams {})
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

    // --- Flight Operations (Takeoff, Land, Hover, Home, Modes) ---

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

    /// Dispatches hover asynchronously, commanding the drone to hold position.
    pub fn hover_async(&self) -> AsyncResult<bool> {
        self.client
            .request_async(&self.method_path("Hover"), &EmptyParams {})
    }

    /// Commands the drone to hold position and hover in place, blocking until complete.
    pub fn hover(&self) -> Result<bool> {
        self.hover_async().get_result()
    }

    /// Dispatches return-to-home flight asynchronously.
    pub fn go_home_async(&self, timeout_sec: f32, velocity: f32) -> AsyncResult<bool> {
        self.client.request_async(
            &self.method_path("GoHome"),
            &GoHomeParams {
                timeout_sec,
                velocity,
            },
        )
    }

    /// Commands return-to-home flight and blocks until complete.
    pub fn go_home(&self, timeout_sec: f32, velocity: f32) -> Result<bool> {
        self.go_home_async(timeout_sec, velocity).get_result()
    }

    /// Requests flight control from the onboard flight computer.
    pub fn request_control(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("RequestControl"), &EmptyParams {})
    }

    /// Sets autopilot mission flight mode.
    pub fn set_mission_mode(&self) -> Result<bool> {
        self.client
            .request(&self.method_path("SetMissionMode"), &EmptyParams {})
    }

    /// Configures VTOL aircraft transition mode (Multirotor vs FixedWing).
    pub fn set_vtol_mode(&self, vtol_mode: VTOLMode) -> Result<bool> {
        self.client.request(
            &self.method_path("SetVTOLMode"),
            &SetVTOLModeParams {
                vtol_mode: vtol_mode as i32,
            },
        )
    }

    // --- Velocity & Trajectory Control ---

    /// Dispatches world-frame velocity command asynchronously.
    pub fn move_by_velocity_async(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
    ) -> AsyncResult<bool> {
        self.client.request_async(
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

    /// Commands velocity in the world frame (North, East, Down in m/s) and blocks.
    pub fn move_by_velocity(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
    ) -> Result<bool> {
        self.move_by_velocity_async(vx, vy, vz, duration_sec)
            .get_result()
    }

    /// Dispatches velocity command with explicit yaw orientation asynchronously.
    pub fn move_by_velocity_with_yaw_async(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> AsyncResult<bool> {
        self.client.request_async(
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
    }

    /// Commands velocity with explicit yaw and blocks until complete.
    pub fn move_by_velocity_with_yaw(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.move_by_velocity_with_yaw_async(
            vx,
            vy,
            vz,
            duration_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
        )
        .get_result()
    }

    /// Dispatches horizontal velocity command at fixed altitude asynchronously.
    pub fn move_by_velocity_z_async(
        &self,
        vx: f64,
        vy: f64,
        z: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> AsyncResult<bool> {
        self.client.request_async(
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
    }

    /// Commands horizontal velocity at fixed altitude and blocks until complete.
    pub fn move_by_velocity_z(
        &self,
        vx: f64,
        vy: f64,
        z: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.move_by_velocity_z_async(vx, vy, z, duration_sec, drivetrain, yaw_is_rate, yaw)
            .get_result()
    }

    /// Dispatches body-frame velocity vector command asynchronously.
    pub fn move_by_velocity_body_frame_async(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> AsyncResult<bool> {
        self.client.request_async(
            &self.method_path("MoveByVelocityBodyFrame"),
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
    }

    /// Commands velocity in the vehicle body frame and blocks until complete.
    pub fn move_by_velocity_body_frame(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.move_by_velocity_body_frame_async(
            vx,
            vy,
            vz,
            duration_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
        )
        .get_result()
    }

    /// Dispatches body-frame horizontal velocity at fixed altitude asynchronously.
    pub fn move_by_velocity_body_frame_z_async(
        &self,
        vx: f64,
        vy: f64,
        z: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> AsyncResult<bool> {
        self.client.request_async(
            &self.method_path("MoveByVelocityBodyFrameZ"),
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
    }

    /// Commands body-frame horizontal velocity at fixed altitude and blocks until complete.
    pub fn move_by_velocity_body_frame_z(
        &self,
        vx: f64,
        vy: f64,
        z: f64,
        duration_sec: f64,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f64,
    ) -> Result<bool> {
        self.move_by_velocity_body_frame_z_async(
            vx,
            vy,
            z,
            duration_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
        )
        .get_result()
    }

    /// Dispatches heading tracking flight asynchronously.
    pub fn move_by_heading_async(
        &self,
        heading: f32,
        speed: f32,
        vz: f32,
        duration_sec: f32,
        heading_margin: f32,
        yaw_rate: f32,
        timeout_sec: f32,
    ) -> AsyncResult<bool> {
        self.client.request_async(
            &self.method_path("MoveByHeading"),
            &MoveByHeadingParams {
                heading,
                speed,
                vz,
                duration: duration_sec,
                heading_margin,
                yaw_rate,
                timeout_sec,
            },
        )
    }

    /// Commands flight along a target heading angle and blocks until complete.
    pub fn move_by_heading(
        &self,
        heading: f32,
        speed: f32,
        vz: f32,
        duration_sec: f32,
        heading_margin: f32,
        yaw_rate: f32,
        timeout_sec: f32,
    ) -> Result<bool> {
        self.move_by_heading_async(
            heading,
            speed,
            vz,
            duration_sec,
            heading_margin,
            yaw_rate,
            timeout_sec,
        )
        .get_result()
    }

    /// Dispatches path-following flight asynchronously along a sequence of 3D waypoints.
    pub fn move_on_path_async(
        &self,
        path: &[Vector3],
        velocity: f32,
        timeout_sec: f32,
        drivetrain: YawControlMode,
        yaw_is_rate: bool,
        yaw: f32,
        lookahead: f32,
        adaptive_lookahead: f32,
    ) -> AsyncResult<bool> {
        self.client.request_async(
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
    }

    /// Follows a trajectory through waypoints and blocks until complete.
    pub fn move_on_path(
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
        self.move_on_path_async(
            path,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
            lookahead,
            adaptive_lookahead,
        )
        .get_result()
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
                x: north,
                y: east,
                z: down,
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

    /// Dispatches position target with explicit yaw orientation asynchronously.
    pub fn move_to_position_with_yaw_async(
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
    ) -> AsyncResult<bool> {
        self.client.request_async(
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
    }

    /// Commands movement to target NED coordinates with yaw and blocks until complete.
    pub fn move_to_position_with_yaw(
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
        self.move_to_position_with_yaw_async(
            north,
            east,
            down,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
            lookahead,
            adaptive_lookahead,
        )
        .get_result()
    }

    /// Dispatches vehicle rotation to an absolute yaw angle asynchronously.
    pub fn rotate_to_yaw_async(
        &self,
        yaw_deg: f32,
        timeout_sec: f32,
        margin_deg: f32,
        yaw_rate_deg_per_sec: f32,
    ) -> AsyncResult<bool> {
        self.client.request_async(
            &self.method_path("RotateToYaw"),
            &RotateToYawParams {
                yaw: yaw_deg,
                timeout_sec,
                margin: margin_deg,
                yaw_rate: yaw_rate_deg_per_sec,
            },
        )
    }

    /// Rotates the drone to a target absolute heading angle in degrees and blocks until complete.
    pub fn rotate_to_yaw(
        &self,
        yaw_deg: f32,
        timeout_sec: f32,
        margin_deg: f32,
        yaw_rate_deg_per_sec: f32,
    ) -> Result<bool> {
        self.rotate_to_yaw_async(yaw_deg, timeout_sec, margin_deg, yaw_rate_deg_per_sec)
            .get_result()
    }

    /// Dispatches vehicle rotation at a continuous angular velocity asynchronously.
    pub fn rotate_by_yaw_rate_async(&self, yaw_rate: f32, duration_sec: f32) -> AsyncResult<bool> {
        self.client.request_async(
            &self.method_path("RotateByYawRate"),
            &RotateByYawRateParams {
                yaw_rate,
                duration: duration_sec,
            },
        )
    }

    /// Rotates the drone at a specified yaw rate for duration and blocks until complete.
    pub fn rotate_by_yaw_rate(&self, yaw_rate: f32, duration_sec: f32) -> Result<bool> {
        self.rotate_by_yaw_rate_async(yaw_rate, duration_sec)
            .get_result()
    }

    // --- Sensor Queries & Battery Controls ---

    /// Queries IMU kinematics (angular velocity, linear acceleration, orientation) from a named sensor.
    pub fn get_imu_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client.request(
            &self.sensor_path(sensor_name, "imu_kinematics"),
            &EmptyParams {},
        )
    }

    /// Queries GPS position and velocity readings from a named sensor.
    pub fn get_gps_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "gps"), &EmptyParams {})
    }

    /// Queries airspeed reading from a named pitot/airspeed sensor.
    pub fn get_airspeed_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "airspeed"), &EmptyParams {})
    }

    /// Queries atmospheric pressure and altitude readings from a named barometer sensor.
    pub fn get_barometer_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "barometer"), &EmptyParams {})
    }

    /// Queries 3-axis magnetic field vector readings from a named magnetometer sensor.
    pub fn get_magnetometer_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client.request(
            &self.sensor_path(sensor_name, "magnetometer"),
            &EmptyParams {},
        )
    }

    /// Queries point cloud data from an onboard lidar sensor.
    pub fn get_lidar_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .request(&self.sensor_path(sensor_name, "lidar"), &EmptyParams {})
    }

    /// Queries raw radar detection points from a named radar sensor.
    pub fn get_radar_detections(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client.request(
            &self.sensor_path(sensor_name, "radar_detections"),
            &EmptyParams {},
        )
    }

    /// Queries tracked object targets from a named radar sensor.
    pub fn get_radar_tracks(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client.request(
            &self.sensor_path(sensor_name, "radar_tracks"),
            &EmptyParams {},
        )
    }

    /// Queries battery telemetry status and remaining charge percentage.
    pub fn get_battery_state(&self) -> Result<serde_json::Value> {
        self.client.request(
            &self.sensor_path("Battery", "GetBatteryStatus"),
            &EmptyParams {},
        )
    }

    /// Overrides simulated battery remaining charge percentage [0.0 - 100.0].
    pub fn set_battery_remaining(&self, desired_battery_remaining: f32) -> Result<bool> {
        self.client.request(
            &self.sensor_path("Battery", "SetBatteryRemaining"),
            &SetBatteryRemainingParams {
                desired_battery_remaining,
            },
        )
    }

    /// Queries the current battery drain rate multiplier.
    pub fn get_battery_drain_rate(&self) -> Result<f32> {
        self.client.request(
            &self.sensor_path("Battery", "GetBatteryDrainRate"),
            &EmptyParams {},
        )
    }

    /// Configures the battery drain rate consumption multiplier.
    pub fn set_battery_drain_rate(&self, desired_drain_rate: f32) -> Result<bool> {
        self.client.request(
            &self.sensor_path("Battery", "SetBatteryDrainRate"),
            &SetBatteryDrainRateParams { desired_drain_rate },
        )
    }

    /// Configures battery hardware health status flag.
    pub fn set_battery_health_status(&self, is_desired_state_healthy: bool) -> Result<bool> {
        self.client.request(
            &self.sensor_path("Battery", "SetBatteryHealthStatus"),
            &SetBatteryHealthStatusParams {
                battery_health_indicator: is_desired_state_healthy,
            },
        )
    }

    // --- Camera Optics & Rendering Controls ---

    /// Captures images from a named onboard camera.
    pub fn get_images(
        &self,
        camera_id: &str,
        image_types: &[ImageType],
    ) -> Result<Vec<ImageResponse>> {
        let type_ids: Vec<i32> = image_types.iter().map(|t| *t as i32).collect();
        self.client.request(
            &self.sensor_path(camera_id, "GetImages"),
            &GetImagesParams {
                image_type_ids: type_ids,
            },
        )
    }

    /// Computes a 3D ray through a specified pixel in an onboard camera sensor.
    pub fn get_camera_ray(
        &self,
        camera_id: &str,
        image_type: ImageType,
        x: i32,
        y: i32,
    ) -> Result<Pose> {
        self.client.request(
            &self.method_path("GetCameraRay"),
            &GetCameraRayParams {
                camera_id,
                image_type: image_type as i32,
                x,
                y,
            },
        )
    }

    /// Commands a camera gimbal to track and point towards a named scene object.
    pub fn camera_look_at_object(
        &self,
        camera_id: &str,
        object_name: &str,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "LookAtObject"),
            &CameraLookAtObjectParams {
                object_name,
                wait_for_pose_update,
            },
        )
    }

    /// Toggles rendering of the camera view frustum in the 3D viewport.
    pub fn camera_draw_frustum(
        &self,
        camera_id: &str,
        to_enable: bool,
        image_type: ImageType,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "DrawFrustum"),
            &CameraDrawFrustumParams {
                image_type: image_type as i32,
                to_enable,
            },
        )
    }

    /// Overrides the 6-DoF mounting pose of an onboard camera sensor.
    pub fn set_camera_pose(
        &self,
        camera_id: &str,
        pose: &Pose,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "SetPose"),
            &SetCameraPoseParams {
                pose,
                wait_for_pose_update,
            },
        )
    }

    /// Resets an onboard camera sensor to its default mounting pose.
    pub fn reset_camera_pose(
        &self,
        camera_id: &str,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "ResetCameraPose"),
            &ResetCameraPoseParams {
                wait_for_pose_update,
            },
        )
    }

    /// Configures the focal length of an onboard camera sensor lens.
    pub fn set_focal_length(
        &self,
        camera_id: &str,
        image_type_id: i32,
        focal_length: f32,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "SetFocalLength"),
            &CameraOpticsFocalLengthParams {
                image_type_id,
                focal_length,
            },
        )
    }

    /// Configures depth of field transition threshold.
    pub fn set_depth_of_field_transition_threshold(
        &self,
        camera_id: &str,
        image_type_id: i32,
        transition_threshold: f32,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "SetDepthOfFieldTransitionRegion"),
            &CameraOpticsTransitionParams {
                image_type_id,
                transition_threshold,
            },
        )
    }

    /// Configures depth of field focal region.
    pub fn set_depth_of_field_focal_region(
        &self,
        camera_id: &str,
        image_type_id: i32,
        max_focal_distance: f32,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "SetDepthOfFieldFocalRegion"),
            &CameraOpticsFocalLengthParams {
                image_type_id,
                focal_length: max_focal_distance,
            },
        )
    }

    /// Configures chromatic aberration intensity on captured camera frames.
    pub fn set_chromatic_aberration_intensity(
        &self,
        camera_id: &str,
        image_type_id: i32,
        intensity: f32,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "SetChromaticAberrationIntensity"),
            &CameraOpticsIntensityParams {
                image_type_id,
                intensity,
            },
        )
    }

    /// Configures the horizontal field of view (FOV) in degrees.
    pub fn set_field_of_view(
        &self,
        camera_id: &str,
        image_type_id: i32,
        field_of_view: f32,
    ) -> Result<bool> {
        self.client.request(
            &self.sensor_path(camera_id, "SetFieldOfView"),
            &CameraOpticsFOVParams {
                image_type_id,
                field_of_view,
            },
        )
    }

    // --- Fault Injection, Disturbances & Kinematics ---

    /// Toggles a simulated actuator fault (e.g. motor failure) on a named rotor actuator.
    pub fn update_actuator_fault_state(
        &self,
        actuator_id: &str,
        fault_configured: bool,
    ) -> Result<bool> {
        self.client.request(
            &self.actuator_path(actuator_id, "ToggleFault"),
            &UpdateActuatorFaultParams {
                enable: fault_configured,
            },
        )
    }

    /// Injects an external force disturbance vector [fx, fy, fz] onto the aircraft body.
    pub fn set_external_force(&self, ext_force: &[f32]) -> Result<bool> {
        self.client.request(
            &self.method_path("SetExternalForce"),
            &SetExternalForceParams { ext_force },
        )
    }

    /// Overrides low-level actuator control signals (e.g. PWM/thrust commands).
    pub fn set_control_signals(
        &self,
        control_signal_map: &HashMap<String, f32>,
    ) -> Result<bool> {
        self.client.request(
            &self.method_path("SetControlSignals"),
            &SetControlSignalsParams { control_signal_map },
        )
    }

    /// Queries ground-truth 6-DoF pose of the drone.
    pub fn get_ground_truth_pose(&self) -> Result<Pose> {
        self.client
            .request(&self.method_path("GetGroundTruthPose"), &EmptyParams {})
    }

    /// Queries ground-truth geographic location (lat, lon, alt).
    pub fn get_ground_truth_geo_location(&self) -> Result<GeoPosition> {
        self.client.request(
            &self.method_path("GetGroundTruthGeoLocation"),
            &EmptyParams {},
        )
    }

    /// Queries estimated geographic location as computed by vehicle state estimators.
    pub fn get_estimated_geo_location(&self) -> Result<GeoPosition> {
        self.client.request(
            &self.method_path("GetEstimatedGeoLocation"),
            &EmptyParams {},
        )
    }

    /// Queries full ground-truth kinematic state dictionary.
    pub fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client.request(
            &self.method_path("GetGroundTruthKinematics"),
            &EmptyParams {},
        )
    }

    /// Queries estimated kinematic state dictionary.
    pub fn get_estimated_kinematics(&self) -> Result<serde_json::Value> {
        self.client
            .request(&self.method_path("GetEstimatedKinematics"), &EmptyParams {})
    }

    /// Overrides vehicle ground-truth kinematics directly in the simulation engine.
    pub fn set_ground_truth_kinematics(
        &self,
        kinematics: &serde_json::Value,
    ) -> Result<bool> {
        self.client.request(
            &self.method_path("SetGroundTruthKinematics"),
            &SetKinematicsParams { kinematics },
        )
    }

    /// Teleports or relocates the vehicle to a target spatial transform.
    pub fn set_pose(&self, transform: &Transform, reset_kinematics: bool) -> Result<bool> {
        self.client.request(
            &self.method_path("SetPose"),
            &SetPoseParams {
                pose: transform,
                reset_kinematics,
            },
        )
    }

    /// Subscribes to a real-time telemetry topic streamed for this vehicle.
    pub fn subscribe_telemetry<F>(&self, subtopic: &str, callback: F) -> Result<()>
    where
        F: Fn(TopicFrame) + Send + Sync + 'static,
    {
        let topic = format!("{}/{}", self.base_topic(), subtopic);
        self.client.subscribe(topic, callback)
    }
}
