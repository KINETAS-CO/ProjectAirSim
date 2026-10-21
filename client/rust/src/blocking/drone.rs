use std::collections::HashMap;

use crate::blocking::async_result::AsyncResult;
use crate::blocking::client::Client;
use crate::error::Result;
use crate::protocol::frame::TopicFrame;
use crate::types::{
    GeoPosition, ImageResponse, ImageType, LandedState, Pose, ReadyState, Transform, VTOLMode,
    Vector3, YawControlMode,
};

/// Synchronous blocking control handle for a drone/multirotor vehicle.
#[derive(Clone)]
pub struct Drone {
    inner: crate::async_api::Drone,
    client: Client,
}

impl Drone {
    /// Creates a new blocking Drone handle.
    pub fn new(
        client: Client,
        drone_name: impl Into<String>,
        parent_topic: impl Into<String>,
    ) -> Self {
        let inner =
            crate::async_api::Drone::new(client.inner().clone(), drone_name, parent_topic);
        Self { inner, client }
    }

    /// Creates a blocking Drone handle wrapping an existing async handle.
    pub fn from_inner(inner: crate::async_api::Drone, client: Client) -> Self {
        Self { inner, client }
    }

    /// Returns the vehicle's unique name.
    pub fn name(&self) -> &str {
        self.inner.name()
    }

    /// Builds the full published topic path for a sensor (e.g. `/Sim/robots/Drone1/sensors/lidar1/lidar`).
    pub fn get_sensor_topic(&self, sensor_name: &str, topic: &str) -> String {
        self.inner.get_sensor_topic(sensor_name, topic)
    }

    // --- API Control & Readiness ---

    /// Requests API control over the vehicle.
    pub fn enable_api_control(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.enable_api_control())
    }

    /// Releases API control back to RC/manual or onboard autonomy.
    pub fn disable_api_control(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.disable_api_control())
    }

    /// Checks if API control is currently granted.
    pub fn is_api_control_enabled(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.is_api_control_enabled())
    }

    /// Cancels the currently executing movement or flight task.
    pub fn cancel_last_task(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.cancel_last_task())
    }

    // --- Arming & Readiness ---

    /// Arms the vehicle motors/propellers.
    pub fn arm(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.arm())
    }

    /// Disarms the vehicle motors/propellers.
    pub fn disarm(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.disarm())
    }

    /// Checks if the vehicle passes safety pre-arm checks and is ready to arm.
    pub fn can_arm(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.can_arm())
    }

    /// Queries the vehicle's overall readiness status.
    pub fn get_ready_state(&self) -> Result<ReadyState> {
        self.client.runtime().block_on(self.inner.get_ready_state())
    }

    /// Queries the vehicle's current landed state (e.g. Landed, Flying).
    pub fn get_landed_state(&self) -> Result<LandedState> {
        self.client.runtime().block_on(self.inner.get_landed_state())
    }

    // --- Takeoff, Land & Basic Maneuvers ---

    /// Dispatches takeoff asynchronously, returning an `AsyncResult` handle.
    pub fn takeoff_async(&self, timeout_sec: f32) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move { inner.takeoff(timeout_sec).await })
    }

    /// Commands the drone to take off and blocks until complete.
    pub fn takeoff(&self, timeout_sec: f32) -> Result<bool> {
        self.client.runtime().block_on(self.inner.takeoff(timeout_sec))
    }

    /// Dispatches landing asynchronously, returning an `AsyncResult` handle.
    pub fn land_async(&self, timeout_sec: f32) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move { inner.land(timeout_sec).await })
    }

    /// Commands the drone to land and blocks until complete.
    pub fn land(&self, timeout_sec: f32) -> Result<bool> {
        self.client.runtime().block_on(self.inner.land(timeout_sec))
    }

    /// Dispatches hover asynchronously, commanding the drone to hold position.
    pub fn hover_async(&self) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move { inner.hover().await })
    }

    /// Commands the drone to hold position and hover in place, blocking until complete.
    pub fn hover(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.hover())
    }

    /// Dispatches return-to-home flight asynchronously.
    pub fn go_home_async(&self, timeout_sec: f32, velocity: f32) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client
            .spawn_async(async move { inner.go_home(timeout_sec, velocity).await })
    }

    /// Commands return-to-home flight and blocks until complete.
    pub fn go_home(&self, timeout_sec: f32, velocity: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.go_home(timeout_sec, velocity))
    }

    /// Requests flight control from the onboard flight computer.
    pub fn request_control(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.request_control())
    }

    /// Sets the flight controller to mission/waypoint tracking mode.
    pub fn set_mission_mode(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_mission_mode())
    }

    /// Sets the VTOL operational transition mode.
    pub fn set_vtol_mode(&self, vtol_mode: VTOLMode) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_vtol_mode(vtol_mode))
    }

    // --- Velocity Flight Commands ---

    /// Dispatches velocity flight command asynchronously.
    pub fn move_by_velocity_async(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
    ) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client
            .spawn_async(async move { inner.move_by_velocity(vx, vy, vz, duration_sec).await })
    }

    /// Commands velocity in the world frame (North, East, Down in m/s).
    pub fn move_by_velocity(
        &self,
        vx: f64,
        vy: f64,
        vz: f64,
        duration_sec: f64,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.move_by_velocity(vx, vy, vz, duration_sec))
    }

    /// Dispatches velocity flight command with specified yaw control asynchronously.
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
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_by_velocity_with_yaw(vx, vy, vz, duration_sec, drivetrain, yaw_is_rate, yaw)
                .await
        })
    }

    /// Commands velocity in the world frame with full yaw control specification.
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
        self.client.runtime().block_on(self.inner.move_by_velocity_with_yaw(
            vx,
            vy,
            vz,
            duration_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
        ))
    }

    /// Dispatches velocity command holding fixed altitude asynchronously.
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
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_by_velocity_z(vx, vy, z, duration_sec, drivetrain, yaw_is_rate, yaw)
                .await
        })
    }

    /// Commands horizontal world velocities (North, East) while holding fixed target altitude Z (Down in meters).
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
        self.client.runtime().block_on(self.inner.move_by_velocity_z(
            vx,
            vy,
            z,
            duration_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
        ))
    }

    /// Dispatches body-frame velocity flight command asynchronously.
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
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_by_velocity_body_frame(vx, vy, vz, duration_sec, drivetrain, yaw_is_rate, yaw)
                .await
        })
    }

    /// Commands velocity in the vehicle body frame (Forward, Right, Down in m/s).
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
        self.client.runtime().block_on(self.inner.move_by_velocity_body_frame(
            vx,
            vy,
            vz,
            duration_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
        ))
    }

    /// Dispatches body-frame horizontal velocity command holding fixed altitude asynchronously.
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
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_by_velocity_body_frame_z(vx, vy, z, duration_sec, drivetrain, yaw_is_rate, yaw)
                .await
        })
    }

    /// Commands body-frame horizontal velocities while holding fixed target altitude Z.
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
        self.client.runtime().block_on(self.inner.move_by_velocity_body_frame_z(
            vx,
            vy,
            z,
            duration_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
        ))
    }

    /// Dispatches heading-directed flight command asynchronously.
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
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_by_heading(
                    heading,
                    speed,
                    vz,
                    duration_sec,
                    heading_margin,
                    yaw_rate,
                    timeout_sec,
                )
                .await
        })
    }

    /// Commands flight along a heading angle at a specified horizontal speed and vertical rate.
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
        self.client.runtime().block_on(self.inner.move_by_heading(
            heading,
            speed,
            vz,
            duration_sec,
            heading_margin,
            yaw_rate,
            timeout_sec,
        ))
    }

    /// Dispatches path-following flight asynchronously.
    #[allow(clippy::too_many_arguments)]
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
        let inner = self.inner.clone();
        let path_vec = path.to_vec();
        self.client.spawn_async(async move {
            inner
                .move_on_path(
                    &path_vec,
                    velocity,
                    timeout_sec,
                    drivetrain,
                    yaw_is_rate,
                    yaw,
                    lookahead,
                    adaptive_lookahead,
                )
                .await
        })
    }

    /// Traverses a sequential 3D waypoint path using carrot-following guidance.
    #[allow(clippy::too_many_arguments)]
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
        self.client.runtime().block_on(self.inner.move_on_path(
            path,
            velocity,
            timeout_sec,
            drivetrain,
            yaw_is_rate,
            yaw,
            lookahead,
            adaptive_lookahead,
        ))
    }

    /// Dispatches navigation to target NED coordinates asynchronously.
    pub fn move_to_position_async(
        &self,
        north: f64,
        east: f64,
        down: f64,
        velocity: f64,
        timeout_sec: f32,
    ) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_to_position(north, east, down, velocity, timeout_sec)
                .await
        })
    }

    /// Moves the drone to target NED coordinates with specified cruise velocity.
    pub fn move_to_position(
        &self,
        north: f64,
        east: f64,
        down: f64,
        velocity: f64,
        timeout_sec: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.move_to_position(
            north,
            east,
            down,
            velocity,
            timeout_sec,
        ))
    }

    /// Dispatches navigation to target NED coordinates with full yaw and lookahead parameters asynchronously.
    #[allow(clippy::too_many_arguments)]
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
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .move_to_position_with_yaw(
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
                .await
        })
    }

    /// Moves the drone to target NED coordinates with full yaw and lookahead parameters.
    #[allow(clippy::too_many_arguments)]
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
        self.client.runtime().block_on(self.inner.move_to_position_with_yaw(
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
        ))
    }

    /// Dispatches yaw rotation to absolute target heading asynchronously.
    pub fn rotate_to_yaw_async(
        &self,
        yaw_deg: f32,
        timeout_sec: f32,
        margin_deg: f32,
        yaw_rate_deg_per_sec: f32,
    ) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .rotate_to_yaw(yaw_deg, timeout_sec, margin_deg, yaw_rate_deg_per_sec)
                .await
        })
    }

    /// Rotates the aircraft to an absolute target yaw heading in radians (blocking).
    pub fn rotate_to_yaw(
        &self,
        yaw_deg: f32,
        timeout_sec: f32,
        margin_deg: f32,
        yaw_rate_deg_per_sec: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.rotate_to_yaw(
            yaw_deg,
            timeout_sec,
            margin_deg,
            yaw_rate_deg_per_sec,
        ))
    }

    /// Dispatches yaw rotation at constant rate asynchronously.
    pub fn rotate_by_yaw_rate_async(&self, yaw_rate: f32, duration_sec: f32) -> AsyncResult<bool> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner.rotate_by_yaw_rate(yaw_rate, duration_sec).await
        })
    }

    /// Rotates the aircraft at a fixed yaw rate for a specified duration (blocking).
    pub fn rotate_by_yaw_rate(&self, yaw_rate: f32, duration_sec: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.rotate_by_yaw_rate(yaw_rate, duration_sec))
    }

    // --- Sensor Access ---

    /// Retrieves IMU measurements from a named onboard sensor.
    pub fn get_imu_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_imu_data(sensor_name))
    }

    /// Retrieves GPS fix data from a named onboard sensor.
    pub fn get_gps_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_gps_data(sensor_name))
    }

    /// Retrieves pitot/airspeed data from a named onboard sensor.
    pub fn get_airspeed_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_airspeed_data(sensor_name))
    }

    /// Retrieves barometric altitude data from a named onboard sensor.
    pub fn get_barometer_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_barometer_data(sensor_name))
    }

    /// Retrieves magnetometer heading data from a named onboard sensor.
    pub fn get_magnetometer_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_magnetometer_data(sensor_name))
    }

    /// Retrieves LiDAR point cloud data from a named onboard sensor.
    pub fn get_lidar_data(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_lidar_data(sensor_name))
    }

    /// Retrieves raw radar detection returns from a named onboard sensor.
    pub fn get_radar_detections(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_radar_detections(sensor_name))
    }

    /// Retrieves tracked radar targets from a named onboard sensor.
    pub fn get_radar_tracks(&self, sensor_name: &str) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_radar_tracks(sensor_name))
    }

    // --- Battery Management ---

    /// Queries battery charge and health state.
    pub fn get_battery_state(&self) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_battery_state())
    }

    /// Overrides current remaining battery capacity (0.0 to 1.0).
    pub fn set_battery_remaining(&self, desired_battery_remaining: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_battery_remaining(desired_battery_remaining))
    }

    /// Queries the current battery drain rate per second.
    pub fn get_battery_drain_rate(&self) -> Result<f32> {
        self.client
            .runtime()
            .block_on(self.inner.get_battery_drain_rate())
    }

    /// Overrides the battery drain multiplier rate per second.
    pub fn set_battery_drain_rate(&self, desired_drain_rate: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_battery_drain_rate(desired_drain_rate))
    }

    /// Flags the battery health status as nominal or degraded.
    pub fn set_battery_health_status(&self, is_desired_state_healthy: bool) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_battery_health_status(is_desired_state_healthy))
    }

    // --- Camera Optics & Rendering Controls ---

    /// Captures images from a named onboard camera.
    pub fn get_images(
        &self,
        camera_id: &str,
        image_types: &[ImageType],
    ) -> Result<Vec<ImageResponse>> {
        self.client
            .runtime()
            .block_on(self.inner.get_images(camera_id, image_types))
    }

    /// Computes a 3D ray through a specified pixel in an onboard camera sensor.
    pub fn get_camera_ray(
        &self,
        camera_id: &str,
        image_type: ImageType,
        x: i32,
        y: i32,
    ) -> Result<Pose> {
        self.client
            .runtime()
            .block_on(self.inner.get_camera_ray(camera_id, image_type, x, y))
    }

    /// Orients an onboard camera toward a named scene object.
    pub fn camera_look_at_object(
        &self,
        camera_id: &str,
        object_name: &str,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .camera_look_at_object(camera_id, object_name, wait_for_pose_update),
        )
    }

    /// Toggles frustum wireframe visualization for an onboard camera.
    pub fn camera_draw_frustum(
        &self,
        camera_id: &str,
        to_enable: bool,
        image_type: ImageType,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .camera_draw_frustum(camera_id, to_enable, image_type),
        )
    }

    /// Sets the relative pose of an onboard camera within the vehicle frame.
    pub fn set_camera_pose(
        &self,
        camera_id: &str,
        pose: &Pose,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_camera_pose(camera_id, pose, wait_for_pose_update),
        )
    }

    /// Resets an onboard camera's pose to its initial mounted configuration.
    pub fn reset_camera_pose(
        &self,
        camera_id: &str,
        wait_for_pose_update: bool,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.reset_camera_pose(camera_id, wait_for_pose_update))
    }

    /// Sets camera optical focal length in millimeters.
    pub fn set_focal_length(
        &self,
        camera_id: &str,
        image_type_id: i32,
        focal_length: f32,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_focal_length(camera_id, image_type_id, focal_length))
    }

    /// Sets camera depth-of-field transition blur threshold.
    pub fn set_depth_of_field_transition_threshold(
        &self,
        camera_id: &str,
        image_type_id: i32,
        transition_threshold: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_depth_of_field_transition_threshold(camera_id, image_type_id, transition_threshold),
        )
    }

    /// Sets camera depth-of-field sharp focal region depth in world units.
    pub fn set_depth_of_field_focal_region(
        &self,
        camera_id: &str,
        image_type_id: i32,
        max_focal_distance: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_depth_of_field_focal_region(camera_id, image_type_id, max_focal_distance),
        )
    }

    /// Sets post-process chromatic aberration effect intensity.
    pub fn set_chromatic_aberration_intensity(
        &self,
        camera_id: &str,
        image_type_id: i32,
        intensity: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_chromatic_aberration_intensity(camera_id, image_type_id, intensity),
        )
    }

    /// Sets camera horizontal field of view in degrees.
    pub fn set_field_of_view(
        &self,
        camera_id: &str,
        image_type_id: i32,
        field_of_view: f32,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_field_of_view(camera_id, image_type_id, field_of_view))
    }

    // --- Actuator & Hardware Simulation ---

    /// Injects or clears a simulated actuator fault.
    pub fn update_actuator_fault_state(
        &self,
        actuator_id: &str,
        fault_configured: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .update_actuator_fault_state(actuator_id, fault_configured),
        )
    }

    /// Injects an external force vector [Fx, Fy, Fz] in Newtons into the physics body.
    pub fn set_external_force(&self, ext_force: &[f32]) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_external_force(ext_force))
    }

    /// Sends raw PWM / control signal values directly to motors.
    pub fn set_control_signals(
        &self,
        control_signals: &HashMap<String, f32>,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_control_signals(control_signals))
    }

    // --- Kinematics & World State ---

    /// Retrieves ground-truth world pose directly from Unreal physics.
    pub fn get_ground_truth_pose(&self) -> Result<Pose> {
        self.client
            .runtime()
            .block_on(self.inner.get_ground_truth_pose())
    }

    /// Retrieves ground truth geographic coordinates (WGS84 lat, lon, alt).
    pub fn get_ground_truth_geo_location(&self) -> Result<GeoPosition> {
        self.client
            .runtime()
            .block_on(self.inner.get_ground_truth_geo_location())
    }

    /// Retrieves estimator-fused geographic coordinates (WGS84 lat, lon, alt).
    pub fn get_estimated_geo_location(&self) -> Result<GeoPosition> {
        self.client
            .runtime()
            .block_on(self.inner.get_estimated_geo_location())
    }

    /// Retrieves full ground-truth kinematic state from physics.
    pub fn get_ground_truth_kinematics(&self) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_ground_truth_kinematics())
    }

    /// Retrieves state-estimator kinematic state.
    pub fn get_estimated_kinematics(&self) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_estimated_kinematics())
    }

    /// Teleports/overrides vehicle physics kinematics directly.
    pub fn set_ground_truth_kinematics(
        &self,
        kinematics: &serde_json::Value,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_ground_truth_kinematics(kinematics))
    }

    /// Teleports vehicle to a new world pose.
    pub fn set_pose(&self, transform: &Transform, reset_kinematics: bool) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_pose(transform, reset_kinematics))
    }

    // --- Telemetry Streaming ---

    /// Subscribes to a vehicle telemetry subtopic with a background callback.
    pub fn subscribe_telemetry<F>(&self, subtopic: &str, callback: F) -> Result<()>
    where
        F: Fn(TopicFrame) + Send + Sync + 'static,
    {
        let mut sub = self
            .client
            .runtime()
            .block_on(self.inner.subscribe_telemetry(subtopic))?;
        let cb = std::sync::Arc::new(callback);
        self.client.runtime().spawn(async move {
            while let Ok(frame) = sub.recv_frame().await {
                cb(frame);
            }
        });
        Ok(())
    }
}
