//! Shared RPC request parameter structures for ProjectAirSim actors.
//!
//! Internal parameter types used across both `async_api` and `blocking` modules
//! to serialize request payloads for the ProjectAirSim simulation server.

use serde::Serialize;
use std::collections::HashMap;

use crate::types::{Pose, Transform, Vector3};

/// Empty parameter payload for RPC methods taking no arguments.
#[derive(Serialize, Default, Debug, Clone, Copy)]
pub struct EmptyParams {}

// --- Drone Parameters ---

#[derive(Serialize, Debug, Clone)]
pub struct TimeoutParams {
    pub timeout_sec: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct GoHomeParams {
    pub timeout_sec: f32,
    pub velocity: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetVTOLModeParams {
    pub vtol_mode: i32,
}

#[derive(Serialize, Debug, Clone)]
pub struct MoveVelocityParams {
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub duration: f64,
    pub drivetrain: i32,
    pub yaw_is_rate: bool,
    pub yaw: f64,
}

#[derive(Serialize, Debug, Clone)]
pub struct MoveVelocityZParams {
    pub vx: f64,
    pub vy: f64,
    pub z: f64,
    pub duration: f64,
    pub drivetrain: i32,
    pub yaw_is_rate: bool,
    pub yaw: f64,
}

#[derive(Serialize, Debug, Clone)]
pub struct MoveByHeadingParams {
    pub heading: f32,
    pub speed: f32,
    pub vz: f32,
    pub duration: f32,
    pub heading_margin: f32,
    pub yaw_rate: f32,
    pub timeout_sec: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct MoveOnPathParams<'a> {
    pub path: &'a [Vector3],
    pub velocity: f32,
    pub timeout_sec: f32,
    pub drivetrain: i32,
    pub yaw_is_rate: bool,
    pub yaw: f32,
    pub lookahead: f32,
    pub adaptive_lookahead: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct MoveToPositionParams {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub velocity: f64,
    pub timeout_sec: f32,
    pub drivetrain: i32,
    pub yaw_is_rate: bool,
    pub yaw: f64,
    pub lookahead: f64,
    pub adaptive_lookahead: f64,
}

#[derive(Serialize, Debug, Clone)]
pub struct RotateToYawParams {
    pub yaw: f32,
    pub timeout_sec: f32,
    pub margin: f32,
    pub yaw_rate: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct RotateByYawRateParams {
    pub yaw_rate: f32,
    pub duration: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetBatteryRemainingParams {
    pub desired_battery_remaining: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetBatteryDrainRateParams {
    pub desired_drain_rate: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetBatteryHealthStatusParams {
    pub battery_health_indicator: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct DroneGetImagesParams {
    pub image_type_ids: Vec<i32>,
}

#[derive(Serialize, Debug, Clone)]
pub struct GetCameraRayParams<'a> {
    pub camera_id: &'a str,
    pub image_type: i32,
    pub x: i32,
    pub y: i32,
}

#[derive(Serialize, Debug, Clone)]
pub struct CameraLookAtObjectParams<'a> {
    pub object_name: &'a str,
    pub wait_for_pose_update: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct CameraDrawFrustumParams {
    pub image_type: i32,
    pub to_enable: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetCameraPoseParams<'a> {
    pub pose: &'a Pose,
    pub wait_for_pose_update: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct ResetCameraPoseParams {
    pub wait_for_pose_update: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct CameraOpticsFocalLengthParams {
    pub image_type_id: i32,
    pub focal_length: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct CameraOpticsTransitionParams {
    pub image_type_id: i32,
    pub transition_threshold: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct CameraOpticsIntensityParams {
    pub image_type_id: i32,
    pub intensity: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct CameraOpticsFOVParams {
    pub image_type_id: i32,
    pub field_of_view: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct UpdateActuatorFaultParams {
    pub enable: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetExternalForceParams<'a> {
    pub ext_force: &'a [f32],
}

#[derive(Serialize, Debug, Clone)]
pub struct SetControlSignalsParams<'a> {
    pub control_signal_map: &'a HashMap<String, f32>,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetKinematicsParams<'a> {
    pub kinematics: &'a serde_json::Value,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetPoseParams<'a> {
    pub pose: &'a Transform,
    pub reset_kinematics: bool,
}

// --- Wheeled Vehicle Parameters ---

#[derive(Serialize, Debug, Clone)]
pub struct SingleValueParams {
    pub value: f32,
}

// --- Rover Parameters ---

#[derive(Serialize, Debug, Clone)]
pub struct RoverSetPoseParams {
    pub pose: Pose,
    pub reset_kinematics: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct RoverMoveToPositionParams {
    pub x: f32,
    pub y: f32,
    pub velocity: f32,
    pub timeout_sec: f32,
    pub yaw_rate_max: f32,
    pub lookahead: f32,
    pub adaptive_lookahead: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetRoverControlsParams {
    pub engine: f32,
    pub steering_angle: f32,
    pub brake: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct RoverMoveByHeadingParams {
    pub heading: f32,
    pub speed: f32,
    pub duration: f32,
    pub heading_margin: f32,
    pub yaw_rate: f32,
    pub timeout_sec: f32,
}

// --- EnvActor Parameters ---

#[derive(Serialize, Debug, Clone)]
pub struct SetTrajectoryParams<'a> {
    pub env_actor_name: &'a str,
    pub traj_name: &'a str,
    pub time_offset: f32,
    pub x_offset: f32,
    pub y_offset: f32,
    pub z_offset: f32,
    pub roll_offset: f32,
    pub pitch_offset: f32,
    pub yaw_offset: f32,
    pub to_loop: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetLinkRotationAngleParams<'a> {
    pub env_actor_name: &'a str,
    pub link_name: &'a str,
    pub angle_deg: f32,
}

#[derive(Serialize, Debug, Clone)]
pub struct SetLinkRotationRateParams<'a> {
    pub env_actor_name: &'a str,
    pub link_name: &'a str,
    pub rotation_deg_per_sec: f32,
}

// --- StaticSensorActor Parameters ---

#[derive(Serialize, Debug, Clone)]
pub struct StaticGetImagesParams<'a> {
    pub image_type_ids: &'a [i32],
}

// --- Client Parameters ---

#[derive(Serialize, Debug, Clone)]
pub struct FeatureParams<'a> {
    pub feature_id: &'a str,
    pub enable: bool,
}

#[derive(Serialize, Debug, Clone)]
pub struct LoadSceneParams<'a> {
    pub scene_config: &'a str,
}

