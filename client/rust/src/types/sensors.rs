use serde::{Deserialize, Serialize};

use crate::types::kinematics::{Quaternion, Vector3};

/// IMU kinematics sensor measurement.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ImuData {
    #[serde(default)]
    pub angular_velocity: Vector3,
    #[serde(default)]
    pub linear_acceleration: Vector3,
    #[serde(default)]
    pub orientation: Option<Quaternion>,
    #[serde(default)]
    pub time_stamp: u64,
}

/// Barometer atmospheric pressure and altitude sensor reading.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct BarometerData {
    #[serde(default)]
    pub altitude: f64,
    #[serde(default)]
    pub pressure: f64,
    #[serde(default)]
    pub qnh: f64,
    #[serde(default)]
    pub time_stamp: u64,
}

/// GNSS / GPS position and velocity measurement.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct GpsData {
    #[serde(default)]
    pub latitude: f64,
    #[serde(default)]
    pub longitude: f64,
    #[serde(default)]
    pub altitude: f64,
    #[serde(default)]
    pub velocity: Vector3,
    #[serde(default)]
    pub time_stamp: u64,
}

/// Airspeed sensor reading.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct AirspeedData {
    #[serde(default)]
    pub airspeed: f64,
    #[serde(default)]
    pub time_stamp: u64,
}

/// Magnetometer magnetic field vector measurement.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct MagnetometerData {
    #[serde(default)]
    pub magnetic_field_body: Vector3,
    #[serde(default)]
    pub time_stamp: u64,
}

/// Battery telemetry and health indicators.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct BatteryState {
    #[serde(default)]
    pub battery_remaining: f32,
    #[serde(default)]
    pub battery_drain_rate: f32,
    #[serde(default)]
    pub is_healthy: bool,
    #[serde(default)]
    pub voltage: f32,
    #[serde(default)]
    pub current: f32,
}

/// Single target radar detection point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct RadarDetection {
    #[serde(default)]
    pub range: f32,
    #[serde(default)]
    pub azimuth: f32,
    #[serde(default)]
    pub elevation: f32,
    #[serde(default)]
    pub radial_velocity: f32,
}

/// Clustered radar track object.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct RadarTrack {
    #[serde(default)]
    pub track_id: u32,
    #[serde(default)]
    pub position: Vector3,
    #[serde(default)]
    pub velocity: Vector3,
}
