use serde::{Deserialize, Serialize};

/// 3D Vector representing coordinates, velocities, or forces.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

/// Unit quaternion representing 3D spatial rotation.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    /// Creates a quaternion from Euler angles (roll, pitch, yaw) in radians.
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let (cr, sr) = ((roll * 0.5).cos(), (roll * 0.5).sin());
        let (cp, sp) = ((pitch * 0.5).cos(), (pitch * 0.5).sin());
        let (cy, sy) = ((yaw * 0.5).cos(), (yaw * 0.5).sin());

        Self {
            w: cr * cp * cy + sr * sp * sy,
            x: sr * cp * cy - cr * sp * sy,
            y: cr * sp * cy + sr * cp * sy,
            z: cr * cp * sy - sr * sp * cy,
        }
    }
}

impl Default for Quaternion {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// 6-DoF Pose consisting of 3D Position and Orientation Quaternion.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct Pose {
    pub position: Vector3,
    pub orientation: Quaternion,
}

impl Pose {
    pub const ZERO: Self = Self {
        position: Vector3::ZERO,
        orientation: Quaternion::IDENTITY,
    };

    pub fn new(position: Vector3, orientation: Quaternion) -> Self {
        Self {
            position,
            orientation,
        }
    }

    pub fn from_translation(x: f64, y: f64, z: f64) -> Self {
        Self {
            position: Vector3::new(x, y, z),
            orientation: Quaternion::IDENTITY,
        }
    }
}

/// Geographic coordinate position.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct GeoPoint {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}

impl GeoPoint {
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            latitude,
            longitude,
            altitude,
        }
    }
}

/// Geographic position coordinates with altitude in meters and lat/lon in degrees.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize, Default)]
pub struct GeoPosition {
    pub altitude: f64,
    pub latitude: f64,
    pub longitude: f64,
}

impl GeoPosition {
    pub fn new(latitude: f64, longitude: f64, altitude: f64) -> Self {
        Self {
            altitude,
            latitude,
            longitude,
        }
    }
}

/// 3D spatial transform including timestamp, frame ID, translation, and rotation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct Transform {
    #[serde(default)]
    pub timestamp: i64,
    #[serde(default)]
    pub frame_id: String,
    pub translation: Vector3,
    pub rotation: Quaternion,
}

impl Transform {
    pub fn new(translation: Vector3, rotation: Quaternion) -> Self {
        Self {
            timestamp: 0,
            frame_id: String::new(),
            translation,
            rotation,
        }
    }

    pub fn with_frame(
        frame_id: impl Into<String>,
        translation: Vector3,
        rotation: Quaternion,
    ) -> Self {
        Self {
            timestamp: 0,
            frame_id: frame_id.into(),
            translation,
            rotation,
        }
    }
}

/// Vehicle landed state indicator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i32)]
pub enum LandedState {
    Unknown = -1,
    Landed = 0,
    Airborne = 1,
}

/// Vehicle readiness status.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct ReadyState {
    pub is_ready: bool,
    pub message: String,
}

/// Yaw control modes for multirotor flight trajectories.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i32)]
pub enum YawControlMode {
    MaxDegreeOfFreedom = 0,
    ForwardOnly = 1,
}

impl Default for YawControlMode {
    fn default() -> Self {
        Self::MaxDegreeOfFreedom
    }
}

/// VTOL flight modes for convertible/tailsitter aircraft.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[repr(i32)]
pub enum VTOLMode {
    Multirotor = 0,
    FixedWing = 1,
}

impl Default for VTOLMode {
    fn default() -> Self {
        Self::Multirotor
    }
}
