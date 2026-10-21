use std::io::{self, Write};
use std::path::Path;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// RGBA Color representation with normalized values [0.0, 1.0].
///
/// Serializes as a 4-element array `[red, green, blue, alpha]` matching the
/// ProjectAirSim wire RPC specification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ColorRGBA {
    pub red: f32,
    pub green: f32,
    pub blue: f32,
    pub alpha: f32,
}

impl ColorRGBA {
    pub const WHITE: Self = Self::new(1.0, 1.0, 1.0, 1.0);
    pub const BLACK: Self = Self::new(0.0, 0.0, 0.0, 1.0);
    pub const RED: Self = Self::new(1.0, 0.0, 0.0, 1.0);
    pub const GREEN: Self = Self::new(0.0, 1.0, 0.0, 1.0);
    pub const BLUE: Self = Self::new(0.0, 0.0, 1.0, 1.0);
    pub const YELLOW: Self = Self::new(1.0, 1.0, 0.0, 1.0);
    pub const CYAN: Self = Self::new(0.0, 1.0, 1.0, 1.0);
    pub const MAGENTA: Self = Self::new(1.0, 0.0, 1.0, 1.0);

    pub const fn new(red: f32, green: f32, blue: f32, alpha: f32) -> Self {
        Self {
            red,
            green,
            blue,
            alpha,
        }
    }

    pub const fn rgb(red: f32, green: f32, blue: f32) -> Self {
        Self {
            red,
            green,
            blue,
            alpha: 1.0,
        }
    }
}

impl Default for ColorRGBA {
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0)
    }
}

impl Serialize for ColorRGBA {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        [self.red, self.green, self.blue, self.alpha].serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for ColorRGBA {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        #[serde(untagged)]
        enum ColorHelper {
            Array([f32; 4]),
            Array3([f32; 3]),
            Object {
                red: f32,
                green: f32,
                blue: f32,
                #[serde(default = "default_alpha")]
                alpha: f32,
            },
            ShortObject {
                r: f32,
                g: f32,
                b: f32,
                #[serde(default = "default_alpha")]
                a: f32,
            },
        }

        fn default_alpha() -> f32 {
            1.0
        }

        match ColorHelper::deserialize(deserializer)? {
            ColorHelper::Array([r, g, b, a]) => Ok(ColorRGBA::new(r, g, b, a)),
            ColorHelper::Array3([r, g, b]) => Ok(ColorRGBA::rgb(r, g, b)),
            ColorHelper::Object {
                red,
                green,
                blue,
                alpha,
            } => Ok(ColorRGBA::new(red, green, blue, alpha)),
            ColorHelper::ShortObject { r, g, b, a } => Ok(ColorRGBA::new(r, g, b, a)),
        }
    }
}

/// Weather visual effect parameter type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum WeatherParameter {
    Enabled = 0,
    Rain = 1,
    RoadWetness = 2,
    Snow = 3,
    RoadSnow = 4,
    MapleLeaf = 5,
    RoadLeaf = 6,
    Dust = 7,
    Fog = 8,
}

impl Serialize for WeatherParameter {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        (*self as i32).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for WeatherParameter {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let val = i32::deserialize(deserializer)?;
        match val {
            0 => Ok(Self::Enabled),
            1 => Ok(Self::Rain),
            2 => Ok(Self::RoadWetness),
            3 => Ok(Self::Snow),
            4 => Ok(Self::RoadSnow),
            5 => Ok(Self::MapleLeaf),
            6 => Ok(Self::RoadLeaf),
            7 => Ok(Self::Dust),
            8 => Ok(Self::Fog),
            other => Err(serde::de::Error::custom(format!(
                "Unknown WeatherParameter value: {other}"
            ))),
        }
    }
}

/// Bounding box alignment relative to world or local object coordinates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BoxAlignment {
    WorldAxis = 0,
    ObjectOriented = 1,
}

impl Serialize for BoxAlignment {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        (*self as i32).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for BoxAlignment {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let val = i32::deserialize(deserializer)?;
        match val {
            0 => Ok(Self::WorldAxis),
            1 => Ok(Self::ObjectOriented),
            other => Err(serde::de::Error::custom(format!(
                "Unknown BoxAlignment value: {other}"
            ))),
        }
    }
}

/// Time of day configuration for simulation sunlight and celestial motion.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TimeOfDay {
    #[serde(rename = "status")]
    pub enabled: bool,
    pub datetime: String,
    pub is_dst: bool,
    pub clock_speed: f32,
    pub update_interval: f32,
    pub move_sun: bool,
}

impl Default for TimeOfDay {
    fn default() -> Self {
        Self {
            enabled: false,
            datetime: String::new(),
            is_dst: false,
            clock_speed: 1.0,
            update_interval: 60.0,
            move_sun: true,
        }
    }
}

/// NED-frame 6-DoF trajectory for autonomous robot or scene actor playback.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct NEDTrajectory {
    pub traj_name: String,
    pub time: Vec<f32>,
    pub pose_x: Vec<f32>,
    pub pose_y: Vec<f32>,
    pub pose_z: Vec<f32>,
    #[serde(default)]
    pub pose_roll: Vec<f32>,
    #[serde(default)]
    pub pose_pitch: Vec<f32>,
    #[serde(default)]
    pub pose_yaw: Vec<f32>,
    #[serde(default, rename = "vel_x_lin")]
    pub vel_lin_x: Vec<f32>,
    #[serde(default, rename = "vel_y_lin")]
    pub vel_lin_y: Vec<f32>,
    #[serde(default, rename = "vel_z_lin")]
    pub vel_lin_z: Vec<f32>,
}

impl NEDTrajectory {
    /// Fills in missing orientation (pitch, yaw) and velocity fields matching C++ client behavior.
    pub fn auto_fill_missing(&mut self) {
        let point_count = self.time.len();
        if point_count == 0 {
            return;
        }
        if self.pose_roll.is_empty() {
            self.pose_roll = vec![0.0; point_count];
        }
        if self.pose_pitch.is_empty() {
            self.pose_pitch.reserve(point_count);
            for idx in 1..point_count {
                let dx = self.pose_x[idx - 1] - self.pose_x[idx];
                let dy = self.pose_y[idx - 1] - self.pose_y[idx];
                let dz = self.pose_z[idx - 1] - self.pose_z[idx];
                self.pose_pitch.push((-dz).atan2(dx.hypot(dy)));
            }
            let last = self.pose_pitch.last().copied().unwrap_or(0.0);
            self.pose_pitch.push(last);
        }
        if self.pose_yaw.is_empty() {
            self.pose_yaw.reserve(point_count);
            for idx in 1..point_count {
                let dx = self.pose_x[idx - 1] - self.pose_x[idx];
                let dy = self.pose_y[idx - 1] - self.pose_y[idx];
                self.pose_yaw.push(dy.atan2(dx));
            }
            let last = self.pose_yaw.last().copied().unwrap_or(0.0);
            self.pose_yaw.push(last);
        }
        if self.vel_lin_x.is_empty() {
            self.vel_lin_x = vec![0.0; point_count];
        }
        if self.vel_lin_y.is_empty() {
            self.vel_lin_y = vec![0.0; point_count];
        }
        if self.vel_lin_z.is_empty() {
            self.vel_lin_z = vec![0.0; point_count];
        }
    }
}

/// Geographic coordinate trajectory for long-range robot or actor playback.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct GeoTrajectory {
    pub traj_name: String,
    pub time: Vec<f32>,
    pub latitudes: Vec<f32>,
    pub longitudes: Vec<f32>,
    pub altitudes: Vec<f32>,
    #[serde(default)]
    pub roll: Vec<f32>,
    #[serde(default)]
    pub pitch: Vec<f32>,
    #[serde(default)]
    pub yaw: Vec<f32>,
    #[serde(default)]
    pub vel_lin_x: Vec<f32>,
    #[serde(default)]
    pub vel_lin_y: Vec<f32>,
    #[serde(default)]
    pub vel_lin_z: Vec<f32>,
}

/// Writes a 3D boolean voxel occupancy map into `.binvox` binary format.
pub fn write_binvox(
    voxels: &[bool],
    x_size: i32,
    y_size: i32,
    z_size: i32,
    resolution: f32,
    file_path: impl AsRef<Path>,
) -> io::Result<()> {
    let mut file = std::fs::File::create(file_path)?;
    let dim_x = (x_size as f32 / resolution) as i32;
    let dim_y = (y_size as f32 / resolution) as i32;
    let dim_z = (z_size as f32 / resolution) as i32;
    let tx = -x_size as f32 * 0.5;
    let ty = -y_size as f32 * 0.5;
    let tz = -z_size as f32 * 0.5;
    let scale = 1.0 / x_size as f32;

    writeln!(file, "#binvox 1")?;
    writeln!(file, "dim {} {} {}", dim_x, dim_z, dim_y)?;
    writeln!(file, "translate {} {} {}", tx, ty, tz)?;
    writeln!(file, "scale {}", scale)?;
    writeln!(file, "data")?;

    if voxels.is_empty() {
        return Ok(());
    }

    let mut state = voxels[0];
    let mut ctr: u8 = 0;

    for &c in voxels {
        if c == state {
            ctr += 1;
            if ctr == 255 {
                file.write_all(&[state as u8, 255])?;
                ctr = 0;
            }
        } else {
            if ctr > 0 {
                file.write_all(&[state as u8, ctr])?;
            }
            state = c;
            ctr = 1;
        }
    }

    if ctr > 0 {
        file.write_all(&[state as u8, ctr])?;
    }

    file.flush()?;
    Ok(())
}
