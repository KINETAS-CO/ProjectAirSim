use serde::{Deserialize, Serialize};

/// Supported camera image modalities.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[repr(i32)]
pub enum ImageType {
    #[default]
    Scene = 0,
    DepthPlanar = 1,
    DepthPerspective = 2,
    Segmentation = 3,
    DepthVis = 4,
    DisparityNormalized = 5,
    SurfaceNormals = 6,
    Infrared = 7,
    OpticalFlow = 8,
    OpticalFlowVis = 9,
}

/// Request for a single camera image capture.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImageRequest {
    pub camera_name: String,
    pub image_type: ImageType,
    pub pixels_as_float: bool,
    pub compress: bool,
}

impl ImageRequest {
    pub fn new(camera_name: impl Into<String>, image_type: ImageType) -> Self {
        Self {
            camera_name: camera_name.into(),
            image_type,
            pixels_as_float: false,
            compress: true,
        }
    }
}

/// Response containing captured camera image data.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct ImageResponse {
    #[serde(default, with = "serde_bytes")]
    pub image_data_uint8: Vec<u8>,
    #[serde(default)]
    pub image_data_float: Vec<f32>,
    #[serde(default)]
    pub camera_name: String,
    #[serde(default)]
    pub time_stamp: u64,
    #[serde(default)]
    pub message: String,
    #[serde(default)]
    pub width: u32,
    #[serde(default)]
    pub height: u32,
    #[serde(default)]
    pub image_type: ImageType,
}
