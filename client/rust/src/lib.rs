//! Native Rust client for the ProjectAirSim simulation platform.
//!
//! Provides both async (Tokio-native) and sync (blocking) interfaces
//! to connect to ProjectAirSim's simulation server over NNG.
#![allow(clippy::too_many_arguments, clippy::type_complexity)]

#[cfg(all(feature = "async", feature = "sync"))]
compile_error!(
    "Features 'async' and 'sync' are mutually exclusive. Please enable either 'async' or 'sync', not both."
);

#[cfg(not(any(feature = "async", feature = "sync")))]
compile_error!(
    "Either feature 'async' or 'sync' must be enabled. Default is 'async'."
);

pub mod config;
pub mod error;
pub mod logging;
pub mod protocol;
pub mod transport;
pub mod types;

pub use logging::{clear_log_sink, has_log_sink, init_logging, set_log_sink, LogSink, Severity};

#[cfg(any(feature = "async", feature = "sync"))]
pub mod async_api;

#[cfg(feature = "sync")]
pub mod blocking;

pub use config::{load_jsonc_file, load_scene_config, parse_jsonc, strip_jsonc_comments};
pub use error::{Result, SimError, Status};

pub use protocol::{
    FrameType, RawResponseEnvelope, RequestEnvelope, ResponseDecoder, TopicFrame, TopicInfo,
};
pub use transport::{MockTransport, NngTransport, Transport};
pub use types::*;

#[cfg(feature = "async")]
pub use async_api::{
    Client, Drone, EnvActor, Rover, StaticSensorActor, TopicSubscription, WheeledVehicle, World,
};

#[cfg(feature = "sync")]
pub use blocking::AsyncResult;

#[cfg(feature = "sync")]
pub use blocking::{Client, Drone, EnvActor, Rover, StaticSensorActor, WheeledVehicle, World};
