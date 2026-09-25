use serde::Serialize;
use tracing::info;

use crate::blocking::client::Client;
use crate::blocking::drone::Drone;
use crate::error::{Result, SimError};
use crate::types::{Pose, Vector3};

/// Synchronous blocking interface for the ProjectAirSim simulation world.
#[derive(Clone)]
pub struct World {
    client: Client,
    parent_topic: String,
    drones: Vec<String>,
    config: Option<serde_json::Value>,
}

#[derive(Serialize)]
struct LoadSceneParams<'a> {
    scene_config: &'a str,
}

#[derive(Serialize)]
struct ObjectNameParams<'a> {
    object_name: &'a str,
}

#[derive(Serialize)]
struct SetObjectPoseParams<'a> {
    object_name: &'a str,
    pose: Pose,
    teleport: bool,
}

#[derive(Serialize)]
struct SetWindParams {
    v_x: f64,
    v_y: f64,
    v_z: f64,
}

#[derive(Serialize)]
struct ContinueSimTimeParams {
    delta_time_nanos: i64,
    wait_until_complete: bool,
}

#[derive(Serialize)]
struct EmptyParams {}

impl World {
    /// Initializes a synchronous simulation world.
    pub fn new(client: Client, scene_config: Option<&str>) -> Result<Self> {
        let mut loaded_config = None;
        let mut drones = Vec::new();
        let mut parent_topic = "/Sim".to_string();

        if let Some(config_str) = scene_config {
            let content = if std::path::Path::new(config_str).exists() {
                std::fs::read_to_string(config_str).map_err(|e| {
                    SimError::SerializationError(format!(
                        "Failed to read scene config file {config_str}: {e}"
                    ))
                })?
            } else {
                config_str.to_string()
            };

            info!("Loading scene into ProjectAirSim server...");
            let _res: serde_json::Value = client.request(
                "/Sim/LoadScene",
                &LoadSceneParams {
                    scene_config: &content,
                },
            )?;
            info!("Scene loaded successfully");

            if let Ok(parsed) = serde_json::from_str::<serde_json::Value>(&content) {
                if let Some(d_list) = parsed.get("drones").and_then(|d| d.as_array()) {
                    for d in d_list {
                        if let Some(name) = d.get("name").and_then(|n| n.as_str()) {
                            drones.push(name.to_string());
                        }
                    }
                }
                if let Some(topic) = parsed.get("parent_topic").and_then(|t| t.as_str()) {
                    parent_topic = topic.to_string();
                }
                loaded_config = Some(parsed);
            }
        }

        Ok(Self {
            client,
            parent_topic,
            drones,
            config: loaded_config,
        })
    }

    /// Returns a reference to the blocking Client.
    pub fn client(&self) -> &Client {
        &self.client
    }

    /// Returns the active parent topic prefix for this world (e.g. `/Sim`).
    pub fn parent_topic(&self) -> &str {
        &self.parent_topic
    }

    /// Returns discovered drone actor names in this scene.
    pub fn drones(&self) -> &[String] {
        &self.drones
    }

    /// Returns the parsed scene configuration JSON, if loaded.
    pub fn configuration(&self) -> Option<&serde_json::Value> {
        self.config.as_ref()
    }

    /// Returns a Drone handle attached to this simulation world.
    pub fn get_drone(&self, name: impl Into<String>) -> Drone {
        Drone::new(self.client.clone(), name, self.parent_topic.clone())
    }

    // --- Simulation Clock & Stepping ---

    /// Pauses simulation execution.
    pub fn pause(&self) -> Result<()> {
        let _: serde_json::Value = self.client.request("/Sim/Pause", &EmptyParams {})?;
        Ok(())
    }

    /// Resumes simulation execution.
    pub fn resume(&self) -> Result<()> {
        let _: serde_json::Value = self.client.request("/Sim/Resume", &EmptyParams {})?;
        Ok(())
    }

    /// Queries whether the simulation clock is paused.
    pub fn is_paused(&self) -> Result<bool> {
        self.client.request("/Sim/IsPaused", &EmptyParams {})
    }

    /// Steps the simulation clock forward by a specified delta time in seconds.
    pub fn step(&self, delta_time_sec: f64) -> Result<i64> {
        let nanos = (delta_time_sec * 1e9) as i64;
        self.client.request(
            "/Sim/ContinueForSimTime",
            &ContinueSimTimeParams {
                delta_time_nanos: nanos,
                wait_until_complete: true,
            },
        )
    }

    /// Retrieves current simulation time in nanoseconds.
    pub fn get_sim_time(&self) -> Result<i64> {
        self.client.request("/Sim/GetSimTime", &EmptyParams {})
    }

    // --- Weather & Environment ---

    /// Sets global simulation wind velocity in m/s.
    pub fn set_wind(&self, wind: Vector3) -> Result<bool> {
        self.client.request(
            "/Sim/SetWindVelocity",
            &SetWindParams {
                v_x: wind.x,
                v_y: wind.y,
                v_z: wind.z,
            },
        )
    }

    /// Retrieves current wind velocity in m/s.
    pub fn get_wind(&self) -> Result<Vector3> {
        self.client.request("/Sim/GetWindVelocity", &EmptyParams {})
    }

    // --- Objects & Actors ---

    /// Lists all actor entities in the current scene.
    pub fn list_actors(&self) -> Result<Vec<String>> {
        self.client.request("/Sim/ListActors", &EmptyParams {})
    }

    /// Retrieves the 6-DoF pose of a named scene object.
    pub fn get_object_pose(&self, object_name: &str) -> Result<Pose> {
        self.client
            .request("/Sim/GetObjectPose", &ObjectNameParams { object_name })
    }

    /// Sets the pose of a named scene object, optionally teleporting without physics collision.
    pub fn set_object_pose(&self, object_name: &str, pose: Pose, teleport: bool) -> Result<bool> {
        self.client.request(
            "/Sim/SetObjectPose",
            &SetObjectPoseParams {
                object_name,
                pose,
                teleport,
            },
        )
    }

    /// Returns a Rover handle attached to this simulation world.
    pub fn get_rover(&self, name: impl Into<String>) -> crate::blocking::rover::Rover {
        crate::blocking::rover::Rover::new(self.client.clone(), name, self.parent_topic.clone())
    }

    /// Returns a WheeledVehicle handle attached to this simulation world.
    pub fn get_wheeled_vehicle(
        &self,
        name: impl Into<String>,
    ) -> crate::blocking::wheeled_vehicle::WheeledVehicle {
        crate::blocking::wheeled_vehicle::WheeledVehicle::new(
            self.client.clone(),
            name,
            self.parent_topic.clone(),
        )
    }

    /// Returns an EnvActor handle attached to this simulation world.
    pub fn get_env_actor(&self, name: impl Into<String>) -> crate::blocking::env_actor::EnvActor {
        crate::blocking::env_actor::EnvActor::new(
            self.client.clone(),
            name,
            self.parent_topic.clone(),
        )
    }

    /// Returns a StaticSensorActor handle attached to this simulation world.
    pub fn get_static_sensor(
        &self,
        name: impl Into<String>,
    ) -> crate::blocking::static_sensor::StaticSensorActor {
        crate::blocking::static_sensor::StaticSensorActor::new(
            self.client.clone(),
            name,
            self.parent_topic.clone(),
        )
    }
}
