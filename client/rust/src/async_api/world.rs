use std::path::Path;
use tracing::info;

use crate::async_api::client::Client;
use crate::async_api::drone::Drone;
use crate::async_api::env_actor::EnvActor;
use crate::async_api::rover::Rover;
use crate::async_api::static_sensor::StaticSensorActor;
use crate::async_api::wheeled_vehicle::WheeledVehicle;
use crate::error::{Result, SimError};
use crate::protocol::params::EmptyParams;
use crate::types::{
    write_binvox, BoxAlignment, ColorRGBA, GeoTrajectory, NEDTrajectory, Pose, TimeOfDay,
    Transform, Vector3, WeatherParameter,
};

/// High-level asynchronous interface for managing the ProjectAirSim simulation world,
/// environment, clock stepping, scene actors, object spawning, and debug visualization.
#[derive(Clone)]
pub struct World {
    client: Client,
    parent_topic: String,
    drones: Vec<String>,
    config: Option<serde_json::Value>,
}

impl World {
    /// Connects to and initializes a simulation world.
    ///
    /// If `scene_config` is provided (either as a path to a JSON/JSONC configuration file
    /// or raw JSON/JSONC text), it is parsed, recursively expanded, and loaded into the simulator.
    pub async fn new(client: Client, scene_config: Option<&str>) -> Result<Self> {
        let mut loaded_config = None;
        let mut drones = Vec::new();
        let mut parent_topic = "/Sim".to_string();

        if let Some(config_str) = scene_config {
            let parsed_config = if Path::new(config_str).exists() {
                crate::config::load_scene_config(config_str, None::<&str>, None)?
            } else {
                crate::config::parse_jsonc(config_str)?
            };

            info!("Loading scene into ProjectAirSim server...");
            let load_res: serde_json::Value = client
                .request(
                    "/Sim/LoadScene",
                    &serde_json::json!({
                        "scene_config": parsed_config.to_string(),
                    }),
                )
                .await?;
            info!("Scene loaded successfully");

            if let Some(res_str) = load_res.as_str() {
                if !res_str.is_empty() {
                    parent_topic = format!("/Sim/{res_str}");
                }
            } else if let Some(topic) = parsed_config.get("parent_topic").and_then(|t| t.as_str()) {
                parent_topic = topic.to_string();
            }

            // Discover drone actor names from config
            if let Some(d_list) = parsed_config.get("drones").and_then(|d| d.as_array()) {
                for d in d_list {
                    if let Some(name) = d.get("name").and_then(|n| n.as_str()) {
                        drones.push(name.to_string());
                    }
                }
            }
            if let Some(actors) = parsed_config.get("actors").and_then(|a| a.as_array()) {
                for a in actors {
                    if a.get("type").and_then(|t| t.as_str()) == Some("robot") {
                        if let Some(name) = a.get("name").and_then(|n| n.as_str()) {
                            if !drones.contains(&name.to_string()) {
                                drones.push(name.to_string());
                            }
                        }
                    }
                }
            }

            loaded_config = Some(parsed_config);
        }

        Ok(Self {
            client,
            parent_topic,
            drones,
            config: loaded_config,
        })
    }

    /// Creates a World handle bound to a known parent topic prefix.
    pub fn with_parent_topic(client: Client, parent_topic: impl Into<String>) -> Self {
        Self {
            client,
            parent_topic: parent_topic.into(),
            drones: Vec::new(),
            config: None,
        }
    }

    /// Returns a reference to the underlying asynchronous Client.
    pub fn client(&self) -> &Client {
        &self.client
    }

    /// Returns the active parent topic prefix for this world (e.g. `/Sim` or `/Sim/Scene_1`).
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

    fn topic(&self, method: &str) -> String {
        format!("{}/{}", self.parent_topic, method)
    }

    // --- Simulation Clock & Stepping ---

    /// Retrieves the current simulation clock type string (e.g. `"steppable"`).
    pub async fn get_sim_clock_type(&self) -> Result<String> {
        self.client
            .request(&self.topic("GetSimClockType"), &EmptyParams {})
            .await
    }

    /// Retrieves the current simulation clock time in nanoseconds.
    pub async fn get_sim_time(&self) -> Result<i64> {
        self.client
            .request(&self.topic("GetSimTime"), &EmptyParams {})
            .await
    }

    /// Pauses simulation execution.
    pub async fn pause(&self) -> Result<String> {
        self.client
            .request(
                &self.topic("Pause"),
                &serde_json::json!({ "do_pause": true }),
            )
            .await
    }

    /// Resumes simulation execution.
    pub async fn resume(&self) -> Result<String> {
        self.client
            .request(
                &self.topic("Pause"),
                &serde_json::json!({ "do_pause": false }),
            )
            .await
    }

    /// Queries whether the simulation clock is currently paused.
    pub async fn is_paused(&self) -> Result<bool> {
        self.client
            .request(&self.topic("IsPaused"), &EmptyParams {})
            .await
    }

    /// Advances the simulation clock by a delta duration in nanoseconds.
    pub async fn continue_for_sim_time(
        &self,
        delta_time_nanos: i64,
        wait_until_complete: bool,
    ) -> Result<i64> {
        self.client
            .request(
                &self.topic("ContinueForSimTime"),
                &serde_json::json!({
                    "delta_time": delta_time_nanos,
                    "wait_until_complete": wait_until_complete,
                }),
            )
            .await
    }

    /// Steps the simulation clock forward by a specified delta time in seconds.
    pub async fn step(&self, delta_time_sec: f64) -> Result<i64> {
        let nanos = (delta_time_sec * 1e9) as i64;
        self.continue_for_sim_time(nanos, true).await
    }

    /// Advances the simulation clock until the target simulation time in nanoseconds is reached.
    pub async fn continue_until_sim_time(
        &self,
        target_time_nanos: i64,
        wait_until_complete: bool,
    ) -> Result<i64> {
        self.client
            .request(
                &self.topic("ContinueUntilSimTime"),
                &serde_json::json!({
                    "target_time": target_time_nanos,
                    "wait_until_complete": wait_until_complete,
                }),
            )
            .await
    }

    /// Advances the simulation clock by a specified number of discrete steps.
    pub async fn continue_for_n_steps(
        &self,
        n_steps: i32,
        wait_until_complete: bool,
    ) -> Result<i64> {
        self.client
            .request(
                &self.topic("ContinueForNSteps"),
                &serde_json::json!({
                    "n_steps": n_steps,
                    "wait_until_complete": wait_until_complete,
                }),
            )
            .await
    }

    /// Advances the simulation clock by a single step.
    pub async fn continue_for_single_step(&self, wait_until_complete: bool) -> Result<i64> {
        self.client
            .request(
                &self.topic("ContinueForSingleStep"),
                &serde_json::json!({
                    "wait_until_complete": wait_until_complete,
                }),
            )
            .await
    }

    // --- Voxel Grid Extraction & .binvox Export ---

    /// Generates a 3D boolean voxel occupancy grid for the specified volume.
    ///
    /// Optionally writes the resulting voxel map to a `.binvox` file on disk.
    pub async fn create_voxel_grid(
        &self,
        position: Pose,
        x_size: i32,
        y_size: i32,
        z_size: i32,
        resolution: f32,
        actors_to_ignore: Vec<String>,
        write_file: bool,
        file_path: Option<&str>,
    ) -> Result<Vec<bool>> {
        let transform = Transform::new(position.position, position.orientation);
        let voxels: Vec<bool> = self
            .client
            .request(
                &self.topic("createVoxelGrid"),
                &serde_json::json!({
                    "position": transform,
                    "x_size": x_size,
                    "y_size": y_size,
                    "z_size": z_size,
                    "res": resolution,
                    "actors_to_ignore": actors_to_ignore,
                }),
            )
            .await?;

        if write_file {
            let path = file_path.unwrap_or("./voxel_grid.binvox");
            write_binvox(&voxels, x_size, y_size, z_size, resolution, path).map_err(|e| {
                SimError::SerializationError(format!("Failed to write binvox file '{path}': {e}"))
            })?;
        }

        Ok(voxels)
    }

    // --- Environment, Lighting & Weather ---

    /// Sets sunlight intensity in the simulation scene.
    pub async fn set_sunlight_intensity(&self, intensity: f32) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetSunLightIntensity"),
                &serde_json::json!({ "intensity": intensity }),
            )
            .await
    }

    /// Retrieves current sunlight intensity.
    pub async fn get_sunlight_intensity(&self) -> Result<f32> {
        self.client
            .request(&self.topic("GetSunLightIntensity"), &EmptyParams {})
            .await
    }

    /// Sets cloud shadow strength.
    pub async fn set_cloud_shadow_strength(&self, strength: f32) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetCloudShadowStrength"),
                &serde_json::json!({ "strength": strength }),
            )
            .await
    }

    /// Retrieves current cloud shadow strength.
    pub async fn get_cloud_shadow_strength(&self) -> Result<f32> {
        self.client
            .request(&self.topic("GetCloudShadowStrength"), &EmptyParams {})
            .await
    }

    /// Sets global simulation wind velocity in m/s.
    pub async fn set_wind_velocity(&self, v_x: f64, v_y: f64, v_z: f64) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetWindVelocity"),
                &serde_json::json!({
                    "v_x": v_x,
                    "v_y": v_y,
                    "v_z": v_z,
                }),
            )
            .await
    }

    /// Convenience wrapper for setting wind velocity from a Vector3.
    pub async fn set_wind(&self, wind: Vector3) -> Result<bool> {
        self.set_wind_velocity(wind.x, wind.y, wind.z).await
    }

    /// Retrieves current wind velocity in m/s.
    pub async fn get_wind_velocity(&self) -> Result<Vector3> {
        let val: serde_json::Value = self
            .client
            .request(&self.topic("GetWindVelocity"), &EmptyParams {})
            .await?;

        if let Some(arr) = val.as_array() {
            if arr.len() >= 3 {
                return Ok(Vector3::new(
                    arr[0].as_f64().unwrap_or(0.0),
                    arr[1].as_f64().unwrap_or(0.0),
                    arr[2].as_f64().unwrap_or(0.0),
                ));
            }
        }
        serde_json::from_value(val).map_err(|e| {
            SimError::SerializationError(format!("Failed to parse wind velocity: {e}"))
        })
    }

    /// Convenience alias for `get_wind_velocity`.
    pub async fn get_wind(&self) -> Result<Vector3> {
        self.get_wind_velocity().await
    }

    /// Enables weather visual effects.
    pub async fn enable_weather_visual_effects(&self) -> Result<bool> {
        self.client
            .request(
                &self.topic("SimSetWeatherVisualEffectsStatus"),
                &serde_json::json!({ "status": true }),
            )
            .await
    }

    /// Disables weather visual effects.
    pub async fn disable_weather_visual_effects(&self) -> Result<bool> {
        self.client
            .request(
                &self.topic("SimSetWeatherVisualEffectsStatus"),
                &serde_json::json!({ "status": false }),
            )
            .await
    }

    /// Resets all weather effects to default values.
    pub async fn reset_weather_effects(&self) -> Result<bool> {
        self.client
            .request(&self.topic("ResetWeatherEffects"), &EmptyParams {})
            .await
    }

    /// Sets a specific weather visual effect parameter.
    pub async fn set_weather_visual_effects_param(
        &self,
        param: WeatherParameter,
        value: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetWeatherVisualEffectsParameter"),
                &serde_json::json!({
                    "param": param as i32,
                    "value": value,
                }),
            )
            .await
    }

    /// Retrieves current weather visual effects parameters.
    pub async fn get_weather_visual_effects_param(&self) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.topic("GetWeatherVisualEffectsParameter"),
                &EmptyParams {},
            )
            .await
    }

    /// Sets time of day and sun position parameters.
    pub async fn set_time_of_day(&self, config: &TimeOfDay) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetTimeOfDay"),
                &serde_json::json!({
                    "status": config.enabled,
                    "datetime": config.datetime,
                    "is_dst": config.is_dst,
                    "clock_speed": config.clock_speed,
                    "update_interval": config.update_interval,
                    "move_sun": config.move_sun,
                }),
            )
            .await
    }

    /// Retrieves current time of day configuration.
    pub async fn get_time_of_day(&self) -> Result<serde_json::Value> {
        self.client
            .request(&self.topic("GetTimeOfDay"), &EmptyParams {})
            .await
    }

    /// Sets celestial sun position based on a date-time string.
    pub async fn set_sun_position_from_date_time(
        &self,
        date_time: &str,
        is_dst: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetSunPositionFromDateTime"),
                &serde_json::json!({
                    "date_time": date_time,
                    "is_dst": is_dst,
                }),
            )
            .await
    }

    /// Switches active streaming view.
    pub async fn switch_streaming_view(&self) -> Result<bool> {
        self.client
            .request(&self.topic("SwitchStreamingView"), &EmptyParams {})
            .await
    }

    // --- Object Spawning & Lifecycle ---

    /// Lists all actor entities in the current scene.
    pub async fn list_actors(&self) -> Result<Vec<String>> {
        self.client
            .request(&self.topic("ListActors"), &EmptyParams {})
            .await
    }

    /// Lists objects in the scene matching a regex filter.
    pub async fn list_objects(&self, name_regex: &str) -> Result<Vec<String>> {
        self.client
            .request(
                &self.topic("ListObjects"),
                &serde_json::json!({ "name": name_regex }),
            )
            .await
    }

    /// Lists assets available in the simulation matching a regex filter.
    pub async fn list_assets(&self, name_regex: &str) -> Result<Vec<String>> {
        self.client
            .request(
                &self.topic("ListAssets"),
                &serde_json::json!({ "name": name_regex }),
            )
            .await
    }

    /// Retrieves the 6-DoF pose of a named scene object.
    pub async fn get_object_pose(&self, object_name: &str) -> Result<Pose> {
        self.client
            .request(
                &self.topic("GetObjectPose"),
                &serde_json::json!({ "object_name": object_name }),
            )
            .await
    }

    /// Retrieves poses for multiple named scene objects in a single batch request.
    pub async fn get_object_poses(&self, object_names: &[String]) -> Result<Vec<Pose>> {
        self.client
            .request(
                &self.topic("GetObjectPoses"),
                &serde_json::json!({ "object_names": object_names }),
            )
            .await
    }

    /// Sets the pose of a named scene object, optionally teleporting without physics collision.
    pub async fn set_object_pose(
        &self,
        object_name: &str,
        pose: Pose,
        teleport: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetObjectPose"),
                &serde_json::json!({
                    "object_name": object_name,
                    "pose": pose,
                    "teleport": teleport,
                }),
            )
            .await
    }

    /// Retrieves the 3D scale vector of a named scene object.
    pub async fn get_object_scale(&self, object_name: &str) -> Result<Vector3> {
        let val: serde_json::Value = self
            .client
            .request(
                &self.topic("GetObjectScale"),
                &serde_json::json!({ "object_name": object_name }),
            )
            .await?;

        if let Some(arr) = val.as_array() {
            if arr.len() >= 3 {
                return Ok(Vector3::new(
                    arr[0].as_f64().unwrap_or(1.0),
                    arr[1].as_f64().unwrap_or(1.0),
                    arr[2].as_f64().unwrap_or(1.0),
                ));
            }
        }
        serde_json::from_value(val).map_err(|e| {
            SimError::SerializationError(format!("Failed to parse object scale: {e}"))
        })
    }

    /// Sets the 3D scale of a named scene object.
    pub async fn set_object_scale(&self, object_name: &str, scale: Vector3) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetObjectScale"),
                &serde_json::json!({
                    "object_name": object_name,
                    "scale": [scale.x as f32, scale.y as f32, scale.z as f32],
                }),
            )
            .await
    }

    /// Spawns a packaged simulation object asset into the scene.
    pub async fn spawn_object(
        &self,
        object_name: &str,
        asset_path: &str,
        pose: Pose,
        scale: Vector3,
        enable_physics: bool,
    ) -> Result<String> {
        self.client
            .request(
                &self.topic("SpawnObject"),
                &serde_json::json!({
                    "object_name": object_name,
                    "asset_path": asset_path,
                    "pose": pose,
                    "scale": [scale.x as f32, scale.y as f32, scale.z as f32],
                    "enable_physics": enable_physics,
                }),
            )
            .await
    }

    /// Spawns a mesh/asset from in-memory byte data into the scene.
    pub async fn spawn_object_from_file(
        &self,
        object_name: &str,
        file_format: &str,
        asset_bytes: &[u8],
        is_binary: bool,
        pose: Pose,
        scale: Vector3,
        enable_physics: bool,
    ) -> Result<String> {
        self.client
            .request(
                &self.topic("spawnObjectFromFile"),
                &serde_json::json!({
                    "object_name": object_name,
                    "file_format": file_format,
                    "byte_array": asset_bytes,
                    "is_binary": is_binary,
                    "pose": pose,
                    "scale": [scale.x as f32, scale.y as f32, scale.z as f32],
                    "enable_physics": enable_physics,
                }),
            )
            .await
    }

    /// Spawns a packaged object asset at a specific geographic coordinate.
    pub async fn spawn_object_at_geo(
        &self,
        object_name: &str,
        asset_path: &str,
        latitude: f32,
        longitude: f32,
        altitude: f32,
        rotation: [f32; 4],
        scale: Vector3,
        enable_physics: bool,
    ) -> Result<String> {
        self.client
            .request(
                &self.topic("spawnObjectAtGeo"),
                &serde_json::json!({
                    "object_name": object_name,
                    "asset_path": asset_path,
                    "latitude": latitude,
                    "longitude": longitude,
                    "altitude": altitude,
                    "rotation": rotation,
                    "scale": [scale.x as f32, scale.y as f32, scale.z as f32],
                    "enable_physics": enable_physics,
                }),
            )
            .await
    }

    /// Spawns an asset from file bytes at a specific geographic coordinate.
    pub async fn spawn_object_from_file_at_geo(
        &self,
        object_name: &str,
        file_format: &str,
        asset_bytes: &[u8],
        is_binary: bool,
        latitude: f32,
        longitude: f32,
        altitude: f32,
        rotation: [f32; 4],
        scale: Vector3,
        enable_physics: bool,
    ) -> Result<String> {
        self.client
            .request(
                &self.topic("spawnObjectFromFileAtGeo"),
                &serde_json::json!({
                    "object_name": object_name,
                    "file_format": file_format,
                    "byte_array": asset_bytes,
                    "is_binary": is_binary,
                    "latitude": latitude,
                    "longitude": longitude,
                    "altitude": altitude,
                    "rotation": rotation,
                    "scale": [scale.x as f32, scale.y as f32, scale.z as f32],
                    "enable_physics": enable_physics,
                }),
            )
            .await
    }

    /// Destroys a named scene object.
    pub async fn destroy_object(&self, object_name: &str) -> Result<bool> {
        self.client
            .request(
                &self.topic("DestroyObject"),
                &serde_json::json!({ "object_name": object_name }),
            )
            .await
    }

    /// Destroys all spawned temporary objects in the scene.
    pub async fn destroy_all_spawned_objects(&self) -> Result<bool> {
        self.client
            .request(&self.topic("DestroyAllSpawnedObjects"), &EmptyParams {})
            .await
    }

    // --- Materials, Textures, Lighting & Segmentation ---

    /// Sets the material asset of a named scene object.
    pub async fn set_object_material(
        &self,
        object_name: &str,
        material_asset_path: &str,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetObjectMaterial"),
                &serde_json::json!({
                    "object_name": object_name,
                    "material_asset_path": material_asset_path,
                }),
            )
            .await
    }

    /// Downloads and applies an object texture from a URL.
    pub async fn set_object_texture_from_url(&self, object_name: &str, url: &str) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetObjectTextureFromUrl"),
                &serde_json::json!({
                    "object_name": object_name,
                    "url": url,
                }),
            )
            .await
    }

    /// Applies an object texture from a local filesystem image file.
    pub async fn set_object_texture_from_file(
        &self,
        object_name: &str,
        texture_file_path: &str,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetObjectTextureFromFile"),
                &serde_json::json!({
                    "object_name": object_name,
                    "texture_file_path": texture_file_path,
                }),
            )
            .await
    }

    /// Applies an object texture from a packaged asset path.
    pub async fn set_object_texture_from_packaged_asset(
        &self,
        object_name: &str,
        texture_asset_path: &str,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetObjectTextureFromPackagedAsset"),
                &serde_json::json!({
                    "object_name": object_name,
                    "texture_asset_path": texture_asset_path,
                }),
            )
            .await
    }

    /// Swaps the active texture of tagged actors to a specific texture ID.
    pub async fn swap_object_texture(&self, tag: &str, tex_id: i32) -> Result<bool> {
        self.client
            .request(
                &self.topic("SwapObjectTexture"),
                &serde_json::json!({
                    "tag": tag,
                    "tex_id": tex_id,
                }),
            )
            .await
    }

    /// Sets the light intensity of a light actor object.
    pub async fn set_light_object_intensity(
        &self,
        object_name: &str,
        new_intensity: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetLightObjectIntensity"),
                &serde_json::json!({
                    "object_name": object_name,
                    "new_intensity": new_intensity,
                }),
            )
            .await
    }

    /// Sets the emission color of a light actor object.
    pub async fn set_light_object_color(
        &self,
        object_name: &str,
        color_rgb: [f32; 3],
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetLightObjectColor"),
                &serde_json::json!({
                    "object_name": object_name,
                    "color_rgb": color_rgb,
                }),
            )
            .await
    }

    /// Sets the light attenuation radius of a light actor object.
    pub async fn set_light_object_radius(&self, object_name: &str, new_radius: f32) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetLightObjectRadius"),
                &serde_json::json!({
                    "object_name": object_name,
                    "new_radius": new_radius,
                }),
            )
            .await
    }

    /// Assigns a segmentation class ID to a mesh.
    pub async fn set_segmentation_id_by_name(
        &self,
        mesh_name: &str,
        segmentation_id: i32,
        is_name_regex: bool,
        use_owner_name: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetSegmentationIDByName"),
                &serde_json::json!({
                    "mesh_name": mesh_name,
                    "segmentation_id": segmentation_id,
                    "is_name_regex": is_name_regex,
                    "use_owner_name": use_owner_name,
                }),
            )
            .await
    }

    /// Retrieves the segmentation class ID assigned to a mesh.
    pub async fn get_segmentation_id_by_name(
        &self,
        mesh_name: &str,
        use_owner_name: bool,
    ) -> Result<i32> {
        self.client
            .request(
                &self.topic("GetSegmentationIDByName"),
                &serde_json::json!({
                    "mesh_name": mesh_name,
                    "use_owner_name": use_owner_name,
                }),
            )
            .await
    }

    /// Retrieves the full mapping of segmentation names to integer IDs.
    pub async fn get_segmentation_id_map(&self) -> Result<serde_json::Value> {
        self.client
            .request(&self.topic("GetSegmentationIDMap"), &EmptyParams {})
            .await
    }

    // --- Trajectories & Spatial Queries ---

    /// Imports a 6-DoF NED trajectory for playback by actors or robot vehicles.
    pub async fn import_ned_trajectory(&self, mut trajectory: NEDTrajectory) -> Result<bool> {
        trajectory.auto_fill_missing();
        self.client
            .request(
                &self.topic("ImportNEDTrajectory"),
                &serde_json::json!({
                    "traj_name": trajectory.traj_name,
                    "time": trajectory.time,
                    "pose_x": trajectory.pose_x,
                    "pose_y": trajectory.pose_y,
                    "pose_z": trajectory.pose_z,
                    "pose_roll": trajectory.pose_roll,
                    "pose_pitch": trajectory.pose_pitch,
                    "pose_yaw": trajectory.pose_yaw,
                    "vel_x_lin": trajectory.vel_lin_x,
                    "vel_y_lin": trajectory.vel_lin_y,
                    "vel_z_lin": trajectory.vel_lin_z,
                }),
            )
            .await
    }

    /// Imports a geographic coordinate trajectory for playback by actors.
    pub async fn import_geo_trajectory(&self, trajectory: GeoTrajectory) -> Result<bool> {
        self.client
            .request(
                &self.topic("ImportGeoTrajectory"),
                &serde_json::json!({
                    "traj_name": trajectory.traj_name,
                    "time": trajectory.time,
                    "latitudes": trajectory.latitudes,
                    "longitudes": trajectory.longitudes,
                    "altitudes": trajectory.altitudes,
                    "roll": trajectory.roll,
                    "pitch": trajectory.pitch,
                    "yaw": trajectory.yaw,
                    "vel_lin_x": trajectory.vel_lin_x,
                    "vel_lin_y": trajectory.vel_lin_y,
                    "vel_lin_z": trajectory.vel_lin_z,
                }),
            )
            .await
    }

    /// Queries the ground surface elevation (Z in NED) at a specified (X, Y) coordinate.
    pub async fn get_surface_elevation_at_point(&self, x: f32, y: f32) -> Result<f32> {
        self.client
            .request(
                &self.topic("GetSurfaceElevationAtPoint"),
                &serde_json::json!({ "x": x, "y": y }),
            )
            .await
    }

    /// Computes the 3D bounding box for an object aligned to either world or object coordinates.
    pub async fn get_3d_bounding_box(
        &self,
        object_name: &str,
        box_alignment: BoxAlignment,
    ) -> Result<serde_json::Value> {
        self.client
            .request(
                &self.topic("Get3DBoundingBox"),
                &serde_json::json!({
                    "object_name": object_name,
                    "box_alignment": box_alignment as i32,
                }),
            )
            .await
    }

    /// Casts a ray along the forward X axis of the given pose and returns the 3D hit point,
    /// or NaN coordinates if no geometry was intersected.
    pub async fn hit_test(&self, pose: Pose) -> Result<Vector3> {
        let transform = Transform::new(pose.position, pose.orientation);
        let val: serde_json::Value = self
            .client
            .request(
                &self.topic("HitTest"),
                &serde_json::json!({ "pose": transform }),
            )
            .await?;

        if let Some(arr) = val.as_array() {
            if arr.len() >= 3 {
                return Ok(Vector3::new(
                    arr[0].as_f64().unwrap_or(f64::NAN),
                    arr[1].as_f64().unwrap_or(f64::NAN),
                    arr[2].as_f64().unwrap_or(f64::NAN),
                ));
            }
        }
        serde_json::from_value(val)
            .map_err(|e| SimError::SerializationError(format!("Failed to parse hit test: {e}")))
    }

    // --- Debug Visualizations & Markers ---

    /// Flushes all persistent debug visual markers from the simulator.
    pub async fn flush_persistent_markers(&self) -> Result<bool> {
        self.client
            .request(&self.topic("debugFlushPersistentMarkers"), &EmptyParams {})
            .await
    }

    /// Plots directional arrows in the simulation world for visual debugging.
    pub async fn plot_debug_arrows(
        &self,
        points_start: &[Vector3],
        points_end: &[Vector3],
        color_rgba: ColorRGBA,
        thickness: f32,
        arrow_size: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("debugPlotArrows"),
                &serde_json::json!({
                    "points_start": points_start,
                    "points_end": points_end,
                    "color_rgba": color_rgba,
                    "thickness": thickness,
                    "arrow_size": arrow_size,
                    "duration": sec_duration,
                    "is_persistent": is_persistent,
                }),
            )
            .await
    }

    /// Plots dashed line segments between consecutive points.
    pub async fn plot_debug_dashed_line(
        &self,
        points: &[Vector3],
        color_rgba: ColorRGBA,
        thickness: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("debugPlotDashedLine"),
                &serde_json::json!({
                    "points": points,
                    "color_rgba": color_rgba,
                    "thickness": thickness,
                    "duration": sec_duration,
                    "is_persistent": is_persistent,
                }),
            )
            .await
    }

    /// Plots point markers in 3D space.
    pub async fn plot_debug_points(
        &self,
        points: &[Vector3],
        color_rgba: ColorRGBA,
        size: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("debugPlotPoints"),
                &serde_json::json!({
                    "points": points,
                    "color_rgba": color_rgba,
                    "size": size,
                    "duration": sec_duration,
                    "is_persistent": is_persistent,
                }),
            )
            .await
    }

    /// Plots connected solid line segments through the given vertices.
    pub async fn plot_debug_solid_line(
        &self,
        points: &[Vector3],
        color_rgba: ColorRGBA,
        thickness: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("debugPlotSolidLine"),
                &serde_json::json!({
                    "points": points,
                    "color_rgba": color_rgba,
                    "thickness": thickness,
                    "duration": sec_duration,
                    "is_persistent": is_persistent,
                }),
            )
            .await
    }

    /// Renders text strings at designated world positions.
    pub async fn plot_debug_strings(
        &self,
        strings: &[String],
        positions: &[Vector3],
        scale: f32,
        color_rgba: ColorRGBA,
        sec_duration: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("debugPlotStrings"),
                &serde_json::json!({
                    "strings": strings,
                    "positions": positions,
                    "scale": scale,
                    "color_rgba": color_rgba,
                    "duration": sec_duration,
                }),
            )
            .await
    }

    /// Draws coordinate frame coordinate triads at specified poses.
    pub async fn plot_debug_transforms(
        &self,
        poses: &[Pose],
        scale: f32,
        thickness: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("debugPlotTransforms"),
                &serde_json::json!({
                    "poses": poses,
                    "scale": scale,
                    "thickness": thickness,
                    "duration": sec_duration,
                    "is_persistent": is_persistent,
                }),
            )
            .await
    }

    /// Draws labeled coordinate frame triads at specified poses.
    pub async fn plot_debug_transforms_with_names(
        &self,
        poses: &[Pose],
        names: &[String],
        tf_scale: f32,
        tf_thickness: f32,
        text_scale: f32,
        text_color_rgba: ColorRGBA,
        sec_duration: f32,
    ) -> Result<bool> {
        self.client
            .request(
                &self.topic("debugPlotTransformsWithNames"),
                &serde_json::json!({
                    "poses": poses,
                    "names": names,
                    "tf_scale": tf_scale,
                    "tf_thickness": tf_thickness,
                    "text_scale": text_scale,
                    "text_color_rgba": text_color_rgba,
                    "duration": sec_duration,
                }),
            )
            .await
    }

    /// Configures the vehicle trajectory trace line color and thickness.
    pub async fn set_trace_line(&self, color_rgba: ColorRGBA, thickness: f32) -> Result<bool> {
        self.client
            .request(
                &self.topic("SetTraceLine"),
                &serde_json::json!({
                    "color_rgba": color_rgba,
                    "thickness": thickness,
                }),
            )
            .await
    }

    /// Toggles trajectory trace line visualization on or off.
    pub async fn toggle_trace(&self) -> Result<bool> {
        self.client
            .request(&self.topic("ToggleTrace"), &EmptyParams {})
            .await
    }

    // --- Actor Handles ---

    /// Returns a Drone handle attached to this simulation world.
    pub fn get_drone(&self, name: impl Into<String>) -> Drone {
        Drone::new(self.client.clone(), name, self.parent_topic.clone())
    }

    /// Returns a Rover handle attached to this simulation world.
    pub fn get_rover(&self, name: impl Into<String>) -> Rover {
        Rover::new(self.client.clone(), name, self.parent_topic.clone())
    }

    /// Returns a WheeledVehicle handle attached to this simulation world.
    pub fn get_wheeled_vehicle(&self, name: impl Into<String>) -> WheeledVehicle {
        WheeledVehicle::new(self.client.clone(), name, self.parent_topic.clone())
    }

    /// Returns an EnvActor handle attached to this simulation world.
    pub fn get_env_actor(&self, name: impl Into<String>) -> EnvActor {
        EnvActor::new(self.client.clone(), name, self.parent_topic.clone())
    }

    /// Returns a StaticSensorActor handle attached to this simulation world.
    pub fn get_static_sensor(&self, name: impl Into<String>) -> StaticSensorActor {
        StaticSensorActor::new(self.client.clone(), name, self.parent_topic.clone())
    }
}
