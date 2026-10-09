use crate::blocking::async_result::AsyncResult;
use crate::blocking::client::Client;
use crate::blocking::drone::Drone;
use crate::blocking::env_actor::EnvActor;
use crate::blocking::rover::Rover;
use crate::blocking::static_sensor::StaticSensorActor;
use crate::blocking::wheeled_vehicle::WheeledVehicle;
use crate::error::Result;
use crate::types::{
    BoxAlignment, ColorRGBA, GeoTrajectory, NEDTrajectory, Pose, TimeOfDay, Vector3,
    WeatherParameter,
};

/// Synchronous blocking simulation world controller.
#[derive(Clone)]
pub struct World {
    inner: crate::async_api::World,
    client: Client,
}

impl World {
    /// Connects to and initializes a synchronous simulation world.
    ///
    /// If `scene_config` is provided (either as a path to a JSON/JSONC configuration file
    /// or raw JSON/JSONC text), it is parsed, recursively expanded, and loaded into the simulator.
    pub fn new(client: Client, scene_config: Option<&str>) -> Result<Self> {
        let async_client = client.inner().clone();
        let inner = client
            .runtime()
            .block_on(crate::async_api::World::new(async_client, scene_config))?;
        Ok(Self { inner, client })
    }

    /// Creates a World handle bound to a known parent topic prefix.
    pub fn with_parent_topic(client: Client, parent_topic: impl Into<String>) -> Self {
        let async_client = client.inner().clone();
        let inner = crate::async_api::World::with_parent_topic(async_client, parent_topic);
        Self { inner, client }
    }

    /// Creates a blocking World handle wrapping an existing async World handle.
    pub fn from_inner(inner: crate::async_api::World, client: Client) -> Self {
        Self { inner, client }
    }

    /// Returns a reference to the blocking Client.
    pub fn client(&self) -> &Client {
        &self.client
    }

    /// Returns the active parent topic prefix for this world (e.g. `/Sim` or `/Sim/Scene_1`).
    pub fn parent_topic(&self) -> &str {
        self.inner.parent_topic()
    }

    /// Returns discovered drone actor names in this scene.
    pub fn drones(&self) -> &[String] {
        self.inner.drones()
    }

    /// Returns the parsed scene configuration JSON, if loaded.
    pub fn configuration(&self) -> Option<&serde_json::Value> {
        self.inner.configuration()
    }

    // --- Clock & Time Stepping Controls ---

    /// Queries the simulation clock type (e.g. "Steppable", "RealTime").
    pub fn get_sim_clock_type(&self) -> Result<String> {
        self.client.runtime().block_on(self.inner.get_sim_clock_type())
    }

    /// Retrieves current simulation time in nanoseconds.
    pub fn get_sim_time(&self) -> Result<i64> {
        self.client.runtime().block_on(self.inner.get_sim_time())
    }

    /// Pauses physics simulation execution.
    pub fn pause(&self) -> Result<String> {
        self.client.runtime().block_on(self.inner.pause())
    }

    /// Resumes physics simulation execution.
    pub fn resume(&self) -> Result<String> {
        self.client.runtime().block_on(self.inner.resume())
    }

    /// Checks whether the simulation physics loop is currently paused.
    pub fn is_paused(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.is_paused())
    }

    /// Advances physics simulation by `delta_time_nanos` nanoseconds, blocking until finished.
    pub fn continue_for_sim_time(
        &self,
        delta_time_nanos: i64,
        wait_until_complete: bool,
    ) -> Result<i64> {
        self.client.runtime().block_on(
            self.inner
                .continue_for_sim_time(delta_time_nanos, wait_until_complete),
        )
    }

    /// Advances physics simulation by `delta_time_nanos` nanoseconds asynchronously.
    pub fn continue_for_sim_time_async(
        &self,
        delta_time_nanos: i64,
        wait_until_complete: bool,
    ) -> AsyncResult<i64> {
        let inner = self.inner.clone();
        self.client.spawn_async(async move {
            inner
                .continue_for_sim_time(delta_time_nanos, wait_until_complete)
                .await
        })
    }

    /// Steps simulation forward by `delta_time_sec` seconds.
    pub fn step(&self, delta_time_sec: f64) -> Result<i64> {
        self.client
            .runtime()
            .block_on(self.inner.step(delta_time_sec))
    }

    /// Advances simulation until absolute timestamp `target_time_nanos` is reached.
    pub fn continue_until_sim_time(
        &self,
        target_time_nanos: i64,
        wait_until_complete: bool,
    ) -> Result<i64> {
        self.client.runtime().block_on(
            self.inner
                .continue_until_sim_time(target_time_nanos, wait_until_complete),
        )
    }

    /// Advances simulation by exactly `n_steps` physics steps.
    pub fn continue_for_n_steps(
        &self,
        n_steps: i32,
        wait_until_complete: bool,
    ) -> Result<i64> {
        self.client.runtime().block_on(
            self.inner
                .continue_for_n_steps(n_steps, wait_until_complete),
        )
    }

    /// Advances simulation by a single step.
    pub fn continue_for_single_step(&self, wait_until_complete: bool) -> Result<i64> {
        self.client
            .runtime()
            .block_on(self.inner.continue_for_single_step(wait_until_complete))
    }

    /// Generates and exports a 3D occupancy voxel grid from scene geometry.
    #[allow(clippy::too_many_arguments)]
    pub fn create_voxel_grid(
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
        self.client.runtime().block_on(self.inner.create_voxel_grid(
            position,
            x_size,
            y_size,
            z_size,
            resolution,
            actors_to_ignore,
            write_file,
            file_path,
        ))
    }

    // --- Weather & Atmosphere Controls ---

    /// Sets directional sunlight illumination intensity.
    pub fn set_sunlight_intensity(&self, intensity: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_sunlight_intensity(intensity))
    }

    /// Queries current directional sunlight illumination intensity.
    pub fn get_sunlight_intensity(&self) -> Result<f32> {
        self.client
            .runtime()
            .block_on(self.inner.get_sunlight_intensity())
    }

    /// Sets cloud shadow darkening strength [0.0 to 1.0].
    pub fn set_cloud_shadow_strength(&self, strength: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_cloud_shadow_strength(strength))
    }

    /// Queries current cloud shadow darkening strength.
    pub fn get_cloud_shadow_strength(&self) -> Result<f32> {
        self.client
            .runtime()
            .block_on(self.inner.get_cloud_shadow_strength())
    }

    /// Sets global ambient wind velocity components in m/s (NED frame).
    pub fn set_wind_velocity(&self, v_x: f64, v_y: f64, v_z: f64) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_wind_velocity(v_x, v_y, v_z))
    }

    /// Sets global wind vector directly.
    pub fn set_wind(&self, wind: Vector3) -> Result<bool> {
        self.client.runtime().block_on(self.inner.set_wind(wind))
    }

    /// Queries global ambient wind velocity in m/s (NED frame).
    pub fn get_wind_velocity(&self) -> Result<Vector3> {
        self.client
            .runtime()
            .block_on(self.inner.get_wind_velocity())
    }

    /// Queries global wind vector directly.
    pub fn get_wind(&self) -> Result<Vector3> {
        self.client.runtime().block_on(self.inner.get_wind())
    }

    /// Enables volumetric visual weather particle systems (rain, snow, fog, dust).
    pub fn enable_weather_visual_effects(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.enable_weather_visual_effects())
    }

    /// Disables volumetric visual weather particle systems.
    pub fn disable_weather_visual_effects(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.disable_weather_visual_effects())
    }

    /// Resets all visual weather effect parameters to default clear-sky settings.
    pub fn reset_weather_effects(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.reset_weather_effects())
    }

    /// Adjusts intensity of a specific weather effect.
    pub fn set_weather_visual_effects_param(
        &self,
        param: WeatherParameter,
        value: f32,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_weather_visual_effects_param(param, value))
    }

    /// Queries all current weather effect intensity values.
    pub fn get_weather_visual_effects_param(&self) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_weather_visual_effects_param())
    }

    /// Sets the simulation time of day and solar celestial positioning.
    pub fn set_time_of_day(&self, config: &TimeOfDay) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_time_of_day(config))
    }

    /// Queries current time of day configuration.
    pub fn get_time_of_day(&self) -> Result<serde_json::Value> {
        self.client.runtime().block_on(self.inner.get_time_of_day())
    }

    /// Sets solar celestial position based on calendar date/time string and daylight savings flag.
    pub fn set_sun_position_from_date_time(
        &self,
        date_time: &str,
        is_dst: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_sun_position_from_date_time(date_time, is_dst),
        )
    }

    /// Cycles the active viewport streaming camera view.
    pub fn switch_streaming_view(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.switch_streaming_view())
    }

    // --- Scene Objects & Assets ---

    /// Lists names of all simulation actors currently instantiated in the level.
    pub fn list_actors(&self) -> Result<Vec<String>> {
        self.client.runtime().block_on(self.inner.list_actors())
    }

    /// Searches for scene objects matching a regex pattern.
    pub fn list_objects(&self, name_regex: &str) -> Result<Vec<String>> {
        self.client
            .runtime()
            .block_on(self.inner.list_objects(name_regex))
    }

    /// Searches for spawnable asset templates matching a regex pattern.
    pub fn list_assets(&self, name_regex: &str) -> Result<Vec<String>> {
        self.client
            .runtime()
            .block_on(self.inner.list_assets(name_regex))
    }

    /// Retrieves world pose for a named object.
    pub fn get_object_pose(&self, object_name: &str) -> Result<Pose> {
        self.client
            .runtime()
            .block_on(self.inner.get_object_pose(object_name))
    }

    /// Retrieves world poses for multiple named objects in a single batch query.
    pub fn get_object_poses(&self, object_names: &[String]) -> Result<Vec<Pose>> {
        self.client
            .runtime()
            .block_on(self.inner.get_object_poses(object_names))
    }

    /// Updates the world pose of an existing scene object.
    pub fn set_object_pose(
        &self,
        object_name: &str,
        pose: Pose,
        teleport: bool,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_object_pose(object_name, pose, teleport))
    }

    /// Queries the 3D scale multipliers [x, y, z] of a scene object.
    pub fn get_object_scale(&self, object_name: &str) -> Result<Vector3> {
        self.client
            .runtime()
            .block_on(self.inner.get_object_scale(object_name))
    }

    /// Sets the 3D scale multipliers of a scene object.
    pub fn set_object_scale(&self, object_name: &str, scale: Vector3) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_object_scale(object_name, scale))
    }

    /// Spawns a packaged simulation object asset into the scene.
    pub fn spawn_object(
        &self,
        object_name: &str,
        asset_path: &str,
        pose: Pose,
        scale: Vector3,
        enable_physics: bool,
    ) -> Result<String> {
        self.client.runtime().block_on(
            self.inner
                .spawn_object(object_name, asset_path, pose, scale, enable_physics),
        )
    }

    /// Spawns a mesh/asset from in-memory byte data into the scene.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_object_from_file(
        &self,
        object_name: &str,
        file_format: &str,
        asset_bytes: &[u8],
        is_binary: bool,
        pose: Pose,
        scale: Vector3,
        enable_physics: bool,
    ) -> Result<String> {
        self.client.runtime().block_on(
            self.inner
                .spawn_object_from_file(
                    object_name,
                    file_format,
                    asset_bytes,
                    is_binary,
                    pose,
                    scale,
                    enable_physics,
                ),
        )
    }

    /// Spawns a packaged object asset at a specific geographic coordinate.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_object_at_geo(
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
        self.client.runtime().block_on(
            self.inner
                .spawn_object_at_geo(
                    object_name,
                    asset_path,
                    latitude,
                    longitude,
                    altitude,
                    rotation,
                    scale,
                    enable_physics,
                ),
        )
    }

    /// Spawns an asset from file bytes at a specific geographic coordinate.
    #[allow(clippy::too_many_arguments)]
    pub fn spawn_object_from_file_at_geo(
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
        self.client.runtime().block_on(
            self.inner
                .spawn_object_from_file_at_geo(
                    object_name,
                    file_format,
                    asset_bytes,
                    is_binary,
                    latitude,
                    longitude,
                    altitude,
                    rotation,
                    scale,
                    enable_physics,
                ),
        )
    }

    /// Deletes a spawned object from the scene.
    pub fn destroy_object(&self, object_name: &str) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.destroy_object(object_name))
    }

    /// Deletes all dynamically spawned objects from the scene.
    pub fn destroy_all_spawned_objects(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.destroy_all_spawned_objects())
    }

    /// Assigns a named material interface to a scene object.
    pub fn set_object_material(
        &self,
        object_name: &str,
        material_name: &str,
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_object_material(object_name, material_name))
    }

    /// Downloads and applies a texture from an HTTP(S) URL to an object's material.
    pub fn set_object_texture_from_url(&self, object_name: &str, url: &str) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_object_texture_from_url(object_name, url))
    }

    /// Applies an image file from local disk as a material texture.
    pub fn set_object_texture_from_file(
        &self,
        object_name: &str,
        file_path: &str,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_object_texture_from_file(object_name, file_path),
        )
    }

    /// Applies a packaged project asset as a material texture.
    pub fn set_object_texture_from_packaged_asset(
        &self,
        object_name: &str,
        asset_path: &str,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_object_texture_from_packaged_asset(object_name, asset_path),
        )
    }

    /// Swaps the active texture variation for an actor identified by tag.
    pub fn swap_object_texture(&self, tag: &str, tex_id: i32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.swap_object_texture(tag, tex_id))
    }

    /// Sets luminous intensity for a light actor.
    pub fn set_light_object_intensity(
        &self,
        object_name: &str,
        intensity: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_light_object_intensity(object_name, intensity),
        )
    }

    /// Sets light emission color for a light actor.
    pub fn set_light_object_color(
        &self,
        object_name: &str,
        color_rgb: [f32; 3],
    ) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_light_object_color(object_name, color_rgb))
    }

    /// Sets illumination attenuation radius for a point or spot light actor.
    pub fn set_light_object_radius(&self, object_name: &str, new_radius: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_light_object_radius(object_name, new_radius))
    }

    /// Assigns semantic segmentation label IDs to meshes matching a name.
    pub fn set_segmentation_id_by_name(
        &self,
        mesh_name: &str,
        seg_id: i32,
        is_name_regex: bool,
        use_owner_name: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner
                .set_segmentation_id_by_name(mesh_name, seg_id, is_name_regex, use_owner_name),
        )
    }

    /// Queries the semantic segmentation class ID assigned to a mesh.
    pub fn get_segmentation_id_by_name(
        &self,
        mesh_name: &str,
        use_owner_name: bool,
    ) -> Result<i32> {
        self.client
            .runtime()
            .block_on(self.inner.get_segmentation_id_by_name(mesh_name, use_owner_name))
    }

    /// Retrieves the complete mapping of mesh names to segmentation IDs.
    pub fn get_segmentation_id_map(&self) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_segmentation_id_map())
    }

    // --- Trajectories & Spatial Queries ---

    /// Imports a 6-DoF NED trajectory for playback by actors or robot vehicles.
    pub fn import_ned_trajectory(&self, trajectory: NEDTrajectory) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.import_ned_trajectory(trajectory))
    }

    /// Imports a geographic coordinate trajectory for playback by actors.
    pub fn import_geo_trajectory(&self, trajectory: GeoTrajectory) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.import_geo_trajectory(trajectory))
    }

    /// Queries the ground surface elevation (Z in NED) at a specified (X, Y) coordinate.
    pub fn get_surface_elevation_at_point(&self, x: f32, y: f32) -> Result<f32> {
        self.client
            .runtime()
            .block_on(self.inner.get_surface_elevation_at_point(x, y))
    }

    /// Computes the 3D bounding box for an object aligned to either world or object coordinates.
    pub fn get_3d_bounding_box(
        &self,
        object_name: &str,
        box_alignment: BoxAlignment,
    ) -> Result<serde_json::Value> {
        self.client
            .runtime()
            .block_on(self.inner.get_3d_bounding_box(object_name, box_alignment))
    }

    /// Casts a ray along the forward X axis of the given pose and returns the 3D hit point,
    /// or NaN coordinates if no geometry was intersected.
    pub fn hit_test(&self, pose: Pose) -> Result<Vector3> {
        self.client.runtime().block_on(self.inner.hit_test(pose))
    }

    // --- Debug Visualizations & Markers ---

    /// Flushes all persistent debug visual markers from the simulator.
    pub fn flush_persistent_markers(&self) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.flush_persistent_markers())
    }

    /// Plots directional arrows in the simulation world for visual debugging.
    #[allow(clippy::too_many_arguments)]
    pub fn plot_debug_arrows(
        &self,
        points_start: &[Vector3],
        points_end: &[Vector3],
        color_rgba: ColorRGBA,
        thickness: f32,
        arrow_size: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.plot_debug_arrows(
            points_start,
            points_end,
            color_rgba,
            thickness,
            arrow_size,
            sec_duration,
            is_persistent,
        ))
    }

    /// Plots dashed line segments between consecutive points.
    pub fn plot_debug_dashed_line(
        &self,
        points: &[Vector3],
        color_rgba: ColorRGBA,
        thickness: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.plot_debug_dashed_line(
            points,
            color_rgba,
            thickness,
            sec_duration,
            is_persistent,
        ))
    }

    /// Plots point markers in 3D space.
    pub fn plot_debug_points(
        &self,
        points: &[Vector3],
        color_rgba: ColorRGBA,
        size: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.plot_debug_points(
            points,
            color_rgba,
            size,
            sec_duration,
            is_persistent,
        ))
    }

    /// Plots connected solid line segments through the given vertices.
    pub fn plot_debug_solid_line(
        &self,
        points: &[Vector3],
        color_rgba: ColorRGBA,
        thickness: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.plot_debug_solid_line(
            points,
            color_rgba,
            thickness,
            sec_duration,
            is_persistent,
        ))
    }

    /// Renders text strings at designated world positions.
    pub fn plot_debug_strings(
        &self,
        strings: &[String],
        positions: &[Vector3],
        scale: f32,
        color_rgba: ColorRGBA,
        sec_duration: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.plot_debug_strings(
            strings,
            positions,
            scale,
            color_rgba,
            sec_duration,
        ))
    }

    /// Draws coordinate frame coordinate triads at specified poses.
    pub fn plot_debug_transforms(
        &self,
        poses: &[Pose],
        scale: f32,
        thickness: f32,
        sec_duration: f32,
        is_persistent: bool,
    ) -> Result<bool> {
        self.client.runtime().block_on(self.inner.plot_debug_transforms(
            poses,
            scale,
            thickness,
            sec_duration,
            is_persistent,
        ))
    }

    /// Draws labeled coordinate frame triads at specified poses.
    #[allow(clippy::too_many_arguments)]
    pub fn plot_debug_transforms_with_names(
        &self,
        poses: &[Pose],
        names: &[String],
        tf_scale: f32,
        tf_thickness: f32,
        text_scale: f32,
        text_color_rgba: ColorRGBA,
        sec_duration: f32,
    ) -> Result<bool> {
        self.client.runtime().block_on(
            self.inner.plot_debug_transforms_with_names(
                poses,
                names,
                tf_scale,
                tf_thickness,
                text_scale,
                text_color_rgba,
                sec_duration,
            ),
        )
    }

    /// Configures the vehicle trajectory trace line color and thickness.
    pub fn set_trace_line(&self, color_rgba: ColorRGBA, thickness: f32) -> Result<bool> {
        self.client
            .runtime()
            .block_on(self.inner.set_trace_line(color_rgba, thickness))
    }

    /// Toggles trajectory trace line visualization on or off.
    pub fn toggle_trace(&self) -> Result<bool> {
        self.client.runtime().block_on(self.inner.toggle_trace())
    }

    // --- Actor Handles ---

    /// Returns a Drone handle attached to this simulation world.
    pub fn get_drone(&self, name: impl Into<String>) -> Drone {
        Drone::new(self.client.clone(), name, self.inner.parent_topic())
    }

    /// Returns a Rover handle attached to this simulation world.
    pub fn get_rover(&self, name: impl Into<String>) -> Rover {
        Rover::new(self.client.clone(), name, self.inner.parent_topic())
    }

    /// Returns a WheeledVehicle handle attached to this simulation world.
    pub fn get_wheeled_vehicle(&self, name: impl Into<String>) -> WheeledVehicle {
        WheeledVehicle::new(self.client.clone(), name, self.inner.parent_topic())
    }

    /// Returns an EnvActor handle attached to this simulation world.
    pub fn get_env_actor(&self, name: impl Into<String>) -> EnvActor {
        EnvActor::new(self.client.clone(), name, self.inner.parent_topic())
    }

    /// Returns a StaticSensorActor handle attached to this simulation world.
    pub fn get_static_sensor(&self, name: impl Into<String>) -> StaticSensorActor {
        StaticSensorActor::new(self.client.clone(), name, self.inner.parent_topic())
    }
}
