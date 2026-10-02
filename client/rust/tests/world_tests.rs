use std::collections::HashMap;
use std::fs;
use std::io::Read;

use nng::{Protocol, Socket};
use projectairsim::config::{load_scene_config, parse_jsonc, strip_jsonc_comments};
use projectairsim::protocol::request::RawDataPayload;
use projectairsim::protocol::response::RawResponseEnvelope;
use projectairsim::protocol::RequestEnvelope;
use projectairsim::types::{
    write_binvox, BoxAlignment, ColorRGBA, GeoTrajectory, NEDTrajectory, Pose, TimeOfDay,
    Vector3, WeatherParameter,
};

#[test]
fn test_strip_jsonc_and_config_loader() {
    let jsonc_sample = r#"
    {
        // Scene ID
        "id": "TestScene",
        /* Block comment
           with multiple lines */
        "parent_topic": "/Sim/TestScene",
        "url": "http://domain.com/path//not_comment", // trailing comment
        "actors": [
            {
                "name": "Drone1",
                "type": "robot"
            }
        ]
    }
    "#;

    let stripped = strip_jsonc_comments(jsonc_sample);
    assert!(!stripped.contains("// Scene ID"));
    assert!(!stripped.contains("/* Block comment"));
    assert!(stripped.contains("http://domain.com/path//not_comment"));

    let parsed = parse_jsonc(jsonc_sample).expect("valid JSONC");
    assert_eq!(parsed["id"], "TestScene");
    assert_eq!(parsed["parent_topic"], "/Sim/TestScene");
    assert_eq!(parsed["actors"][0]["name"], "Drone1");

    // Test load_scene_config with temporary directory structure
    let temp_dir = tempfile::tempdir().expect("temp dir");
    let scene_path = temp_dir.path().join("scene.jsonc");
    let sim_cfg_dir = temp_dir.path().join("sim_config");
    fs::create_dir_all(&sim_cfg_dir).expect("create sim_cfg_dir");

    let robot_cfg_path = sim_cfg_dir.join("robot1.jsonc");
    fs::write(
        &robot_cfg_path,
        r#"{
            // Robot internal sensors
            "mass": 1.5,
            "drivetrain": "multirotor"
        }"#,
    )
    .expect("write robot config");

    let env_cfg_path = sim_cfg_dir.join("env1.jsonc");
    fs::write(
        &env_cfg_path,
        r#"{
            // Env actor config
            "radius": 10.0
        }"#,
    )
    .expect("write env config");

    let scene_content = r#"{
        "id": "SceneWithIncludes",
        "actors": [
            {
                "name": "DroneAlpha",
                "type": "robot",
                "robot-config": "robot1.jsonc"
            }
        ],
        "environment-actors": [
            {
                "name": "ObstacleZone",
                "type": "env_actor",
                "env-actor-config": "env1.jsonc"
            }
        ]
    }"#;
    fs::write(&scene_path, scene_content).expect("write scene");

    let loaded = load_scene_config(&scene_path, Some(&sim_cfg_dir), Some(42))
        .expect("loaded expanded scene");
    assert_eq!(loaded["id"], "SceneWithIncludes-42");
    assert_eq!(loaded["actors"][0]["robot-config"]["mass"], 1.5);
    assert_eq!(
        loaded["environment-actors"][0]["env-actor-config"]["radius"],
        10.0
    );
}

#[test]
fn test_binvox_serializer() {
    let temp_dir = tempfile::tempdir().expect("temp dir");
    let file_path = temp_dir.path().join("test.binvox");

    // 2x2x2 = 8 voxels: 4 true, 4 false
    let voxels = vec![true, true, true, true, false, false, false, false];
    write_binvox(&voxels, 2, 2, 2, 1.0, &file_path).expect("write binvox");

    let mut file_bytes = Vec::new();
    let mut file = fs::File::open(&file_path).expect("open binvox file");
    file.read_to_end(&mut file_bytes).expect("read binvox");

    let content_str = String::from_utf8_lossy(&file_bytes);
    assert!(content_str.contains("#binvox 1"));
    assert!(content_str.contains("dim 2 2 2"));
    assert!(content_str.contains("translate -1 -1 -1"));
    assert!(content_str.contains("scale 0.5"));
    assert!(content_str.contains("data\n"));

    // Check binary pairs after "data\n"
    let data_pos = file_bytes
        .windows(5)
        .position(|w| w == b"data\n")
        .expect("data header marker")
        + 5;
    let binary_data = &file_bytes[data_pos..];
    // Should be pair (1, 4) followed by pair (0, 4)
    assert_eq!(binary_data, &[1, 4, 0, 4]);
}

#[test]
fn test_color_rgba_and_enums() {
    let color = ColorRGBA::new(1.0, 0.5, 0.25, 0.8);
    let serialized = serde_json::to_string(&color).expect("serialize color");
    assert_eq!(serialized, "[1.0,0.5,0.25,0.8]");

    let from_arr: ColorRGBA = serde_json::from_str("[0.1,0.2,0.3,1.0]").expect("deserialize arr");
    assert_eq!(from_arr, ColorRGBA::new(0.1, 0.2, 0.3, 1.0));

    let from_arr3: ColorRGBA = serde_json::from_str("[0.1,0.2,0.3]").expect("deserialize arr3");
    assert_eq!(from_arr3, ColorRGBA::rgb(0.1, 0.2, 0.3));

    let from_obj: ColorRGBA = serde_json::from_str(
        r#"{"red": 0.4, "green": 0.5, "blue": 0.6, "alpha": 0.7}"#,
    )
    .expect("deserialize obj");
    assert_eq!(from_obj, ColorRGBA::new(0.4, 0.5, 0.6, 0.7));

    // Test WeatherParameter
    let wp = WeatherParameter::Rain;
    let wp_str = serde_json::to_string(&wp).expect("serialize wp");
    assert_eq!(wp_str, "1");
    let wp_back: WeatherParameter = serde_json::from_str("1").expect("deserialize wp");
    assert_eq!(wp_back, WeatherParameter::Rain);

    // Test BoxAlignment
    let ba = BoxAlignment::ObjectOriented;
    let ba_str = serde_json::to_string(&ba).expect("serialize ba");
    assert_eq!(ba_str, "1");
    let ba_back: BoxAlignment = serde_json::from_str("0").expect("deserialize ba");
    assert_eq!(ba_back, BoxAlignment::WorldAxis);
}

#[cfg(feature = "async")]
#[tokio::test]
async fn test_async_world_phase3_complete_suite() {
    let rep_port = 49990;
    let pair_port = 49989;

    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");

    let rep_socket = Socket::new(Protocol::Rep0).expect("failed to open Rep0 socket");
    rep_socket.listen(&rep_url).expect("failed to listen Rep0");

    let pair_server = Socket::new(Protocol::Pair0).expect("failed to open Pair0 socket");
    pair_server
        .listen(&pair_url)
        .expect("failed to listen Pair0");

    let server_thread = std::thread::spawn(move || {
        let expected_methods = vec![
            // Clock
            "/Sim/SceneUnit/GetSimClockType",
            "/Sim/SceneUnit/GetSimTime",
            "/Sim/SceneUnit/Pause",
            "/Sim/SceneUnit/Pause",
            "/Sim/SceneUnit/IsPaused",
            "/Sim/SceneUnit/ContinueForSimTime",
            "/Sim/SceneUnit/ContinueUntilSimTime",
            "/Sim/SceneUnit/ContinueForNSteps",
            "/Sim/SceneUnit/ContinueForSingleStep",
            // Voxel Grid
            "/Sim/SceneUnit/createVoxelGrid",
            // Environment & Weather
            "/Sim/SceneUnit/SetSunLightIntensity",
            "/Sim/SceneUnit/GetSunLightIntensity",
            "/Sim/SceneUnit/SetCloudShadowStrength",
            "/Sim/SceneUnit/GetCloudShadowStrength",
            "/Sim/SceneUnit/SetWindVelocity",
            "/Sim/SceneUnit/GetWindVelocity",
            "/Sim/SceneUnit/SimSetWeatherVisualEffectsStatus",
            "/Sim/SceneUnit/SimSetWeatherVisualEffectsStatus",
            "/Sim/SceneUnit/ResetWeatherEffects",
            "/Sim/SceneUnit/SetWeatherVisualEffectsParameter",
            "/Sim/SceneUnit/GetWeatherVisualEffectsParameter",
            "/Sim/SceneUnit/SetTimeOfDay",
            "/Sim/SceneUnit/GetTimeOfDay",
            "/Sim/SceneUnit/SetSunPositionFromDateTime",
            "/Sim/SceneUnit/SwitchStreamingView",
            // Object Spawning & Lifecycle
            "/Sim/SceneUnit/ListActors",
            "/Sim/SceneUnit/ListObjects",
            "/Sim/SceneUnit/ListAssets",
            "/Sim/SceneUnit/GetObjectPose",
            "/Sim/SceneUnit/GetObjectPoses",
            "/Sim/SceneUnit/SetObjectPose",
            "/Sim/SceneUnit/GetObjectScale",
            "/Sim/SceneUnit/SetObjectScale",
            "/Sim/SceneUnit/SpawnObject",
            "/Sim/SceneUnit/spawnObjectFromFile",
            "/Sim/SceneUnit/spawnObjectAtGeo",
            "/Sim/SceneUnit/spawnObjectFromFileAtGeo",
            "/Sim/SceneUnit/DestroyObject",
            "/Sim/SceneUnit/DestroyAllSpawnedObjects",
            // Materials, Textures, Lighting, Segmentation
            "/Sim/SceneUnit/SetObjectMaterial",
            "/Sim/SceneUnit/SetObjectTextureFromUrl",
            "/Sim/SceneUnit/SetObjectTextureFromFile",
            "/Sim/SceneUnit/SetObjectTextureFromPackagedAsset",
            "/Sim/SceneUnit/SwapObjectTexture",
            "/Sim/SceneUnit/SetLightObjectIntensity",
            "/Sim/SceneUnit/SetLightObjectColor",
            "/Sim/SceneUnit/SetLightObjectRadius",
            "/Sim/SceneUnit/SetSegmentationIDByName",
            "/Sim/SceneUnit/GetSegmentationIDByName",
            "/Sim/SceneUnit/GetSegmentationIDMap",
            // Trajectories & Spatial Queries
            "/Sim/SceneUnit/ImportNEDTrajectory",
            "/Sim/SceneUnit/ImportGeoTrajectory",
            "/Sim/SceneUnit/GetSurfaceElevationAtPoint",
            "/Sim/SceneUnit/Get3DBoundingBox",
            "/Sim/SceneUnit/HitTest",
            // Debug Plots
            "/Sim/SceneUnit/debugFlushPersistentMarkers",
            "/Sim/SceneUnit/debugPlotArrows",
            "/Sim/SceneUnit/debugPlotDashedLine",
            "/Sim/SceneUnit/debugPlotPoints",
            "/Sim/SceneUnit/debugPlotSolidLine",
            "/Sim/SceneUnit/debugPlotStrings",
            "/Sim/SceneUnit/debugPlotTransforms",
            "/Sim/SceneUnit/debugPlotTransformsWithNames",
            "/Sim/SceneUnit/SetTraceLine",
            "/Sim/SceneUnit/ToggleTrace",
        ];

        for expected in expected_methods {
            if let Ok(msg) = rep_socket.recv() {
                let req: RequestEnvelope =
                    rmp_serde::from_slice(msg.as_slice()).expect("valid RequestEnvelope");
                assert_eq!(req.method, expected, "Unexpected RPC method in mock server");

                let reply_data = match req.method {
                    "/Sim/SceneUnit/GetSimClockType" => rmp_serde::to_vec(&"steppable").unwrap(),
                    "/Sim/SceneUnit/GetSimTime" => rmp_serde::to_vec(&10_000_000i64).unwrap(),
                    "/Sim/SceneUnit/Pause" => rmp_serde::to_vec(&"paused").unwrap(),
                    "/Sim/SceneUnit/IsPaused" => rmp_serde::to_vec(&true).unwrap(),
                    "/Sim/SceneUnit/ContinueForSimTime"
                    | "/Sim/SceneUnit/ContinueUntilSimTime"
                    | "/Sim/SceneUnit/ContinueForNSteps"
                    | "/Sim/SceneUnit/ContinueForSingleStep" => {
                        rmp_serde::to_vec(&20_000_000i64).unwrap()
                    }
                    "/Sim/SceneUnit/createVoxelGrid" => {
                        rmp_serde::to_vec(&vec![true, false, true, false]).unwrap()
                    }
                    "/Sim/SceneUnit/GetSunLightIntensity"
                    | "/Sim/SceneUnit/GetCloudShadowStrength" => {
                        rmp_serde::to_vec(&0.75f32).unwrap()
                    }
                    "/Sim/SceneUnit/GetWindVelocity" => {
                        rmp_serde::to_vec(&vec![1.5, -2.0, 0.5]).unwrap()
                    }
                    "/Sim/SceneUnit/GetWeatherVisualEffectsParameter" => {
                        let mut map = HashMap::new();
                        map.insert("1", 0.8);
                        rmp_serde::to_vec(&map).unwrap()
                    }
                    "/Sim/SceneUnit/GetTimeOfDay" => {
                        let mut map = HashMap::new();
                        map.insert("datetime", "2026-10-01 12:00:00");
                        rmp_serde::to_vec(&map).unwrap()
                    }
                    "/Sim/SceneUnit/ListActors" => {
                        rmp_serde::to_vec(&vec!["Drone1", "Rover1"]).unwrap()
                    }
                    "/Sim/SceneUnit/ListObjects" => rmp_serde::to_vec(&vec!["Cube", "Cylinder"]).unwrap(),
                    "/Sim/SceneUnit/ListAssets" => rmp_serde::to_vec(&vec!["Asset_A", "Asset_B"]).unwrap(),
                    "/Sim/SceneUnit/GetObjectPose" => rmp_serde::to_vec(&Pose::default()).unwrap(),
                    "/Sim/SceneUnit/GetObjectPoses" => {
                        rmp_serde::to_vec(&vec![Pose::default(), Pose::default()]).unwrap()
                    }
                    "/Sim/SceneUnit/GetObjectScale" => rmp_serde::to_vec(&vec![1.0, 1.0, 1.0]).unwrap(),
                    "/Sim/SceneUnit/SpawnObject"
                    | "/Sim/SceneUnit/spawnObjectFromFile"
                    | "/Sim/SceneUnit/spawnObjectAtGeo"
                    | "/Sim/SceneUnit/spawnObjectFromFileAtGeo" => {
                        rmp_serde::to_vec(&"SpawnedObj_01").unwrap()
                    }
                    "/Sim/SceneUnit/GetSegmentationIDByName" => rmp_serde::to_vec(&42).unwrap(),
                    "/Sim/SceneUnit/GetSegmentationIDMap" => {
                        let mut map = HashMap::new();
                        map.insert("Cube", 42);
                        rmp_serde::to_vec(&map).unwrap()
                    }
                    "/Sim/SceneUnit/GetSurfaceElevationAtPoint" => {
                        rmp_serde::to_vec(&-12.5f32).unwrap()
                    }
                    "/Sim/SceneUnit/Get3DBoundingBox" => {
                        let mut map = HashMap::new();
                        map.insert("min_x", -1.0);
                        map.insert("max_x", 1.0);
                        rmp_serde::to_vec(&map).unwrap()
                    }
                    "/Sim/SceneUnit/HitTest" => rmp_serde::to_vec(&vec![10.0, 20.0, -5.0]).unwrap(),
                    _ => rmp_serde::to_vec(&true).unwrap(),
                };

                let resp = RawResponseEnvelope {
                    id: Some(req.id),
                    version: Some(1.0),
                    result: Some(RawDataPayload { data: reply_data }),
                    error: None,
                };
                let resp_bytes = rmp_serde::to_vec(&resp).expect("valid response bytes");
                let _ = rep_socket.send(&resp_bytes);

            }
        }
    });

    // Connect client and wrap with World
    let client = projectairsim::async_api::Client::connect_with_ports(
        "127.0.0.1",
        pair_port,
        rep_port,
    )
    .await
    .expect("connect client");

    let world = projectairsim::async_api::World::with_parent_topic(client, "/Sim/SceneUnit");

    // 1. Clock
    assert_eq!(world.get_sim_clock_type().await.unwrap(), "steppable");
    assert_eq!(world.get_sim_time().await.unwrap(), 10_000_000);
    assert_eq!(world.pause().await.unwrap(), "paused");
    assert_eq!(world.resume().await.unwrap(), "paused");
    assert!(world.is_paused().await.unwrap());
    assert_eq!(world.continue_for_sim_time(100_000, true).await.unwrap(), 20_000_000);
    assert_eq!(world.continue_until_sim_time(500_000, true).await.unwrap(), 20_000_000);
    assert_eq!(world.continue_for_n_steps(5, true).await.unwrap(), 20_000_000);
    assert_eq!(world.continue_for_single_step(true).await.unwrap(), 20_000_000);

    // 2. Voxel Grid
    let vox = world
        .create_voxel_grid(Pose::default(), 2, 2, 2, 1.0, vec![], false, None)
        .await
        .unwrap();
    assert_eq!(vox.len(), 4);

    // 3. Environment & Weather
    assert!(world.set_sunlight_intensity(1.2).await.unwrap());
    assert_eq!(world.get_sunlight_intensity().await.unwrap(), 0.75);
    assert!(world.set_cloud_shadow_strength(0.5).await.unwrap());
    assert_eq!(world.get_cloud_shadow_strength().await.unwrap(), 0.75);
    assert!(world.set_wind_velocity(1.0, 2.0, 3.0).await.unwrap());
    let wind = world.get_wind().await.unwrap();
    assert_eq!(wind, Vector3::new(1.5, -2.0, 0.5));
    assert!(world.enable_weather_visual_effects().await.unwrap());
    assert!(world.disable_weather_visual_effects().await.unwrap());
    assert!(world.reset_weather_effects().await.unwrap());
    assert!(world.set_weather_visual_effects_param(WeatherParameter::Rain, 0.8).await.unwrap());
    assert!(world.get_weather_visual_effects_param().await.is_ok());
    assert!(world.set_time_of_day(&TimeOfDay::default()).await.unwrap());
    assert!(world.get_time_of_day().await.is_ok());
    assert!(world.set_sun_position_from_date_time("2026-10-01 12:00:00", false).await.unwrap());
    assert!(world.switch_streaming_view().await.unwrap());

    // 4. Object Spawning & Lifecycle
    assert_eq!(world.list_actors().await.unwrap(), vec!["Drone1", "Rover1"]);
    assert_eq!(world.list_objects(".*").await.unwrap(), vec!["Cube", "Cylinder"]);
    assert_eq!(world.list_assets(".*").await.unwrap(), vec!["Asset_A", "Asset_B"]);
    assert_eq!(world.get_object_pose("Cube").await.unwrap(), Pose::default());
    assert_eq!(
        world.get_object_poses(&["Cube".into(), "Cylinder".into()]).await.unwrap().len(),
        2
    );
    assert!(world.set_object_pose("Cube", Pose::default(), true).await.unwrap());
    assert_eq!(world.get_object_scale("Cube").await.unwrap(), Vector3::new(1.0, 1.0, 1.0));
    assert!(world.set_object_scale("Cube", Vector3::new(2.0, 2.0, 2.0)).await.unwrap());
    assert_eq!(
        world.spawn_object("Cube", "path/asset", Pose::default(), Vector3::new(1.0, 1.0, 1.0), true).await.unwrap(),
        "SpawnedObj_01"
    );
    assert_eq!(
        world.spawn_object_from_file("Cube", "obj", b"binarydata", true, Pose::default(), Vector3::new(1.0, 1.0, 1.0), true).await.unwrap(),
        "SpawnedObj_01"
    );
    assert_eq!(
        world.spawn_object_at_geo("Cube", "path/asset", 47.0, -122.0, 100.0, [1.0, 0.0, 0.0, 0.0], Vector3::new(1.0, 1.0, 1.0), true).await.unwrap(),
        "SpawnedObj_01"
    );
    assert_eq!(
        world.spawn_object_from_file_at_geo("Cube", "obj", b"bindata", true, 47.0, -122.0, 100.0, [1.0, 0.0, 0.0, 0.0], Vector3::new(1.0, 1.0, 1.0), true).await.unwrap(),
        "SpawnedObj_01"
    );
    assert!(world.destroy_object("Cube").await.unwrap());
    assert!(world.destroy_all_spawned_objects().await.unwrap());

    // 5. Materials, Textures, Lighting, Segmentation
    assert!(world.set_object_material("Cube", "mat/path").await.unwrap());
    assert!(world.set_object_texture_from_url("Cube", "http://tex.png").await.unwrap());
    assert!(world.set_object_texture_from_file("Cube", "/tmp/tex.png").await.unwrap());
    assert!(world.set_object_texture_from_packaged_asset("Cube", "asset/tex").await.unwrap());
    assert!(world.swap_object_texture("tag1", 2).await.unwrap());
    assert!(world.set_light_object_intensity("Light1", 500.0).await.unwrap());
    assert!(world.set_light_object_color("Light1", [1.0, 0.8, 0.6]).await.unwrap());
    assert!(world.set_light_object_radius("Light1", 10.0).await.unwrap());
    assert!(world.set_segmentation_id_by_name("Cube", 42, false, false).await.unwrap());
    assert_eq!(world.get_segmentation_id_by_name("Cube", false).await.unwrap(), 42);
    assert!(world.get_segmentation_id_map().await.is_ok());

    // 6. Trajectories & Spatial Queries
    let ned_traj = NEDTrajectory {
        traj_name: "test_traj".into(),
        time: vec![0.0, 1.0],
        pose_x: vec![0.0, 10.0],
        pose_y: vec![0.0, 0.0],
        pose_z: vec![0.0, -5.0],
        ..Default::default()
    };
    assert!(world.import_ned_trajectory(ned_traj).await.unwrap());

    let geo_traj = GeoTrajectory {
        traj_name: "geo_traj".into(),
        time: vec![0.0, 1.0],
        latitudes: vec![47.0, 47.1],
        longitudes: vec![-122.0, -122.1],
        altitudes: vec![100.0, 110.0],
        ..Default::default()
    };
    assert!(world.import_geo_trajectory(geo_traj).await.unwrap());

    assert_eq!(world.get_surface_elevation_at_point(5.0, 10.0).await.unwrap(), -12.5);
    assert!(world.get_3d_bounding_box("Cube", BoxAlignment::WorldAxis).await.is_ok());
    assert_eq!(world.hit_test(Pose::default()).await.unwrap(), Vector3::new(10.0, 20.0, -5.0));

    // 7. Debug Plots
    assert!(world.flush_persistent_markers().await.unwrap());
    assert!(world.plot_debug_arrows(&[Vector3::ZERO], &[Vector3::new(1.0, 0.0, 0.0)], ColorRGBA::RED, 2.0, 0.5, 10.0, false).await.unwrap());
    assert!(world.plot_debug_dashed_line(&[Vector3::ZERO, Vector3::new(1.0, 1.0, 0.0)], ColorRGBA::GREEN, 2.0, 10.0, false).await.unwrap());
    assert!(world.plot_debug_points(&[Vector3::ZERO], ColorRGBA::BLUE, 5.0, 10.0, false).await.unwrap());
    assert!(world.plot_debug_solid_line(&[Vector3::ZERO, Vector3::new(2.0, 2.0, 0.0)], ColorRGBA::WHITE, 2.0, 10.0, false).await.unwrap());
    assert!(world.plot_debug_strings(&["Waypoint".into()], &[Vector3::ZERO], 1.0, ColorRGBA::YELLOW, 10.0).await.unwrap());
    assert!(world.plot_debug_transforms(&[Pose::default()], 1.0, 2.0, 10.0, false).await.unwrap());
    assert!(world.plot_debug_transforms_with_names(&[Pose::default()], &["Origin".into()], 1.0, 2.0, 1.0, ColorRGBA::WHITE, 10.0).await.unwrap());
    assert!(world.set_trace_line(ColorRGBA::CYAN, 3.0).await.unwrap());
    assert!(world.toggle_trace().await.unwrap());

    server_thread.join().expect("server thread finished cleanly");
}

#[cfg(feature = "sync")]
#[test]
fn test_blocking_world_phase3_suite() {
    let rep_port = 49992;
    let pair_port = 49991;

    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");

    let rep_socket = Socket::new(Protocol::Rep0).expect("failed to open Rep0 socket");
    rep_socket.listen(&rep_url).expect("failed to listen Rep0");

    let pair_server = Socket::new(Protocol::Pair0).expect("failed to open Pair0 socket");
    pair_server
        .listen(&pair_url)
        .expect("failed to listen Pair0");

    let server_thread = std::thread::spawn(move || {
        let expected_methods = vec![
            "/Sim/SceneSync/GetSimClockType",
            "/Sim/SceneSync/GetSimTime",
            "/Sim/SceneSync/Pause",
            "/Sim/SceneSync/Pause",
            "/Sim/SceneSync/IsPaused",
            "/Sim/SceneSync/ContinueForSimTime",
            "/Sim/SceneSync/createVoxelGrid",
            "/Sim/SceneSync/SetSunLightIntensity",
            "/Sim/SceneSync/GetSunLightIntensity",
            "/Sim/SceneSync/SetWindVelocity",
            "/Sim/SceneSync/GetWindVelocity",
            "/Sim/SceneSync/ListActors",
            "/Sim/SceneSync/GetObjectPose",
            "/Sim/SceneSync/SetObjectPose",
            "/Sim/SceneSync/SpawnObject",
            "/Sim/SceneSync/DestroyObject",
            "/Sim/SceneSync/SetObjectMaterial",
            "/Sim/SceneSync/SetSegmentationIDByName",
            "/Sim/SceneSync/GetSegmentationIDByName",
            "/Sim/SceneSync/ImportNEDTrajectory",
            "/Sim/SceneSync/GetSurfaceElevationAtPoint",
            "/Sim/SceneSync/HitTest",
            "/Sim/SceneSync/debugFlushPersistentMarkers",
            "/Sim/SceneSync/debugPlotPoints",
            "/Sim/SceneSync/SetTraceLine",
            "/Sim/SceneSync/ToggleTrace",
        ];

        for expected in expected_methods {
            if let Ok(msg) = rep_socket.recv() {
                let req: RequestEnvelope =
                    rmp_serde::from_slice(msg.as_slice()).expect("valid RequestEnvelope");
                assert_eq!(req.method, expected);

                let reply_data = match req.method {
                    "/Sim/SceneSync/GetSimClockType" => rmp_serde::to_vec(&"wallclock").unwrap(),
                    "/Sim/SceneSync/GetSimTime" => rmp_serde::to_vec(&50_000_000i64).unwrap(),
                    "/Sim/SceneSync/Pause" => rmp_serde::to_vec(&"paused").unwrap(),
                    "/Sim/SceneSync/IsPaused" => rmp_serde::to_vec(&false).unwrap(),
                    "/Sim/SceneSync/ContinueForSimTime" => rmp_serde::to_vec(&60_000_000i64).unwrap(),
                    "/Sim/SceneSync/createVoxelGrid" => rmp_serde::to_vec(&vec![false, true]).unwrap(),
                    "/Sim/SceneSync/GetSunLightIntensity" => rmp_serde::to_vec(&1.0f32).unwrap(),
                    "/Sim/SceneSync/GetWindVelocity" => rmp_serde::to_vec(&vec![0.0, 0.0, 0.0]).unwrap(),
                    "/Sim/SceneSync/ListActors" => rmp_serde::to_vec(&vec!["DroneSync"]).unwrap(),
                    "/Sim/SceneSync/GetObjectPose" => rmp_serde::to_vec(&Pose::default()).unwrap(),
                    "/Sim/SceneSync/SpawnObject" => rmp_serde::to_vec(&"SpawnedSync").unwrap(),
                    "/Sim/SceneSync/GetSegmentationIDByName" => rmp_serde::to_vec(&101).unwrap(),
                    "/Sim/SceneSync/GetSurfaceElevationAtPoint" => rmp_serde::to_vec(&0.0f32).unwrap(),
                    "/Sim/SceneSync/HitTest" => rmp_serde::to_vec(&vec![0.0, 0.0, 0.0]).unwrap(),
                    _ => rmp_serde::to_vec(&true).unwrap(),
                };

                let resp = RawResponseEnvelope {
                    id: Some(req.id),
                    version: Some(1.0),
                    result: Some(RawDataPayload { data: reply_data }),
                    error: None,
                };
                let resp_bytes = rmp_serde::to_vec(&resp).expect("valid response bytes");
                let _ = rep_socket.send(&resp_bytes);
            }
        }
    });

    let client = projectairsim::blocking::Client::connect_with_ports(
        "127.0.0.1",
        pair_port,
        rep_port,
    )
    .expect("connect blocking client");

    let world = projectairsim::blocking::World::with_parent_topic(client, "/Sim/SceneSync");

    assert_eq!(world.get_sim_clock_type().unwrap(), "wallclock");
    assert_eq!(world.get_sim_time().unwrap(), 50_000_000);
    assert_eq!(world.pause().unwrap(), "paused");
    assert_eq!(world.resume().unwrap(), "paused");
    assert!(!world.is_paused().unwrap());
    assert_eq!(world.continue_for_sim_time(10_000, true).unwrap(), 60_000_000);

    let vox = world
        .create_voxel_grid(Pose::default(), 1, 1, 1, 1.0, vec![], false, None)
        .unwrap();
    assert_eq!(vox.len(), 2);

    assert!(world.set_sunlight_intensity(1.0).unwrap());
    assert_eq!(world.get_sunlight_intensity().unwrap(), 1.0);
    assert!(world.set_wind_velocity(0.0, 0.0, 0.0).unwrap());
    assert_eq!(world.get_wind_velocity().unwrap(), Vector3::ZERO);

    assert_eq!(world.list_actors().unwrap(), vec!["DroneSync"]);
    assert_eq!(world.get_object_pose("Obj").unwrap(), Pose::default());
    assert!(world.set_object_pose("Obj", Pose::default(), false).unwrap());
    assert_eq!(
        world.spawn_object("Obj", "asset", Pose::default(), Vector3::new(1.0, 1.0, 1.0), false).unwrap(),
        "SpawnedSync"
    );
    assert!(world.destroy_object("Obj").unwrap());

    assert!(world.set_object_material("Obj", "mat").unwrap());
    assert!(world.set_segmentation_id_by_name("Obj", 101, false, false).unwrap());
    assert_eq!(world.get_segmentation_id_by_name("Obj", false).unwrap(), 101);

    let ned = NEDTrajectory {
        traj_name: "traj".into(),
        time: vec![0.0],
        pose_x: vec![0.0],
        pose_y: vec![0.0],
        pose_z: vec![0.0],
        ..Default::default()
    };
    assert!(world.import_ned_trajectory(ned).unwrap());
    assert_eq!(world.get_surface_elevation_at_point(0.0, 0.0).unwrap(), 0.0);
    assert_eq!(world.hit_test(Pose::default()).unwrap(), Vector3::ZERO);

    assert!(world.flush_persistent_markers().unwrap());
    assert!(world.plot_debug_points(&[Vector3::ZERO], ColorRGBA::WHITE, 1.0, 1.0, false).unwrap());
    assert!(world.set_trace_line(ColorRGBA::BLACK, 1.0).unwrap());
    assert!(world.toggle_trace().unwrap());

    let _drone = world.get_drone("DroneSync");
    let _rover = world.get_rover("RoverSync");
    let _vehicle = world.get_wheeled_vehicle("WheeledSync");
    let _env = world.get_env_actor("EnvSync");
    let _sensor = world.get_static_sensor("SensorSync");

    server_thread.join().expect("server thread finished cleanly");
}

