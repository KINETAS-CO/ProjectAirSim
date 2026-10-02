//! Example: User Radar Scenario matching C++ `UserRadarScenario`.
//!
//! Demonstrates drone takeoff, subscribing to radar detections and tracks topics,
//! and parsing sensor output streams.
//!
//! Run with:
//! ```bash
//! cargo run --example user_radar_scenario
//! ```

use std::path::Path;
use std::time::Duration;
use projectairsim::{Client, World};

struct Config {
    sim_host: String,
    sim_config: String,
    scene_file: String,
    vehicle_name: String,
    radar_name: String,
}

impl Config {
    fn parse_args() -> Option<Self> {
        let mut sim_host = "127.0.0.1".to_string();
        let mut sim_config = "client/python/example_user_scripts/sim_config".to_string();
        let mut scene_file = "scene_radar_drone.jsonc".to_string();
        let mut vehicle_name = "Drone1".to_string();
        let mut radar_name = "radar1".to_string();

        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--simhost" => {
                    sim_host = args.next().expect("Missing value for --simhost");
                }
                "--simconfig" => {
                    sim_config = args.next().expect("Missing value for --simconfig");
                }
                "--scene" => {
                    scene_file = args.next().expect("Missing value for --scene");
                }
                "--vehicle" => {
                    vehicle_name = args.next().expect("Missing value for --vehicle");
                }
                "--radar" => {
                    radar_name = args.next().expect("Missing value for --radar");
                }
                "-h" | "--help" | "/?" => {
                    println!("Usage: user_radar_scenario [--simhost <host>] [--simconfig <dir>] [--scene <file>] [--vehicle <name>] [--radar <name>]");
                    return None;
                }
                other => {
                    eprintln!("Unknown argument: {other}");
                    return None;
                }
            }
        }

        Some(Self {
            sim_host,
            sim_config,
            scene_file,
            vehicle_name,
            radar_name,
        })
    }
}

fn count_msgpack_field(payload: &[u8], field_name: &str) -> usize {
    if let Ok(value) = rmp_serde::from_slice::<serde_json::Value>(payload) {
        if let Some(arr) = value.get(field_name).and_then(|v| v.as_array()) {
            return arr.len();
        }
    }
    0
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    let config = match Config::parse_args() {
        Some(cfg) => cfg,
        None => return Ok(()),
    };

    println!("[INFO] Connecting to ProjectAirSim at {}:8990...", config.sim_host);
    let client = Client::connect(&config.sim_host).await?;

    let scene_path = Path::new(&config.sim_config).join(&config.scene_file);
    let world = if scene_path.exists() {
        println!("[INFO] Loading scene from {}", scene_path.display());
        World::new(client.clone(), Some(scene_path.to_str().unwrap())).await?
    } else {
        println!("[INFO] Attaching to existing simulation scene...");
        World::new(client.clone(), None).await?
    };

    let drone = world.get_drone(&config.vehicle_name);
    println!("[OK] Connected to drone '{}'", drone.name());

    println!("[..] Enabling API control and arming...");
    drone.enable_api_control().await?;
    drone.arm().await?;
    println!("[OK] Armed");

    println!("[..] Commanding takeoff...");
    drone.takeoff(10.0).await?;
    println!("[OK] Drone at mission altitude");

    let detections_topic = drone.get_sensor_topic(&config.radar_name, "radar_detections");
    let tracks_topic = drone.get_sensor_topic(&config.radar_name, "radar_tracks");

    println!("[INFO] Subscribing to Radar detections: {detections_topic}");
    let mut det_sub = client.subscribe(&detections_topic).await?;

    println!("[INFO] Subscribing to Radar tracks: {tracks_topic}");
    let mut track_sub = client.subscribe(&tracks_topic).await?;

    let mut detection_frames = 0;
    let mut track_frames = 0;
    let timeout = Duration::from_secs(10);
    let deadline = tokio::time::Instant::now() + timeout;

    while (detection_frames < 3 || track_frames < 3) && tokio::time::Instant::now() < deadline {
        tokio::select! {
            res = det_sub.recv() => {
                if let Ok(msg_bytes) = res {
                    detection_frames += 1;
                    let count = count_msgpack_field(&msg_bytes, "radar_detections");
                    println!(
                        "[OK] Radar detections frame {} received: {} targets ({} bytes)",
                        detection_frames, count, msg_bytes.len()
                    );
                }
            }
            res = track_sub.recv() => {
                if let Ok(msg_bytes) = res {
                    track_frames += 1;
                    let count = count_msgpack_field(&msg_bytes, "radar_tracks");
                    println!(
                        "[OK] Radar tracks frame {} received: {} tracks ({} bytes)",
                        track_frames, count, msg_bytes.len()
                    );
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(100)) => {}
        }
    }

    println!("[..] Unsubscribing from Radar topics...");
    let _ = client.unsubscribe(&detections_topic).await;
    let _ = client.unsubscribe(&tracks_topic).await;

    println!("[..] Commanding landing...");
    drone.land(10.0).await?;
    println!("[OK] Drone landed");

    drone.disarm().await?;
    drone.disable_api_control().await?;

    println!("[PASS] User Radar scenario completed successfully!");
    Ok(())
}
