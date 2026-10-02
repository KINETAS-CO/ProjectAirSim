//! Example: User Lidar Scenario matching C++ `UserLidarScenario`.
//!
//! Demonstrates drone takeoff, subscribing to real-time lidar point cloud streaming,
//! and parsing point clouds from MessagePack frames.
//!
//! Run with:
//! ```bash
//! cargo run --example user_lidar_scenario
//! ```

use projectairsim::{Client, World};
use std::path::Path;
use std::time::Duration;
use tracing::{info, warn};

struct Config {
    sim_host: String,
    sim_config: String,
    scene_file: String,
    vehicle_name: String,
    lidar_name: String,
}

impl Config {
    fn parse_args() -> Option<Self> {
        let mut sim_host = "127.0.0.1".to_string();
        let mut sim_config = "client/python/example_user_scripts/sim_config".to_string();
        let mut scene_file = "scene_lidar_drone.jsonc".to_string();
        let mut vehicle_name = "Drone1".to_string();
        let mut lidar_name = "lidar1".to_string();

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
                "--lidar" => {
                    lidar_name = args.next().expect("Missing value for --lidar");
                }
                "-h" | "--help" | "/?" => {
                    println!("Usage: user_lidar_scenario [--simhost <host>] [--simconfig <dir>] [--scene <file>] [--vehicle <name>] [--lidar <name>]");
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
            lidar_name,
        })
    }
}

fn count_lidar_points(payload: &[u8]) -> usize {
    if let Ok(value) = rmp_serde::from_slice::<serde_json::Value>(payload) {
        if let Some(pc) = value.get("point_cloud").and_then(|v| v.as_array()) {
            return pc.len() / 3;
        }
    }
    0
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    projectairsim::init_logging();

    let config = match Config::parse_args() {
        Some(cfg) => cfg,
        None => return Ok(()),
    };

    info!("Connecting to ProjectAirSim at {}:8990...", config.sim_host);
    let client = Client::connect(&config.sim_host).await?;

    let scene_path = Path::new(&config.sim_config).join(&config.scene_file);
    let world = if scene_path.exists() {
        info!("Loading scene from {}", scene_path.display());
        World::new(client.clone(), Some(scene_path.to_str().unwrap())).await?
    } else {
        info!("Attaching to existing simulation scene...");
        World::new(client.clone(), None).await?
    };

    let drone = world.get_drone(&config.vehicle_name);
    info!("Connected to drone '{}'", drone.name());

    info!("Enabling API control and arming...");
    drone.enable_api_control().await?;
    drone.arm().await?;
    info!("Armed");

    info!("Commanding takeoff...");
    drone.takeoff(10.0).await?;
    info!("Drone at mission altitude");

    let lidar_topic = drone.get_sensor_topic(&config.lidar_name, "lidar");
    info!("Subscribing to Lidar topic: {lidar_topic}");
    let mut lidar_sub = client.subscribe(&lidar_topic).await?;

    let mut frames_received = 0;
    let timeout = Duration::from_secs(8);
    let deadline = tokio::time::Instant::now() + timeout;

    while frames_received < 3 && tokio::time::Instant::now() < deadline {
        let remaining = deadline.saturating_duration_since(tokio::time::Instant::now());
        match tokio::time::timeout(remaining, lidar_sub.recv()).await {
            Ok(Ok(msg_bytes)) => {
                frames_received += 1;
                let points = count_lidar_points(&msg_bytes);
                info!(
                    "Lidar frame {} received: {} points ({} raw bytes)",
                    frames_received,
                    points,
                    msg_bytes.len()
                );
            }
            Ok(Err(e)) => {
                warn!("Lidar subscription error: {e}");
                break;
            }
            Err(_) => {
                warn!("Timeout waiting for next Lidar frame");
                break;
            }
        }
    }

    info!("Unsubscribing from Lidar topic...");
    let _ = client.unsubscribe(&lidar_topic).await;

    info!("Commanding landing...");
    drone.land(10.0).await?;
    info!("Drone landed");

    drone.disarm().await?;
    drone.disable_api_control().await?;

    info!("User Lidar scenario completed successfully!");
    Ok(())
}
