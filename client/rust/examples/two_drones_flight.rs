//! Example: Two Drones Flight scenario matching C++ `RunTwoDrones` and Python `two_drones.py`.
//!
//! Demonstrates simultaneous multi-vehicle coordination with two drones in a single simulation world.
//!
//! Run with:
//! ```bash
//! cargo run --example two_drones_flight
//! ```

use projectairsim::{Client, World};
use std::path::Path;
use tracing::info;

struct Config {
    sim_host: String,
    sim_config: String,
    scene_file: String,
}

impl Config {
    fn parse_args() -> Option<Self> {
        let mut sim_host = "127.0.0.1".to_string();
        let mut sim_config = "client/python/example_user_scripts/sim_config".to_string();
        let mut scene_file = "scene_two_drones.jsonc".to_string();

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
                "-h" | "--help" | "/?" => {
                    println!("Usage: two_drones_flight [--simhost <host>] [--simconfig <dir>] [--scene <file>]");
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
        })
    }
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

    let drone1 = world.get_drone("Drone1");
    let drone2 = world.get_drone("Drone2");
    info!("Handles created for '{}' and '{}'", drone1.name(), drone2.name());

    info!("Enabling API control and arming both drones...");
    drone1.enable_api_control().await?;
    drone2.enable_api_control().await?;
    drone1.arm().await?;
    drone2.arm().await?;
    info!("Both drones armed");

    info!("Commanding concurrent takeoff for both drones...");
    let takeoff1 = drone1.takeoff(15.0);
    let takeoff2 = drone2.takeoff(15.0);
    let (t1_res, t2_res) = tokio::join!(takeoff1, takeoff2);
    t1_res?;
    t2_res?;
    info!("Both drones in hover");

    info!("Executing concurrent velocity maneuvers (Drone1 North, Drone2 East)...");
    let move1 = drone1.move_by_velocity(1.0, 0.0, 0.0, 2.5);
    let move2 = drone2.move_by_velocity(0.0, 1.0, 0.0, 2.5);
    let (m1_res, m2_res) = tokio::join!(move1, move2);
    m1_res?;
    m2_res?;
    info!("Concurrent flight maneuvers completed");

    let k1 = drone1.get_ground_truth_kinematics().await?;
    let k2 = drone2.get_ground_truth_kinematics().await?;
    info!("Drone1 kinematics: {}", serde_json::to_string(&k1)?);
    info!("Drone2 kinematics: {}", serde_json::to_string(&k2)?);

    info!("Commanding concurrent landing...");
    let land1 = drone1.land(15.0);
    let land2 = drone2.land(15.0);
    let (l1_res, l2_res) = tokio::join!(land1, land2);
    l1_res?;
    l2_res?;
    info!("Both drones landed");

    drone1.disarm().await?;
    drone2.disarm().await?;
    drone1.disable_api_control().await?;
    drone2.disable_api_control().await?;

    info!("Two Drones Flight scenario completed successfully!");
    Ok(())
}
