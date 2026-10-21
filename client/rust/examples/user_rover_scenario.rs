//! Example: User Rover Scenario matching C++ `UserRoverScenario`.
//!
//! Demonstrates arming, forward/reverse maneuvers, steering commands, and braking on a ground rover.
//!
//! Run with:
//! ```bash
//! cargo run --example user_rover_scenario
//! ```

use projectairsim::{Client, World};
use std::path::Path;
use std::time::Duration;
use tracing::info;

struct Config {
    sim_host: String,
    sim_config: String,
    scene_file: String,
    vehicle_name: String,
}

impl Config {
    fn parse_args() -> Option<Self> {
        let mut sim_host = "127.0.0.1".to_string();
        let mut sim_config = "client/python/example_user_scripts/sim_config".to_string();
        let mut scene_file = "scene_basic_rover.jsonc".to_string();
        let mut vehicle_name = "Rover1".to_string();

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
                "-h" | "--help" | "/?" => {
                    println!("Usage: user_rover_scenario [--simhost <host>] [--simconfig <dir>] [--scene <file>] [--vehicle <name>]");
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

    let rover = world.get_rover(&config.vehicle_name);
    info!("Connected to rover '{}'", rover.name());

    info!("Enabling API control...");
    let api_ok = rover.enable_api_control().await?;
    info!("API control enabled: {}", api_ok);

    info!("Arming rover motors...");
    let armed = rover.arm().await?;
    info!("Armed: {}", armed);

    info!("Setting rover controls to zero (idling for 2s)...");
    rover.set_rover_controls(0.0, 0.0, 0.0).await?;
    tokio::time::sleep(Duration::from_secs(2)).await;

    info!("Driving forward and right (engine=0.5, steering=1.0)...");
    rover.set_rover_controls(0.5, 1.0, 0.0).await?;
    tokio::time::sleep(Duration::from_secs(3)).await;

    info!("Driving forward and left (engine=0.5, steering=-1.0)...");
    rover.set_rover_controls(0.5, -1.0, 0.0).await?;
    tokio::time::sleep(Duration::from_secs(3)).await;

    info!("Applying rover brakes...");
    rover.set_rover_controls(0.0, 0.0, 0.5).await?;
    tokio::time::sleep(Duration::from_secs(2)).await;

    info!("Driving backward and right (engine=-0.5, steering=1.0)...");
    rover.set_rover_controls(-0.5, 1.0, 0.0).await?;
    tokio::time::sleep(Duration::from_secs(3)).await;

    info!("Driving backward and left (engine=-0.5, steering=-1.0)...");
    rover.set_rover_controls(-0.5, -1.0, 0.0).await?;
    tokio::time::sleep(Duration::from_secs(3)).await;

    info!("Applying rover brakes...");
    rover.set_rover_controls(0.0, 0.0, 0.5).await?;
    tokio::time::sleep(Duration::from_secs(2)).await;

    info!("Neutralizing controls...");
    rover.set_rover_controls(0.0, 0.0, 0.0).await?;

    info!("Disarming motors...");
    let disarmed = rover.disarm().await?;
    info!("Disarmed: {}", disarmed);

    info!("Disabling API control...");
    let disabled = rover.disable_api_control().await?;
    info!("API control disabled: {}", disabled);

    info!("User Rover scenario completed successfully!");
    Ok(())
}
