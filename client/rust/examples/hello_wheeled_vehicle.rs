//! Example: Hello Wheeled Vehicle scenario matching C++ `HelloWheeledVehicle`.
//!
//! Demonstrates throttle, steering, and braking on ground wheeled vehicles.
//!
//! Run with:
//! ```bash
//! cargo run --example hello_wheeled_vehicle
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
        let mut scene_file = "scene_basic_wheeled_vehicle.jsonc".to_string();
        let mut vehicle_name = "Vehicle1".to_string();

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
                    println!("Usage: hello_wheeled_vehicle [--simhost <host>] [--simconfig <dir>] [--scene <file>] [--vehicle <name>]");
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

fn extract_position(kinematics: &serde_json::Value) -> Option<(f64, f64, f64)> {
    let pos = kinematics.get("pose")?.get("position")?;
    let x = pos.get("x")?.as_f64()?;
    let y = pos.get("y")?.as_f64()?;
    let z = pos.get("z")?.as_f64()?;
    Some((x, y, z))
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

    let vehicle = world.get_wheeled_vehicle(&config.vehicle_name);
    info!("Connected to wheeled vehicle '{}'", vehicle.name());

    // Allow physics settling
    tokio::time::sleep(Duration::from_secs(2)).await;

    let init_kinematics = vehicle.get_ground_truth_kinematics().await?;
    let init_pos = extract_position(&init_kinematics).unwrap_or((0.0, 0.0, 0.0));
    info!(
        "Initial position: X={:.2}, Y={:.2}, Z={:.2}",
        init_pos.0, init_pos.1, init_pos.2
    );

    info!("Driving with throttle=0.7 and steering=0.45 for 5 seconds...");
    vehicle.set_controls(0.7, 0.45, 0.0).await?;

    tokio::time::sleep(Duration::from_secs(5)).await;

    let final_kinematics = vehicle.get_ground_truth_kinematics().await?;
    let final_pos = extract_position(&final_kinematics).unwrap_or((0.0, 0.0, 0.0));
    info!(
        "Final position: X={:.2}, Y={:.2}, Z={:.2}",
        final_pos.0, final_pos.1, final_pos.2
    );

    let dx = final_pos.0 - init_pos.0;
    let dy = final_pos.1 - init_pos.1;
    let distance = (dx * dx + dy * dy).sqrt();
    info!("Total horizontal distance traveled: {:.2} m", distance);

    info!("Applying full brakes...");
    vehicle.set_controls(0.0, 0.0, 1.0).await?;
    tokio::time::sleep(Duration::from_secs(1)).await;

    info!("Neutralizing controls...");
    vehicle.set_controls(0.0, 0.0, 0.0).await?;

    info!("Wheeled vehicle scenario completed successfully!");
    Ok(())
}
