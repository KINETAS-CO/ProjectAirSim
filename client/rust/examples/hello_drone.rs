//! Example: Hello Drone scenario matching C++ `HelloDrone`.
//!
//! Demonstrates arming, takeoff, velocity climbs, landing, and disarming a quadrotor.
//!
//! Run with:
//! ```bash
//! cargo run --example hello_drone
//! ```

use std::path::Path;
use projectairsim::{Client, LandedState, World};

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
        let mut scene_file = "scene_basic_drone.jsonc".to_string();
        let mut vehicle_name = "Drone1".to_string();

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
                    println!("Usage: hello_drone [--simhost <host>] [--simconfig <dir>] [--scene <file>] [--vehicle <name>]");
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
    println!("[OK] Connected to vehicle '{}'", drone.name());

    println!("[..] Enabling API control...");
    drone.enable_api_control().await?;

    println!("[..] Arming vehicle motors...");
    drone.arm().await?;

    let ready_state = drone.get_ready_state().await?;
    println!(
        "[OK] Ready state: is_ready={}, message='{}'",
        ready_state.is_ready, ready_state.message
    );

    println!("[..] Commanding takeoff (altitude hold)...");
    drone.takeoff(15.0).await?;
    println!("[OK] Takeoff completed");

    println!("[..] Moving up (NED vz = -1.0 m/s for 4.0 seconds)...");
    drone.move_by_velocity(0.0, 0.0, -1.0, 4.0).await?;
    println!("[OK] Climb completed");

    println!("[..] Commanding landing...");
    drone.land(15.0).await?;
    let landed_state = drone.get_landed_state().await?;
    println!(
        "[OK] Landed state: {}",
        match landed_state {
            LandedState::Landed => "landed",
            LandedState::Airborne => "airborne",
            LandedState::Unknown => "unknown",
        }
    );

    println!("[..] Disarming motors...");
    drone.disarm().await?;

    println!("[..] Disabling API control...");
    drone.disable_api_control().await?;

    println!("[PASS] Hello Drone scenario completed successfully!");
    Ok(())
}
