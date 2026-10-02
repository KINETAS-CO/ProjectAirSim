//! Example: Synchronous (blocking) drone control in ProjectAirSim.
//!
//! Run with:
//! ```bash
//! cargo run --example sync_drone_flight --features sync
//! ```

use projectairsim::blocking::{AsyncResult, Client, World};
use std::time::Duration;
use tracing::info;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    projectairsim::init_logging();

    info!("Connecting (blocking) to ProjectAirSim simulation server...");
    let client = Client::connect("127.0.0.1")?;

    info!("Server ping: {}", client.ping()?);

    let world = World::new(client.clone(), None)?;
    let drone = world.get_drone("Drone1");

    info!("Enabling API control...");
    drone.enable_api_control()?;

    info!("Arming...");
    drone.arm()?;

    info!("Initiating takeoff with C++-style AsyncResult handle...");
    let mut takeoff_ar: AsyncResult<bool> = drone.takeoff_async(15.0);

    // Poll status while waiting
    while !takeoff_ar.is_done() {
        info!("Waiting for takeoff to complete...");
        let _ = takeoff_ar.wait_timeout(Duration::from_millis(500));
    }
    info!("Takeoff finished with status: {}", takeoff_ar.get_result()?);

    info!("Commanding velocity move: 2 m/s North for 2 seconds...");
    drone.move_by_velocity(2.0, 0.0, 0.0, 2.0)?;

    info!("Landing...");
    drone.land(15.0)?;

    info!("Disarming...");
    drone.disarm()?;

    info!("Blocking flight mission complete!");
    Ok(())
}
