//! Example: Synchronous (blocking) drone control in ProjectAirSim.
//!
//! Run with:
//! ```bash
//! cargo run --example sync_drone_flight --features sync
//! ```

use std::time::Duration;
use projectairsim::blocking::{AsyncResult, Client, World};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("Connecting (blocking) to ProjectAirSim simulation server...");
    let client = Client::connect("127.0.0.1")?;

    println!("Server ping: {}", client.ping()?);

    let world = World::new(client.clone(), None)?;
    let drone = world.get_drone("Drone1");

    println!("Enabling API control...");
    drone.enable_api_control()?;

    println!("Arming...");
    drone.arm()?;

    println!("Initiating takeoff with C++-style AsyncResult handle...");
    let mut takeoff_ar: AsyncResult<bool> = drone.takeoff_async(15.0);

    // Poll status while waiting
    while !takeoff_ar.is_done() {
        println!("Waiting for takeoff to complete...");
        let _ = takeoff_ar.wait_timeout(Duration::from_millis(500));
    }
    println!("Takeoff finished with status: {}", takeoff_ar.get_result()?);

    println!("Commanding velocity move: 2 m/s North for 2 seconds...");
    drone.move_by_velocity(2.0, 0.0, 0.0, 2.0)?;

    println!("Landing...");
    drone.land(15.0)?;

    println!("Disarming...");
    drone.disarm()?;

    println!("Blocking flight mission complete!");
    Ok(())
}
