//! Example: Asynchronous drone control and telemetry in ProjectAirSim.
//!
//! Run with:
//! ```bash
//! cargo run --example async_drone_flight
//! ```

use projectairsim::{Client, Pose, World};
use tracing::info;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    projectairsim::init_logging();

    info!("Connecting to ProjectAirSim simulation server...");
    let client = Client::connect("127.0.0.1").await?;

    info!("Server ping: {}", client.ping().await?);

    // Initialize the world (reflecting the currently loaded simulation scene)
    let world = World::new(client.clone(), None).await?;
    info!("Active scene parent topic: {}", world.parent_topic());

    // Connect to vehicle Drone1
    let drone = world.get_drone("Drone1");

    info!("Requesting API control...");
    drone.enable_api_control().await?;

    info!("Arming vehicle...");
    drone.arm().await?;

    info!("Commanding takeoff...");
    drone.takeoff(15.0).await?;

    info!("Subscribing to real-time ground truth pose telemetry...");
    let mut pose_sub = drone
        .subscribe_telemetry("robot_info/ground_truth_pose")
        .await?;

    // Spawn telemetry reader task
    let telemetry_task = tokio::spawn(async move {
        for _ in 0..10 {
            if let Ok(pose) = pose_sub.recv_typed::<Pose>().await {
                info!(
                    "Telemetry -> Position: [N: {:.2}, E: {:.2}, D: {:.2}]",
                    pose.position.x, pose.position.y, pose.position.z
                );
            }
        }
    });

    info!("Flying forward at 3.0 m/s for 3 seconds...");
    drone.move_by_velocity(3.0, 0.0, 0.0, 3.0).await?;

    let _ = telemetry_task.await;

    info!("Landing vehicle...");
    drone.land(15.0).await?;

    info!("Disarming...");
    drone.disarm().await?;

    info!("Flight mission complete!");
    Ok(())
}
