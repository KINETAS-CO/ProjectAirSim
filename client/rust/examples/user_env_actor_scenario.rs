//! Example: User Environment Actor Scenario matching C++ `UserEnvActorScenario`.
//!
//! Demonstrates trajectory playback and articulated link rotations on scenery actors.
//!
//! Run with:
//! ```bash
//! cargo run --example user_env_actor_scenario
//! ```

use projectairsim::{Client, NEDTrajectory, World};
use std::collections::HashMap;
use std::f32::consts::PI;
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
        let mut scene_file = "scene_env_actor.jsonc".to_string();

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
                    println!("Usage: user_env_actor_scenario [--simhost <host>] [--simconfig <dir>] [--scene <file>]");
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

fn generate_loop_coords(
    radius: f32,
    center_y: f32,
    center_z: f32,
    num_points: usize,
) -> (Vec<f32>, Vec<f32>) {
    let angle_step = 2.0 * PI / num_points as f32;
    let mut angles = Vec::with_capacity(num_points + num_points / 2 + 1);

    for idx in 0..=num_points {
        angles.push(-PI + angle_step * idx as f32);
    }
    let angle_start = *angles.last().unwrap();
    for idx in 1..=(num_points / 2) {
        angles.push(angle_start + angle_step * idx as f32);
    }

    let mut y_coords = Vec::with_capacity(angles.len());
    let mut z_coords = Vec::with_capacity(angles.len());
    for angle in angles {
        y_coords.push(center_y + radius * angle.cos());
        z_coords.push(center_z + radius * angle.sin());
    }
    (y_coords, z_coords)
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

    let tiltrotor = world.get_env_actor("TiltrotorWithConfigTrajOffset");
    let actor_api = world.get_env_actor("ActorWithApiTraj");
    let actor_api_offset = world.get_env_actor("ActorWithApiTrajAndOffset");

    info!("Articulating tiltrotor shroud rotation angles to 90 degrees...");
    let mut shroud_angles = HashMap::new();
    shroud_angles.insert("Shroud_FL".to_string(), 90.0f32);
    shroud_angles.insert("Shroud_RL".to_string(), 90.0f32);
    shroud_angles.insert("Shroud_FR".to_string(), 90.0f32);
    shroud_angles.insert("Shroud_RR".to_string(), 90.0f32);
    tiltrotor.set_link_rotation_angles(&shroud_angles).await?;
    info!("Articulated links positioned");

    info!("Setting TiltrotorWithConfigTrajOffset trajectory with spatial offset...");
    tiltrotor
        .set_trajectory("right_and_descend_config", true, 3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.57)
        .await?;
    info!("Configured trajectory assigned to tiltrotor");

    info!("Generating programmatic 3D loop-the-loop trajectory coordinates...");
    let (y_loop, z_loop) = generate_loop_coords(3.0, 10.0, -6.0, 20);

    let mut time_sec = vec![4.0f32];
    let mut pose_x = vec![1.0f32];
    let mut pose_y = vec![0.0f32];
    let mut pose_z = vec![-6.0f32];

    let step_size = 0.2f32;
    let loop_start = *time_sec.last().unwrap() + 2.0;
    for (idx, (&y, &z)) in y_loop.iter().zip(z_loop.iter()).enumerate() {
        time_sec.push(loop_start + step_size * (idx + 1) as f32);
        pose_x.push(*pose_x.last().unwrap());
        pose_y.push(y);
        pose_z.push(z);
    }
    time_sec.push(*time_sec.last().unwrap() + 2.0);
    pose_x.push(*pose_x.last().unwrap());
    pose_y.push(*pose_y.last().unwrap() + 3.0);
    pose_z.push(*pose_z.last().unwrap());

    info!("Importing generated NED trajectory into world simulation engine...");
    let traj = NEDTrajectory {
        traj_name: "looptheloop".to_string(),
        time: time_sec,
        pose_x,
        pose_y,
        pose_z,
        ..Default::default()
    };
    world.import_ned_trajectory(traj).await?;
    info!("Trajectory 'looptheloop' imported");

    info!("Assigning 'looptheloop' trajectory to environment actors...");
    actor_api
        .set_trajectory("looptheloop", true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .await?;
    actor_api_offset
        .set_trajectory("looptheloop", true, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .await?;
    info!("Trajectories bound to actors");

    info!("User Environment Actor scenario completed successfully!");
    Ok(())
}
