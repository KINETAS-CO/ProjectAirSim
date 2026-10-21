//! Example: User Static Sensor Scenario matching C++ `UserStaticSensorScenario`.
//!
//! Demonstrates image collection and metadata retrieval from fixed sensor towers and stationary camera actors.
//!
//! Run with:
//! ```bash
//! cargo run --example user_static_sensor_scenario
//! ```

use projectairsim::{Client, ImageType, World};
use std::path::Path;
use tracing::info;

struct Config {
    sim_host: String,
    sim_config: String,
    scene_file: String,
    actor_name: String,
    camera_name: String,
}

impl Config {
    fn parse_args() -> Option<Self> {
        let mut sim_host = "127.0.0.1".to_string();
        let mut sim_config = "client/python/example_user_scripts/sim_config".to_string();
        let mut scene_file = "scene_computer_vision.jsonc".to_string();
        let mut actor_name = "CV".to_string();
        let mut camera_name = "Camera".to_string();

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
                "--actor" => {
                    actor_name = args.next().expect("Missing value for --actor");
                }
                "--camera" => {
                    camera_name = args.next().expect("Missing value for --camera");
                }
                "-h" | "--help" | "/?" => {
                    println!("Usage: user_static_sensor_scenario [--simhost <host>] [--simconfig <dir>] [--scene <file>] [--actor <name>] [--camera <name>]");
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
            actor_name,
            camera_name,
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

    let sensor_actor = world.get_static_sensor(&config.actor_name);
    info!("Connected to static sensor actor '{}'", sensor_actor.name());

    info!(
        "Capturing Scene images from camera '{}'...",
        config.camera_name
    );
    let responses = sensor_actor
        .get_camera_images(&config.camera_name, &[ImageType::Scene])
        .await?;

    info!("Retrieved {} image response frame(s)", responses.len());
    for (i, img) in responses.iter().enumerate() {
        info!(
            "  Frame {}: camera='{}', width={}, height={}, bytes={}, message='{}'",
            i,
            img.camera_name,
            img.width,
            img.height,
            img.image_data_uint8.len(),
            img.message
        );
    }

    info!("Static sensor scenario completed successfully!");
    Ok(())
}
