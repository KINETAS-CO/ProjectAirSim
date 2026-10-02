//! Example: World Debug Plots scenario matching Python `debug_utils.py` and C++ debug methods.
//!
//! Demonstrates debug points, arrows, solid lines, dashed lines, strings, transforms,
//! voxel grid extraction, and `.binvox` file export.
//!
//! Run with:
//! ```bash
//! cargo run --example world_debug_plots
//! ```

use std::path::Path;
use std::time::Duration;
use projectairsim::{Client, ColorRGBA, Pose, Quaternion, Vector3, World};

struct Config {
    sim_host: String,
    sim_config: String,
    scene_file: String,
}

impl Config {
    fn parse_args() -> Option<Self> {
        let mut sim_host = "127.0.0.1".to_string();
        let mut sim_config = "client/python/example_user_scripts/sim_config".to_string();
        let mut scene_file = "scene_basic_drone.jsonc".to_string();

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
                    println!("Usage: world_debug_plots [--simhost <host>] [--simconfig <dir>] [--scene <file>]");
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

    println!("[..] Plotting persistent red 3D points in simulation world...");
    let points: Vec<Vector3> = (0..20)
        .map(|i| {
            let t = i as f64 / 19.0;
            Vector3::new(-10.0 * t, -20.0 * t, -5.0)
        })
        .collect();

    world
        .plot_debug_points(&points, ColorRGBA::RED, 10.0, 10.0, true)
        .await?;
    println!("[OK] 20 points plotted");

    tokio::time::sleep(Duration::from_millis(500)).await;

    println!("[..] Flushing persistent markers from viewport...");
    world.flush_persistent_markers().await?;
    println!("[OK] Markers flushed");

    println!("[..] Plotting magenta debug 3D arrows...");
    let points_start: Vec<Vector3> = (0..10)
        .map(|i| {
            let t = i as f64 / 9.0;
            Vector3::new(10.0 * t, 0.0, -3.0 - 7.0 * t)
        })
        .collect();
    let points_end: Vec<Vector3> = (0..10)
        .map(|i| {
            let t = i as f64 / 9.0;
            Vector3::new(10.0 * t, 10.0 + 10.0 * t, -5.0 - 3.0 * t)
        })
        .collect();

    world
        .plot_debug_arrows(
            &points_start,
            &points_end,
            ColorRGBA::MAGENTA,
            3.0,
            15.0,
            10.0,
            false,
        )
        .await?;
    println!("[OK] 10 arrows plotted");

    println!("[..] Plotting solid red line strip...");
    world
        .plot_debug_solid_line(&points[..10], ColorRGBA::RED, 5.0, 10.0, false)
        .await?;
    println!("[OK] Solid line plotted");

    println!("[..] Plotting green dashed line segments...");
    world
        .plot_debug_dashed_line(&points[10..], ColorRGBA::GREEN, 5.0, 10.0, false)
        .await?;
    println!("[OK] Dashed line plotted");

    println!("[..] Plotting debug text strings at target waypoints...");
    let strings = vec![
        "Waypoint Alpha".to_string(),
        "Waypoint Beta".to_string(),
        "Waypoint Gamma".to_string(),
    ];
    let text_positions = vec![
        Vector3::new(0.0, 0.0, -5.0),
        Vector3::new(5.0, 5.0, -5.0),
        Vector3::new(10.0, 10.0, -5.0),
    ];
    world
        .plot_debug_strings(&strings, &text_positions, 1.5, ColorRGBA::YELLOW, 10.0)
        .await?;
    println!("[OK] Debug strings plotted");

    println!("[..] Plotting debug coordinate transforms triads...");
    let poses = vec![
        Pose::new(Vector3::new(2.0, 0.0, -2.0), Quaternion::default()),
        Pose::new(Vector3::new(4.0, 0.0, -2.0), Quaternion::default()),
    ];
    world
        .plot_debug_transforms(&poses, 3.0, 5.0, 10.0, false)
        .await?;
    println!("[OK] Coordinate frame transforms plotted");

    println!("[..] Extracting 3D voxel occupancy grid from scene geometry...");
    let voxel_center = Pose::new(Vector3::new(0.0, 0.0, -2.0), Quaternion::default());
    let binvox_path = "target/world_debug.binvox";
    let voxels = world
        .create_voxel_grid(
            voxel_center,
            10,
            10,
            5,
            0.5,
            Vec::new(),
            true,
            Some(binvox_path),
        )
        .await?;
    println!(
        "[OK] Voxel grid extracted ({} total cells). Exported to {}",
        voxels.len(),
        binvox_path
    );

    println!("[PASS] World debug plots scenario completed successfully!");
    Ok(())
}
