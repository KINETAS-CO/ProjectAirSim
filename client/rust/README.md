# ProjectAirSim Rust Client

The `projectairsim` library provides a Rust interface for the ProjectAirSim simulation platform.
The library communicates with the simulation server through the Nanomsg Next Generation (NNG) protocol.
You can use asynchronous Tokio tasks, or you can use synchronous blocking function calls.

## Features

The library includes these features:
- Asynchronous client with Tokio runtime support
- Synchronous client with blocking `AsyncResult` handles
- Flight controls for multirotor drones
- Drive and brake controls for wheeled vehicles and ground rovers
- Trajectory and articulation controls for environment actors
- Camera image capture for stationary sensors
- Real-time topic data streaming for lidar and radar sensors
- Debug visual markers and voxel grid export
- Structured logging with custom callback sinks

## Features in Cargo

The crate defines three features in `Cargo.toml`:
- `async`: This feature is the default. It enables the Tokio asynchronous client.
- `sync`: This feature enables the blocking client.
- `full`: This feature enables both asynchronous and synchronous interfaces.

## Installation

To add the library to a project, add this line to your `Cargo.toml` file:

```toml
[dependencies]
projectairsim = { path = "path/to/ProjectAirSim/client/rust" }
```

If you need blocking calls, disable default features and enable `sync`:

```toml
[dependencies]
projectairsim = { path = "path/to/ProjectAirSim/client/rust", default-features = false, features = ["sync"] }
```

## Examples

The repository provides eleven example applications in the `examples/` directory.
If you want to run an example, use the `cargo run` command.

- `hello_drone`: Quadrotor takeoff, climb, landing, and custom log sink
- `hello_wheeled_vehicle`: Ground vehicle throttle, steering, and braking
- `user_rover_scenario`: Ground rover directional maneuvers
- `user_env_actor_scenario`: Scenery actor trajectories and articulated links
- `user_static_sensor_scenario`: Camera images from stationary sensor towers
- `user_lidar_scenario`: Real-time lidar point cloud streaming
- `user_radar_scenario`: Streaming radar detections and tracks
- `two_drones_flight`: Concurrent flight control of two drones
- `world_debug_plots`: 3D debug arrows, points, lines, and voxel export
- `async_drone_flight`: Asynchronous drone telemetry streaming
- `sync_drone_flight`: Synchronous blocking drone flight

Run an example with this command:

```bash
cargo run --example hello_drone
```

If you run `sync_drone_flight`, include the `sync` feature flag:

```bash
cargo run --example sync_drone_flight --features sync
```

## Build and Test

You can build and test the client without a running simulation server.
The repository includes mock transports for hermetic tests.

1. To build the client library, run `cargo build --all-features`.
2. To run the test suite, run `./build_rust_client.sh debug --tests`.
3. To test code cleanliness, run `cargo clippy --all-targets --all-features -- -D warnings`.
