# ProjectAirSim Rust Client (`projectairsim`)

A high-performance, dual-mode (`async` and `sync`) native Rust client library for the [ProjectAirSim](https://github.com/microsoft/ProjectAirSim) simulation platform.

---

## Features

- **Dual-Mode Engine via Cargo Features**:
  - **`async` (Default)**: Tokio-native asynchronous actor architecture with typed RPC methods and broadcast streams for real-time telemetry topics.
  - **`sync`**: Blocking interface mirroring the C++ ProjectAirSim API with `AsyncResult<T>` handle mechanics for waiting, timeouts, and completion polling.
  - **`full`**: Enables both `async` and `sync` modules simultaneously.
- **Native NNG Transport**: Powered by `nng-rs` (C `libnng` binding) matching the simulation server's NNG engine bug-for-bug.
  - Port `8990`: Req0 RPC Services (`RequestEnvelope` MessagePack/JSON wire framing).
  - Port `8989`: Pair0 Topics Pub/Sub (`TopicFrame` 3-tuple MessagePack arrays).
- **First-Class Vehicle & World APIs**:
  - Scene configuration parser (`World`) with clock stepping (`pause`, `resume`, `step`).
  - Multirotor flight control (`Drone`): API control, arming, takeoff, landing, velocity vectors, waypoints.
  - Sensor queries and camera image capture.
- **Mock Transport**: Built-in mock transport and unit test suite that can be run hermetically without a running simulator.

---

## Installation

Add `projectairsim` to your `Cargo.toml`:

```toml
[dependencies]
# Tokio async mode (default)
projectairsim = { path = "path/to/ProjectAirSim/client/rust" }

# Or for blocking mode:
# projectairsim = { path = "path/to/ProjectAirSim/client/rust", default-features = false, features = ["sync"] }
```

---

## Usage Examples

### 1. Asynchronous (Tokio) Client

```rust
use projectairsim::{Client, World, Pose};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Connect to simulation server (default: 8989 topics, 8990 services)
    let client = Client::connect("127.0.0.1").await?;

    // Initialize world scene
    let world = World::new(client.clone(), None).await?;
    let drone = world.get_drone("Drone1");

    // Flight mission sequence
    drone.enable_api_control().await?;
    drone.arm().await?;
    drone.takeoff(15.0).await?;

    // Subscribe to streaming telemetry
    let mut pose_sub = drone.subscribe_telemetry("robot_info/ground_truth_pose").await?;
    tokio::spawn(async move {
        while let Ok(pose) = pose_sub.recv_typed::<Pose>().await {
            println!("Position: [N: {:.2}, E: {:.2}, D: {:.2}]", pose.position.x, pose.position.y, pose.position.z);
        }
    });

    // Velocity flight command
    drone.move_by_velocity(3.0, 0.0, 0.0, 3.0).await?;

    // Land and disarm
    drone.land(15.0).await?;
    drone.disarm().await?;

    Ok(())
}
```

### 2. Synchronous (Blocking) Client with `AsyncResult`

```rust
use std::time::Duration;
use projectairsim::blocking::{AsyncResult, Client, World};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Connect blocking client
    let client = Client::connect("127.0.0.1")?;
    let world = World::new(client.clone(), None)?;
    let drone = world.get_drone("Drone1");

    drone.enable_api_control()?;
    drone.arm()?;

    // C++-style asynchronous handle
    let mut takeoff_ar: AsyncResult<bool> = drone.takeoff_async(15.0);

    // Poll status while waiting
    while !takeoff_ar.is_done() {
        println!("Waiting for takeoff...");
        let _ = takeoff_ar.wait_timeout(Duration::from_millis(500));
    }
    println!("Takeoff finished: {}", takeoff_ar.get_result()?);

    drone.move_by_velocity(2.0, 0.0, 0.0, 2.0)?;
    drone.land(15.0)?;
    drone.disarm()?;

    Ok(())
}
```

---

## Running the Examples

```bash
# Run async example
cargo run --example async_drone_flight

# Run sync example
cargo run --example sync_drone_flight --features sync
```

---

## Testing

```bash
# Run tests with default async feature
cargo test

# Run tests in blocking mode
cargo test --no-default-features --features sync

# Run all tests across both modes
cargo test --all-features
```
