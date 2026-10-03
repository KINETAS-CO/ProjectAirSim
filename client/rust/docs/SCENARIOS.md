# ProjectAirSim Rust Scenarios Guide

This document describes the example scenario applications provided with the Rust client library.
You can find the source code for each scenario in the `examples/` directory.

## Command Line Arguments

All scenario applications accept these standard command line arguments:
- `--simhost <host>`: IP address of the simulation server. The default is `127.0.0.1`.
- `--simconfig <dir>`: Directory containing scene configuration files.
- `--scene <file>`: Scene configuration file to load into the world.
- `--vehicle <name>`: Target vehicle name within the scene.

If you specify `-h` or `--help`, the program prints usage instructions.

## Hello Drone (`hello_drone.rs`)

This scenario matches the C++ `HelloDrone` application.
It demonstrates basic quadrotor flight operations.

### Operations in this Scenario

1. Registers a custom log sink to route messages.
2. Connects to the simulation server on port 8990.
3. Obtains a handle for vehicle `Drone1`.
4. Enables software API control and arms the motors.
5. Commands takeoff to hover altitude.
6. Commands vertical ascent at 1.0 meter per second for 4.0 seconds.
7. Commands landing until surface touchdown.
8. Disarms motors and releases software control.

### How to Operate

```bash
cargo run --example hello_drone
```

## Hello Wheeled Vehicle (`hello_wheeled_vehicle.rs`)

This scenario matches the C++ `HelloWheeledVehicle` application.
It demonstrates driving and braking operations on ground automobiles.

### Operations in this Scenario

1. Connects to the simulation server and loads `scene_basic_wheeled_vehicle.jsonc`.
2. Obtains a handle for vehicle `Vehicle1`.
3. Pauses for two seconds to allow physics settling.
4. Queries initial ground truth position.
5. Applies throttle of 0.7 and steering of 0.45 for 5.0 seconds.
6. Queries final position and calculates horizontal distance traveled.
7. Applies full brakes and then neutralizes controls.

### How to Operate

```bash
cargo run --example hello_wheeled_vehicle
```

## User Rover Scenario (`user_rover_scenario.rs`)

This scenario matches the C++ `UserRoverScenario` application.
It demonstrates directional maneuvers and braking on ground rovers.

### Operations in this Scenario

1. Connects to the simulation server and loads `scene_basic_rover.jsonc`.
2. Obtains a handle for rover `Rover1`.
3. Enables software control authority and arms the drive motors.
4. Commands forward drive with right steering.
5. Commands forward drive with left steering.
6. Applies rover brakes.
7. Commands reverse drive with right steering.
8. Commands reverse drive with left steering.
9. Applies brakes, disarms motors, and releases API control.

### How to Operate

```bash
cargo run --example user_rover_scenario
```

## User Environment Actor Scenario (`user_env_actor_scenario.rs`)

This scenario matches the C++ `UserEnvActorScenario` application.
It demonstrates trajectory assignment and articulated link rotations on scenery actors.

### Operations in this Scenario

1. Connects to the simulation server and loads `scene_env_actor.jsonc`.
2. Obtains handles for scenery actors.
3. Sets rotation angles of four tiltrotor shrouds to 90 degrees.
4. Assigns a configured trajectory with a spatial offset to the tiltrotor actor.
5. Generates a programmatic 3D loop-the-loop trajectory in coordinates.
6. Uploads the generated trajectory into the world simulation engine.
7. Binds the imported trajectory to environment actors.

### How to Operate

```bash
cargo run --example user_env_actor_scenario
```

## User Static Sensor Scenario (`user_static_sensor_scenario.rs`)

This scenario matches the C++ `UserStaticSensorScenario` application.
It demonstrates image capture from stationary camera towers.

### Operations in this Scenario

1. Connects to the simulation server and loads `scene_computer_vision.jsonc`.
2. Obtains a handle for static sensor actor `CameraTower1`.
3. Requests RGB scene image frames from mounted camera `Camera1`.
4. Receives raw image buffers and metadata from the simulation server.
5. Logs image dimensions, byte sizes, and frame statuses.

### How to Operate

```bash
cargo run --example user_static_sensor_scenario
```

## User Lidar Scenario (`user_lidar_scenario.rs`)

This scenario matches the C++ `UserLidarScenario` application.
It demonstrates real-time lidar point cloud streaming over topic channels.

### Operations in this Scenario

1. Connects to the simulation server and arms drone `Drone1`.
2. Commands takeoff to mission altitude.
3. Obtains the sensor topic name for lidar device `lidar1`.
4. Subscribes to the lidar topic on port 8989.
5. Receives MessagePack streaming frames and unpacks point count data.
6. Unsubscribes from the topic.
7. Commands landing and releases control.

### How to Operate

```bash
cargo run --example user_lidar_scenario
```

## User Radar Scenario (`user_radar_scenario.rs`)

This scenario matches the C++ `UserRadarScenario` application.
It demonstrates dual-topic streaming for radar detections and target tracks.

### Operations in this Scenario

1. Connects to the simulation server and takes off with `Drone1`.
2. Resolves topic names for `radar_detections` and `radar_tracks`.
3. Subscribes to both radar data streams concurrently.
4. Parses MessagePack detection targets and track state vectors.
5. Unsubscribes from both topics after receiving target data.
6. Commands landing and releases control.

### How to Operate

```bash
cargo run --example user_radar_scenario
```

## Two Drones Flight (`two_drones_flight.rs`)

This scenario matches the C++ `RunTwoDrones` application.
It demonstrates concurrent multi-aircraft control in one simulation scene.

### Operations in this Scenario

1. Connects to the simulation server and loads `scene_two_drones.jsonc`.
2. Obtains handles for both `Drone1` and `Drone2`.
3. Enables API control and arms motors on both drones concurrently.
4. Commands concurrent takeoff for both aircraft.
5. Commands divergent velocity vectors: `Drone1` flies North, `Drone2` flies East.
6. Queries and logs kinematics for both aircraft simultaneously.
7. Commands concurrent landing and disarms both vehicles.

### How to Operate

```bash
cargo run --example two_drones_flight
```

## World Debug Plots (`world_debug_plots.rs`)

This scenario matches Python and C++ debug plotting utilities.
It demonstrates 3D visual markers, text labels, and voxel extraction.

### Operations in this Scenario

1. Connects to the simulation server.
2. Plots 20 persistent red 3D points in the world viewport.
3. Flushes persistent markers from the viewport.
4. Plots magenta directional 3D arrows.
5. Plots solid red line strips and green dashed lines.
6. Renders yellow text waypoint labels at 3D positions.
7. Plots coordinate transform axis triads.
8. Extracts a 3D voxel grid and exports the data to a `.binvox` file.

### How to Operate

```bash
cargo run --example world_debug_plots
```

## Synchronous Drone Flight (`sync_drone_flight.rs`)

This scenario demonstrates the blocking synchronous API.
It uses `AsyncResult` handles that match C++ future polling patterns.

### How to Operate

```bash
cargo run --example sync_drone_flight --features sync
```
