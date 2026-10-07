# ProjectAirSim Rust API Reference

This document describes the public interface of the `projectairsim` library.
The library provides both asynchronous and synchronous interfaces.

## Client

The `Client` manages network connections to the ProjectAirSim simulation server.
It connects to port 8990 for remote procedure calls and to port 8989 for topic data.

### Methods

- `Client::connect(sim_host)`:
  Connects to the simulation server at the specified host address.

- `client.ping()`:
  Sends a ping message to the simulation server.
  Returns true when the server responds.

- `Client::get_version()`:
  Returns the crate package version string of the client library.

- `Client::get_nng_version()`:
  Returns the underlying NNG messaging library version string.

- `client.get_build_commit_hash()`:
  Returns the build commit hash reported by the simulation server.

- `client.subscribe(topic)`:
  Subscribes to a real-time topic on port 8989.
  Returns a `TopicSubscription` receiver stream.

- `client.unsubscribe(topic)`:
  Removes a previously registered topic subscription.

## World

The `World` interface manages the simulation environment, time stepping, and scene actors.

### Methods

- `World::new(client, scene_path)`:
  Initializes the simulation world.
  If you specify a scene file path, the server loads that scene.
  If you pass `None`, the client attaches to the active scene.

- `world.pause()`:
  Pauses the simulation clock.

- `world.resume()`:
  Resumes the simulation clock.

- `world.step(duration_seconds)`:
  Advances the simulation clock by the specified duration.

- `world.reset()`:
  Resets all vehicles and actors to their starting states.

- `world.import_ned_trajectory(trajectory)`:
  Uploads a programmatic North-East-Down (NED) path trajectory to the world.

- `world.get_drone(vehicle_name)`:
  Returns a `Drone` handle for a quadrotor or multirotor vehicle.

- `world.get_wheeled_vehicle(vehicle_name)`:
  Returns a `WheeledVehicle` handle for a ground automobile or truck.

- `world.get_rover(vehicle_name)`:
  Returns a `Rover` handle for an exploration ground vehicle.

- `world.get_env_actor(actor_name)`:
  Returns an `EnvActor` handle for an articulated scenery actor.

- `world.get_static_sensor(actor_name)`:
  Returns a `StaticSensorActor` handle for a stationary camera tower.

### Debug Plotting Methods

- `world.plot_debug_points(points, color, size, duration, is_persistent)`:
  Draws 3D spheres at the specified coordinates in the simulation viewport.

- `world.plot_debug_arrows(points_start, points_end, color, thickness, arrow_size, duration, is_persistent)`:
  Draws directional arrows between pairs of start and end coordinates.

- `world.plot_debug_solid_line(points, color, thickness, duration, is_persistent)`:
  Draws a continuous solid line strip through the specified coordinates.

- `world.plot_debug_dashed_line(points, color, thickness, duration, is_persistent)`:
  Draws dashed line segments connecting the specified coordinates.

- `world.plot_debug_strings(strings, positions, scale, color, duration)`:
  Renders text labels at the specified 3D coordinates.

- `world.plot_debug_transforms(poses, scale, thickness, duration, is_persistent)`:
  Draws red, green, and blue coordinate axes at each pose.

- `world.flush_persistent_markers()`:
  Removes all persistent debug visual markers from the simulation viewport.

- `world.create_voxel_grid(center_pose, x_size, y_size, z_size, resolution, ignore_actors, export_binvox, output_path)`:
  Extracts 3D occupancy voxels from the scene geometry.
  If you specify an output path, the method saves the grid as a `.binvox` file.

## Drone

The `Drone` interface controls multirotor aircraft.

### Methods

- `drone.enable_api_control()`:
  Requests software control authority from the simulation server.

- `drone.disable_api_control()`:
  Releases software control authority back to manual control.

- `drone.arm()`:
  Arms the vehicle propulsion motors.

- `drone.disarm()`:
  Disarms the vehicle propulsion motors.

- `drone.takeoff(timeout_seconds)`:
  Commands vertical ascent to default hover altitude.

- `drone.land(timeout_seconds)`:
  Commands vertical descent until surface touchdown.

- `drone.move_by_velocity(vx, vy, vz, duration_seconds)`:
  Commands autonomous flight with a velocity vector in the NED coordinate frame.

- `drone.move_to_position(x, y, z, velocity)`:
  Commands flight to an absolute target position in the NED frame.

- `drone.get_ready_state()`:
  Queries whether vehicle sub-systems report readiness for motor arming.

- `drone.get_landed_state()`:
  Queries whether the vehicle is landed or airborne.

- `drone.get_ground_truth_kinematics()`:
  Retrieves position, orientation, linear velocity, and angular velocity.

- `drone.get_sensor_topic(sensor_name, topic)`:
  Constructs the topic name for a mounted sensor.

- `drone.subscribe_telemetry(subtopic)`:
  Subscribes directly to vehicle telemetry streams.

## WheeledVehicle

The `WheeledVehicle` interface controls wheeled vehicles such as cars and trucks.

### Methods

- `vehicle.set_controls(throttle, steering, brake)`:
  Sets vehicle drive controls.
  The `throttle` value ranges from 0.0 to 1.0.
  The `steering` value ranges from -1.0 to 1.0.
  The `brake` value ranges from 0.0 to 1.0.

- `vehicle.get_ground_truth_kinematics()`:
  Retrieves position, orientation, and velocity for the vehicle.

## Rover

The `Rover` interface controls ground exploration rovers.

### Methods

- `rover.enable_api_control()`:
  Requests software control authority for the rover.

- `rover.disable_api_control()`:
  Releases software control authority.

- `rover.arm()`:
  Arms rover drive motors.

- `rover.disarm()`:
  Disarms rover drive motors.

- `rover.set_rover_controls(engine, steering, brake)`:
  Sets directional drive controls.
  The `engine` value ranges from -1.0 (reverse) to 1.0 (forward).
  The `steering` value ranges from -1.0 (left) to 1.0 (right).
  The `brake` value ranges from 0.0 to 1.0.

- `rover.get_ground_truth_kinematics()`:
  Retrieves position, orientation, and velocity for the rover.

## EnvActor

The `EnvActor` interface controls non-player environment actors and scenery objects.

### Methods

- `actor.set_trajectory(traj_name, to_loop, time_offset, x_offset, y_offset, z_offset, roll_offset, pitch_offset, yaw_offset)`:
  Assigns a named trajectory asset with time offset, spatial offsets, and Euler rotation offsets.

- `actor.set_link_rotation_angles(angles_map)`:
  Rotates articulated joints on the actor to specified degree angles.

## StaticSensorActor

The `StaticSensorActor` interface interacts with stationary sensor towers.

### Methods

- `sensor.get_camera_images(camera_name, image_types)`:
  Captures image frames from cameras mounted on the stationary actor.
  Supported types include `ImageType::Scene`, `ImageType::DepthPlanar`, and `ImageType::Segmentation`.

## Logging

The logging module provides structured logging and custom callback sinks.

### Functions and Types

- `init_logging()`:
  Initializes the logging system.
  It reads filter levels from the `RUST_LOG` environment variable.

- `set_log_sink(callback)`:
  Registers a custom callback for log messages.
  The callback receives the message severity and the message string.
  When an active sink exists, the library suppresses duplicate terminal output.

- `clear_log_sink()`:
  Removes the custom log sink and restores terminal output.

- `has_log_sink()`:
  Returns true if a custom sink callback is currently registered.

- `Severity`:
  Enum matching log severity levels: `Trace`, `Debug`, `Info`, `Warning`, `Error`, `Critical`.
