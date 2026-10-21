#![cfg(feature = "async")]

use std::collections::HashMap;
use std::net::TcpListener;

use nng::{Protocol, Socket};
use projectairsim::protocol::request::RawDataPayload;
use projectairsim::protocol::response::RawResponseEnvelope;
use projectairsim::protocol::RequestEnvelope;
use projectairsim::types::{
    GeoPosition, ImageType, Pose, Transform, VTOLMode, Vector3, YawControlMode,
};

fn get_available_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local addr").port()
}

#[cfg(feature = "async")]
#[tokio::test]
async fn test_async_drone_phase2_complete_suite() {
    let rep_port = get_available_port();
    let pair_port = get_available_port();

    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");

    let rep_socket = Socket::new(Protocol::Rep0).expect("failed to open Rep0 socket");
    rep_socket
        .listen(&rep_url)
        .expect("failed to listen on Rep0");

    let pair_server = Socket::new(Protocol::Pair0).expect("failed to open Pair0 socket");
    pair_server
        .listen(&pair_url)
        .expect("failed to listen on Pair0");

    let server_thread = std::thread::spawn(move || {
        let expected_methods = vec![
            "/Sim/SceneUnit/robots/Drone1/EnableApiControl",
            "/Sim/SceneUnit/robots/Drone1/Arm",
            "/Sim/SceneUnit/robots/Drone1/Takeoff",
            "/Sim/SceneUnit/robots/Drone1/Hover",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocity",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocityZ",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocityBodyFrame",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocityBodyFrameZ",
            "/Sim/SceneUnit/robots/Drone1/MoveByHeading",
            "/Sim/SceneUnit/robots/Drone1/MoveOnPath",
            "/Sim/SceneUnit/robots/Drone1/MoveToPosition",
            "/Sim/SceneUnit/robots/Drone1/RotateToYaw",
            "/Sim/SceneUnit/robots/Drone1/RotateByYawRate",
            "/Sim/SceneUnit/robots/Drone1/GoHome",
            "/Sim/SceneUnit/robots/Drone1/RequestControl",
            "/Sim/SceneUnit/robots/Drone1/SetMissionMode",
            "/Sim/SceneUnit/robots/Drone1/SetVTOLMode",
            "/Sim/SceneUnit/robots/Drone1/CancelLastTask",
            // Sensors & Battery
            "/Sim/SceneUnit/robots/Drone1/sensors/IMU1/imu_kinematics",
            "/Sim/SceneUnit/robots/Drone1/sensors/GPS1/gps",
            "/Sim/SceneUnit/robots/Drone1/sensors/Airspeed1/airspeed",
            "/Sim/SceneUnit/robots/Drone1/sensors/Baro1/barometer",
            "/Sim/SceneUnit/robots/Drone1/sensors/Mag1/magnetometer",
            "/Sim/SceneUnit/robots/Drone1/sensors/Lidar1/lidar",
            "/Sim/SceneUnit/robots/Drone1/sensors/Radar1/radar_detections",
            "/Sim/SceneUnit/robots/Drone1/sensors/Radar1/radar_tracks",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/GetBatteryStatus",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/SetBatteryRemaining",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/GetBatteryDrainRate",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/SetBatteryDrainRate",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/SetBatteryHealthStatus",
            // Camera Optics
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/GetImages",
            "/Sim/SceneUnit/robots/Drone1/GetCameraRay",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/LookAtObject",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/DrawFrustum",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetPose",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/ResetCameraPose",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetFocalLength",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetDepthOfFieldTransitionRegion",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetDepthOfFieldFocalRegion",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetChromaticAberrationIntensity",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetFieldOfView",
            // Kinematics & Disturbances
            "/Sim/SceneUnit/robots/Drone1/actuators/0/ToggleFault",
            "/Sim/SceneUnit/robots/Drone1/SetExternalForce",
            "/Sim/SceneUnit/robots/Drone1/SetControlSignals",
            "/Sim/SceneUnit/robots/Drone1/GetGroundTruthPose",
            "/Sim/SceneUnit/robots/Drone1/GetGroundTruthGeoLocation",
            "/Sim/SceneUnit/robots/Drone1/GetEstimatedGeoLocation",
            "/Sim/SceneUnit/robots/Drone1/GetGroundTruthKinematics",
            "/Sim/SceneUnit/robots/Drone1/GetEstimatedKinematics",
            "/Sim/SceneUnit/robots/Drone1/SetGroundTruthKinematics",
            "/Sim/SceneUnit/robots/Drone1/SetPose",
            "/Sim/SceneUnit/robots/Drone1/Land",
            "/Sim/SceneUnit/robots/Drone1/Disarm",
            "/Sim/SceneUnit/robots/Drone1/DisableApiControl",
        ];

        for expected in expected_methods {
            if let Ok(msg) = rep_socket.recv() {
                let req: RequestEnvelope =
                    rmp_serde::from_slice(msg.as_slice()).expect("valid RequestEnvelope");
                assert_eq!(
                    req.method, expected,
                    "Unexpected method dispatched to mock server"
                );

                let reply_data = match req.method {
                    "/Sim/SceneUnit/robots/Drone1/sensors/IMU1/imu_kinematics" => {
                        rmp_serde::to_vec(&serde_json::json!({ "angular_velocity": [0.0, 0.0, 0.0] })).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/sensors/GPS1/gps" => {
                        rmp_serde::to_vec(&serde_json::json!({ "latitude": 47.6, "longitude": -122.3 })).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/sensors/Battery/GetBatteryStatus" => {
                        rmp_serde::to_vec(&serde_json::json!({ "battery_remaining": 95.5 })).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/sensors/Battery/GetBatteryDrainRate" => {
                        rmp_serde::to_vec(&1.25f32).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/GetImages" => {
                        rmp_serde::to_vec(&serde_json::json!([{ "image_type": 0, "width": 640, "height": 480 }])).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/GetCameraRay" => {
                        rmp_serde::to_vec(&Pose::default()).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/GetGroundTruthPose" => {
                        rmp_serde::to_vec(&Pose::default()).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/GetGroundTruthGeoLocation"
                    | "/Sim/SceneUnit/robots/Drone1/GetEstimatedGeoLocation" => {
                        rmp_serde::to_vec(&GeoPosition::new(47.6, -122.3, 100.0)).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/GetGroundTruthKinematics"
                    | "/Sim/SceneUnit/robots/Drone1/GetEstimatedKinematics" => {
                        rmp_serde::to_vec(&serde_json::json!({ "linear_velocity": [1.0, 0.0, 0.0] })).unwrap()
                    }
                    _ => rmp_serde::to_vec(&true).unwrap(),
                };

                let reply_envelope = RawResponseEnvelope {
                    id: Some(req.id),
                    version: Some(1.0),
                    result: Some(RawDataPayload { data: reply_data }),
                    error: None,
                };
                let reply_bytes = rmp_serde::to_vec(&reply_envelope).unwrap();
                rep_socket.send(&reply_bytes).expect("send reply failed");
            }
        }
    });

    let client = projectairsim::Client::connect_with_ports("127.0.0.1", pair_port, rep_port)
        .await
        .expect("client connect failed");

    let drone = projectairsim::Drone::new(client.clone(), "Drone1", "/Sim/SceneUnit");
    assert_eq!(drone.name(), "Drone1");

    // Flight & Trajectory
    assert!(drone.enable_api_control().await.unwrap());
    assert!(drone.arm().await.unwrap());
    assert!(drone.takeoff(10.0).await.unwrap());
    assert!(drone.hover().await.unwrap());
    assert!(drone.move_by_velocity(1.0, 0.0, 0.0, 2.0).await.unwrap());
    assert!(drone.move_by_velocity_z(1.0, 0.0, -5.0, 2.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0).await.unwrap());
    assert!(drone.move_by_velocity_body_frame(1.0, 0.0, 0.0, 2.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0).await.unwrap());
    assert!(drone.move_by_velocity_body_frame_z(1.0, 0.0, -5.0, 2.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0).await.unwrap());
    assert!(drone.move_by_heading(90.0, 5.0, 0.0, 3.0, 1.0, 10.0, 5.0).await.unwrap());
    assert!(drone.move_on_path(&[Vector3::new(1.0, 2.0, -5.0)], 5.0, 10.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0, -1.0, 1.0).await.unwrap());
    assert!(drone.move_to_position(10.0, 10.0, -5.0, 3.0, 10.0).await.unwrap());
    assert!(drone.rotate_to_yaw(180.0, 5.0, 1.0, 30.0).await.unwrap());
    assert!(drone.rotate_by_yaw_rate(15.0, 2.0).await.unwrap());
    assert!(drone.go_home(15.0, 5.0).await.unwrap());
    assert!(drone.request_control().await.unwrap());
    assert!(drone.set_mission_mode().await.unwrap());
    assert!(drone.set_vtol_mode(VTOLMode::Multirotor).await.unwrap());
    assert!(drone.cancel_last_task().await.unwrap());

    // Sensors & Battery
    assert!(drone.get_imu_data("IMU1").await.unwrap().is_object());
    assert!(drone.get_gps_data("GPS1").await.unwrap().is_object());
    assert!(drone.get_airspeed_data("Airspeed1").await.unwrap().is_boolean());
    assert!(drone.get_barometer_data("Baro1").await.unwrap().is_boolean());
    assert!(drone.get_magnetometer_data("Mag1").await.unwrap().is_boolean());
    assert!(drone.get_lidar_data("Lidar1").await.unwrap().is_boolean());
    assert!(drone.get_radar_detections("Radar1").await.unwrap().is_boolean());
    assert!(drone.get_radar_tracks("Radar1").await.unwrap().is_boolean());
    assert!(drone.get_battery_state().await.unwrap().is_object());
    assert!(drone.set_battery_remaining(90.0).await.unwrap());
    assert_eq!(drone.get_battery_drain_rate().await.unwrap(), 1.25);
    assert!(drone.set_battery_drain_rate(1.0).await.unwrap());
    assert!(drone.set_battery_health_status(true).await.unwrap());

    // Camera Optics
    assert_eq!(drone.get_images("Camera1", &[ImageType::Scene]).await.unwrap().len(), 1);
    assert_eq!(drone.get_camera_ray("Camera1", ImageType::Scene, 320, 240).await.unwrap(), Pose::default());
    assert!(drone.camera_look_at_object("Camera1", "TargetObj", true).await.unwrap());
    assert!(drone.camera_draw_frustum("Camera1", true, ImageType::Scene).await.unwrap());
    assert!(drone.set_camera_pose("Camera1", &Pose::default(), true).await.unwrap());
    assert!(drone.reset_camera_pose("Camera1", true).await.unwrap());
    assert!(drone.set_focal_length("Camera1", 0, 35.0).await.unwrap());
    assert!(drone.set_depth_of_field_transition_threshold("Camera1", 0, 0.5).await.unwrap());
    assert!(drone.set_depth_of_field_focal_region("Camera1", 0, 100.0).await.unwrap());
    assert!(drone.set_chromatic_aberration_intensity("Camera1", 0, 0.2).await.unwrap());
    assert!(drone.set_field_of_view("Camera1", 0, 90.0).await.unwrap());

    // Kinematics & Disturbances
    assert!(drone.update_actuator_fault_state("0", true).await.unwrap());
    assert!(drone.set_external_force(&[0.0, 0.0, -10.0]).await.unwrap());
    let mut signals = HashMap::new();
    signals.insert("0".to_string(), 0.8f32);
    assert!(drone.set_control_signals(&signals).await.unwrap());
    assert_eq!(drone.get_ground_truth_pose().await.unwrap(), Pose::default());
    assert_eq!(drone.get_ground_truth_geo_location().await.unwrap().altitude, 100.0);
    assert_eq!(drone.get_estimated_geo_location().await.unwrap().latitude, 47.6);
    assert!(drone.get_ground_truth_kinematics().await.unwrap().is_object());
    assert!(drone.get_estimated_kinematics().await.unwrap().is_object());
    assert!(drone.set_ground_truth_kinematics(&serde_json::json!({ "velocity": [0.0, 0.0, 0.0] })).await.unwrap());
    assert!(drone.set_pose(&Transform::default(), true).await.unwrap());
    assert!(drone.land(10.0).await.unwrap());
    assert!(drone.disarm().await.unwrap());
    assert!(drone.disable_api_control().await.unwrap());

    server_thread.join().expect("server thread failed");
}

#[cfg(feature = "sync")]
#[test]
fn test_sync_drone_phase2_complete_suite() {
    let rep_port = get_available_port();
    let pair_port = get_available_port();

    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");

    let rep_socket = Socket::new(Protocol::Rep0).expect("failed to open Rep0 socket");
    rep_socket
        .listen(&rep_url)
        .expect("failed to listen on Rep0");

    let pair_server = Socket::new(Protocol::Pair0).expect("failed to open Pair0 socket");
    pair_server
        .listen(&pair_url)
        .expect("failed to listen on Pair0");

    let server_thread = std::thread::spawn(move || {
        let expected_methods = vec![
            "/Sim/SceneUnit/robots/Drone1/EnableApiControl",
            "/Sim/SceneUnit/robots/Drone1/Arm",
            "/Sim/SceneUnit/robots/Drone1/Takeoff",
            "/Sim/SceneUnit/robots/Drone1/Hover",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocity",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocityZ",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocityBodyFrame",
            "/Sim/SceneUnit/robots/Drone1/MoveByVelocityBodyFrameZ",
            "/Sim/SceneUnit/robots/Drone1/MoveByHeading",
            "/Sim/SceneUnit/robots/Drone1/MoveOnPath",
            "/Sim/SceneUnit/robots/Drone1/MoveToPosition",
            "/Sim/SceneUnit/robots/Drone1/RotateToYaw",
            "/Sim/SceneUnit/robots/Drone1/RotateByYawRate",
            "/Sim/SceneUnit/robots/Drone1/GoHome",
            "/Sim/SceneUnit/robots/Drone1/RequestControl",
            "/Sim/SceneUnit/robots/Drone1/SetMissionMode",
            "/Sim/SceneUnit/robots/Drone1/SetVTOLMode",
            "/Sim/SceneUnit/robots/Drone1/CancelLastTask",
            // Sensors & Battery
            "/Sim/SceneUnit/robots/Drone1/sensors/IMU1/imu_kinematics",
            "/Sim/SceneUnit/robots/Drone1/sensors/GPS1/gps",
            "/Sim/SceneUnit/robots/Drone1/sensors/Airspeed1/airspeed",
            "/Sim/SceneUnit/robots/Drone1/sensors/Baro1/barometer",
            "/Sim/SceneUnit/robots/Drone1/sensors/Mag1/magnetometer",
            "/Sim/SceneUnit/robots/Drone1/sensors/Lidar1/lidar",
            "/Sim/SceneUnit/robots/Drone1/sensors/Radar1/radar_detections",
            "/Sim/SceneUnit/robots/Drone1/sensors/Radar1/radar_tracks",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/GetBatteryStatus",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/SetBatteryRemaining",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/GetBatteryDrainRate",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/SetBatteryDrainRate",
            "/Sim/SceneUnit/robots/Drone1/sensors/Battery/SetBatteryHealthStatus",
            // Camera Optics
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/GetImages",
            "/Sim/SceneUnit/robots/Drone1/GetCameraRay",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/LookAtObject",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/DrawFrustum",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetPose",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/ResetCameraPose",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetFocalLength",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetDepthOfFieldTransitionRegion",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetDepthOfFieldFocalRegion",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetChromaticAberrationIntensity",
            "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/SetFieldOfView",
            // Kinematics & Disturbances
            "/Sim/SceneUnit/robots/Drone1/actuators/0/ToggleFault",
            "/Sim/SceneUnit/robots/Drone1/SetExternalForce",
            "/Sim/SceneUnit/robots/Drone1/SetControlSignals",
            "/Sim/SceneUnit/robots/Drone1/GetGroundTruthPose",
            "/Sim/SceneUnit/robots/Drone1/GetGroundTruthGeoLocation",
            "/Sim/SceneUnit/robots/Drone1/GetEstimatedGeoLocation",
            "/Sim/SceneUnit/robots/Drone1/GetGroundTruthKinematics",
            "/Sim/SceneUnit/robots/Drone1/GetEstimatedKinematics",
            "/Sim/SceneUnit/robots/Drone1/SetGroundTruthKinematics",
            "/Sim/SceneUnit/robots/Drone1/SetPose",
            "/Sim/SceneUnit/robots/Drone1/Land",
            "/Sim/SceneUnit/robots/Drone1/Disarm",
            "/Sim/SceneUnit/robots/Drone1/DisableApiControl",
        ];

        for expected in expected_methods {
            if let Ok(msg) = rep_socket.recv() {
                let req: RequestEnvelope =
                    rmp_serde::from_slice(msg.as_slice()).expect("valid RequestEnvelope");
                assert_eq!(
                    req.method, expected,
                    "Unexpected method dispatched to mock server"
                );

                let reply_data = match req.method {
                    "/Sim/SceneUnit/robots/Drone1/sensors/Battery/GetBatteryDrainRate" => {
                        rmp_serde::to_vec(&2.5f32).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/sensors/Camera1/GetImages" => {
                        rmp_serde::to_vec(&serde_json::json!([{ "image_type": 0, "width": 1280, "height": 720 }])).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/GetCameraRay" => {
                        rmp_serde::to_vec(&Pose::default()).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/GetGroundTruthPose" => {
                        rmp_serde::to_vec(&Pose::default()).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Drone1/GetGroundTruthGeoLocation"
                    | "/Sim/SceneUnit/robots/Drone1/GetEstimatedGeoLocation" => {
                        rmp_serde::to_vec(&GeoPosition::new(47.6, -122.3, 50.0)).unwrap()
                    }
                    _ => rmp_serde::to_vec(&true).unwrap(),
                };

                let reply_envelope = RawResponseEnvelope {
                    id: Some(req.id),
                    version: Some(1.0),
                    result: Some(RawDataPayload { data: reply_data }),
                    error: None,
                };
                let reply_bytes = rmp_serde::to_vec(&reply_envelope).unwrap();
                rep_socket.send(&reply_bytes).expect("send reply failed");
            }
        }
    });

    let client = projectairsim::blocking::Client::connect_with_ports("127.0.0.1", pair_port, rep_port)
        .expect("client connect failed");

    let drone = projectairsim::blocking::Drone::new(client.clone(), "Drone1", "/Sim/SceneUnit");
    assert_eq!(drone.name(), "Drone1");

    // Flight & Trajectory
    assert!(drone.enable_api_control().unwrap());
    assert!(drone.arm().unwrap());
    assert!(drone.takeoff(10.0).unwrap());
    assert!(drone.hover().unwrap());
    assert!(drone.move_by_velocity(1.0, 0.0, 0.0, 2.0).unwrap());
    assert!(drone.move_by_velocity_z(1.0, 0.0, -5.0, 2.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0).unwrap());
    assert!(drone.move_by_velocity_body_frame(1.0, 0.0, 0.0, 2.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0).unwrap());
    assert!(drone.move_by_velocity_body_frame_z(1.0, 0.0, -5.0, 2.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0).unwrap());
    assert!(drone.move_by_heading(90.0, 5.0, 0.0, 3.0, 1.0, 10.0, 5.0).unwrap());
    assert!(drone.move_on_path(&[Vector3::new(1.0, 2.0, -5.0)], 5.0, 10.0, YawControlMode::MaxDegreeOfFreedom, false, 0.0, -1.0, 1.0).unwrap());
    assert!(drone.move_to_position(10.0, 10.0, -5.0, 3.0, 10.0).unwrap());
    assert!(drone.rotate_to_yaw(180.0, 5.0, 1.0, 30.0).unwrap());
    assert!(drone.rotate_by_yaw_rate(15.0, 2.0).unwrap());
    assert!(drone.go_home(15.0, 5.0).unwrap());
    assert!(drone.request_control().unwrap());
    assert!(drone.set_mission_mode().unwrap());
    assert!(drone.set_vtol_mode(VTOLMode::Multirotor).unwrap());
    assert!(drone.cancel_last_task().unwrap());

    // Sensors & Battery
    assert!(drone.get_imu_data("IMU1").unwrap().is_boolean());
    assert!(drone.get_gps_data("GPS1").unwrap().is_boolean());
    assert!(drone.get_airspeed_data("Airspeed1").unwrap().is_boolean());
    assert!(drone.get_barometer_data("Baro1").unwrap().is_boolean());
    assert!(drone.get_magnetometer_data("Mag1").unwrap().is_boolean());
    assert!(drone.get_lidar_data("Lidar1").unwrap().is_boolean());
    assert!(drone.get_radar_detections("Radar1").unwrap().is_boolean());
    assert!(drone.get_radar_tracks("Radar1").unwrap().is_boolean());
    assert!(drone.get_battery_state().unwrap().is_boolean());
    assert!(drone.set_battery_remaining(90.0).unwrap());
    assert_eq!(drone.get_battery_drain_rate().unwrap(), 2.5);
    assert!(drone.set_battery_drain_rate(1.0).unwrap());
    assert!(drone.set_battery_health_status(true).unwrap());

    // Camera Optics
    assert_eq!(drone.get_images("Camera1", &[ImageType::Scene]).unwrap().len(), 1);
    assert_eq!(drone.get_camera_ray("Camera1", ImageType::Scene, 320, 240).unwrap(), Pose::default());
    assert!(drone.camera_look_at_object("Camera1", "TargetObj", true).unwrap());
    assert!(drone.camera_draw_frustum("Camera1", true, ImageType::Scene).unwrap());
    assert!(drone.set_camera_pose("Camera1", &Pose::default(), true).unwrap());
    assert!(drone.reset_camera_pose("Camera1", true).unwrap());
    assert!(drone.set_focal_length("Camera1", 0, 35.0).unwrap());
    assert!(drone.set_depth_of_field_transition_threshold("Camera1", 0, 0.5).unwrap());
    assert!(drone.set_depth_of_field_focal_region("Camera1", 0, 100.0).unwrap());
    assert!(drone.set_chromatic_aberration_intensity("Camera1", 0, 0.2).unwrap());
    assert!(drone.set_field_of_view("Camera1", 0, 90.0).unwrap());

    // Kinematics & Disturbances
    assert!(drone.update_actuator_fault_state("0", true).unwrap());
    assert!(drone.set_external_force(&[0.0, 0.0, -10.0]).unwrap());
    let mut signals = HashMap::new();
    signals.insert("0".to_string(), 0.8f32);
    assert!(drone.set_control_signals(&signals).unwrap());
    assert_eq!(drone.get_ground_truth_pose().unwrap(), Pose::default());
    assert_eq!(drone.get_ground_truth_geo_location().unwrap().altitude, 50.0);
    assert_eq!(drone.get_estimated_geo_location().unwrap().latitude, 47.6);
    assert!(drone.get_ground_truth_kinematics().unwrap().is_boolean());
    assert!(drone.get_estimated_kinematics().unwrap().is_boolean());
    assert!(drone.set_ground_truth_kinematics(&serde_json::json!({ "velocity": [0.0, 0.0, 0.0] })).unwrap());
    assert!(drone.set_pose(&Transform::default(), true).unwrap());
    assert!(drone.land(10.0).unwrap());
    assert!(drone.disarm().unwrap());
    assert!(drone.disable_api_control().unwrap());

    server_thread.join().expect("server thread failed");
}
