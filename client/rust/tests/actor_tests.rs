use nng::{Protocol, Socket};
use projectairsim::protocol::request::RawDataPayload;
use projectairsim::protocol::response::RawResponseEnvelope;
use projectairsim::protocol::RequestEnvelope;
#[allow(unused_imports)]
use std::time::Duration;

#[cfg(feature = "async")]
#[tokio::test]
async fn test_async_actors_complete_suite() {
    let rep_port = 38990;
    let pair_port = 38989;

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

    // Server thread handling expected RPC calls
    let server_thread = std::thread::spawn(move || {
        let expected_methods = vec![
            // Rover calls
            "/Sim/SceneUnit/robots/Rover1/EnableApiControl",
            "/Sim/SceneUnit/robots/Rover1/Arm",
            "/Sim/SceneUnit/robots/Rover1/SetRoverControls",
            "/Sim/SceneUnit/robots/Rover1/MoveToPosition",
            "/Sim/SceneUnit/robots/Rover1/MoveByHeading",
            "/Sim/SceneUnit/robots/Rover1/GetGroundTruthKinematics",
            // WheeledVehicle calls
            "/Sim/SceneUnit/robots/Car1/SetThrottle",
            "/Sim/SceneUnit/robots/Car1/SetSteering",
            "/Sim/SceneUnit/robots/Car1/SetBrakes",
            "/Sim/SceneUnit/robots/Car1/GetGroundTruthKinematics",
            // EnvActor calls
            "/Sim/SceneUnit/SetEnvActorTrajectory",
            "/Sim/SceneUnit/SetEnvActorLinkRotAngle",
            "/Sim/SceneUnit/SetEnvActorLinkRotRate",
            // StaticSensorActor calls
            "/Sim/SceneUnit/robots/Static1/sensors/Camera1/GetImages",
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
                    "/Sim/SceneUnit/robots/Rover1/GetGroundTruthKinematics" => {
                        rmp_serde::to_vec(&serde_json::json!({ "velocity": [1.0, 0.0, 0.0] }))
                            .unwrap()
                    }
                    "/Sim/SceneUnit/robots/Car1/GetGroundTruthKinematics" => {
                        rmp_serde::to_vec(&serde_json::json!({ "speed": 15.0 })).unwrap()
                    }
                    "/Sim/SceneUnit/robots/Static1/sensors/Camera1/GetImages" => rmp_serde::to_vec(
                        &serde_json::json!([{ "image_type": 0, "width": 640, "height": 480 }]),
                    )
                    .unwrap(),
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

    let _world = projectairsim::World::new(client.clone(), None)
        .await
        .expect("world init failed");
    // World defaults to "/Sim", but actors can be created with explicit scene topic "/Sim/SceneUnit"
    let scene_topic = "/Sim/SceneUnit";

    // 1. Rover tests
    let rover = projectairsim::Rover::new(client.clone(), "Rover1", scene_topic);
    assert_eq!(rover.name(), "Rover1");
    assert!(rover
        .enable_api_control()
        .await
        .expect("enable api control failed"));
    assert!(rover.arm().await.expect("arm failed"));
    assert!(rover
        .set_rover_controls(0.5, 0.1, 0.0)
        .await
        .expect("set rover controls failed"));
    assert!(rover
        .move_to_position(10.0, 20.0, 3.0, None, None, None, None)
        .await
        .expect("move to position failed"));
    assert!(rover
        .move_by_heading(1.57, 2.0, None, None, None, None)
        .await
        .expect("move by heading failed"));
    let rover_kinematics = rover
        .get_ground_truth_kinematics()
        .await
        .expect("get kinematics failed");
    assert_eq!(rover_kinematics["velocity"][0], 1.0);

    // 2. WheeledVehicle tests
    let vehicle = projectairsim::WheeledVehicle::new(client.clone(), "Car1", scene_topic);
    assert_eq!(vehicle.name(), "Car1");
    assert!(vehicle
        .set_throttle(0.8)
        .await
        .expect("set throttle failed"));
    assert!(vehicle
        .set_steering(-0.2)
        .await
        .expect("set steering failed"));
    assert!(vehicle.set_brakes(0.5).await.expect("set brakes failed"));
    let car_kinematics = vehicle
        .get_ground_truth_kinematics()
        .await
        .expect("get ground truth kinematics failed");
    assert_eq!(car_kinematics["speed"], 15.0);

    // 3. EnvActor tests
    let env_actor = projectairsim::EnvActor::new(client.clone(), "Arm1", scene_topic);
    assert_eq!(env_actor.name(), "Arm1");
    assert!(env_actor
        .set_trajectory("traj_loop", true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .await
        .expect("set trajectory failed"));
    assert!(env_actor
        .set_link_rotation_angle("joint1", 45.0)
        .await
        .expect("set link angle failed"));
    assert!(env_actor
        .set_link_rotation_rate("joint2", 10.0)
        .await
        .expect("set link rate failed"));

    // 4. StaticSensorActor tests
    let static_sensor =
        projectairsim::StaticSensorActor::new(client.clone(), "Static1", scene_topic);
    assert_eq!(static_sensor.name(), "Static1");
    let images_json = static_sensor
        .get_images("Camera1", &[0, 1])
        .await
        .expect("get images failed");
    assert!(images_json.is_array());
    assert_eq!(images_json[0]["width"], 640);

    server_thread.join().expect("server thread failed");
}

#[cfg(feature = "sync")]
#[test]
fn test_sync_actors_complete_suite() {
    let rep_port = 38992;
    let pair_port = 38991;

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
            // Rover calls
            "/Sim/SceneUnit/robots/Rover1/EnableApiControl",
            "/Sim/SceneUnit/robots/Rover1/Arm",
            "/Sim/SceneUnit/robots/Rover1/SetRoverControls",
            "/Sim/SceneUnit/robots/Rover1/MoveToPosition",
            "/Sim/SceneUnit/robots/Rover1/MoveByHeading",
            // WheeledVehicle calls
            "/Sim/SceneUnit/robots/Car1/SetThrottle",
            "/Sim/SceneUnit/robots/Car1/SetSteering",
            "/Sim/SceneUnit/robots/Car1/SetBrakes",
            // EnvActor calls
            "/Sim/SceneUnit/SetEnvActorTrajectory",
            "/Sim/SceneUnit/SetEnvActorLinkRotAngle",
            // StaticSensorActor calls
            "/Sim/SceneUnit/robots/Static1/sensors/Camera1/GetImages",
        ];

        for expected in expected_methods {
            if let Ok(msg) = rep_socket.recv() {
                let req: RequestEnvelope =
                    rmp_serde::from_slice(msg.as_slice()).expect("valid RequestEnvelope");
                assert_eq!(req.method, expected);

                let reply_data = match req.method {
                    "/Sim/SceneUnit/robots/Static1/sensors/Camera1/GetImages" => rmp_serde::to_vec(
                        &serde_json::json!([{ "image_type": 0, "width": 1920, "height": 1080 }]),
                    )
                    .unwrap(),
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

    let client =
        projectairsim::blocking::Client::connect_with_ports("127.0.0.1", pair_port, rep_port)
            .expect("sync client connect failed");

    let scene_topic = "/Sim/SceneUnit";

    // 1. Rover sync tests
    let rover = projectairsim::blocking::Rover::new(client.clone(), "Rover1", scene_topic);
    assert!(rover
        .enable_api_control()
        .expect("enable api control failed"));
    assert!(rover.arm().expect("arm failed"));
    assert!(rover
        .set_rover_controls(0.7, -0.1, 0.0)
        .expect("set controls failed"));

    // AsyncResult handle for MoveToPosition
    let mut move_ar = rover.move_to_position_async(5.0, 10.0, 2.0, None, None, None, None);
    assert!(move_ar.wait_timeout(Duration::from_secs(2)).is_ok());
    assert!(move_ar.get_result().unwrap());

    assert!(rover
        .move_by_heading(std::f32::consts::PI, 1.5, None, None, None, None)
        .expect("move by heading failed"));


    // 2. WheeledVehicle sync tests
    let vehicle = projectairsim::blocking::WheeledVehicle::new(client.clone(), "Car1", scene_topic);
    assert!(vehicle.set_throttle(1.0).expect("set throttle failed"));
    assert!(vehicle.set_steering(0.5).expect("set steering failed"));
    assert!(vehicle.set_brakes(0.0).expect("set brakes failed"));

    // 3. EnvActor sync tests
    let env_actor = projectairsim::blocking::EnvActor::new(client.clone(), "Arm1", scene_topic);
    assert!(env_actor
        .set_trajectory("crane_motion", false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .expect("set trajectory failed"));
    assert!(env_actor
        .set_link_rotation_angle("link_base", 90.0)
        .expect("set link angle failed"));

    // 4. StaticSensorActor sync tests
    let static_sensor =
        projectairsim::blocking::StaticSensorActor::new(client.clone(), "Static1", scene_topic);
    let images = static_sensor
        .get_images("Camera1", &[0])
        .expect("get images failed");
    assert_eq!(images[0]["width"], 1920);

    server_thread.join().expect("server thread failed");
}
