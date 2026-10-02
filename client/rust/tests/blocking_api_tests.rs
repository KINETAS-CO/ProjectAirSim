#![cfg(feature = "sync")]

use nng::{Protocol, Socket};
use projectairsim::blocking::{AsyncResult, Client, Drone, World};
use projectairsim::protocol::request::RawDataPayload;
use projectairsim::protocol::response::RawResponseEnvelope;
use std::time::Duration;

#[test]
fn test_async_result_mechanics() {
    let (tx, rx) = std::sync::mpsc::channel();
    let mut ar = AsyncResult::new(rx);

    assert!(!ar.is_done());
    tx.send(Ok(42)).unwrap();
    assert!(ar.wait().is_ok());
    assert!(ar.is_done());
    assert_eq!(ar.get_result().unwrap(), 42);
}

#[test]
fn test_blocking_client_rpc_and_drone() {
    let rep_port = 28990;
    let pair_port = 28989;

    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");

    // Start mock server sockets
    let rep_socket = Socket::new(Protocol::Rep0).expect("failed to open Rep0 socket");
    rep_socket
        .listen(&rep_url)
        .expect("failed to listen on Rep0");

    let pair_server = Socket::new(Protocol::Pair0).expect("failed to open Pair0 socket");
    pair_server
        .listen(&pair_url)
        .expect("failed to listen on Pair0");

    // Server worker thread
    let server_thread = std::thread::spawn(move || {
        // Handle 3 requests: /Sim/Ping, /Sim/Drone1/EnableApiControl, /Sim/Drone1/Takeoff
        for _ in 0..3 {
            if let Ok(msg) = rep_socket.recv() {
                let req: projectairsim::RequestEnvelope =
                    rmp_serde::from_slice(msg.as_slice()).expect("valid request");
                let reply_data = match req.method {
                    "/Sim/Ping" => rmp_serde::to_vec(&true).unwrap(),
                    "/Sim/Drone1/EnableApiControl" => rmp_serde::to_vec(&true).unwrap(),
                    "/Sim/Drone1/Takeoff" => rmp_serde::to_vec(&true).unwrap(),
                    other => panic!("Unexpected method {other}"),
                };

                let reply_envelope = RawResponseEnvelope {
                    id: Some(req.id),
                    version: Some(1.0),
                    result: Some(RawDataPayload { data: reply_data }),
                    error: None,
                };
                let reply_bytes = rmp_serde::to_vec(&reply_envelope).unwrap();
                let _ = rep_socket.send(&reply_bytes);
            }
        }
    });

    // Connect blocking client
    let client =
        Client::connect_with_ports("127.0.0.1", pair_port, rep_port).expect("connect failed");

    // 1. Direct synchronous ping
    assert!(client.ping().expect("ping failed"));

    // 2. World and Drone interaction
    let world = World::new(client.clone(), None).expect("world init failed");
    let drone: Drone = world.get_drone("Drone1");

    // 3. API Control
    assert!(drone
        .enable_api_control()
        .expect("enable api control failed"));

    // 4. Takeoff with AsyncResult handle (C++ style)
    let mut takeoff_ar: AsyncResult<bool> = drone.takeoff_async(10.0);
    assert!(takeoff_ar.wait_timeout(Duration::from_secs(2)).is_ok());
    assert!(takeoff_ar.is_done());
    assert!(takeoff_ar.get_result().unwrap());

    server_thread.join().expect("server thread failed");
}
