#![cfg(feature = "sync")]

use nng::options::Options;
use nng::{Protocol, Socket};
use projectairsim::blocking::{AsyncResult, Client, Drone, World};
use projectairsim::protocol::request::RawDataPayload;
use projectairsim::protocol::response::RawResponseEnvelope;
use std::net::TcpListener;
use std::time::Duration;

fn get_available_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local addr").port()
}

#[test]
fn test_sync_client_version_apis() {
    assert_eq!(Client::get_version(), env!("CARGO_PKG_VERSION"));
    assert_eq!(Client::get_nng_version(), "1.0");
}

#[test]
fn test_async_result_mechanics() {
    let (tx, rx) = std::sync::mpsc::channel();
    let ar = AsyncResult::new(rx);

    assert!(!ar.is_done());
    tx.send(Ok(42)).unwrap();
    assert!(ar.is_done());
    assert!(ar.wait().is_ok());
    assert!(ar.is_done());
    assert_eq!(ar.get_result().unwrap(), 42);
}

#[test]
fn test_async_result_concurrent_is_done() {
    let (tx, rx) = std::sync::mpsc::channel();
    let ar = std::sync::Arc::new(AsyncResult::new(rx));

    let ar_clone = std::sync::Arc::clone(&ar);
    let waiter = std::thread::spawn(move || {
        ar_clone.wait().unwrap();
    });

    // While waiter is blocking on recv, is_done() must return false without blocking
    std::thread::sleep(Duration::from_millis(50));
    assert!(!ar.is_done());

    tx.send(Ok(100)).unwrap();
    waiter.join().unwrap();

    assert!(ar.is_done());
}

#[test]
fn test_blocking_client_rpc_and_drone() {
    let rep_port = get_available_port();
    let pair_port = get_available_port();

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
    let takeoff_ar: AsyncResult<bool> = drone.takeoff_async(10.0);
    assert!(takeoff_ar.wait_timeout(Duration::from_secs(2)).is_ok());
    assert!(takeoff_ar.is_done());
    assert!(takeoff_ar.get_result().unwrap());

    server_thread.join().expect("server thread failed");
}

#[test]
fn test_blocking_client_cancel_all_requests() {
    let rep_port = get_available_port();
    let pair_port = get_available_port();

    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");

    let is_running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let is_running_clone = std::sync::Arc::clone(&is_running);

    let rep_socket = Socket::new(Protocol::Rep0).expect("failed to open Rep0 socket");
    let _ = rep_socket.set_opt::<nng::options::RecvTimeout>(Some(Duration::from_millis(50)));
    rep_socket.listen(&rep_url).expect("failed to listen on Rep0");

    let pair_server = Socket::new(Protocol::Pair0).expect("failed to open Pair0 socket");
    pair_server.listen(&pair_url).expect("failed to listen on Pair0");

    let server_thread = std::thread::spawn(move || {
        // Delay to allow queue accumulation and cancellation
        std::thread::sleep(Duration::from_millis(150));
        while is_running_clone.load(std::sync::atomic::Ordering::Relaxed) {
            let msg = match rep_socket.recv() {
                Ok(m) => m,
                Err(nng::Error::TimedOut) => continue,
                Err(_) => break,
            };
            let req: projectairsim::protocol::RequestEnvelope =
                rmp_serde::from_slice(msg.as_slice()).unwrap();
            let resp = RawResponseEnvelope {
                id: Some(req.id),
                version: Some(1.0),
                result: Some(RawDataPayload {
                    data: rmp_serde::to_vec(&true).unwrap(),
                }),
                error: None,
            };
            let _ = rep_socket.send(&rmp_serde::to_vec(&resp).unwrap());
        }
    });

    let client =
        Client::connect_with_ports("127.0.0.1", pair_port, rep_port).expect("connect failed");

    // Queue up requests
    let ar1: AsyncResult<bool> = client.request_async("/Sim/Ping", &serde_json::json!({}));
    let ar2: AsyncResult<bool> = client.request_async("/Sim/Ping", &serde_json::json!({}));
    let ar3: AsyncResult<bool> = client.request_async("/Sim/Ping", &serde_json::json!({}));

    // Cancel queued requests
    client.cancel_all_requests();

    // The in-flight request completes or cancels, and queued requests get cancelled
    let _ = ar1.wait();
    let _ = ar2.wait();
    let _ = ar3.wait();
    assert!(ar1.is_done());
    assert!(ar2.is_done());
    assert!(ar3.is_done());

    is_running.store(false, std::sync::atomic::Ordering::Relaxed);
    drop(client);
    let _ = server_thread.join();
}
