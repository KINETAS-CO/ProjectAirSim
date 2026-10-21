#![cfg(feature = "async")]

use nng::{Protocol, Socket};
use projectairsim::protocol::frame::TopicFrame;
use projectairsim::protocol::request::RawDataPayload;
use projectairsim::protocol::response::RawResponseEnvelope;
use projectairsim::Client;
use std::net::TcpListener;
use std::time::Duration;

fn get_available_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local addr").port()
}

#[tokio::test]
async fn test_nng_actor_real_socket_rpc_and_topics() {
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

    // Spawn server worker thread to reply to RPC requests
    let server_thread = std::thread::spawn(move || {
        if let Ok(msg) = rep_socket.recv() {
            let req: projectairsim::RequestEnvelope =
                rmp_serde::from_slice(msg.as_slice()).expect("valid RequestEnvelope");
            assert_eq!(req.method, "/Sim/Ping");

            // Build reply envelope: {"id": 1, "result": {"data": <msgpack(true)>}, "version": 1.0}
            let success_data = rmp_serde::to_vec(&true).unwrap();
            let reply_envelope = RawResponseEnvelope {
                id: Some(req.id),
                version: Some(1.0),
                result: Some(RawDataPayload { data: success_data }),
                error: None,
            };
            let reply_bytes = rmp_serde::to_vec(&reply_envelope).unwrap();
            let _ = rep_socket.send(&reply_bytes);
        }

        // 2. Publish a test topic message over Pair0
        let test_payload = rmp_serde::to_vec(&"telemetry_data_123").unwrap();
        let frame = TopicFrame::message("world/drone1/pose", test_payload);
        let frame_bytes = frame.to_bytes().unwrap();
        std::thread::sleep(Duration::from_millis(50));
        let _ = pair_server.send(&frame_bytes);
    });

    // Connect client
    let client = Client::connect_with_ports("127.0.0.1", pair_port, rep_port)
        .await
        .expect("client connect failed");

    // Test 1: Subscribe to topic
    let mut sub = client
        .subscribe("world/drone1/pose")
        .await
        .expect("subscribe failed");

    // Test 2: Perform RPC request
    #[derive(serde::Serialize)]
    struct EmptyParams {}
    let ping_res: bool = client
        .request("/Sim/Ping", &EmptyParams {})
        .await
        .expect("ping failed");
    assert!(ping_res);

    // Test 3: Receive published topic frame
    let msg_bytes = tokio::time::timeout(Duration::from_secs(2), sub.recv())
        .await
        .expect("timed out waiting for topic frame")
        .expect("failed to receive topic frame");

    let text: String = rmp_serde::from_slice(&msg_bytes).expect("decode payload");
    assert_eq!(text, "telemetry_data_123");

    server_thread.join().expect("server thread failed");
}
