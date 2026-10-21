#![cfg(feature = "async")]

use std::net::TcpListener;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use futures_util::StreamExt;
use nng::options::Options;
use nng::{Protocol, Socket};
use projectairsim::protocol::frame::{FrameType, TopicFrame, TopicInfo};
use projectairsim::protocol::request::RawDataPayload;
use projectairsim::protocol::response::RawResponseEnvelope;
use projectairsim::protocol::RequestEnvelope;
use serde_json::json;

fn get_available_port() -> u16 {
    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    listener.local_addr().expect("local addr").port()
}

#[test]
fn test_client_version_apis() {
    assert_eq!(
        projectairsim::async_api::Client::get_version(),
        env!("CARGO_PKG_VERSION")
    );
    assert_eq!(projectairsim::async_api::Client::get_nng_version(), "1.0");
}

#[cfg(feature = "async")]
#[tokio::test]
async fn test_async_client_controls_and_features() {
    let rep_port = get_available_port();
    let pair_port = get_available_port();
    let is_running = Arc::new(AtomicBool::new(true));

    // Spawn mock server for RPC (Rep0)
    let rep_socket = Socket::new(Protocol::Rep0).expect("create Rep0");
    let _ = rep_socket.set_opt::<nng::options::RecvTimeout>(Some(Duration::from_millis(50)));
    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    rep_socket.listen(&rep_url).expect("listen rep");

    let is_running_clone = Arc::clone(&is_running);
    let rpc_server = std::thread::spawn(move || {
        while is_running_clone.load(Ordering::Relaxed) {
            let msg = match rep_socket.recv() {
                Ok(m) => m,
                Err(nng::Error::TimedOut) => continue,
                Err(_) => break,
            };

            let req: RequestEnvelope =
                rmp_serde::from_slice(msg.as_slice()).expect("valid request envelope");
            let params: serde_json::Value =
                rmp_serde::from_slice(&req.params.data).unwrap_or(serde_json::Value::Null);

            let reply_data: Vec<u8> = match req.method {
                "/Sim/Ping" => rmp_serde::to_vec(&true).unwrap(),
                "/Sim/GetBuildCommitHash" => rmp_serde::to_vec(&"git-commit-hash-789abc").unwrap(),
                "/Sim/SetInteractiveFeature" => {
                    let feature_id = params["feature_id"].as_str().unwrap();
                    let enable = params["enable"].as_bool().unwrap();
                    assert_eq!(feature_id, "weather");
                    rmp_serde::to_vec(&enable).unwrap()
                }
                "/Sim/LoadScene" => {
                    let scene_cfg = params["scene_config"].as_str().unwrap();
                    assert!(scene_cfg.contains("TestSceneId"));
                    rmp_serde::to_vec(&"SceneLoadedRoot").unwrap()
                }
                "/Sim/Unsubscribe" => {
                    let paths = params["topic_paths"].as_array().unwrap();
                    assert!(!paths.is_empty());
                    rmp_serde::to_vec(&true).unwrap()
                }
                "/Sim/PriorityMethod" => rmp_serde::to_vec(&"PrioritySuccess").unwrap(),
                _ => rmp_serde::to_vec(&true).unwrap(),
            };

            let resp = RawResponseEnvelope {
                id: Some(req.id),
                version: Some(1.0),
                result: Some(RawDataPayload { data: reply_data }),
                error: None,
            };
            let resp_bytes = rmp_serde::to_vec(&resp).expect("valid response bytes");
            let _ = rep_socket.send(&resp_bytes);
        }
    });

    // Spawn mock server for Topics (Pair0)
    let pair_socket = Socket::new(Protocol::Pair0).expect("create Pair0");
    let _ = pair_socket.set_opt::<nng::options::RecvTimeout>(Some(Duration::from_millis(50)));
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");
    pair_socket.listen(&pair_url).expect("listen pair");

    let is_running_clone2 = Arc::clone(&is_running);
    let pair_server = std::thread::spawn(move || {
        while is_running_clone2.load(Ordering::Relaxed) {
            match pair_socket.recv() {
                Ok(msg) => {
                    // Echo or acknowledge topic frame
                    if let Ok(frame) = TopicFrame::from_bytes(&msg) {
                        if frame.frame_type() == FrameType::Subscribe {
                            if frame.topic() == "/Sim/TestTopic" {
                                // Send simulated message frame
                                let msg_frame = TopicFrame::message(
                                    "/Sim/TestTopic",
                                    rmp_serde::to_vec(&json!({ "sensor_val": 42.5 })).unwrap(),
                                );
                                let _ = pair_socket.send(&msg_frame.to_bytes().unwrap());
                            } else if frame.topic() == "/$topics" {
                                let infos = vec![TopicInfo {
                                    path: "/Sim/Drone1/pose".into(),
                                    topic_type: "published".into(),
                                    message_type: "PoseMessage".into(),
                                    frequency: 50,
                                }];
                                let dir_frame = TopicFrame::message(
                                    "/$topics",
                                    rmp_serde::to_vec(&infos).unwrap(),
                                );
                                let _ = pair_socket.send(&dir_frame.to_bytes().unwrap());
                            }
                        }
                    }
                }
                Err(nng::Error::TimedOut) => continue,
                Err(_) => break,
            }
        }
    });

    // Connect async Client
    let client = projectairsim::async_api::Client::connect_with_ports(
        "127.0.0.1",
        pair_port,
        rep_port,
    )
    .await
    .expect("connect client");

    // 1. Diagnostics & Liveness
    assert!(client.ping().await.expect("ping success"));
    let hash = client.get_build_commit_hash().await.expect("commit hash");
    assert_eq!(hash, "git-commit-hash-789abc");

    // 2. Interactive Features
    let enabled = client
        .set_interactive_feature("weather", true)
        .await
        .expect("set interactive feature");
    assert!(enabled);

    // 3. Priority Request
    let prio_res: String = client
        .request_priority("/Sim/PriorityMethod", &json!({ "arg": 1 }))
        .await
        .expect("priority request");
    assert_eq!(prio_res, "PrioritySuccess");

    // 4. Pub-Sub Topic Streaming (Subscription, Stream, Typed receive, Unsubscribe)
    let mut sub = client
        .subscribe("/Sim/TestTopic")
        .await
        .expect("subscribe to topic");
    assert_eq!(sub.topic(), "/Sim/TestTopic");

    // Receive typed message
    let typed_val: serde_json::Value = sub.recv_typed().await.expect("recv typed payload");
    assert_eq!(typed_val["sensor_val"], 42.5);

    // Stream trait validation via futures_util::StreamExt
    let mut stream_sub = client
        .subscribe("/Sim/TestTopic")
        .await
        .expect("stream_sub");
    let stream_frame = stream_sub
        .next()
        .await
        .expect("stream item")
        .expect("valid frame");
    assert_eq!(stream_frame.topic(), "/Sim/TestTopic");

    // Active subscriptions list
    let active = client.get_active_subscriptions().await;
    assert!(active.contains(&"/Sim/TestTopic".to_string()));

    // Publish to topic
    client
        .publish("/Sim/DroneCmd", &json!({ "throttle": 0.8 }))
        .await
        .expect("publish payload");

    // Unsubscribe single topic
    client
        .unsubscribe("/Sim/TestTopic")
        .await
        .expect("unsubscribe single");
    let active_after = client.get_active_subscriptions().await;
    assert!(!active_after.contains(&"/Sim/TestTopic".to_string()));

    // Subscribe and Unsubscribe All
    let _sub2 = client.subscribe("/Sim/Topic2").await.expect("sub2");
    let _sub3 = client.subscribe("/Sim/Topic3").await.expect("sub3");
    assert_eq!(client.get_active_subscriptions().await.len(), 2);
    client.unsubscribe_all().await.expect("unsubscribe all");
    assert_eq!(client.get_active_subscriptions().await.len(), 0);

    // 5. Request Load Scene
    let scene_root = client
        .request_load_scene("{\"id\": \"TestSceneId\"}")
        .await
        .expect("load scene");
    assert_eq!(scene_root, "SceneLoadedRoot");

    // 6. Topic Directory Discovery
    tokio::time::sleep(Duration::from_millis(150)).await;
    let paths = client.get_topic_paths().await.expect("topic paths");
    assert!(paths.contains(&"/Sim/Drone1/pose".to_string()));

    // Cancel all requests
    client.cancel_all_requests();

    // Cleanup servers
    is_running.store(false, Ordering::Relaxed);
    drop(client);
    let _ = rpc_server.join();
    let _ = pair_server.join();
}

#[cfg(feature = "sync")]
#[test]
fn test_sync_client_controls_and_features() {
    let rep_port = get_available_port();
    let pair_port = get_available_port();
    let is_running = Arc::new(AtomicBool::new(true));

    // Spawn mock server for RPC (Rep0)
    let rep_socket = Socket::new(Protocol::Rep0).expect("create Rep0");
    let _ = rep_socket.set_opt::<nng::options::RecvTimeout>(Some(Duration::from_millis(50)));
    let rep_url = format!("tcp://127.0.0.1:{rep_port}");
    rep_socket.listen(&rep_url).expect("listen rep");

    let is_running_clone = Arc::clone(&is_running);
    let rpc_server = std::thread::spawn(move || {
        while is_running_clone.load(Ordering::Relaxed) {
            let msg = match rep_socket.recv() {
                Ok(m) => m,
                Err(nng::Error::TimedOut) => continue,
                Err(_) => break,
            };

            let req: RequestEnvelope =
                rmp_serde::from_slice(msg.as_slice()).expect("valid request envelope");
            let params: serde_json::Value =
                rmp_serde::from_slice(&req.params.data).unwrap_or(serde_json::Value::Null);

            let reply_data: Vec<u8> = match req.method {
                "/Sim/Ping" => rmp_serde::to_vec(&true).unwrap(),
                "/Sim/GetBuildCommitHash" => rmp_serde::to_vec(&"git-commit-hash-sync-123").unwrap(),
                "/Sim/SetInteractiveFeature" => {
                    let feature_id = params["feature_id"].as_str().unwrap();
                    let enable = params["enable"].as_bool().unwrap();
                    assert_eq!(feature_id, "physics");
                    rmp_serde::to_vec(&enable).unwrap()
                }
                "/Sim/LoadScene" => {
                    let scene_cfg = params["scene_config"].as_str().unwrap();
                    assert!(scene_cfg.contains("SyncSceneCfg"));
                    rmp_serde::to_vec(&"SyncSceneRoot").unwrap()
                }
                "/Sim/Unsubscribe" => rmp_serde::to_vec(&true).unwrap(),
                _ => rmp_serde::to_vec(&true).unwrap(),
            };

            let resp = RawResponseEnvelope {
                id: Some(req.id),
                version: Some(1.0),
                result: Some(RawDataPayload { data: reply_data }),
                error: None,
            };
            let resp_bytes = rmp_serde::to_vec(&resp).expect("valid response bytes");
            let _ = rep_socket.send(&resp_bytes);
        }
    });

    // Spawn mock server for Topics (Pair0)
    let pair_socket = Socket::new(Protocol::Pair0).expect("create Pair0");
    let _ = pair_socket.set_opt::<nng::options::RecvTimeout>(Some(Duration::from_millis(50)));
    let pair_url = format!("tcp://127.0.0.1:{pair_port}");
    pair_socket.listen(&pair_url).expect("listen pair");

    let is_running_clone2 = Arc::clone(&is_running);
    let pair_server = std::thread::spawn(move || {
        while is_running_clone2.load(Ordering::Relaxed) {
            match pair_socket.recv() {
                Ok(msg) => {
                    if let Ok(frame) = TopicFrame::from_bytes(&msg) {
                        if frame.frame_type() == FrameType::Subscribe {
                            if frame.topic() == "/Sim/SyncTopic" {
                                let msg_frame = TopicFrame::message(
                                    "/Sim/SyncTopic",
                                    rmp_serde::to_vec(&"SyncMsgPayload").unwrap(),
                                );
                                let _ = pair_socket.send(&msg_frame.to_bytes().unwrap());
                            } else if frame.topic() == "/$topics" {
                                let infos = vec![TopicInfo {
                                    path: "/Sim/Rover1/velocity".into(),
                                    topic_type: "published".into(),
                                    message_type: "TwistMessage".into(),
                                    frequency: 20,
                                }];
                                let dir_frame = TopicFrame::message(
                                    "/$topics",
                                    rmp_serde::to_vec(&infos).unwrap(),
                                );
                                let _ = pair_socket.send(&dir_frame.to_bytes().unwrap());
                            }
                        }
                    }
                }
                Err(nng::Error::TimedOut) => continue,
                Err(_) => break,
            }
        }
    });

    // Connect blocking Client
    let client = projectairsim::blocking::Client::connect_with_ports(
        "127.0.0.1",
        pair_port,
        rep_port,
    )
    .expect("connect blocking client");

    // 1. Diagnostics & Liveness
    assert!(client.ping().expect("ping success"));
    let hash = client.get_build_commit_hash().expect("commit hash");
    assert_eq!(hash, "git-commit-hash-sync-123");

    // 2. Interactive Features
    let enabled = client
        .set_interactive_feature("physics", true)
        .expect("set interactive feature");
    assert!(enabled);

    // 3. Priority Request & Async Result
    let prio_res: bool = client
        .request_priority("/Sim/Ping", &json!({}))
        .expect("priority request");
    assert!(prio_res);

    let async_res = client.request_priority_async::<_, bool>("/Sim/Ping", &json!({}));
    async_res.wait().expect("wait async result");
    let val = async_res.get_result().expect("get result");
    assert!(val);

    // 4. Pub-Sub Topic Streaming (Iterator)
    let iter = client
        .subscribe_iter("/Sim/SyncTopic")
        .expect("subscribe_iter");
    assert_eq!(iter.topic(), "/Sim/SyncTopic");

    let frame = iter
        .recv_timeout(Duration::from_millis(500))
        .expect("recv frame from iterator");
    assert_eq!(frame.topic(), "/Sim/SyncTopic");
    let payload: String = rmp_serde::from_slice(frame.body()).expect("deserialize body");
    assert_eq!(payload, "SyncMsgPayload");

    // 5. Callback subscription
    let (tx, _rx) = std::sync::mpsc::channel();
    client
        .subscribe("/Sim/SyncTopic", move |frame| {
            let _ = tx.send(frame.topic().to_string());
        })
        .expect("subscribe callback");

    // Publish to topic
    client
        .publish("/Sim/RoverCmd", &json!({ "steer": 0.5 }))
        .expect("publish");

    // Active subscriptions
    let active = client.get_active_subscriptions();
    assert!(active.contains(&"/Sim/SyncTopic".to_string()));

    // Unsubscribe all
    client.unsubscribe_all().expect("unsubscribe all");

    // 6. Request Load Scene
    let root = client
        .request_load_scene("{\"id\": \"SyncSceneCfg\"}")
        .expect("load scene");
    assert_eq!(root, "SyncSceneRoot");

    // 7. Topic paths
    std::thread::sleep(Duration::from_millis(150));
    let paths = client.get_topic_paths().expect("topic paths");
    assert!(paths.contains(&"/Sim/Rover1/velocity".to_string()));

    // Cleanup
    is_running.store(false, Ordering::Relaxed);
    drop(client);
    let _ = rpc_server.join();
    let _ = pair_server.join();
}
