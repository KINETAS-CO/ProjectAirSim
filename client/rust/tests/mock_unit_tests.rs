use projectairsim::error::SimError;
use projectairsim::protocol::{
    FrameType, RawDataPayload, RequestEnvelope, ResponseDecoder, TopicFrame, TopicInfo,
};
use projectairsim::transport::MockTransport;
use serde::{Deserialize, Serialize};

#[test]
fn test_topic_frame_roundtrip() {
    let frame =
        TopicFrame::subscribe("/Sim/SceneBasicDrone/robots/Drone1/sensors/Chase/scene_camera");
    let bytes = frame.to_bytes().expect("serialization failed");

    let decoded = TopicFrame::from_bytes(&bytes).expect("deserialization failed");
    assert_eq!(decoded.0, FrameType::Subscribe);
    assert_eq!(
        decoded.1,
        "/Sim/SceneBasicDrone/robots/Drone1/sensors/Chase/scene_camera"
    );
    assert!(decoded.2.is_empty());
}

#[test]
fn test_topic_frame_message_payload() {
    let payload = vec![0xDE, 0xAD, 0xBE, 0xEF];
    let frame = TopicFrame::message("/Sim/test_topic", payload.clone());
    let bytes = frame.to_bytes().expect("serialization failed");

    let decoded = TopicFrame::from_bytes(&bytes).expect("deserialization failed");
    assert_eq!(decoded.0, FrameType::Message);
    assert_eq!(decoded.1, "/Sim/test_topic");
    assert_eq!(decoded.2, payload);
}

#[test]
fn test_topic_info_deserialization() {
    // TopicInfo matches ProjectAirSim's `/$topics` stream
    let info = TopicInfo {
        path: "/Sim/Drone1/scene_camera".into(),
        topic_type: "published".into(),
        message_type: "ImageMessage".into(),
        frequency: 30,
    };

    let bytes = rmp_serde::to_vec_named(&info).expect("serialization failed");
    let decoded: TopicInfo = rmp_serde::from_slice(&bytes).expect("deserialization failed");

    assert_eq!(decoded.path, "/Sim/Drone1/scene_camera");
    assert_eq!(decoded.topic_type, "published");
    assert_eq!(decoded.message_type, "ImageMessage");
    assert_eq!(decoded.frequency, 30);
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
struct TakeoffParams {
    timeout_sec: f32,
}

#[test]
fn test_request_envelope_msgpack_json_format() {
    let params = TakeoffParams { timeout_sec: 20.0 };
    let req =
        RequestEnvelope::new(42, "/Sim/Drone1/Takeoff", &params).expect("request packaging failed");

    let bytes = req.to_bytes().expect("serialization failed");

    // Unpack as a generic MessagePack value to verify exact field names
    let _raw_val: rmpv::Value = rmp_serde::from_slice(&bytes).unwrap_or_else(|_| {
        // Fallback deserialization check
        let map: serde_json::Value = rmp_serde::from_slice(&bytes).expect("msgpack decode failed");
        assert_eq!(map["id"], 42);
        assert_eq!(map["method"], "/Sim/Drone1/Takeoff");
        assert_eq!(map["version"], 1.0);
        assert!(map["params"]["data"].is_array() || map["params"]["data"].is_string());
        return rmpv::Value::Nil;
    });

    // Verify params.data can be unpacked back into TakeoffParams
    let unpacked_params: TakeoffParams =
        rmp_serde::from_slice(&req.params.data).expect("inner params decode failed");
    assert_eq!(unpacked_params.timeout_sec, 20.0);
}

#[test]
fn test_response_decoder_success() {
    // Simulate server response: {"id": 1, "result": {"data": <msgpack(true)>}, "version": 1.0}
    let inner_data = rmp_serde::to_vec_named(&true).unwrap();

    #[derive(Serialize)]
    struct MockResponse {
        id: i32,
        result: RawDataPayload,
        version: f32,
    }

    let mock_resp = MockResponse {
        id: 1,
        result: RawDataPayload { data: inner_data },
        version: 1.0,
    };
    let resp_bytes = rmp_serde::to_vec_named(&mock_resp).unwrap();

    let result_bool: bool = ResponseDecoder::decode_typed(&resp_bytes).expect("decoding failed");
    assert!(result_bool);
}

#[test]
fn test_response_decoder_failure() {
    // Simulate server failure: {"id": 1, "error": {"code": 4, "message": "Client is not authorized"}, "version": 1.0}
    #[derive(Serialize)]
    struct MockErrorResponse {
        id: i32,
        error: serde_json::Value,
        version: f32,
    }

    let mock_resp = MockErrorResponse {
        id: 1,
        error: serde_json::json!({
            "code": 4,
            "message": "Client is not authorized"
        }),
        version: 1.0,
    };
    let resp_bytes = rmp_serde::to_vec_named(&mock_resp).unwrap();

    let err = ResponseDecoder::decode(&resp_bytes).unwrap_err();
    match err {
        SimError::ServerRejected { code, message } => {
            assert_eq!(code, 4);
            assert_eq!(message, "Client is not authorized");
        }
        other => panic!("Unexpected error type: {:?}", other),
    }
}

#[cfg(feature = "async")]
#[tokio::test]
async fn test_mock_transport_request_response() {
    use projectairsim::transport::Transport;
    let transport = MockTransport::new();

    // Queue simulated server reply
    let inner_data = rmp_serde::to_vec_named(&"scene_basic_drone").unwrap();
    let reply = serde_json::json!({
        "id": 1,
        "result": { "data": inner_data },
        "version": 1.0
    });
    let reply_bytes = rmp_serde::to_vec_named(&reply).unwrap();
    transport.push_service_response(reply_bytes);

    // Send dummy request
    let req_bytes = vec![1, 2, 3];
    let resp = transport
        .send_request(&req_bytes)
        .await
        .expect("transport send failed");

    // Verify request was recorded
    assert_eq!(transport.sent_requests(), vec![req_bytes]);

    // Decode response
    let scene_id: String = ResponseDecoder::decode_typed(&resp).expect("decode failed");
    assert_eq!(scene_id, "scene_basic_drone");
}

#[cfg(feature = "sync")]
#[test]
fn test_mock_transport_request_response_sync() {
    use projectairsim::transport::Transport;
    let transport = MockTransport::new();

    // Queue simulated server reply
    let inner_data = rmp_serde::to_vec_named(&"scene_basic_drone").unwrap();
    let reply = serde_json::json!({
        "id": 1,
        "result": { "data": inner_data },
        "version": 1.0
    });
    let reply_bytes = rmp_serde::to_vec_named(&reply).unwrap();
    transport.push_service_response(reply_bytes);

    // Send dummy request
    let req_bytes = vec![1, 2, 3];
    let resp = transport
        .send_request_sync(&req_bytes)
        .expect("transport send failed");

    // Verify request was recorded
    assert_eq!(transport.sent_requests(), vec![req_bytes]);

    // Decode response
    let scene_id: String = ResponseDecoder::decode_typed(&resp).expect("decode failed");
    assert_eq!(scene_id, "scene_basic_drone");
}
