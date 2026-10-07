# Branch Review: `feature/rust-client` vs `main`

- **Branch**: `feature/rust-client`
- **Base**: `main`
- **Scope**: 66 files changed, +14,552 lines (Rust client implementation, build workflows, documentation)
- **Review Criteria**: Functionality, Robustness, Simplicity, Readability

---

## 1. Executive Summary

The `feature/rust-client` branch introduces a native, idiomatic Rust client library (`projectairsim`) for ProjectAirSim located under [`client/rust/`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust). The implementation enables both asynchronous (Tokio-native) and synchronous (blocking) client workflows with full simulation actor and scenario parity with the existing C++ and Python client libraries.

### Key Strengths
- **Dual Async/Sync Support**: Complete Tokio async implementation alongside a blocking client utilizing channels and an `AsyncResult` handle model matching C++.
- **Comprehensive API Parity**: Implements Drones, Wheeled Vehicles, Rovers, Environment Actors, Static Sensor Actors, World management, time-stepping, debug markers, and `.binvox` export.
- **Offline Mocking & Hermetic Testing**: Features an in-memory [`MockTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs) and mock socket integration test suite that tests wire serialization without requiring an active Unreal simulator.
- **Code Cleanliness**: Passes `cargo clippy --all-targets --all-features -- -D warnings` with zero warnings.

---

## 2. Functionality Review

### 2.1 Actor and Vehicle APIs
The crate implements full API parity with ProjectAirSim simulation servers across both execution modes:
- **[`Drone`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/drone.rs)** / **[`blocking::Drone`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/drone.rs)**: Takeoff, landing, hover, return-to-home, velocity control (NED and Body frames), position navigation, yaw rate orientation, VTOL flight mode toggles, camera image capture, FOV/optics tuning, actuator fault injection, and telemetry subscriptions.
- **[`WheeledVehicle`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/wheeled_vehicle.rs)** / **[`blocking::WheeledVehicle`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/wheeled_vehicle.rs)**: Throttle, steering angle, brake control, and kinematics retrieval.
- **[`Rover`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/rover.rs)** / **[`blocking::Rover`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/rover.rs)**: Differential drive controls, heading/position maneuvers, and arming.
- **[`EnvActor`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/env_actor.rs)** / **[`blocking::EnvActor`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/env_actor.rs)**: 6-DoF trajectory execution and articulated joint rotation angles and rates.
- **[`StaticSensorActor`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/static_sensor.rs)** / **[`blocking::StaticSensorActor`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/static_sensor.rs)**: Multi-modal camera image capture from stationary towers.
- **[`World`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/world.rs)** / **[`blocking::World`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/world.rs)**: Steppable clock management, pause/resume, scene loading, weather visual effects, time-of-day control, 3D debug shapes (points, arrows, lines, transforms), and 3D voxel grid extraction.

### 2.2 Configuration & JSONC Parser
[`client/rust/src/config.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/config.rs) implements a custom comment-stripping parser that removes line (`//`) and block (`/* */`) comments while preserving string literals and escape sequences. [`load_scene_config`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/config.rs#L132) recursively resolves and inlines referenced `robot-config` and `env-actor-config` JSON files matching C++ `World::Impl::LoadSceneConfig`.

### 2.3 Identified Functional Deficiencies
1. **Broken Compilation for Sync-Only Feature (`--no-default-features --features sync`)**:
   In [`client/rust/tests/client_controls_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/client_controls_tests.rs):
   - Line 6 imports `use futures_util::StreamExt;` unconditionally, but `futures-util` is an optional dependency enabled only under `feature = "async"`.
   - Lines 23 & 26 invoke `projectairsim::async_api::Client::get_version()` without `#[cfg(feature = "async")]`.
   - Line 38 defines `#[tokio::test] async fn test_async_client_controls_and_features` without `#[cfg(feature = "async")]`.
   Running `cargo test --no-default-features --features sync` consequently fails to compile.
2. **Default `cargo test` Skips Blocking Suite**:
   [`client/rust/tests/blocking_api_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/blocking_api_tests.rs) is guarded by `#![cfg(feature = "sync")]` at line 1. Because the default feature set is `default = ["async"]`, running standard `cargo test` executes 0 tests in this file. It only runs if `--all-features` or `--features sync` is explicitly passed.
3. **No-op Cancellation in Blocking Client**:
   [`client/rust/src/blocking/client.rs:L349-351`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L349-L351) defines:
   ```rust
   pub fn cancel_all_requests(&self) {
       // In blocking mode, in-flight requests on detached threads complete or are ignored by the caller.
   }
   ```
   Unlike [`NngActor::cancel_all_requests`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_actor.rs#L233), in-flight blocking requests cannot be cancelled or drained.

---

## 3. Robustness & Concurrency

### 3.1 Mutex Contention in [`AsyncResult`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs)
In [`client/rust/src/blocking/async_result.rs:L25-32`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L25-L32) and [`L41-51`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L41-L51):
```rust
pub fn wait(&mut self) -> Result<()> {
    let mut guard = match self.cached.lock() {
        Ok(g) => g,
        Err(poisoned) => poisoned.into_inner(),
    };
    if guard.is_none() {
        let res = self.receiver.recv().map_err(|_| SimError::Cancelled)?;
        *guard = Some(res);
    }
    // ...
}
```
**Issue**: `guard` holds the lock on `self.cached` across the blocking `self.receiver.recv()` call. If another thread calls non-blocking status check `is_done(&self)` ([`L71-90`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L71-L90)), `is_done` blocks attempting to acquire `self.cached.lock()`.
**Fix**: Perform the channel receive outside the cached lock, or make `is_done()` attempt `try_lock()`.

### 3.2 Unbounded Thread Spawning in Blocking [`request_async`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs)
In [`client/rust/src/blocking/client.rs:L307`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L307):
```rust
let transport = Arc::clone(&self.transport);
std::thread::spawn(move || {
    let res = transport
        .send_request_sync(&req_bytes)
        .and_then(|resp_bytes| ResponseDecoder::decode_typed::<R>(&resp_bytes));
    let _ = tx.send(res);
});
```
Every invocation of `request_async` in blocking mode creates a new detached OS thread. High-frequency RPCs or tight polling loops can cause thread exhaustion. Additionally, multiple detached threads concurrently contend on `req_socket.lock()`, leading to non-deterministic request order across the half-duplex NNG socket.

### 3.3 Hardcoded Port Binding in Unit/Integration Tests
While [`tests/client_controls_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/client_controls_tests.rs#L15-L18) dynamically binds ephemeral ports using `TcpListener::bind("127.0.0.1:0")`, several test suites hardcode static ports:
- [`tests/nng_actor_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/nng_actor_tests.rs#L12): `18990`, `18989`
- [`tests/blocking_api_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/blocking_api_tests.rs#L24): `28990`, `28989`
- [`tests/actor_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/actor_tests.rs#L11): `38990`, `38989`
- [`tests/drone_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/drone_tests.rs#L14): `48990`, `48989`, `58990`
- [`tests/world_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/world_tests.rs#L167): `49990`, `49989`, `59990`

When running tests in parallel (e.g. `cargo nextest` or concurrent CI jobs), fixed port numbers risk intermittent `AddressInUse` failures. All test suites should adopt the ephemeral port allocator.

---

## 4. Simplicity & API Design

### 4.1 Parameter Struct Consolidation
Commit `108fb6d` centralized request parameters into [`client/rust/src/protocol/params.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/protocol/params.rs). This significantly reduced duplicate structs across `async_api` and `blocking` modules, keeping message schemas unified.

### 4.2 Module Re-exports
[`client/rust/src/lib.rs:L31-41`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/lib.rs#L31-L41) provides clean conditional re-exports:
- Default (`async` enabled): Exposes async types at the crate root (`projectairsim::Client`, `projectairsim::Drone`), with blocking types available under `projectairsim::blocking`.
- Sync-only (`sync` enabled, `async` disabled): Automatically promotes blocking types to the crate root.

### 4.3 Incomplete `Transport` Trait Abstraction
[`client/rust/src/transport/mod.rs:L15-39`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L15-L39) defines a `Transport` trait, but only [`MockTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L53) implements it. [`NngTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_transport.rs#L10) implements inherent methods rather than the trait. Either `NngTransport` should implement `Transport` to enable full polymorphism, or the trait should be simplified to target mock requirements.

---

## 5. Readability & Documentation

### 5.1 Codebase Styling
The Rust code adheres strictly to Rust standard style guides and passes strict Clippy enforcement. Module structure is intuitive, and comments/docstrings are clear and informative.

### 5.2 Documentation Inaccuracies

1. **[`client/rust/docs/API_REFERENCE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/API_REFERENCE.md)**:
   - Lines 20–24: Documents `client.get_client_version()` and `client.get_server_version()`. These do not exist as instance methods. The actual implementations are static associated functions [`Client::get_version()`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/client.rs#L92), [`Client::get_nng_version()`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/client.rs#L97), and the async method [`client.get_build_commit_hash()`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/client.rs#L328).
   - Line 194: Documents `actor.set_trajectory(traj_name, loop, offset_x, offset_y, offset_z, pitch, roll, yaw, duration)`. In [`async_api/env_actor.rs:L39-50`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/env_actor.rs#L39-L50), `time_offset` is missing from the documentation, `duration` does not exist, and the order of `roll` and `pitch` is inverted.
2. **[`client/rust/docs/ARCHITECTURE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/ARCHITECTURE.md)**:
   - Line 27: Claims `TopicFrame` contains `"a topic string, a time stamp, and raw payload bytes"`. The actual protocol ([`protocol/frame.rs:L15-20`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/protocol/frame.rs#L15-L20)) defines `[FrameType, String, Vec<u8>]`.
3. **[`.gitignore`](file:///home/nrakoski/git_ws/ProjectAirSim/.gitignore)**:
   - Lines 33 and 395 both add `client/rust/target/`. The second occurrence is redundant.

---

## 6. Actionable Recommendations

| Priority | Category | File & Location | Recommendation |
|:---|:---|:---|:---|
| **High** | Functionality | [`tests/client_controls_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/client_controls_tests.rs#L6) | Gate `use futures_util::StreamExt;`, `test_async_client_controls_and_features`, and `async_api` calls behind `#[cfg(feature = "async")]`. |
| **High** | Robustness | [`src/blocking/async_result.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L25-L56) | Release `self.cached` lock before invoking blocking `self.receiver.recv()` so concurrent `is_done()` calls remain non-blocking. |
| **Medium** | Robustness | [`tests/*_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests) | Replace hardcoded ports (`18990`, `28990`, `38990`, `48990`, etc.) with ephemeral port allocator `TcpListener::bind("127.0.0.1:0")`. |
| **Medium** | Architecture | [`src/blocking/client.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L307) | Replace unbounded `std::thread::spawn` in `request_async` with a worker queue or bounded thread pool. |
| **Medium** | Documentation | [`docs/API_REFERENCE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/API_REFERENCE.md#L20-L24) | Align `Client` versioning methods and `EnvActor::set_trajectory` signature with actual Rust code. |
| **Medium** | Documentation | [`docs/ARCHITECTURE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/ARCHITECTURE.md#L27) | Correct `TopicFrame` layout description from `[topic, timestamp, payload]` to `[frame_type, topic, payload]`. |
| **Low** | Cleanup | [`.gitignore`](file:///home/nrakoski/git_ws/ProjectAirSim/.gitignore#L395) | Deduplicate `client/rust/target/` entry. |
