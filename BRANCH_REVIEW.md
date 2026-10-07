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

---

# Branch Review (Round 2): `feature/rust-client` vs `main`

- **Branch**: `feature/rust-client`
- **Base**: `main`
- **Scope**: 66 files changed, +14,552 lines (Native Rust client library `projectairsim`, test harness, build scripts, documentation)
- **Review Criteria**: Functionality, Robustness, Simplicity, Readability

---

## 1. Executive Summary

This second-round review evaluates branch `feature/rust-client` against `main`, incorporating verification of fixes implemented for the Round 1 findings (both committed and working-tree changes).

### Verification of Round 1 Fixes
1. **Compilation Under Sync-Only Feature (`--no-default-features --features sync`)**: **Verified Fixed**. [`client/rust/tests/client_controls_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/client_controls_tests.rs#L6) now properly guards `futures_util::StreamExt`, the async test case, and `async_api::Client::get_version()` behind `#[cfg(feature = "async")]`. The sync test suite compiles cleanly and passes.
2. **Ephemeral Port Allocation in Tests**: **Verified Fixed**. All integration tests across [`actor_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/actor_tests.rs#L9-L12), [`blocking_api_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/blocking_api_tests.rs#L11-L14), [`client_controls_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/client_controls_tests.rs#L16-L19), [`drone_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/drone_tests.rs#L12-L15), [`nng_actor_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/nng_actor_tests.rs#L10-L13), and [`world_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/world_tests.rs#L19-L22) now dynamically acquire ephemeral ports via `TcpListener::bind("127.0.0.1:0")`.
3. **Non-Blocking Mutex Starvation in [`AsyncResult`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs)**: **Verified Fixed**. [`AsyncResult::is_done`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L133-L158) now uses `try_lock()` on the receiver mutex. Concurrent `is_done()` calls return immediately (`false`) rather than hanging when a background waiter is blocked inside `recv()`. Moreover, [`wait`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L24) and [`wait_timeout`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L71) now take `&self` rather than `&mut self`.
4. **Worker Queue in Blocking Client**: **Verified Fixed**. A dedicated worker [`BlockingRpcWorker`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L257-L346) was introduced with prioritized queues (`priority` and `normal`) and condition-variable synchronization, eliminating unbounded OS thread spawning and protecting the half-duplex `Req0` socket from multi-thread interleaving.
5. **[`Transport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L15) Trait Implementation**: **Verified Fixed**. [`NngTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_transport.rs#L103-L134) now implements the [`Transport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L15) trait alongside [`MockTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L53).
6. **Documentation & .gitignore Alignment**: **Verified Fixed**. The duplicate `.gitignore` target directory was removed, `TopicFrame` layout in [`ARCHITECTURE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/ARCHITECTURE.md#L27) was corrected, and [`API_REFERENCE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/API_REFERENCE.md#L20-L27) was aligned with actual method signatures.

### Key New & Remaining Findings in Round 2
- **Deadlock Hazard in Blocking Callbacks**: [`BlockingTopicsHub`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L112-L118) executes user-supplied callbacks while holding the `callbacks` `Mutex`. Any attempt by the callback to call [`client.subscribe_callback`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L156), [`client.unsubscribe`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L180), or [`client.get_active_subscriptions`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L217) results in immediate deadlock.
- **Worker Drain Behavior on Shutdown**: [`BlockingRpcWorker::stop`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L333-L346) does not clear or cancel pending queue items. Dropping the blocking client waits for all queued jobs to execute across the network instead of cancelling pending jobs immediately like [`NngActor::shutdown`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_actor.rs#L269-L297).
- **Default Test Coverage**: [`client/rust/Cargo.toml`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/Cargo.toml#L9) defaults to `default = ["async"]`. Running standard `cargo test` skips the blocking test suite entirely.
- **Clippy Warning on `--no-default-features`**: Running `cargo clippy --all-targets --no-default-features -- -D warnings` fails due to unused imports in [`mock_transport.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L2) and [`transport/mod.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L12) when both `async` and `sync` features are disabled.
- **Lack of Dependency Injection with `Transport`**: Although [`NngTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_transport.rs#L10) and [`MockTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L11) implement [`Transport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L15), neither [`async_api::Client`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/client.rs#L84) nor [`blocking::Client`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L356) can accept a [`MockTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L11) instance.

---

## 2. Functionality Review

### 2.1 Feature Permutations & Test Verification
All primary test matrices were executed and validated:
- **`cargo test --all-features`**: 23 tests pass across unit and integration targets (0 failed, 0 ignored).
- **`cargo test --no-default-features --features sync`**: 18 tests pass (async-only tests are properly ignored).
- **`cargo test --no-default-features --features async`**: 18 tests pass (sync-only tests are properly ignored).
- **`cargo clippy --all-targets --all-features -- -D warnings`**: 0 warnings.
- **`cargo clippy --all-targets --no-default-features --features sync -- -D warnings`**: 0 warnings.
- **`cargo clippy --all-targets --no-default-features --features async -- -D warnings`**: 0 warnings.

### 2.2 Default Feature Set Skips Blocking Tests
In [`client/rust/Cargo.toml:L8-13`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/Cargo.toml#L8-L13):
```toml
[features]
default = ["async"]
async = ["dep:tokio", "dep:futures-util", "dep:async-trait"]
sync = []
full = ["async", "sync"]
```
Because `default` only includes `"async"`, running plain `cargo test` in `client/rust/` ignores [`tests/blocking_api_tests.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/tests/blocking_api_tests.rs). Because the `sync` feature has zero external dependencies (it uses only `std` threads and synchronization primitives), enabling both by default (`default = ["async", "sync"]`) would ensure complete regression testing on standard `cargo test` invocations without increasing external dependencies.

### 2.3 Clippy Failure Under `--no-default-features`
When compiling without default features (`cargo clippy --all-targets --no-default-features -- -D warnings`):
```text
error: unused imports: `Result` and `SimError`
 --> client/rust/src/transport/mock_transport.rs:2:20
  |
2 | use crate::error::{Result, SimError};
```
In [`client/rust/src/transport/mock_transport.rs:L2`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L2) and [`client/rust/src/transport/mod.rs:L12`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L12), all methods using `Result` are gated behind `#[cfg(feature = "async")]` or `#[cfg(feature = "sync")]`. If neither feature is enabled, these imports trigger compiler warnings and break CI gates configured with `-D warnings`.

---

## 3. Robustness & Concurrency

### 3.1 Deadlock Hazard in [`BlockingTopicsHub`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs) Callbacks
In [`client/rust/src/blocking/client.rs:L112-L118`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L112-L118):
```rust
// 1. Specific callbacks
if let Ok(mut cbs_lock) = t_callbacks.lock() {
    if let Some(cbs) = cbs_lock.get_mut(frame.topic()) {
        for cb in cbs.iter_mut() {
            cb(frame.clone());
        }
    }
}
```
**Issue**: `cbs_lock` holds an exclusive `MutexGuard` over `t_callbacks` while invoking the closure `cb`.
If the user's callback closure calls any client method that accesses `self.callbacks`:
- [`client.subscribe_callback(topic, ...)` (L159)](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L159)
- [`client.unsubscribe(topic)` (L183)](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L183)
- [`client.unsubscribe_all()` (L191)](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L191)
- [`client.get_active_subscriptions()` (L219)](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L219)

The thread deadlocks because `std::sync::Mutex` is not re-entrant. Furthermore, while the callback is running, no other threads can subscribe or unsubscribe.

**Fix**: Decouple dispatch from the registration lock. For instance, clone references to callbacks or use a deferred action queue, or document explicitly that callbacks must not synchronously mutate topic subscriptions on the same client.

### 3.2 Inconsistent Shutdown Drain in [`BlockingRpcWorker`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs)
Compare [`BlockingRpcWorker::stop`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L333-L345) with [`NngActor::shutdown`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_actor.rs#L269-L297):
```rust
// In BlockingRpcWorker::stop (L333-L345):
pub fn stop(&mut self) {
    let (lock, cvar) = &*self.queue;
    if let Ok(mut state) = lock.lock() {
        if !state.is_running {
            return;
        }
        state.is_running = false;
        cvar.notify_all();
    }
    if let Some(thread) = self.thread.take() {
        let _ = thread.join();
    }
}
```
When `state.is_running` is set to `false`, the worker thread loop ([`L281-292`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L281-L292)) does not terminate until `state.priority.is_empty() && state.normal.is_empty()`. Consequently, any requests queued prior to client drop will continue to be sent across the network sequentially, blocking `drop(client)` and `thread.join()`.

In contrast, [`NngActor::shutdown`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_actor.rs#L279-L284) drains `priority` and `normal`, immediately replying with [`SimError::Cancelled`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/error.rs#L40) and exiting cleanly.

**Fix**: Clear the queues in `BlockingRpcWorker::stop`:
```rust
if let Ok(mut state) = lock.lock() {
    if !state.is_running {
        return;
    }
    state.is_running = false;
    state.priority.clear();
    state.normal.clear();
    cvar.notify_all();
}
```
Clearing the closures automatically drops their embedded `tx` channels, which immediately delivers [`SimError::Cancelled`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/error.rs#L40) to any waiting [`AsyncResult`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L10) handles without incurring network delays.

### 3.3 [`AsyncResult::wait_timeout`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs) Lock Contention Across Threads
In [`client/rust/src/blocking/async_result.rs:L85-88`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L85-L88):
```rust
let rx = match self.receiver.lock() {
    Ok(g) => g,
    Err(poisoned) => poisoned.into_inner(),
};
```
If multiple threads share an `Arc<AsyncResult<T>>`:
- Thread A calls `wait_timeout(Duration::from_secs(10))` and holds `self.receiver`.
- Thread B calls `wait_timeout(Duration::from_millis(50))`.
Thread B blocks unconditionally on `self.receiver.lock()`, failing to respect its requested 50ms timeout until Thread A releases the lock.

Additionally, in [`wait`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L56), if `rx.recv()` fails because the sender disconnected:
```rust
let res = rx.recv().map_err(|_| SimError::Cancelled)?;
```
Because `?` returns early, `self.cached` remains `None`. Subsequent calls to `wait()` must acquire `self.receiver.lock()` again rather than hitting the fast-path in `self.cached`. Setting `*guard = Some(Err(SimError::Cancelled))` permanently marks the handle finished and short-circuits future checks.

---

## 4. Simplicity & API Design

### 4.1 Incomplete Dependency Injection with [`Transport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L15)
While [`NngTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_transport.rs#L10) now implements [`Transport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L15):
- [`async_api::Client`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/client.rs#L85) hardcodes `actor: Arc<NngActor>`.
- [`blocking::Client`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L357) hardcodes `transport: Arc<NngTransport>`.
- Neither client offers a constructor accepting `Arc<dyn Transport>` or a generic `T: Transport`.

As a result, [`MockTransport`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L11) cannot be used to construct a `Client`, `Drone`, or `World`. All actor unit/integration tests must bind real TCP sockets over localhost. For true mock testing, either allow `Client` to take an abstract transport or make `NngActor` wrap `Arc<dyn Transport>`.

### 4.2 Lack of Per-Handle Cancellation on [`AsyncResult`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs)
The C++ client provides [`AsyncResult::Cancel()`](file:///home/nrakoski/git_ws/ProjectAirSim/client/cpp/ProjectAirsimClientLib/Include/ProjectAirsimClient/AsyncResult.h#L67), which cancels an individual task.
In the Rust client:
- `Client::cancel_all_requests()` ([`client.rs:L479`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L479)) exists.
- `Drone::cancel_last_task()` ([`drone.rs:L112`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/drone.rs#L112)) exists.
- However, [`AsyncResult<T>`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L10) does not expose a `.cancel()` method because it is a pure receiver wrapper without a backward channel to the worker queue.

Documenting this architectural difference in [`client/rust/docs/API_REFERENCE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/API_REFERENCE.md) will prevent confusion for developers transitioning from C++.

---

## 5. Readability & Documentation

### 5.1 [`ARCHITECTURE.md`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/ARCHITECTURE.md#L37-L40) Accuracy
In [`client/rust/docs/ARCHITECTURE.md:L37-L40`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/ARCHITECTURE.md#L37-L40):
> *"In asynchronous mode, the `Client` spawns background Tokio tasks. One task manages the `Req0` socket for request and response handling. A second task manages the `Pair0` socket for incoming topic frames."*

**Correction**: [`NngActor`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/nng_actor.rs#L62-L64) spawns dedicated **OS threads** (`std::thread::Builder::new().spawn(...)`), not Tokio tasks (`tokio::spawn`). This design is deliberate and correct because NNG socket operations perform blocking FFI calls, which would block cooperative Tokio runtime workers. The documentation should be updated to state that dedicated OS threads are spawned and bridged to Tokio via asynchronous channels.

### 5.2 Code Comments and Styling
- The codebase consistently adheres to standard Rust formatting (`rustfmt`) and idiomatic patterns.
- Error handling with [`thiserror`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/error.rs#L19) is clean and maps well to ProjectAirSim C++ [`Status`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/error.rs#L6) codes.
- The state-machine JSONC parser in [`client/rust/src/config.rs`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/config.rs#L8) is well documented and handles escape sequences properly.

---

## 6. Actionable Recommendations

| Priority | Category | File & Location | Recommendation |
|:---|:---|:---|:---|
| **High** | Robustness | [`src/blocking/client.rs:L112-L118`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L112-L118) | Release the `callbacks` `Mutex` before invoking user callbacks (`cb(frame)`), or clone callback lists, to avoid deadlocks when callbacks invoke client APIs. |
| **High** | Robustness | [`src/blocking/client.rs:L333-L345`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L333-L345) | Clear `state.priority` and `state.normal` in `BlockingRpcWorker::stop()` so pending unstarted requests are cancelled immediately without delaying client shutdown. |
| **Medium** | Quality | [`src/transport/mock_transport.rs:L2`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mock_transport.rs#L2)<br>[`src/transport/mod.rs:L12`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/transport/mod.rs#L12) | Gate `use crate::error::{Result, SimError};` behind `#[cfg(any(feature = "async", feature = "sync"))]` to ensure `cargo clippy --all-targets --no-default-features -- -D warnings` passes. |
| **Medium** | Ergonomics | [`Cargo.toml:L9`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/Cargo.toml#L9) | Set `default = ["async", "sync"]` so standard `cargo test` runs both async and blocking test suites out-of-the-box without extra flags. |
| **Medium** | Robustness | [`src/blocking/async_result.rs:L56-62`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/async_result.rs#L56-L62) | Cache `Some(Err(SimError::Cancelled))` in `self.cached` when `rx.recv()` disconnects so subsequent calls immediately hit the cache. |
| **Low** | Architecture | [`src/async_api/client.rs:L84`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/async_api/client.rs#L84)<br>[`src/blocking/client.rs:L356`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/src/blocking/client.rs#L356) | Allow `Client` to accept an abstract `Arc<dyn Transport>` to enable mock integration testing without real sockets. |
| **Low** | Documentation | [`docs/ARCHITECTURE.md:L37`](file:///home/nrakoski/git_ws/ProjectAirSim/client/rust/docs/ARCHITECTURE.md#L37) | Clarify that `NngActor` spawns dedicated OS worker threads (not cooperative Tokio tasks) with channel bridges. |
