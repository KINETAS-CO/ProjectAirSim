# ProjectAirSim Rust Client Architecture

This document describes the internal software architecture of the `projectairsim` client library.

## Architectural Overview

The library connects Rust applications to the ProjectAirSim simulation server.
It provides two execution modes through Cargo features:
- `async`: An asynchronous mode based on the Tokio runtime.
- `sync`: A synchronous mode based on blocking threads and channels.

Both modes use the same wire protocol and data types.

## Network Transport Layer

The client communicates with the simulation server through Nanomsg Next Generation (NNG) sockets.
The server opens two network ports:

1. Port 8990 (RPC Services):
   This port uses the NNG `Req0` protocol for remote procedure calls.
   The client sends a `RequestEnvelope` encoded with MessagePack.
   The server returns a `RawResponseEnvelope` containing the result or error data.

2. Port 8989 (Topic Data Streams):
   This port uses the NNG `Pair0` protocol for high-throughput sensor telemetry.
   The server streams three-element MessagePack arrays called `TopicFrame` structures.
   Each frame contains a frame type identifier, a topic string, and raw payload bytes.

### Mock Transport for Unit Tests

The `MockTransport` trait allows complete simulation of network communication in unit tests.
Tests register canned responses for expected remote procedure calls.
Tests can run hermetically on developer machines without an installed simulator binary.

## Asynchronous Architecture (`async`)

In asynchronous mode, the `Client` wraps an `NngActor` that coordinates network traffic.
Because low-level NNG C FFI calls are synchronous blocking operations, the actor manages
two dedicated OS worker threads (`airsim-rpc-actor` and `airsim-topics-actor`) rather than
running directly on Tokio's cooperative task threads.

Asynchronous Tokio channels bridge the actor's dedicated threads to async client code:
- An `mpsc` queue and paired `oneshot` reply channels route serialized RPC requests over `Req0`.
- An incoming `Pair0` socket reader forwards topic payloads to registered `broadcast` channel receivers.
Application code calls `subscribe` to receive these broadcast streams.

## Synchronous Architecture (`sync`)

In synchronous mode, the library mirrors the C++ client design while providing thread-safe concurrency.
Each asynchronous request returns a cloneable `AsyncResult<T>` handle.
The handle is backed by an `Arc`-managed synchronization state with a `Condvar` and atomic status flags:

- `is_done()` provides lock-free polling of completion status.
- `wait_timeout()` blocks the caller for up to a specified duration.
- `get_result()` blocks until completion and retrieves the unpacked value.
- Dropping the worker's sender handle automatically marks pending results as cancelled.

Topic stream handling is managed by `BlockingTopicsHub`, which runs a background listener thread.
Callbacks are stored as `Arc<dyn Fn(TopicFrame) + Send + Sync>` closures and snapshotted under a brief
lock before invocation, ensuring user callbacks can safely invoke client APIs without re-entrant deadlocks.

## Configuration Parser

The client loads scene files written in JSON with Comments (JSONC).
The `config` module removes single-line and multi-line comments from the file text.
The parser then deserializes the clean JSON text into typed scenario data structures.

## Logging Subsystem

The logging module integrates with the Rust `tracing` ecosystem.
It provides semantic parity with the C++ `pasc::Log` subsystem.

The module provides a `Severity` enum matching C++ severity levels.
The `init_logging` function configures a subscriber with environment variable filtering.

Applications can call `set_log_sink` with a custom callback closure.
When an active sink exists, the custom layer sends formatted events to the callback.
At the same time, the subscriber suppresses default console output to prevent duplicated messages.
If the application calls `clear_log_sink`, standard console printing resumes immediately.
