#!/usr/bin/env bash
# Copyright (C) Microsoft Corporation.
# Copyright (C) 2025 IAMAI CONSULTING CORP
# MIT License.

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: ./build_rust_client.sh [debug|release] [--tests]

Build the standalone ProjectAirSim Rust client without building SimLibs.
  debug       Build Debug artifacts (default).
  release     Build Release artifacts.
  --tests     Build and run the mocked unit tests. A simulator is not required.
EOF
}

build_type=debug
run_tests=false

for arg in "$@"; do
    case "$arg" in
        debug) build_type=debug ;;
        release) build_type=release ;;
        --tests|--test) run_tests=true ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $arg" >&2; usage >&2; exit 2 ;;
    esac
done

root_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
manifest_path="$root_dir/client/rust/Cargo.toml"

cargo_flags=()
if [ "$build_type" = "release" ]; then
    cargo_flags+=("--release")
fi

echo "Building ProjectAirSim Rust Client (async)..."
cargo build --manifest-path "$manifest_path" --no-default-features --features async "${cargo_flags[@]}"

echo "Building ProjectAirSim Rust Client (sync)..."
cargo build --manifest-path "$manifest_path" --no-default-features --features sync "${cargo_flags[@]}"

if "$run_tests"; then
    if cargo nextest --version >/dev/null 2>&1; then
        echo "Running async tests with nextest..."
        CI=1 cargo nextest run --manifest-path "$manifest_path" --no-default-features --features async "${cargo_flags[@]}"
        echo "Running sync tests with nextest..."
        CI=1 cargo nextest run --manifest-path "$manifest_path" --no-default-features --features sync "${cargo_flags[@]}"
    else
        echo "Running async tests with cargo test..."
        cargo test --manifest-path "$manifest_path" --no-default-features --features async "${cargo_flags[@]}"
        echo "Running sync tests with cargo test..."
        cargo test --manifest-path "$manifest_path" --no-default-features --features sync "${cargo_flags[@]}"
    fi
fi
