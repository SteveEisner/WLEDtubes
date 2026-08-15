#!/usr/bin/env bash
set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$test_dir/../.." && pwd)"
build_dir="$(mktemp -d "${TMPDIR:-/tmp}/wled-tubes-mesh-tests.XXXXXX")"
trap 'rm -rf "$build_dir"' EXIT

compile_and_run() {
  local test_name="$1"
  "${CXX:-c++}" \
    -std=c++17 \
    -Wall \
    -Wextra \
    -Werror \
    -pedantic \
    -I"$repo_dir/usermods/Tubes" \
    -I"$test_dir" \
    "$test_dir/$test_name.cpp" \
    -o "$build_dir/$test_name"
  "$build_dir/$test_name"
}

compile_and_run mesh_routing_test
compile_and_run device_report_protocol_test
compile_and_run firmware_target_contract_test
compile_and_run firmware_image_source_test
compile_and_run firmware_http_source_test
compile_and_run firmware_update_session_test
compile_and_run running_image_source_test
compile_and_run deferred_bpm_broadcast_test
