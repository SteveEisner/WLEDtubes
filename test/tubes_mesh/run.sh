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

check_dig2go_peer_config() {
  local header="$repo_dir/usermods/Tubes/dig2go_peer_config.h"
  local macros="$build_dir/dig2go-peer-default.macros"
  "${CXX:-c++}" -std=c++17 -E -dM -x c++ -include "$header" /dev/null > "$macros"
  grep -q '^#define TUBES_ENABLE_DIG2GO_PEER_PROPAGATION 0$' "$macros"

  if "${CXX:-c++}" -std=c++17 -E -x c++ \
      -DTUBES_ENABLE_DIG2GO_PEER_PROPAGATION=1 \
      -DTUBES_DIG2GO_LEGACY_PULL_HOST=1 \
      -include "$header" /dev/null > /dev/null 2>&1; then
    echo "Dig2Go peer config accepted a host without dynamic enrollment" >&2
    return 1
  fi

  "${CXX:-c++}" -std=c++17 -E -x c++ \
    -DTUBES_ENABLE_DIG2GO_PEER_PROPAGATION=1 \
    -DTUBES_DIG2GO_LEGACY_PULL_HOST=1 \
    -DTUBES_DIG2GO_DYNAMIC_ENROLLMENT=1 \
    -include "$header" /dev/null > /dev/null
}

compile_and_run mesh_routing_test
compile_and_run device_report_protocol_test
compile_and_run legacy_auto_update_wire_test
compile_and_run legacy_pull_rendezvous_test
compile_and_run legacy_pull_host_lifecycle_test
compile_and_run legacy_propagation_model_test
compile_and_run modern_propagation_lease_test
compile_and_run modern_peer_request_test
compile_and_run firmware_target_contract_test
compile_and_run firmware_image_source_test
compile_and_run firmware_http_source_test
compile_and_run running_image_source_test
compile_and_run deferred_bpm_broadcast_test
compile_and_run rubber_band_beat_clock_test
compile_and_run downbeat_tracker_test
compile_and_run effect_chance_test
compile_and_run v3_protocol_test
compile_and_run dig2go_peer_propagation_test
check_dig2go_peer_config
