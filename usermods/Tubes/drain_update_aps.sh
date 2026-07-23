#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/firmware.sh"

update_ssid="${TUBES_UPDATE_SSID:-WLED-UPDATE}"
update_password="${TUBES_UPDATE_PASSWORD:-update1234}"
connect_timeout="${TUBES_BATCH_CONNECT_TIMEOUT:-20}"
wifi_device="${TUBES_WIFI_DEVICE:-}"
drain_dir=""
drained_count=0

find_wifi_device() {
  if [[ -n "$wifi_device" ]]; then
    return 0
  fi
  wifi_device="$(networksetup -listallhardwareports | awk '
    $0 == "Hardware Port: Wi-Fi" {
      getline
      sub(/^Device: /, "")
      print
      exit
    }
  ')"
  [[ -n "$wifi_device" ]] || fail "could not find the Wi-Fi interface"
}

connect_to_update_ap() {
  local deadline=$((SECONDS + connect_timeout))
  local probe_file="$upgrade_work_dir/selection-probe.json"
  while (( SECONDS < deadline )); do
    networksetup -setairportnetwork "$wifi_device" "$update_ssid" "$update_password" >/dev/null 2>&1 || true
    if "$curl_bin" --fail --silent --connect-timeout 1 --max-time 2 \
      "$device_url/json/si" -o "$probe_file"; then
      "$jq_bin" -e 'type == "object" and (.info.mac | type == "string")' "$probe_file" >/dev/null
      return 0
    fi
    sleep 1
  done
  return 1
}

archive_and_dismiss() {
  fetch_device_files "$upgrade_work_dir"
  load_raw_device_identity "$upgrade_work_dir/info.json"
  require_selected_access_point "$upgrade_work_dir/info.json"
  local observation_dir="$drain_dir/$device_mac"
  local visit_file="$observation_dir/visits"
  local visits=0
  mkdir -p "$observation_dir"
  cp "$upgrade_work_dir/info.json" "$observation_dir/info.json"
  cp "$upgrade_work_dir/cfg.json" "$observation_dir/cfg.json"
  chmod 600 "$observation_dir/info.json" "$observation_dir/cfg.json"
  if [[ -f "$visit_file" ]]; then
    visits="$(<"$visit_file")"
  fi
  visits=$((visits + 1))
  printf '%s\n' "$visits" > "$visit_file"
  if (( visits > 3 )); then
    fail "device $device_mac remained on $update_ssid after three local dismissals"
  fi
  echo "Closing update AP mac=$device_mac name='$device_name' release='${device_release:-none}'."
  dismiss_selected_device
  drained_count=$((drained_count + 1))
}

main_drain() {
  [[ $# -eq 0 ]] || { echo "Usage: ./drain_update_aps.sh" >&2; return 2; }
  require_tools
  command -v networksetup >/dev/null || fail "networksetup is required for the shared-SSID updater"
  find_wifi_device
  drain_dir="$backup_root/drain-$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$drain_dir"
  chmod 700 "$drain_dir"
  trap cleanup EXIT

  while true; do
    cleanup
    upgrade_work_dir="$(mktemp -d "${TMPDIR:-/tmp}/tubes-upgrade.XXXXXX")"
    if ! connect_to_update_ap; then
      break
    fi
    archive_and_dismiss
    sleep 2
  done

  cleanup
  upgrade_work_dir=""
  echo "DRAIN_COMPLETE observations=$drained_count archive=$drain_dir"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main_drain "$@"
fi
