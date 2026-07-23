#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/firmware.sh"

mesh_reporter="${TUBES_MESH_REPORTER:-$script_dir/mesh_device_report.py}"
update_ssid="${TUBES_UPDATE_SSID:-WLED-UPDATE}"
update_password="${TUBES_UPDATE_PASSWORD:-update1234}"
selection_timeout="${TUBES_SELECTION_TIMEOUT:-25}"
verification_timeout="${TUBES_VERIFICATION_TIMEOUT:-45}"
wifi_device="${TUBES_WIFI_DEVICE:-}"

usage_fast() {
  cat <<'EOF'
Usage:
  ./upgrade_fast.sh PROFILE SERIAL_DEVICE [--loop]

PROFILE is dig2go, athom-c3, christmas, or golden. It is the operator's physical
identification of the devices being upgraded. The script discovers and locks
every write to the selected MAC. The USB controller must already run firmware
with device-report support.
EOF
}

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

wait_for_selected_device() {
  local deadline=$((SECONDS + selection_timeout))
  local probe_file="$upgrade_work_dir/selection-probe.json"

  echo "Double-click one $profile_name now. Waiting for $update_ssid..."
  while (( SECONDS < deadline )); do
    networksetup -setairportnetwork "$wifi_device" "$update_ssid" "$update_password" >/dev/null 2>&1 || true
    if "$curl_bin" --fail --silent --connect-timeout 1 --max-time 2 \
      "$device_url/json/si" -o "$probe_file"; then
      "$jq_bin" -e 'type == "object" and (.info.mac | type == "string")' "$probe_file" >/dev/null
      return 0
    fi
    sleep 1
  done
  fail "selection timed out before a device served $update_ssid"
}

select_and_connect() {
  "$python_bin" "$mesh_reporter" select "$serial_device"
  wait_for_selected_device
}

enroll_selected_device() {
  fetch_device_files "$upgrade_work_dir"
  load_raw_device_identity "$upgrade_work_dir/info.json"
  require_selected_access_point "$upgrade_work_dir/info.json"
  select_profile "$requested_profile"
  save_device_profile "$profile_id"
  profile_source="operator-confirmed hardware, enrolled MAC"
  print_identity "$upgrade_work_dir/info.json"
}

wait_for_reboot_start() {
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if ! "$curl_bin" --fail --silent --connect-timeout 1 --max-time 2 \
      "$device_url/json/si" >/dev/null; then
      return 0
    fi
    sleep 1
  done
  fail "device never left its update AP after accepting firmware"
}

profile_family_id() {
  case "$1" in
    dig2go|christmas|golden) echo 1 ;;
    athom-c3) echo 3 ;;
    *) fail "fast upgrade does not yet support profile '$1'" ;;
  esac
}

profile_display_name() {
  case "$1" in
    dig2go) echo "Dig2Go" ;;
    christmas) echo "Dig2Go Christmas" ;;
    golden) echo "Dig2Go Golden" ;;
    athom-c3) echo "ATHOM ESP32-C3" ;;
    *) fail "fast upgrade does not yet support profile '$1'" ;;
  esac
}

upgrade_one() {
  upgrade_work_dir="$(mktemp -d "${TMPDIR:-/tmp}/tubes-upgrade.XXXXXX")"
  local started_at=$SECONDS
  select_and_connect
  enroll_selected_device
  local selected_mac="$device_mac"

  local needed_config_migration=false
  if ! config_has_explicit_output "$upgrade_work_dir/cfg.json"; then
    needed_config_migration=true
  fi
  prepare_device "$upgrade_work_dir" "$selected_mac" true

  if [[ "$needed_config_migration" == "true" ]]; then
    echo "Configuration migration rebooted the device; selecting the same MAC once more."
    sleep 6
    select_and_connect
    fetch_device_files "$upgrade_work_dir"
    load_device_identity "$upgrade_work_dir/info.json"
    require_selected_access_point "$upgrade_work_dir/info.json"
    require_expected_mac "$selected_mac"
  fi

  load_device_identity "$upgrade_work_dir/info.json"
  require_selected_access_point "$upgrade_work_dir/info.json"
  require_expected_mac "$selected_mac"
  validate_explicit_output "$upgrade_work_dir/cfg.json"
  local expected_led_count
  expected_led_count="$(configured_led_count "$upgrade_work_dir/cfg.json")"
  local expected_tubes_version
  expected_tubes_version="$(sed -n 's/^#define RELEASE_VERSION //p' "$script_dir/updater.h")"
  [[ "$expected_tubes_version" =~ ^[0-9]+$ ]] || fail "could not read the Tubes release version"
  local adoption_requested=false
  if [[ "$device_release" == "Custom" ]]; then
    adoption_requested=true
  fi

  # Selection cleanup guarantees this is the only update AP, so the validated
  # files remain authoritative until this connection is used for the upload.
  upload_device "$upgrade_work_dir" "$selected_mac" "$adoption_requested" true
  wait_for_reboot_start

  "$python_bin" "$mesh_reporter" verify "$serial_device" "$selected_mac" \
    --family "$requested_profile" \
    --variant "$profile_variant" \
    --release "$profile_release" \
    --tubes "$expected_tubes_version" \
    --leds "$expected_led_count" \
    --pin "$profile_led_pin" \
    --type "$profile_led_type" \
    --timeout "$verification_timeout"

  echo "FAST_UPGRADE_OK mac=$selected_mac leds=$expected_led_count seconds=$((SECONDS - started_at))"
  cleanup
  upgrade_work_dir=""
}

main_fast() {
  [[ $# -eq 2 || ($# -eq 3 && "$3" == "--loop") ]] || { usage_fast; return 2; }
  requested_profile="$1"
  serial_device="$2"
  local loop_requested=false
  if [[ $# -eq 3 ]]; then
    loop_requested=true
  fi
  [[ -e "$serial_device" ]] || fail "serial device not found: $serial_device"

  require_tools
  command -v networksetup >/dev/null || fail "networksetup is required for the shared-SSID updater"
  find_wifi_device
  profile_name="$(profile_display_name "$requested_profile")"
  profile_family_id "$requested_profile" >/dev/null
  trap cleanup EXIT

  # A patched USB controller is the trusted bridge for post-reboot proof. Fail
  # before physical selection if it cannot issue and recognize report probes.
  "$python_bin" "$mesh_reporter" check "$serial_device"

  while true; do
    upgrade_one
    if [[ "$loop_requested" != "true" ]]; then
      break
    fi
    echo "Ready for the next $profile_name. Press Ctrl-C to stop."
  done
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main_fast "$@"
fi
