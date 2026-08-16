#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$script_dir/firmware.sh"

mesh_reporter="${TUBES_MESH_REPORTER:-$script_dir/mesh_device_report.py}"
update_ssid="${TUBES_UPDATE_SSID:-WLED-UPDATE}"
update_password="${TUBES_UPDATE_PASSWORD:-update1234}"
connect_timeout="${TUBES_BATCH_CONNECT_TIMEOUT:-12}"
verification_timeout="${TUBES_VERIFICATION_TIMEOUT:-45}"
mesh_settle_delay="${TUBES_MESH_SETTLE_DELAY:-30}"
max_rounds="${TUBES_BATCH_ROUNDS:-5}"
wifi_device="${TUBES_WIFI_DEVICE:-}"
fleet_hardware_set="${TUBES_FLEET_HARDWARE_SET:-}"
batch_profiles="${TUBES_BATCH_PROFILES:-dig2go,christmas,golden,athom-c3}"
serial_device=""
batch_dir=""
expected_tubes_version=""
upgraded_count=0
migrated_count=0
skipped_count=0
failed_count=0
batch_phase="canary"
canary_mac=""

usage_batch() {
  cat <<'EOF'
Usage:
  ./upgrade_batch.sh SERIAL_DEVICE

Sends a version offer over the mesh, then drains the shared WLED-UPDATE SSID.
Unattended firmware selection requires an exact-MAC enrollment or a distinct
running Tubes release. Ambiguous legacy devices are backed up and skipped.
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

validate_batch_images() {
  local candidate_profile image_identity image_release
  device_arch="esp32"
  for candidate_profile in dig2go christmas golden athom-c3; do
    profile_is_enabled "$candidate_profile" || continue
    select_profile "$candidate_profile"
    [[ -s "$firmware_dir/$profile_firmware" ]] \
      || fail "firmware image not found: $firmware_dir/$profile_firmware"
    image_identity="$(read_image_identity "$firmware_dir/$profile_firmware")"
    IFS=$'\t' read -r _ image_release <<< "$image_identity"
    [[ "$image_release" == "$profile_release" ]] \
      || fail "firmware release '$image_release' does not match '$profile_release'"
  done
}

profile_is_enabled() {
  [[ ",$batch_profiles," == *",$1,"* ]]
}

connect_to_next_device() {
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

archive_observation() {
  local result="$1"
  local observation_dir="$batch_dir/$device_mac"
  mkdir -p "$observation_dir"
  cp "$upgrade_work_dir/info.json" "$observation_dir/info.json"
  cp "$upgrade_work_dir/cfg.json" "$observation_dir/cfg.json"
  chmod 600 "$observation_dir/info.json" "$observation_dir/cfg.json"
  "$jq_bin" -n \
    --arg mac "$device_mac" \
    --arg name "$device_name" \
    --arg release "$device_release" \
    --arg result "$result" \
    '{mac:$mac,name:$name,release:$release,result:$result}' \
    >> "$batch_dir/results.jsonl"
}

verify_upgraded_device() {
  local selected_mac="$1"
  local selected_profile="$2"
  local selected_variant="$3"
  local selected_release="$4"
  local selected_led_count="$5"
  "$python_bin" "$mesh_reporter" verify "$serial_device" "$selected_mac" \
    --family "$selected_profile" \
    --variant "$selected_variant" \
    --release "$selected_release" \
    --tubes "$expected_tubes_version" \
    --leds "$selected_led_count" \
    --pin 16 \
    --type 22 \
    --timeout "$verification_timeout"
}

save_pending_verification() {
  local selected_mac="$1"
  local selected_profile="$2"
  local selected_variant="$3"
  local selected_release="$4"
  local selected_led_count="$5"
  "$jq_bin" -n \
    --arg mac "$selected_mac" \
    --arg profile "$selected_profile" \
    --arg release "$selected_release" \
    --arg variant "$selected_variant" \
    --arg leds "$selected_led_count" \
    '{
      mac: $mac,
      profile: $profile,
      variant: ($variant | tonumber),
      release: $release,
      leds: ($leds | tonumber)
    }' \
    > "$batch_dir/$selected_mac/expected.json"
}

verify_pending_devices() {
  local expected_file selected_mac selected_profile selected_variant selected_release selected_led_count
  for expected_file in "$batch_dir"/*/expected.json; do
    [[ -f "$expected_file" ]] || continue
    [[ ! -f "$(dirname "$expected_file")/verified" ]] || continue
    selected_mac="$("$jq_bin" -r '.mac' "$expected_file")"
    selected_profile="$("$jq_bin" -r '.profile' "$expected_file")"
    selected_variant="$("$jq_bin" -r '.variant' "$expected_file")"
    selected_release="$("$jq_bin" -r '.release' "$expected_file")"
    selected_led_count="$("$jq_bin" -r '.leds' "$expected_file")"
    if ! verify_upgraded_device "$selected_mac" "$selected_profile" "$selected_variant" \
        "$selected_release" "$selected_led_count"; then
      failed_count=$((failed_count + 1))
      echo "FAILED mac=$selected_mac: firmware was accepted but post-wave mesh verification failed."
      return 1
    fi
    touch "$(dirname "$expected_file")/verified"
    "$jq_bin" -n --arg mac "$selected_mac" '{mac:$mac,result:"upgraded-and-verified"}' \
      >> "$batch_dir/results.jsonl"
    upgraded_count=$((upgraded_count + 1))
    echo "BATCH_UPGRADE_OK mac=$selected_mac profile=$selected_profile leds=$selected_led_count"
  done
}

has_pending_verification() {
  local expected_file
  for expected_file in "$batch_dir"/*/expected.json; do
    [[ -f "$expected_file" ]] || continue
    if [[ ! -f "$(dirname "$expected_file")/verified" ]]; then
      return 0
    fi
  done
  return 1
}

process_connected_device() {
  fetch_device_files "$upgrade_work_dir"
  load_raw_device_identity "$upgrade_work_dir/info.json"
  require_selected_access_point "$upgrade_work_dir/info.json"
  local selected_mac="$device_mac"

  if [[ -f "$batch_dir/$selected_mac/completed" ]]; then
    echo "Already handled $selected_mac in this batch; closing its update AP."
    dismiss_selected_device
    return 0
  fi

  if ! load_unattended_device_identity "$upgrade_work_dir/info.json"; then
    if [[ "$fleet_hardware_set" == "dig2go" || "$fleet_hardware_set" == "dig2go,athom-c3" ]] \
        && infer_two_family_hardware_profile "$upgrade_work_dir/cfg.json" \
        && [[ "$fleet_hardware_set" == "dig2go,athom-c3" || "$profile_id" == "dig2go" ]]; then
      save_device_profile "$profile_id"
      profile_source="$profile_source, enrolled for current two-family fleet"
    else
      archive_observation "skipped-ambiguous-identity"
      touch "$batch_dir/$selected_mac/completed"
      skipped_count=$((skipped_count + 1))
      echo "SKIPPED mac=$selected_mac: identity is ambiguous; configuration was backed up."
      dismiss_selected_device
      return 0
    fi
  fi

  local selected_profile="$profile_id"
  local selected_variant="$profile_variant"
  local selected_release="$profile_release"
  local selected_led_count
  if ! profile_is_enabled "$selected_profile"; then
    archive_observation "skipped-profile-outside-batch"
    touch "$batch_dir/$selected_mac/completed"
    skipped_count=$((skipped_count + 1))
    echo "SKIPPED mac=$selected_mac: profile $selected_profile is outside this batch."
    dismiss_selected_device
    return 0
  fi
  echo "Selected $selected_mac as $profile_name via $profile_source."

  if [[ "$batch_phase" == "canary" ]]; then
    if [[ -z "$canary_mac" ]]; then
      canary_mac="$selected_mac"
      echo "Using $canary_mac as the fleet canary."
    elif [[ "$selected_mac" != "$canary_mac" ]]; then
      archive_observation "deferred-until-canary-verifies"
      echo "DEFERRED mac=$selected_mac until canary $canary_mac verifies."
      dismiss_selected_device
      return 0
    fi
  fi

  if ! config_has_explicit_output "$upgrade_work_dir/cfg.json"; then
    if prepare_device "$upgrade_work_dir" "$selected_mac" true; then
      archive_observation "configuration-migrated"
      migrated_count=$((migrated_count + 1))
      echo "MIGRATED mac=$selected_mac; it will be offered the update again next round."
      return 0
    fi
    archive_observation "failed-configuration-migration"
    touch "$batch_dir/$selected_mac/completed"
    failed_count=$((failed_count + 1))
    dismiss_selected_device
    return 0
  fi

  if ! validate_explicit_output "$upgrade_work_dir/cfg.json"; then
    archive_observation "failed-output-validation"
    touch "$batch_dir/$selected_mac/completed"
    failed_count=$((failed_count + 1))
    echo "FAILED mac=$selected_mac: unsupported LED output; no firmware was written."
    dismiss_selected_device
    return 0
  fi
  selected_led_count="$(configured_led_count "$upgrade_work_dir/cfg.json")"

  local adoption_requested=false
  if [[ "$device_release" == "Custom" ]]; then
    adoption_requested=true
  fi
  if ! upload_device "$upgrade_work_dir" "$selected_mac" "$adoption_requested" true; then
    archive_observation "failed-firmware-upload"
    touch "$batch_dir/$selected_mac/completed"
    failed_count=$((failed_count + 1))
    echo "FAILED mac=$selected_mac: firmware upload was rejected."
    dismiss_selected_device
    return 0
  fi

  archive_observation "uploaded-awaiting-wave-verification"
  save_pending_verification "$selected_mac" "$selected_profile" "$selected_variant" \
    "$selected_release" "$selected_led_count"
  echo "UPLOADED mac=$selected_mac; verification is deferred until all update APs close."
  touch "$batch_dir/$selected_mac/completed"
}

main_batch() {
  [[ $# -eq 1 ]] || { usage_batch; return 2; }
  serial_device="$1"
  [[ -e "$serial_device" ]] || fail "serial device not found: $serial_device"
  [[ "$max_rounds" =~ ^[1-9][0-9]*$ ]] || fail "TUBES_BATCH_ROUNDS must be a positive integer"

  require_tools
  command -v networksetup >/dev/null || fail "networksetup is required for the shared-SSID updater"
  find_wifi_device
  expected_tubes_version="$(sed -n 's/^#define RELEASE_VERSION //p' "$script_dir/updater.h")"
  [[ "$expected_tubes_version" =~ ^[0-9]+$ ]] || fail "could not read the Tubes release version"
  validate_batch_images

  batch_dir="$backup_root/batch-$(date -u +%Y%m%dT%H%M%SZ)"
  mkdir -p "$batch_dir"
  chmod 700 "$batch_dir"
  : > "$batch_dir/results.jsonl"
  trap cleanup EXIT

  "$python_bin" "$mesh_reporter" check "$serial_device"
  local round
  for ((round = 1; round <= max_rounds; round++)); do
    echo "Batch round $round/$max_rounds: sending V$expected_tubes_version."
    "$python_bin" "$mesh_reporter" offer "$serial_device" "$expected_tubes_version"
    sleep 3

    while true; do
      cleanup
      upgrade_work_dir="$(mktemp -d "${TMPDIR:-/tmp}/tubes-upgrade.XXXXXX")"
      if ! connect_to_next_device; then
        break
      fi
      process_connected_device
      sleep 2
    done

    if has_pending_verification; then
      echo "Update-AP wave drained; waiting $mesh_settle_delay seconds for the mesh to reform."
      sleep "$mesh_settle_delay"
      verify_pending_devices
      if [[ "$batch_phase" == "canary" && -f "$batch_dir/$canary_mac/verified" ]]; then
        batch_phase="fleet"
        echo "Canary $canary_mac verified; fleet uploads are now enabled."
      fi
    fi
  done

  cleanup
  upgrade_work_dir=""
  if [[ -n "$canary_mac" && "$batch_phase" == "canary" ]]; then
    fail "canary $canary_mac never reached verified firmware within $max_rounds rounds"
  fi
  echo "BATCH_COMPLETE upgraded=$upgraded_count migrated=$migrated_count skipped=$skipped_count failed=$failed_count"
  echo "Batch log: $batch_dir/results.jsonl"
  [[ "$failed_count" -eq 0 ]]
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main_batch "$@"
fi
