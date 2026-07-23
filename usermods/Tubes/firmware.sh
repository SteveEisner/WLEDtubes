#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$script_dir/../.." && pwd)"

device_url="${TUBES_DEVICE_URL:-http://4.3.2.1}"
firmware_dir="${TUBES_FIRMWARE_DIR:-$repo_dir/build_output/firmware}"
backup_root="${TUBES_BACKUP_DIR:-$repo_dir/build_output/device-backups}"
inventory_file="${TUBES_DEVICE_INVENTORY:-$backup_root/device-inventory.json}"
curl_bin="${TUBES_CURL_BIN:-curl}"
jq_bin="${TUBES_JQ_BIN:-jq}"
python_bin="${TUBES_PYTHON_BIN:-python3}"
metadata_reader="${TUBES_METADATA_READER:-$script_dir/read_firmware_metadata.py}"

device_mac=""
device_name=""
device_arch=""
device_release=""
profile_id=""
profile_name=""
profile_source=""
profile_release=""
profile_firmware=""
profile_variant=0
profile_led_pin=0
profile_led_type=0
upgrade_work_dir=""

fail() {
  echo "ERROR: $*" >&2
  return 1
}

usage() {
  cat <<'EOF'
Usage:
  ./firmware.sh inspect
  ./firmware.sh enroll DEVICE_MAC PROFILE
  ./firmware.sh prepare DEVICE_MAC
  ./firmware.sh upload DEVICE_MAC
  ./firmware.sh adopt DEVICE_MAC
  ./firmware.sh verify DEVICE_MAC

The computer must already be connected to the one selected device's
WLED-UPDATE access point. Run `inspect` first and copy the reported MAC into
every mutating command so reconnecting to the wrong device fails closed.
EOF
}

cleanup() {
  local temporary_root="${TMPDIR:-/tmp}/tubes-upgrade."
  if [[ -n "$upgrade_work_dir" && "$upgrade_work_dir" == "$temporary_root"* ]]; then
    rm -rf -- "$upgrade_work_dir"
  fi
}

require_tools() {
  command -v "$curl_bin" >/dev/null || fail "curl command not found: $curl_bin"
  command -v "$jq_bin" >/dev/null || fail "jq command not found: $jq_bin"
  command -v "$python_bin" >/dev/null || fail "Python command not found: $python_bin"
}

normalize_mac() {
  tr '[:upper:]' '[:lower:]' | tr -d ':-'
}

load_raw_device_identity() {
  local info_file="$1"
  device_mac="$("$jq_bin" -er '.info.mac | strings' "$info_file" | normalize_mac)"
  device_name="$("$jq_bin" -er '.info.name | strings' "$info_file")"
  device_arch="$("$jq_bin" -er '.info.arch | strings' "$info_file")"
  device_release="$("$jq_bin" -r '.info.release // ""' "$info_file")"
}

select_profile() {
  local requested_profile="$1"
  profile_id=""
  profile_name=""
  profile_release=""
  profile_firmware=""
  profile_variant=0
  profile_led_pin=0
  profile_led_type=0

  case "$requested_profile" in
    dig2go|christmas|golden)
      [[ "$device_arch" == "esp32" ]] \
        || { fail "profile '$requested_profile' does not support device architecture '$device_arch'"; return 1; }
      case "$requested_profile" in
        dig2go)
        profile_name="Dig2Go"
        profile_release="DIG2GO_TUBES"
        profile_firmware="esp32_quinled_dig2go_tubes.bin"
        profile_variant=0
          ;;
        christmas)
        profile_name="Dig2Go Christmas"
        profile_release="CHRISTMAS_TUBES"
        profile_firmware="christmas.bin"
        profile_variant=1
          ;;
        golden)
        profile_name="Dig2Go Golden"
        profile_release="GOLDEN_TUBES"
        profile_firmware="golden.bin"
        profile_variant=2
          ;;
      esac
      profile_id="$requested_profile"
      profile_led_pin=16
      profile_led_type=22
      ;;
    athom-c3)
      case "$device_arch" in
        esp32|esp32c3|ESP32-C3) ;;
        *) fail "profile '$requested_profile' does not support device architecture '$device_arch'"; return 1 ;;
      esac
      profile_id="athom-c3"
      profile_name="ATHOM ESP32-C3"
      profile_release="ESP32-C3_ATHOM_TUBES"
      profile_firmware="esp32-c3-athom_tubes.bin"
      profile_led_pin=10
      profile_led_type=22
      ;;
    *)
      fail "unknown hardware profile '$requested_profile'"
      return 1
      ;;
  esac
}

infer_two_family_hardware_profile() {
  local config_file="$1"
  local led_pin button_pin
  led_pin="$("$jq_bin" -er 'select((.hw.led.ins // []) | length == 1) | .hw.led.ins[0].pin | select(length == 1) | .[0]' "$config_file")" \
    || return 1
  button_pin="$("$jq_bin" -er 'select((.hw.btn.ins // []) | length > 0) | .hw.btn.ins[0].pin | select(length == 1) | .[0]' "$config_file")" \
    || return 1

  if [[ "$led_pin" == "16" && "$button_pin" == "0" ]]; then
    select_profile "dig2go"
  elif [[ "$led_pin" == "10" && "$button_pin" == "9" ]]; then
    select_profile "athom-c3"
  else
    return 1
  fi
  profile_source="explicit LED and button pin signature"
}

load_device_identity() {
  local info_file="$1"
  local enrolled_profile=""
  load_raw_device_identity "$info_file"

  if [[ -f "$inventory_file" ]]; then
    enrolled_profile="$("$jq_bin" -r --arg mac "$device_mac" '.[$mac] // ""' "$inventory_file")"
  fi
  if [[ -n "$enrolled_profile" ]]; then
    select_profile "$enrolled_profile"
    profile_source="enrolled MAC"
    return 0
  fi

  # A classic ESP32 architecture does not distinguish Dig2Go from Gledopto,
  # and a native board name cannot distinguish a special installation build.
  # Exact-MAC enrollment therefore takes precedence over this standard fallback.
  if [[ "$device_name" == "dig2go" && "$device_arch" == "esp32" ]]; then
    select_profile "dig2go"
    profile_source="reported board name"
    return 0
  fi

  fail "unsupported device MAC='$device_mac' name='$device_name' arch='$device_arch'; physically identify and enroll this MAC before selecting firmware"
}

load_unattended_device_identity() {
  local info_file="$1"
  local enrolled_profile=""
  load_raw_device_identity "$info_file"

  if [[ -f "$inventory_file" ]]; then
    enrolled_profile="$("$jq_bin" -r --arg mac "$device_mac" '.[$mac] // ""' "$inventory_file")"
  fi
  if [[ -n "$enrolled_profile" ]]; then
    select_profile "$enrolled_profile"
    profile_source="enrolled MAC"
    return 0
  fi

  # Mutable legacy names such as Custom and Light Tube do not distinguish the
  # standard, Christmas, and Golden builds. Only a release unique to one image
  # is strong enough to authorize unattended firmware selection.
  case "$device_release" in
    DIG2GO_TUBES) select_profile "dig2go" ;;
    CHRISTMAS_TUBES) select_profile "christmas" ;;
    GOLDEN_TUBES) select_profile "golden" ;;
    *)
      fail "device $device_mac has ambiguous unattended identity name='$device_name' release='${device_release:-none}'"
      return 1
      ;;
  esac
  profile_source="distinct running release"
}

require_selected_access_point() {
  local info_file="$1"
  if "$jq_bin" -e '.info.wifi | has("ap")' "$info_file" >/dev/null; then
    "$jq_bin" -e '.info.wifi.ap == true' "$info_file" >/dev/null \
      || fail "device is not serving its selected update access point"
    return 0
  fi

  # WLED 0.14 omits wifi.ap. In its selected-AP state it reports no station
  # BSSID or station IP, while remaining reachable at the fixed device URL.
  "$jq_bin" -e '.info.ip == "" and .info.wifi.bssid == ""' "$info_file" >/dev/null \
    || fail "legacy device does not report selected update-access-point state"
}

require_expected_mac() {
  local expected_mac
  expected_mac="$(printf '%s' "$1" | normalize_mac)"
  [[ "$device_mac" == "$expected_mac" ]] \
    || fail "selected device MAC $device_mac does not match requested MAC $expected_mac"
}

fetch_device_files() {
  local work_dir="$1"
  "$curl_bin" --fail-with-body --silent --show-error --connect-timeout 5 --max-time 15 \
    "$device_url/json/si" -o "$work_dir/info.json"
  "$curl_bin" --fail-with-body --silent --show-error --connect-timeout 5 --max-time 15 \
    "$device_url/cfg.json" -o "$work_dir/cfg.json"
  "$jq_bin" -e 'type == "object" and (.info | type == "object")' "$work_dir/info.json" >/dev/null
  "$jq_bin" -e 'type == "object" and (.hw.led | type == "object")' "$work_dir/cfg.json" >/dev/null
}

dismiss_selected_device() {
  # The reboot endpoint exists in both legacy and current WLED. Rebooting only
  # the HTTP-connected device clears update mode without depending on newer
  # Tubes JSON commands that an old fleet device may not understand.
  "$curl_bin" --silent --show-error --connect-timeout 5 --max-time 15 \
    "$device_url/reset" >/dev/null 2>&1 || true
}

save_config_backup() {
  local config_file="$1"
  local device_backup_dir="$backup_root/$device_mac"
  local backup_file
  mkdir -p "$device_backup_dir"
  backup_file="$(mktemp "$device_backup_dir/cfg-before-upgrade.XXXXXX")"
  cp "$config_file" "$backup_file"
  chmod 600 "$backup_file"
  echo "$backup_file"
}

save_device_profile() {
  local requested_profile="$1"
  local inventory_dir
  local temporary_inventory
  local existing_profile=""
  inventory_dir="$(dirname "$inventory_file")"
  mkdir -p "$inventory_dir"
  temporary_inventory="$(mktemp "$inventory_file.XXXXXX")"

  if [[ -f "$inventory_file" ]]; then
    existing_profile="$("$jq_bin" -r --arg mac "$device_mac" '.[$mac] // ""' "$inventory_file")"
    if [[ -n "$existing_profile" && "$existing_profile" != "$requested_profile" ]]; then
      rm -f -- "$temporary_inventory"
      fail "device $device_mac is already enrolled as '$existing_profile', refusing '$requested_profile'"
      return 1
    fi
    "$jq_bin" --arg mac "$device_mac" --arg profile "$requested_profile" \
      '.[$mac] = $profile' "$inventory_file" > "$temporary_inventory"
  else
    "$jq_bin" -n --arg mac "$device_mac" --arg profile "$requested_profile" \
      '{($mac): $profile}' > "$temporary_inventory"
  fi
  chmod 600 "$temporary_inventory"
  mv "$temporary_inventory" "$inventory_file"
}

config_has_explicit_output() {
  local config_file="$1"
  "$jq_bin" -e '(.hw.led.ins // []) | length > 0' "$config_file" >/dev/null
}

validate_explicit_output() {
  local config_file="$1"
  "$jq_bin" -e \
    --argjson pin "$profile_led_pin" \
    --argjson type "$profile_led_type" \
    '
      ((.hw.led.ins // []) | length == 1)
      and (.hw.led.ins[0].pin == [$pin])
      and (.hw.led.ins[0].type == $type)
      and ((.hw.led.ins[0].start // 0) == 0)
      and ((.hw.led.ins[0].len | numbers) > 0)
      and (.hw.led.ins[0].len <= 1500)
    ' "$config_file" >/dev/null \
    || fail "LED output does not match the supported $profile_name single-strip profile"
}

configured_led_count() {
  local config_file="$1"
  "$jq_bin" -er '
    if ((.hw.led.ins // []) | length) > 0
    then [.hw.led.ins[].len] | add
    else .hw.led.total
    end | numbers
  ' "$config_file"
}

make_legacy_config_explicit() {
  local source_file="$1"
  local destination_file="$2"
  local total
  local max_current
  total="$("$jq_bin" -er '.hw.led.total | numbers' "$source_file")"
  max_current="$("$jq_bin" -er '.hw.led.maxpwr | numbers' "$source_file")"

  # These migration-era labels identify the 112-LED default only when there is
  # no explicit bus to preserve. Explicit output lengths always remain authoritative.
  if [[ "$device_name" == "Light Tube" || "$device_release" == "Custom" ]]; then
    total=112
  fi

  [[ "$total" -gt 0 && "$total" -le 1500 ]] \
    || fail "legacy LED total $total is outside the supported range"
  [[ "$max_current" -ge 100 && "$max_current" -le 20000 ]] \
    || fail "legacy current limit $max_current mA is outside the supported range"

  # The old `total` and current limit are authoritative. Making the compiled
  # Dig2Go bus explicit prevents a newer WLED release from replacing them with
  # its own fallback length during first boot.
  "$jq_bin" \
    --argjson total "$total" \
    --argjson max_current "$max_current" \
    --argjson pin "$profile_led_pin" \
    --argjson type "$profile_led_type" \
    '
      .hw.led.ins = [{
        start: 0,
        len: $total,
        pin: [$pin],
        order: 0,
        rev: false,
        skip: 0,
        type: $type,
        ref: false,
        rgbwm: 0,
        freq: 0,
        maxpwr: $max_current,
        ledma: 55
      }]
    ' "$source_file" > "$destination_file"
  validate_explicit_output "$destination_file"
}

print_identity() {
  local info_file="$1"
  local version
  local led_count
  version="$("$jq_bin" -r '.info.ver // "unknown"' "$info_file")"
  led_count="$("$jq_bin" -r '.info.leds.count // "unknown"' "$info_file")"
  echo "MAC:      $device_mac"
  echo "Board:    $profile_name ($device_name, $device_arch)"
  echo "Identity: $profile_source"
  echo "Firmware: WLED $version${device_release:+, release $device_release}"
  echo "LEDs:     $led_count"
  echo "Image:    $firmware_dir/$profile_firmware"
}

enroll_device() {
  local work_dir="$1"
  local requested_mac="$2"
  local requested_profile="$3"
  fetch_device_files "$work_dir"
  load_raw_device_identity "$work_dir/info.json"
  require_selected_access_point "$work_dir/info.json"
  require_expected_mac "$requested_mac"
  select_profile "$requested_profile"
  save_device_profile "$profile_id"
  profile_source="enrolled MAC"
  print_identity "$work_dir/info.json"
  echo "Enrolled: $device_mac is $profile_name. No device changes were made."
}

inspect_device() {
  local work_dir="$1"
  local active_count
  local legacy_total
  fetch_device_files "$work_dir"
  load_device_identity "$work_dir/info.json"
  require_selected_access_point "$work_dir/info.json"
  print_identity "$work_dir/info.json"
  if config_has_explicit_output "$work_dir/cfg.json"; then
    validate_explicit_output "$work_dir/cfg.json"
    active_count="$(configured_led_count "$work_dir/cfg.json")"
    legacy_total="$("$jq_bin" -r '.hw.led.total // "unset"' "$work_dir/cfg.json")"
    if [[ "$legacy_total" == "$active_count" ]]; then
      echo "Config:   explicit $active_count-LED output; ready to upload"
    else
      echo "Config:   explicit $active_count-LED output; ignoring legacy total $legacy_total"
    fi
  else
    echo "Config:   legacy LED total without an output; run prepare before upload"
  fi
}

prepare_device() {
  local work_dir="$1"
  local requested_mac="$2"
  local files_are_current="${3:-false}"
  local backup_file
  local migrated_file="$work_dir/cfg-migrated.json"
  if [[ "$files_are_current" != "true" ]]; then
    fetch_device_files "$work_dir"
  fi
  load_device_identity "$work_dir/info.json"
  require_selected_access_point "$work_dir/info.json"
  require_expected_mac "$requested_mac"
  backup_file="$(save_config_backup "$work_dir/cfg.json")"
  echo "Saved configuration backup: $backup_file"

  if config_has_explicit_output "$work_dir/cfg.json"; then
    validate_explicit_output "$work_dir/cfg.json"
    echo "Configuration already has an explicit $profile_name LED output; no device changes made."
    return 0
  fi

  make_legacy_config_explicit "$work_dir/cfg.json" "$migrated_file"
  "$curl_bin" --fail-with-body --silent --show-error --connect-timeout 10 --max-time 60 \
    "$device_url/upload" \
    -F "data=@$migrated_file;filename=/cfg.json" \
    -H "Connection: close" --no-keepalive >/dev/null
  echo "Legacy configuration migrated. The device is rebooting; select it again before upload."
}

read_image_identity() {
  local firmware_file="$1"
  "$python_bin" "$metadata_reader" "$firmware_file"
}

require_running_release() {
  local adoption_requested="$1"
  if [[ "$adoption_requested" == "true" ]]; then
    if [[ "$device_release" != "Custom" ]]; then
      fail "adopt is only valid for the one-time transition from release 'Custom'"
      return 1
    fi
    return 0
  fi

  [[ -z "$device_release" || "$device_release" == "$profile_release" ]] \
    || fail "running release '$device_release' does not match '$profile_release'; use adopt only for a known Custom build"
}

upload_device() {
  local work_dir="$1"
  local requested_mac="$2"
  local adoption_requested="${3:-false}"
  local files_are_current="${4:-false}"
  local backup_file
  local firmware_file
  local image_identity
  local image_version
  local image_release
  if [[ "$files_are_current" != "true" ]]; then
    fetch_device_files "$work_dir"
  fi
  load_device_identity "$work_dir/info.json"
  require_selected_access_point "$work_dir/info.json"
  require_expected_mac "$requested_mac"
  require_running_release "$adoption_requested"
  validate_explicit_output "$work_dir/cfg.json"
  backup_file="$(save_config_backup "$work_dir/cfg.json")"
  echo "Saved configuration backup: $backup_file"

  firmware_file="$firmware_dir/$profile_firmware"
  [[ -s "$firmware_file" ]] || fail "firmware image not found: $firmware_file"
  image_identity="$(read_image_identity "$firmware_file")"
  IFS=$'\t' read -r image_version image_release <<< "$image_identity"
  [[ "$image_release" == "$profile_release" ]] \
    || fail "firmware release '$image_release' does not match required release '$profile_release'"

  if [[ "$adoption_requested" == "true" ]]; then
    echo "Applying the one-time Custom to $image_release release transition."
  fi
  echo "Uploading WLED $image_version release $image_release to $device_mac..."
  if [[ "$adoption_requested" == "true" ]]; then
    "$curl_bin" --fail-with-body --silent --show-error --connect-timeout 10 --max-time 180 \
      -F "skipValidation=1" -F "update=@$firmware_file" "$device_url/update" >/dev/null
  else
    "$curl_bin" --fail-with-body --silent --show-error --connect-timeout 10 --max-time 180 \
      -F "update=@$firmware_file" "$device_url/update" >/dev/null
  fi
  echo "Firmware accepted. The device is rebooting."
}

adopt_device() {
  upload_device "$1" "$2" true
}

verify_device() {
  local work_dir="$1"
  local requested_mac="$2"
  local configured_count
  local running_count
  fetch_device_files "$work_dir"
  load_device_identity "$work_dir/info.json"
  require_selected_access_point "$work_dir/info.json"
  require_expected_mac "$requested_mac"
  validate_explicit_output "$work_dir/cfg.json"

  [[ "$device_release" == "$profile_release" ]] \
    || fail "running release '$device_release' does not match '$profile_release'"
  configured_count="$(configured_led_count "$work_dir/cfg.json")"
  running_count="$("$jq_bin" -er '.info.leds.count | numbers' "$work_dir/info.json")"
  [[ "$configured_count" == "$running_count" ]] \
    || fail "running LED count $running_count does not match configured count $configured_count"

  print_identity "$work_dir/info.json"
  echo "Verified: firmware family, MAC, and live LED count match."
}

main() {
  local command="${1:-}"
  local requested_mac="${2:-}"
  require_tools
  upgrade_work_dir="$(mktemp -d "${TMPDIR:-/tmp}/tubes-upgrade.XXXXXX")"
  trap cleanup EXIT

  case "$command" in
    inspect)
      [[ $# -eq 1 ]] || { usage; return 2; }
      inspect_device "$upgrade_work_dir"
      ;;
    enroll)
      [[ $# -eq 3 ]] || { usage; return 2; }
      enroll_device "$upgrade_work_dir" "$requested_mac" "$3"
      ;;
    prepare|upload|adopt|verify)
      [[ $# -eq 2 ]] || { usage; return 2; }
      "${command}_device" "$upgrade_work_dir" "$requested_mac"
      ;;
    *)
      usage
      return 2
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
