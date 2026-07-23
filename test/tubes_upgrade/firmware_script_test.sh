#!/usr/bin/env bash

set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$test_dir/../.." && pwd)"
fixtures="$test_dir/fixtures"

source "$repo_dir/usermods/Tubes/firmware.sh"

fail_test() {
  echo "FAIL: $1" >&2
  return 1
}

run_test() {
  local name="$1"
  local test_function="$2"
  "$test_function"
  echo "PASS: $name"
}

identifies_dig2go_without_architecture_guessing() {
  # The observed board name plus architecture selects Dig2Go; architecture alone
  # cannot distinguish it from another classic ESP32 controller.
  load_device_identity "$fixtures/dig2go_info.json"

  [[ "$device_mac" == "a0b765ca2e28" ]] || fail_test "Dig2Go MAC was not normalized"
  [[ "$profile_name" == "Dig2Go" ]] || fail_test "Dig2Go profile was not selected"
  [[ "$profile_release" == "DIG2GO_TUBES" ]] || fail_test "wrong firmware family selected"
  [[ "$profile_firmware" == "esp32_quinled_dig2go_tubes.bin" ]] \
    || fail_test "wrong Dig2Go image selected"
}

rejects_unknown_classic_esp32_board() {
  # An unsupported classic ESP32 must stop before image selection, even though its
  # architecture matches Dig2Go.
  if load_device_identity "$fixtures/gledopto_info.json" 2>/dev/null; then
    fail_test "unsupported classic ESP32 was accepted as Dig2Go"
  fi
}

uses_an_enrolled_mac_for_a_legacy_device_name() {
  # Legacy configs use a display name shared across hardware families. A person
  # can identify the physical board once and bind that exact MAC to its profile.
  load_raw_device_identity "$fixtures/legacy_named_dig2go_info.json"
  require_selected_access_point "$fixtures/legacy_named_dig2go_info.json"
  select_profile "dig2go"
  save_device_profile "$profile_id"

  load_device_identity "$fixtures/legacy_named_dig2go_info.json"

  [[ "$profile_name" == "Dig2Go" ]] || fail_test "enrolled Dig2Go profile was not selected"
  [[ "$profile_source" == "enrolled MAC" ]] || fail_test "enrolled identity source was not reported"
}

preserves_an_enrolled_special_variant() {
  # An exact-MAC special profile overrides the native Dig2Go board name and
  # cannot be silently replaced by a later standard fleet run.
  local special_info="$temporary_test_dir/special-info.json"
  "$jq_bin" '.info.mac = "11:22:33:44:55:66"' "$fixtures/dig2go_info.json" > "$special_info"
  load_raw_device_identity "$special_info"
  select_profile "christmas"
  save_device_profile "$profile_id"

  load_device_identity "$special_info"
  [[ "$profile_id" == "christmas" ]] || fail_test "special profile lost to native board fallback"
  [[ "$profile_release" == "CHRISTMAS_TUBES" ]] || fail_test "wrong special release selected"
  [[ "$profile_firmware" == "christmas.bin" ]] || fail_test "wrong special image selected"

  if save_device_profile "dig2go" 2>/dev/null; then
    fail_test "standard profile replaced an enrolled special build"
  fi
}

unattended_identity_requires_enrollment_or_distinct_release() {
  # A mutable legacy label cannot authorize a fleet flash, but the exact MAC
  # inventory can do so without weakening interactive physical enrollment.
  local ambiguous_info="$temporary_test_dir/ambiguous-info.json"
  "$jq_bin" '.info.mac = "22:33:44:55:66:77"' \
    "$fixtures/legacy_named_dig2go_info.json" > "$ambiguous_info"

  if load_unattended_device_identity "$ambiguous_info" 2>/dev/null; then
    fail_test "ambiguous legacy identity authorized an unattended flash"
  fi

  load_raw_device_identity "$ambiguous_info"
  select_profile "christmas"
  save_device_profile "$profile_id"
  load_unattended_device_identity "$ambiguous_info"
  [[ "$profile_id" == "christmas" ]] || fail_test "enrolled special profile was not preserved"
  [[ "$profile_source" == "enrolled MAC" ]] || fail_test "wrong unattended identity source"
}

unattended_identity_accepts_a_distinct_release() {
  local released_info="$temporary_test_dir/released-info.json"
  "$jq_bin" '.info.mac = "12:34:56:78:9a:bc" | .info.release = "GOLDEN_TUBES"' \
    "$fixtures/dig2go_info.json" > "$released_info"

  load_unattended_device_identity "$released_info"

  [[ "$profile_id" == "golden" ]] || fail_test "distinct Golden release selected the wrong profile"
  [[ "$profile_source" == "distinct running release" ]] || fail_test "wrong release identity source"
}

classifies_the_current_two_family_fleet_by_hardware_pins() {
  # Generic legacy names are safe to classify only under the operator's stated
  # two-family scope, where LED and button pins distinguish the hardware.
  local dig2go_info="$temporary_test_dir/two-family-dig2go-info.json"
  local dig2go_config="$temporary_test_dir/two-family-dig2go-config.json"
  cp "$fixtures/dig2go_info.json" "$dig2go_info"
  "$jq_bin" '.hw.btn.ins = [{pin:[0]}]' "$fixtures/explicit_dig2go_config.json" > "$dig2go_config"
  load_raw_device_identity "$dig2go_info"
  infer_two_family_hardware_profile "$dig2go_config"
  [[ "$profile_id" == "dig2go" ]] || fail_test "Dig2Go pin signature selected the wrong profile"

  local athom_config="$temporary_test_dir/athom-config.json"
  "$jq_bin" '.hw.led.ins[0].pin = [10] | .hw.btn.ins[0].pin = [9]' \
    "$dig2go_config" > "$athom_config"
  infer_two_family_hardware_profile "$athom_config"
  [[ "$profile_id" == "athom-c3" ]] || fail_test "ATHOM-C3 pin signature selected the wrong profile"
  [[ "$profile_firmware" == "esp32-c3-athom_tubes.bin" ]] || fail_test "wrong ATHOM-C3 image selected"
}

requires_the_selected_device_mac() {
  # The MAC copied from inspect is the write authorization. Formatting may vary,
  # but a different physical device must be rejected before any mutation.
  load_device_identity "$fixtures/dig2go_info.json"
  require_expected_mac "A0:B7:65:CA:2E:28"

  if require_expected_mac "00:11:22:33:44:55" 2>/dev/null; then
    fail_test "a different selected-device MAC was accepted"
  fi
}

migrates_legacy_led_total_into_explicit_bus() {
  # A legacy 300-LED config with no outputs becomes one GPIO-16 Dig2Go output while
  # preserving its device identity and original current limit.
  local migrated_file="$temporary_test_dir/migrated.json"
  load_device_identity "$fixtures/dig2go_info.json"

  make_legacy_config_explicit "$fixtures/legacy_dig2go_config.json" "$migrated_file"

  "$jq_bin" -e '
    .id.name == "dig2go"
    and .hw.led.total == 300
    and .hw.led.maxpwr == 1250
    and (.hw.led.ins | length == 1)
    and .hw.led.ins[0].pin == [16]
    and .hw.led.ins[0].len == 300
    and .hw.led.ins[0].maxpwr == 1250
  ' "$migrated_file" >/dev/null || fail_test "legacy config migration changed the wrong fields"
}

uses_112_for_unconfigured_migration_era_devices() {
  # Light Tube and Custom builds use 112 only when no explicit output exists;
  # this supplies a deterministic fallback without overwriting configured buses.
  local migrated_file="$temporary_test_dir/migrated-light-tube.json"
  load_raw_device_identity "$fixtures/legacy_named_dig2go_info.json"
  select_profile "dig2go"

  make_legacy_config_explicit "$fixtures/legacy_dig2go_config.json" "$migrated_file"

  [[ "$(configured_led_count "$migrated_file")" == "112" ]] \
    || fail_test "migration-era device did not receive the 112-LED fallback"
}

rejects_explicit_output_for_wrong_pin() {
  # A nominal Dig2Go config cannot pass upload validation when its only LED output
  # points at hardware outside the Dig2Go profile.
  local wrong_pin_file="$temporary_test_dir/wrong-pin.json"
  load_device_identity "$fixtures/dig2go_info.json"
  "$jq_bin" '.hw.led.ins[0].pin = [10]' "$fixtures/explicit_dig2go_config.json" > "$wrong_pin_file"

  if validate_explicit_output "$wrong_pin_file" 2>/dev/null; then
    fail_test "wrong Dig2Go output pin passed validation"
  fi
}

uses_the_explicit_bus_length_instead_of_legacy_total() {
  # Once a bus is explicit, its active length is authoritative. Older WLED
  # configs can retain an unrelated aggregate total without affecting output.
  load_device_identity "$fixtures/dig2go_info.json"
  validate_explicit_output "$fixtures/explicit_dig2go_config.json"

  [[ "$(configured_led_count "$fixtures/explicit_dig2go_config.json")" == "300" ]] \
    || fail_test "legacy total replaced the explicit bus length"
}

requires_explicit_adoption_for_custom_release() {
  # Normal upgrades preserve WLED's release-family check. Only the deliberate
  # adoption path accepts the legacy Custom identity, and only once.
  load_device_identity "$fixtures/dig2go_info.json"
  device_release="Custom"

  if require_running_release false 2>/dev/null; then
    fail_test "normal upload accepted the legacy Custom release"
  fi
  require_running_release true

  device_release="DIG2GO_TUBES"
  require_running_release false
  if require_running_release true 2>/dev/null; then
    fail_test "adopt accepted an already identified release"
  fi
}

temporary_test_dir="$(mktemp -d "${TMPDIR:-/tmp}/tubes-upgrade-tests.XXXXXX")"
inventory_file="$temporary_test_dir/device-inventory.json"
cleanup_test_dir() {
  local temporary_root="${TMPDIR:-/tmp}/tubes-upgrade-tests."
  if [[ "$temporary_test_dir" == "$temporary_root"* ]]; then
    rm -rf -- "$temporary_test_dir"
  fi
}
trap cleanup_test_dir EXIT

run_test "identifies Dig2Go without architecture guessing" identifies_dig2go_without_architecture_guessing
run_test "rejects unknown classic ESP32 board" rejects_unknown_classic_esp32_board
run_test "uses an enrolled MAC for a legacy device name" uses_an_enrolled_mac_for_a_legacy_device_name
run_test "preserves an enrolled special variant" preserves_an_enrolled_special_variant
run_test "unattended identity requires enrollment or distinct release" unattended_identity_requires_enrollment_or_distinct_release
run_test "unattended identity accepts a distinct release" unattended_identity_accepts_a_distinct_release
run_test "classifies current two-family fleet by hardware pins" classifies_the_current_two_family_fleet_by_hardware_pins
run_test "requires the selected device MAC" requires_the_selected_device_mac
run_test "migrates legacy LED total into explicit bus" migrates_legacy_led_total_into_explicit_bus
run_test "uses 112 for unconfigured migration-era devices" uses_112_for_unconfigured_migration_era_devices
run_test "rejects explicit output for wrong pin" rejects_explicit_output_for_wrong_pin
run_test "uses explicit bus length instead of legacy total" uses_the_explicit_bus_length_instead_of_legacy_total
run_test "requires explicit adoption for Custom release" requires_explicit_adoption_for_custom_release
