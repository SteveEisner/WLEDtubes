#!/usr/bin/env bash

set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$test_dir/../.." && pwd)"
temporary_test_dir="$(mktemp -d "${TMPDIR:-/tmp}/tubes-batch-upgrade-tests.XXXXXX")"

cleanup_test_dir() {
  local temporary_root="${TMPDIR:-/tmp}/tubes-batch-upgrade-tests."
  if [[ "$temporary_test_dir" == "$temporary_root"* ]]; then
    rm -rf -- "$temporary_test_dir"
  fi
}
trap cleanup_test_dir EXIT

fake_bin="$temporary_test_dir/bin"
fake_state="$temporary_test_dir/state"
firmware_dir="$temporary_test_dir/firmware"
backup_dir="$temporary_test_dir/backups"
mkdir -p "$fake_bin" "$fake_state" "$firmware_dir" "$backup_dir"
touch "$temporary_test_dir/usbserial"
printf '1' > "$fake_state/device"
printf 'test' > "$firmware_dir/esp32_quinled_dig2go_tubes.bin"
printf 'test' > "$firmware_dir/christmas.bin"
printf 'test' > "$firmware_dir/golden.bin"
printf 'test' > "$firmware_dir/esp32-c3-athom_tubes.bin"

jq '.info.mac = "111111111111" | .info.name = "Light Tube" | .info.release = null' \
  "$test_dir/fixtures/dig2go_info.json" > "$fake_state/info-1.json"
jq '.info.mac = "222222222222" | .info.name = "Light Tube" | .info.release = "Custom" | .info.leds.count = 112' \
  "$test_dir/fixtures/dig2go_info.json" > "$fake_state/info-2.json"
cp "$test_dir/fixtures/legacy_named_explicit_config.json" "$fake_state/cfg-1.json"
cp "$test_dir/fixtures/legacy_named_explicit_config.json" "$fake_state/cfg-2.json"
jq -n '{"222222222222":"dig2go"}' > "$backup_dir/device-inventory.json"

cat > "$fake_bin/networksetup" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF

cat > "$fake_bin/curl" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
output_file=""
url=""
is_upload=false
is_dismiss=false
previous=""
for argument in "$@"; do
  if [[ "$previous" == "-o" ]]; then
    output_file="$argument"
  elif [[ "$previous" == "-F" && "$argument" == update=@* ]]; then
    is_upload=true
  fi
  if [[ "$argument" == http://* ]]; then
    url="$argument"
  fi
  previous="$argument"
done

if [[ "$url" == */reset ]]; then
  is_dismiss=true
fi

device="$(cat "$TUBES_FAKE_STATE/device")"
if [[ "$is_dismiss" == "true" ]]; then
  printf 'dismiss %s\n' "$device" >> "$TUBES_FAKE_STATE/writes.log"
  printf '%s' "$((device + 1))" > "$TUBES_FAKE_STATE/device"
  exit 0
fi
if [[ "$is_upload" == "true" ]]; then
  printf 'upload %s\n' "$device" >> "$TUBES_FAKE_STATE/writes.log"
  printf '%s' "$((device + 1))" > "$TUBES_FAKE_STATE/device"
  exit 0
fi
if (( device > 2 )); then
  exit 22
fi
case "$url" in
  */json/si) source_file="$TUBES_FAKE_STATE/info-$device.json" ;;
  */cfg.json) source_file="$TUBES_FAKE_STATE/cfg-$device.json" ;;
  *) exit 22 ;;
esac
if [[ -n "$output_file" ]]; then
  cp "$source_file" "$output_file"
else
  cat "$source_file"
fi
EOF

cat > "$temporary_test_dir/fake_metadata.py" <<'EOF'
import pathlib
import sys

releases = {
    "esp32_quinled_dig2go_tubes.bin": "DIG2GO_TUBES",
    "christmas.bin": "CHRISTMAS_TUBES",
    "golden.bin": "GOLDEN_TUBES",
    "esp32-c3-athom_tubes.bin": "ESP32-C3_ATHOM_TUBES",
}
print(f"0.15.3\t{releases[pathlib.Path(sys.argv[1]).name]}")
EOF

cat > "$temporary_test_dir/fake_mesh_reporter.py" <<'EOF'
import os
import pathlib
import sys

with pathlib.Path(os.environ["TUBES_FAKE_MESH_LOG"]).open("a") as log:
    log.write(" ".join(sys.argv[1:]) + "\n")
EOF

chmod +x "$fake_bin/networksetup" "$fake_bin/curl" "$temporary_test_dir/fake_mesh_reporter.py"

workflow_output="$temporary_test_dir/workflow.out"
PATH="$fake_bin:$PATH" \
TUBES_CURL_BIN="$fake_bin/curl" \
TUBES_METADATA_READER="$temporary_test_dir/fake_metadata.py" \
TUBES_MESH_REPORTER="$temporary_test_dir/fake_mesh_reporter.py" \
TUBES_FIRMWARE_DIR="$firmware_dir" \
TUBES_BACKUP_DIR="$backup_dir" \
TUBES_DEVICE_INVENTORY="$backup_dir/device-inventory.json" \
TUBES_WIFI_DEVICE="en1" \
TUBES_BATCH_CONNECT_TIMEOUT=1 \
TUBES_BATCH_ROUNDS=1 \
TUBES_MESH_SETTLE_DELAY=0 \
TUBES_FAKE_STATE="$fake_state" \
TUBES_FAKE_MESH_LOG="$fake_state/mesh.log" \
  "$repo_dir/usermods/Tubes/upgrade_batch.sh" "$temporary_test_dir/usbserial" > "$workflow_output"

grep -q 'SKIPPED mac=111111111111' "$workflow_output"
grep -q 'BATCH_UPGRADE_OK mac=222222222222 profile=dig2go leds=112' "$workflow_output"
grep -q 'BATCH_COMPLETE upgraded=1 migrated=0 skipped=1 failed=0' "$workflow_output"
grep -qx 'dismiss 1' "$fake_state/writes.log"
grep -qx 'upload 2' "$fake_state/writes.log"
grep -q '^offer .* 14$' "$fake_state/mesh.log"
grep -q '^verify .*222222222222 .*--family dig2go .*--variant 0 .*--release DIG2GO_TUBES' "$fake_state/mesh.log"
test -f "$backup_dir"/batch-*/111111111111/info.json
test -f "$backup_dir"/batch-*/111111111111/cfg.json
jq -s -e 'map(select(.mac == "111111111111" and .result == "skipped-ambiguous-identity")) | length == 1' \
  "$backup_dir"/batch-*/results.jsonl >/dev/null

echo "PASS: unattended batch skips ambiguous identity and verifies enrolled devices"
