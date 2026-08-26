#!/usr/bin/env bash

set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$test_dir/../.." && pwd)"
temporary_test_dir="$(mktemp -d "${TMPDIR:-/tmp}/tubes-fast-upgrade-tests.XXXXXX")"

cleanup_test_dir() {
  local temporary_root="${TMPDIR:-/tmp}/tubes-fast-upgrade-tests."
  if [[ "$temporary_test_dir" == "$temporary_root"* ]]; then
    rm -rf -- "$temporary_test_dir"
  fi
}
trap cleanup_test_dir EXIT

fake_bin="$temporary_test_dir/bin"
fake_state="$temporary_test_dir/state"
firmware_dir="$temporary_test_dir/firmware"
backup_dir="$temporary_test_dir/backups"
mkdir -p "$fake_bin" "$fake_state" "$firmware_dir"
touch "$temporary_test_dir/usbserial"
printf 'test firmware' > "$firmware_dir/esp32_quinled_dig2go_tubes.bin"

cat > "$fake_bin/networksetup" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF

cat > "$fake_bin/curl" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
output_file=""
url=""
upload=false
previous=""
for argument in "$@"; do
  if [[ "$previous" == "-o" ]]; then
    output_file="$argument"
  fi
  if [[ "$argument" == -F && "$previous" == "" ]]; then
    previous="$argument"
    continue
  fi
  if [[ "$argument" == update=@* ]]; then
    upload=true
  fi
  if [[ "$argument" == http://* ]]; then
    url="$argument"
  fi
  previous="$argument"
done

if [[ "$upload" == "true" ]]; then
  touch "$TUBES_FAKE_STATE/uploaded"
  exit 0
fi
printf '%s\n' "$url" >> "$TUBES_FAKE_STATE/http-gets.log"
if [[ -f "$TUBES_FAKE_STATE/uploaded" ]]; then
  exit 22
fi
case "$url" in
  */json/si) source_file="$TUBES_FAKE_INFO" ;;
  */json/cfg|*/cfg.json) source_file="$TUBES_FAKE_CONFIG" ;;
  *) exit 22 ;;
esac
if [[ -n "$output_file" ]]; then
  cp "$source_file" "$output_file"
else
  cat "$source_file"
fi
EOF

cat > "$temporary_test_dir/fake_metadata.py" <<'EOF'
print("0.15.3\tDIG2GO_TUBES")
EOF

cat > "$temporary_test_dir/fake_mesh_reporter.py" <<'EOF'
import os
import pathlib
import sys

with pathlib.Path(os.environ["TUBES_FAKE_MESH_LOG"]).open("a") as log:
    log.write(" ".join(sys.argv[1:]) + "\n")
if sys.argv[1] == "check":
    print("USB controller supports device reports")
elif sys.argv[1] == "select":
    print("selection window open")
elif sys.argv[1] == "verify":
    print('{"mac": "5443b2b542f4", "leds": 112}')
else:
    raise SystemExit(2)
EOF

chmod +x "$fake_bin/networksetup" "$fake_bin/curl"

workflow_output="$temporary_test_dir/workflow.out"
if ! PATH="$fake_bin:$PATH" \
TUBES_CURL_BIN="$fake_bin/curl" \
TUBES_METADATA_READER="$temporary_test_dir/fake_metadata.py" \
TUBES_MESH_REPORTER="$temporary_test_dir/fake_mesh_reporter.py" \
TUBES_FIRMWARE_DIR="$firmware_dir" \
TUBES_BACKUP_DIR="$backup_dir" \
TUBES_DEVICE_INVENTORY="$backup_dir/device-inventory.json" \
TUBES_WIFI_DEVICE="en1" \
TUBES_FAKE_STATE="$fake_state" \
TUBES_FAKE_INFO="$test_dir/fixtures/legacy_named_dig2go_info.json" \
TUBES_FAKE_CONFIG="$test_dir/fixtures/legacy_named_explicit_config.json" \
TUBES_FAKE_MESH_LOG="$fake_state/mesh.log" \
  "$repo_dir/usermods/Tubes/upgrade_fast.sh" \
    dig2go "$temporary_test_dir/usbserial" > "$workflow_output" 2>&1; then
  cat "$workflow_output" >&2
  if [[ -f "$fake_state/http-gets.log" ]]; then
    cat "$fake_state/http-gets.log" >&2
  fi
  exit 1
fi

grep -q 'FAST_UPGRADE_OK mac=5443b2b542f4 leds=112' "$workflow_output"
 expected_release="$(sed -n 's/^#define RELEASE_VERSION //p' "$repo_dir/usermods/Tubes/updater.h")"
 grep -q "^verify .*5443b2b542f4 .*--family dig2go .*--variant 0 .*--release DIG2GO_TUBES .*--tubes $expected_release .*--leds 112 .*--pin 16 .*--type 22" "$fake_state/mesh.log"
 jq -e '."5443b2b542f4" == "dig2go"' "$backup_dir/device-inventory.json" >/dev/null
 test -f "$fake_state/uploaded"
 test "$(find "$backup_dir/5443b2b542f4" -type f -name 'cfg-before-upgrade.*' | wc -l | tr -d ' ')" -ge 2
 test "$(grep -E -c '/(json/cfg|cfg.json)$' "$fake_state/http-gets.log")" -eq 1

echo "PASS: one selection reuses one config fetch through upload and mesh verification"
