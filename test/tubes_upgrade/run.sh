#!/usr/bin/env bash

set -euo pipefail

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"$test_dir/firmware_script_test.sh"
"$test_dir/fast_upgrade_workflow_test.sh"
"$test_dir/batch_upgrade_workflow_test.sh"
python3 "$test_dir/firmware_metadata_test.py"
python3 "$test_dir/mesh_device_report_test.py"
python3 "$test_dir/fleet_update_server_test.py"
