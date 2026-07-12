#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST="${1:-4.3.2.1}"

"$SCRIPT_DIR/safe_ota.py" update "$HOST"
