#!/usr/bin/env bash
# JANDI versioned repository entrypoint; install the tools once before use.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$script_dir/../src/vision/scripts/jandi_deploy.py" select "$@"
