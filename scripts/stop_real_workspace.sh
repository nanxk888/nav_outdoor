#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
exec python3 scripts/stop_real_workspace.py "$@"
