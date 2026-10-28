#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if python3 - 2>/dev/null <<'PY'
import json
import sys
sys.path.insert(0,'.local/patchworkpp')
from importlib.metadata import distribution
import pypatchworkpp
d=distribution('pypatchworkpp')
assert json.loads(d.read_text('direct_url.json'))['vcs_info']['commit_id']=='3e6903a1d5537a4cc2ace897b0bbb98a92d6014c'
PY
then
  echo '[READY] Official Patchwork++ pinned binding available'
else
  python3 -m venv .local/patchworkpp-build || { echo 'Install dependencies: sudo apt-get install python3-venv python3-dev build-essential cmake libeigen3-dev git' >&2; exit 1; }
  export CMAKE_BUILD_PARALLEL_LEVEL=1
  env -u PYTHONPATH .local/patchworkpp-build/bin/python -m pip install 'pip==24.3.1' 'packaging==24.2' 'scikit-build-core==0.10.7' 'pybind11==2.13.6'
  env -u PYTHONPATH .local/patchworkpp-build/bin/python -m pip install --no-build-isolation --upgrade --no-deps --config-settings=cmake.define.USE_SYSTEM_EIGEN3=ON --target "$PWD/.local/patchworkpp" -r scripts/requirements_patchworkpp.txt
fi
