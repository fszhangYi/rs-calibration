#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
export PYTHONPATH="${ROOT}/src:${ROOT}/vendor:${PYTHONPATH:-}"
exec python3 "${ROOT}/scripts/build_desktop.py" "$@"
