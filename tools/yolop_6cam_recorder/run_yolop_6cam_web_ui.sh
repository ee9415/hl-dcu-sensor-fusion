#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  echo "ERROR: Python executable not found: ${PYTHON_BIN}" >&2
  exit 127
fi

"${PYTHON_BIN}" -c 'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)' || {
  echo "ERROR: Python 3.10 is required. Current: $("${PYTHON_BIN}" --version 2>&1)" >&2
  exit 2
}

exec "${PYTHON_BIN}" "${SCRIPT_DIR}/yolop_6cam_web_ui.py" "$@"
