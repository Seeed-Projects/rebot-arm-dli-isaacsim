#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_ROOT="${ISAACSIM_ROOT:-${HOME}/isaacsim}"
ISAACSIM_PYTHON="${ISAACSIM_ROOT}/python.sh"
export PYTHONUNBUFFERED=1

if [[ ! -x "${ISAACSIM_PYTHON}" ]]; then
  echo "[error] Isaac Sim Python was not found: ${ISAACSIM_PYTHON}" >&2
  echo "[hint] Set a custom installation with ISAACSIM_ROOT=/path/to/isaacsim ./run_so101.sh" >&2
  exit 1
fi

cd "${SCRIPT_DIR}"
exec "${ISAACSIM_PYTHON}" "${SCRIPT_DIR}/scripts/so101_teleop.py" "$@"
