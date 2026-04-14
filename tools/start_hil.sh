#!/usr/bin/env bash
set -euo pipefail

# Resolve paths relative to this script's location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${REPO_ROOT}/venv"

if [ ! -f "${VENV_DIR}/bin/activate" ]; then
    echo "Error: virtual environment not found at ${VENV_DIR}"
    echo "Create it with:"
    echo "  python3 -m venv venv && pip install -r tools/requirements.txt"
    exit 1
fi

# Activate the virtual environment
source "${VENV_DIR}/bin/activate"

# Add tools/ to PYTHONPATH so 'hil.*' package imports resolve correctly
export PYTHONPATH="${SCRIPT_DIR}:${PYTHONPATH:-}"

# Forward all arguments to run_hil.py
python "${SCRIPT_DIR}/run_hil.py" "$@"
