#!/bin/bash
# NUC robot server launcher
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DROID_ROOT="$( cd "$SCRIPT_DIR/../.." && pwd )"

echo "[LAUNCH] DROID root: $DROID_ROOT"

# Activate conda env if available
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    conda activate droid 2>/dev/null || conda activate base
fi

cd "$DROID_ROOT"
export PYTHONPATH="$DROID_ROOT:$PYTHONPATH"

# Check robot IP reachable
ROBOT_IP="${ROBOT_IP:-172.16.0.3}"
echo "[LAUNCH] Checking robot at $ROBOT_IP ..."
if ! ping -c 1 -W 2 "$ROBOT_IP" > /dev/null 2>&1; then
    echo "[LAUNCH] ERROR: Cannot reach robot at $ROBOT_IP. Check network."
    exit 1
fi
echo "[LAUNCH] Robot reachable. Starting server..."

exec python3 scripts/server/run_server.py
