#!/usr/bin/env bash
# Launch the motor GUI with the fastsleep shim (see runtime/fastsleep.c —
# without it the DAMIAO SDK batches CAN feedback into ~100 ms clumps).
# Builds the shim on first run / when the source changes.
#
#   sudo -E env "PATH=$PATH" ./run_gui.sh
set -euo pipefail
cd "$(dirname "$0")"
if [[ ! -f runtime/fastsleep.so || runtime/fastsleep.c -nt runtime/fastsleep.so ]]; then
    gcc -shared -fPIC -O2 -o runtime/fastsleep.so runtime/fastsleep.c -ldl
    echo "built runtime/fastsleep.so"
fi
export LD_PRELOAD="$PWD/runtime/fastsleep.so"
exec uv run motor_gui.py
