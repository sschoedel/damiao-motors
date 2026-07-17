#!/usr/bin/env bash
# Launch the motor GUI with the fastsleep shim (see runtime/fastsleep.c —
# without it the DAMIAO SDK batches CAN feedback into ~100 ms clumps).
# Builds the shim on first run / when the source changes.
#
#   sudo -E env "PATH=$PATH" ./run_gui.sh
set -euo pipefail
cd "$(dirname "$0")"

# The DM-USB2FDCAN adapter is claim-once: a stale/duplicate GUI instance
# blocks dmcan_device_open for everyone else. If that instance is driving
# motors, killing it drops the command stream and the 200 ms motor CAN
# watchdog freewheels them.
existing="$(pgrep -f 'motor_gui\.py' || true)"
if [[ -n "$existing" ]]; then
    echo "motor_gui already running (pid: ${existing//$'\n'/ })"
    read -r -p "kill and continue? [Y/n] " ans
    if [[ "${ans:-Y}" =~ ^[Yy]$|^$ ]]; then
        # shellcheck disable=SC2086
        kill $existing 2>/dev/null || true
        for _ in $(seq 1 20); do
            pgrep -f 'motor_gui\.py' >/dev/null || break
            sleep 0.25
        done
        if pgrep -f 'motor_gui\.py' >/dev/null; then
            echo "still running after 5 s — sending SIGKILL"
            pkill -9 -f 'motor_gui\.py' || true
            sleep 0.5
        fi
        if pgrep -f 'motor_gui\.py' >/dev/null; then
            echo "could not kill it — if it was started with sudo," \
                 "rerun this script with sudo too"
            exit 1
        fi
        echo "old instance stopped"
    else
        echo "leaving it alone — aborting"
        exit 1
    fi
fi

if [[ ! -f runtime/fastsleep.so || runtime/fastsleep.c -nt runtime/fastsleep.so ]]; then
    gcc -shared -fPIC -O2 -o runtime/fastsleep.so runtime/fastsleep.c -ldl
    echo "built runtime/fastsleep.so"
fi
export LD_PRELOAD="$PWD/runtime/fastsleep.so"
exec uv run motor_gui.py
