#!/usr/bin/env python3
"""Measure CAN feedback latency/burstiness (no motor motion).

    sudo -E env "PATH=$PATH" uv run can_probe.py [seconds]

Requires the GUI to be STOPPED (single SDK context). Sends passive status
queries to every motor at 100 Hz and logs reply arrival gaps + hardware
timestamps. Healthy: replies ~10 ms apart per motor, arrival gap p95 well
under 20 ms. Bursting (like the IMU FTDI issue): arrival gaps clustered at
some large value while hardware timestamps stay evenly spaced.
"""

import statistics
import sys
import time

import dmcan
import damiao

SLAVES_L = [0x01, 0x03, 0x05, 0x07]
SLAVES_R = [0x09, 0x0B, 0x0D, 0x0F]


def main(seconds: float = 8.0):
    adapters = dmcan.open_all()
    if not adapters:
        raise SystemExit("no adapters (GUI running? sudo?)")
    for a in adapters:
        a.set_classic_can(channel=0, bitrate=1_000_000, sample_point=0.8)
        a.enable_channel(0)
        a.drain()

    arrivals = {}   # (adapter_idx, motor_id) -> [monotonic arrival times]
    hw_ts = {}      # -> [hardware timestamps]
    t_end = time.monotonic() + seconds
    dt = 0.01
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        for a in adapters:
            for s in SLAVES_L + SLAVES_R:
                try:
                    a.send(damiao.BROADCAST_ID, damiao.refresh_query(s))
                except Exception:
                    pass
        for idx, a in enumerate(adapters):
            for frame in a.drain():
                if len(frame.data) < 8:
                    continue
                mid = frame.data[0] & 0x0F
                key = (idx, mid)
                arrivals.setdefault(key, []).append(time.monotonic())
                hw_ts.setdefault(key, []).append(frame.time_stamp)
        sleep = dt - (time.monotonic() - t0)
        if sleep > 0:
            time.sleep(sleep)

    for key in sorted(arrivals):
        ts = arrivals[key]
        if len(ts) < 10:
            print(f"bus{key[0]} motor 0x{key[1]:02X}: only {len(ts)} replies!")
            continue
        gaps = sorted(ts[i + 1] - ts[i] for i in range(len(ts) - 1))
        hw = hw_ts[key]
        hw_gaps = sorted(hw[i + 1] - hw[i] for i in range(len(hw) - 1))
        print(
            f"bus{key[0]} motor 0x{key[1]:02X}: {len(ts)/seconds:5.0f} Hz | "
            f"arrival gap med {statistics.median(gaps)*1e3:5.1f} ms  "
            f"p95 {gaps[int(0.95*len(gaps))]*1e3:6.1f} ms  "
            f"max {gaps[-1]*1e3:6.1f} ms | hw-ts gap med {statistics.median(hw_gaps)}"
        )
    for a in adapters:
        a._shutting_down = True
    time.sleep(0.2)


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 8.0)
