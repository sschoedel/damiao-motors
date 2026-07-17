#!/usr/bin/env python3
"""Dump Xsens IMU stream diagnostics for a few seconds.

    uv run imu_debug.py [seconds]

Prints per-second field rates, checksum/bad-quaternion counters, the
coordinate mode on the wire, and sample roll/pitch values — run this when
the orientation estimate looks wrong or jumpy. Healthy stream: quat ~100
Hz, gyro/accel ~200 Hz, zero bad counters, steady roll/pitch when still.
"""

import sys
import time

import numpy as np

import imu_interface
from lqr_runtime import tilt_from_quat

COORD = {0: "ENU", 1: "NED (auto-converted)", 2: "NWU", -1: "unknown"}


def main(seconds: float = 5.0):
    imu = imu_interface.create()
    if not getattr(imu, "available", False):
        print("no IMU found")
        return
    prev = dict(imu.stats)
    t_end = time.time() + seconds
    while time.time() < t_end:
        time.sleep(1.0)
        s = imu.stats
        rates = {k: s[k] - prev[k] for k in ("n_quat", "n_gyro", "n_accel",
                                             "bad_quat")}
        prev = dict(s)
        sample = imu.read()
        if sample is None:
            print(f"rates {rates} | coord={COORD.get(s['coord'])} | NO FRESH SAMPLE")
            continue
        quat, gyro, accel = sample
        roll, pitch = tilt_from_quat(quat)
        print(
            f"quat {rates['n_quat']:4d}/s gyro {rates['n_gyro']:4d}/s "
            f"accel {rates['n_accel']:4d}/s badquat {rates['bad_quat']:3d}/s"
            f" | coord={COORD.get(s['coord']):5s}"
            f" | roll {roll:+.4f} pitch {pitch:+.4f} rad"
            f" | |gyro| {np.linalg.norm(gyro):.3f} |acc| {np.linalg.norm(accel):.2f}"
        )
    imu.close()


if __name__ == "__main__":
    main(float(sys.argv[1]) if len(sys.argv) > 1 else 5.0)
