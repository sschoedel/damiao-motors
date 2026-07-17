#!/usr/bin/env python3
"""MIT-mode sinusoidal spin test.

Talks to the Damiao USB-CANFD adapter directly via ctypes (dmcan.py)
and speaks the Damiao motor protocol (damiao.py).

First run setup_runtime.py to download the platform-appropriate
libdm_device runtime.
"""

import argparse
import math
import os
import time

from dmcan import Adapter, USB2CANFD
import damiao


DEFAULT_MOTOR_ID = 0x01

# --- motion params — tame for a first spin ---
KP        = 5.0     # spring stiffness (0-500)
KD        = 0.5     # damping (0-5)
AMPL_RAD  = 0.5     # ~28.6°
FREQ_HZ   = 0.25    # 4 s period
DURATION  = 20.0
LOOP_HZ   = 200.0
PRINT_HZ  = 10.0


def read_initial_position(a: Adapter, motor_id: int, timeout: float = 1.0) -> float:
    """Send a zero-effort MIT frame, wait for feedback, return current pos."""
    a.drain()
    a.send(motor_id, damiao.pack_mit(0.0, 0.0, 0.0, 0.0, 0.0))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = a.recv(timeout=0.1)
        if frame is None:
            continue
        try:
            fb = damiao.Feedback.parse(frame.data)
        except ValueError:
            continue
        print(f"initial feedback: pos={fb.pos:+.3f}  err={fb.err_name}  "
              f"T_mos={fb.t_mos}°C  T_rotor={fb.t_rotor}°C")
        return fb.pos
    print("warning: no feedback frame received; assuming q_center=0.0")
    return 0.0


def latest_feedback(a: Adapter) -> damiao.Feedback | None:
    fb = None
    for frame in a.drain():
        try:
            fb = damiao.Feedback.parse(frame.data)
        except ValueError:
            pass
    return fb


def main() -> None:
    p = argparse.ArgumentParser(description="MIT-mode sinusoidal spin test")
    p.add_argument("--motor-id", type=lambda x: int(x, 0), default=DEFAULT_MOTOR_ID,
                   help=f"motor ESC_ID, hex or decimal (default 0x{DEFAULT_MOTOR_ID:02X})")
    args = p.parse_args()
    motor_id = args.motor_id

    a = Adapter()
    a.open(device_type=USB2CANFD, index=0)
    a.set_classic_can(channel=0, bitrate=1_000_000, sample_point=0.8)
    a.enable_channel(0)
    print("adapter open + CAN channel enabled")

    try:
        print(f"enabling motor 0x{motor_id:02X}")
        a.send(motor_id, damiao.ENABLE_CMD)
        time.sleep(0.1)

        q_center = read_initial_position(a, motor_id)

        dt = 1.0 / LOOP_HZ
        print_every = max(1, int(LOOP_HZ / PRINT_HZ))
        t0 = time.monotonic()
        i = 0
        while True:
            t = time.monotonic() - t0
            if t > DURATION:
                break

            q_des = q_center + AMPL_RAD * math.sin(2 * math.pi * FREQ_HZ * t)
            a.send(motor_id, damiao.pack_mit(q_des, 0.0, KP, KD, 0.0))

            if i % print_every == 0:
                fb = latest_feedback(a)
                if fb is not None:
                    print(f"t={t:5.2f}  q_des={q_des:+.3f}  q={fb.pos:+.3f}  "
                          f"dq={fb.vel:+.3f}  tau={fb.tau:+.3f}  err={fb.err_name}")
                else:
                    print(f"t={t:5.2f}  q_des={q_des:+.3f}  (no feedback)")
            i += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        try:
            a.send(motor_id, damiao.DISABLE_CMD)
            time.sleep(0.05)
        except Exception:
            pass
        a._shutting_down = True
        print("motor disabled, adapter quiescing...")
        time.sleep(0.3)
        os._exit(0)


if __name__ == "__main__":
    main()
