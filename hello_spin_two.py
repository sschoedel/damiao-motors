#!/usr/bin/env python3
"""MIT-mode sinusoidal spin of two daisy-chained Damiao motors.

Motor A oscillates in-phase, motor B oscillates in anti-phase, so at any
moment they're moving in opposite directions. That makes it visually
obvious that both are being controlled independently rather than mirroring
the same command.

Prereq: each motor must have a unique slave_id (ESC_ID) *and* a unique
master_id (MST_ID). Run `uv run scan_bus.py` first — it will warn if any
IDs collide. To reassign a motor's IDs on Linux, the simplest path is
motorbridge's CLI (installed as a dep):

    motorbridge-cli id-set --vendor damiao --transport dm-device \\
        --dm-device-type usb2canfd --dm-channel 0 --model 6006 \\
        --motor-id 0x01 --feedback-id 0x11 \\
        --set-motor-id 0x02 --set-feedback-id 0x12 --store 1

(Adjust --motor-id / --feedback-id to the motor's *current* IDs, and
--set-motor-id / --set-feedback-id to the desired new ones.)
"""

import math
import os
import time
from dataclasses import dataclass, field

from dmcan import Adapter, USB2CANFD
import damiao


# Edit these to match what scan_bus.py reports for your two motors.
@dataclass
class MotorCfg:
    slave_id: int
    master_id: int
    label: str


MOTORS: list[MotorCfg] = [
    MotorCfg(slave_id=0x01, master_id=0x02, label="A"),
    MotorCfg(slave_id=0x03, master_id=0x04, label="B"),
    MotorCfg(slave_id=0x05, master_id=0x06, label="C"),
    MotorCfg(slave_id=0x07, master_id=0x08, label="D"),
]

# --- motion params — tame for a first spin ---
KP        = 5.0     # spring stiffness (0-500)
KD        = 0.5     # damping (0-5)
AMPL_RAD  = 0.5     # ~28.6°
FREQ_HZ   = 0.25    # 4 s period
DURATION  = 20.0
LOOP_HZ   = 200.0
PRINT_HZ  = 5.0


@dataclass
class MotorRt:
    cfg: MotorCfg
    q_center: float = 0.0
    last_fb: damiao.Feedback | None = field(default=None)
    saw_feedback: bool = False


def drain_feedback(a: Adapter, motors: list[MotorRt]) -> None:
    """Route any pending feedback frames to their motor by motor_id."""
    for frame in a.drain():
        try:
            fb = damiao.Feedback.parse(frame.data)
        except ValueError:
            continue
        for m in motors:
            if fb.motor_id == m.cfg.slave_id:
                m.last_fb = fb
                m.saw_feedback = True
                break


def read_initial_positions(a: Adapter, motors: list[MotorRt],
                           timeout: float = 1.5) -> None:
    """Poke each motor with a zero-effort MIT frame, then collect replies."""
    a.drain()
    for m in motors:
        a.send(m.cfg.slave_id, damiao.pack_mit(0.0, 0.0, 0.0, 0.0, 0.0))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not all(m.saw_feedback for m in motors):
        time.sleep(0.05)
        drain_feedback(a, motors)
    for m in motors:
        if m.saw_feedback and m.last_fb is not None:
            m.q_center = m.last_fb.pos
            print(f"motor {m.cfg.label} (0x{m.cfg.slave_id:02X}): initial "
                  f"pos={m.last_fb.pos:+.3f}  err={m.last_fb.err_name}")
        else:
            print(f"motor {m.cfg.label} (0x{m.cfg.slave_id:02X}): NO FEEDBACK "
                  f"— will command assuming q_center=0")


def main() -> None:
    motors = [MotorRt(cfg=c) for c in MOTORS]

    a = Adapter()
    a.open(device_type=USB2CANFD, index=0)
    a.set_classic_can(channel=0, bitrate=1_000_000, sample_point=0.8)
    a.enable_channel(0)
    print("adapter open + CAN channel enabled")

    try:
        for m in motors:
            print(f"enabling motor {m.cfg.label} (0x{m.cfg.slave_id:02X})")
            a.send(m.cfg.slave_id, damiao.ENABLE_CMD)
        time.sleep(0.15)

        read_initial_positions(a, motors)

        dt = 1.0 / LOOP_HZ
        print_every = max(1, int(LOOP_HZ / PRINT_HZ))
        t0 = time.monotonic()
        i = 0
        while True:
            t = time.monotonic() - t0
            if t > DURATION:
                break

            sine = AMPL_RAD * math.sin(2 * math.pi * FREQ_HZ * t)
            for idx, m in enumerate(motors):
                phase = 1.0 if idx % 2 == 0 else -1.0
                q_des = m.q_center + phase * sine
                a.send(m.cfg.slave_id, damiao.pack_mit(q_des, 0.0, KP, KD, 0.0))

            drain_feedback(a, motors)

            if i % print_every == 0:
                parts = [f"t={t:5.2f}"]
                for m in motors:
                    if m.last_fb is not None:
                        parts.append(
                            f"{m.cfg.label} q={m.last_fb.pos:+.3f} "
                            f"dq={m.last_fb.vel:+.3f}"
                        )
                    else:
                        parts.append(f"{m.cfg.label} <no fb>")
                print("  ".join(parts))
            i += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        for m in motors:
            try:
                a.send(m.cfg.slave_id, damiao.DISABLE_CMD)
            except Exception:
                pass
        a._shutting_down = True
        print("motors disabled, adapter quiescing...")
        time.sleep(0.3)
        os._exit(0)


if __name__ == "__main__":
    main()
