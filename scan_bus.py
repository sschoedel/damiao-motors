#!/usr/bin/env python3
"""Scan the CAN bus for Damiao motors.

Sends a broadcast status query (CAN ID 0x7FF) for each candidate slave_id
in 0x01..0x20. Motors that exist reply on their MST_ID (feedback CAN ID).
Prints slave_id, master_id (feedback ID), current position, and status.

This is safe to run any time — the query doesn't enable or move the motor.
"""

import os
import time

from dmcan import Adapter, USB2CANFD
import damiao


SLAVE_RANGE = range(0x01, 0x21)
REPLY_WAIT_S = 0.15


def main() -> None:
    a = Adapter()
    a.open(device_type=USB2CANFD, index=0)
    a.set_classic_can(channel=0, bitrate=1_000_000, sample_point=0.8)
    a.enable_channel(0)
    print(f"scanning slave IDs 0x{SLAVE_RANGE.start:02X}..0x{SLAVE_RANGE.stop - 1:02X}")

    found: list[tuple[int, int, damiao.Feedback]] = []
    try:
        for slave in SLAVE_RANGE:
            a.drain()
            try:
                a.send(damiao.BROADCAST_ID, damiao.refresh_query(slave))
            except Exception as e:
                print(f"  0x{slave:02X}: send failed ({e})")
                continue

            reply = None
            deadline = time.monotonic() + REPLY_WAIT_S
            while time.monotonic() < deadline:
                frame = a.recv(timeout=0.05)
                if frame is None:
                    continue
                try:
                    fb = damiao.Feedback.parse(frame.data)
                except ValueError:
                    continue
                if fb.motor_id == slave:
                    reply = (frame.can_id, fb)
                    break
            if reply is not None:
                master_id, fb = reply
                found.append((slave, master_id, fb))
                print(f"  slave=0x{slave:02X}  master=0x{master_id:03X}  "
                      f"pos={fb.pos:+.3f}  err={fb.err_name}  "
                      f"T_mos={fb.t_mos}°C  T_rotor={fb.t_rotor}°C")
    finally:
        a._shutting_down = True

    print()
    if not found:
        print("no motors found. Check power, wiring, and that at least one motor")
        print("has a slave_id in 0x01..0x20.")
        return

    print(f"found {len(found)} motor(s).")
    masters = {master for _, master, _ in found}
    if len(masters) < len(found):
        print()
        print("WARNING: two or more motors share the same master (feedback) ID.")
        print("Their feedback frames will collide on the CAN bus and corrupt each")
        print("other. Give each motor a unique MST_ID before running multi-motor")
        print("control. See README-style note in hello_spin_two.py.")

    slaves = {slave for slave, _, _ in found}
    if len(slaves) < len(found):
        print()
        print("WARNING: two or more motors share the same slave (command) ID.")
        print("Commands to that ID would drive both motors simultaneously. Give")
        print("each motor a unique ESC_ID before running multi-motor control.")

    time.sleep(0.3)
    os._exit(0)


if __name__ == "__main__":
    main()
