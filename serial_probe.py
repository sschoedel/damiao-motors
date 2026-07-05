#!/usr/bin/env python3
"""Raw serial probe: send a broadcast refresh, dump anything that comes back.

Distinguishes 'adapter is deaf' vs 'adapter talks but motor is deaf'.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "DM_Control_Python"))

import serial  # noqa: E402
from DM_CAN import MotorControl, Motor, DM_Motor_Type  # noqa: E402

SERIAL_PORT = "/dev/cu.usbmodem31404"
SERIAL_BAUD = 921600
LISTEN_S    = 2.0


def hexdump(prefix: str, data: bytes) -> None:
    if not data:
        print(f"{prefix}<nothing>")
        return
    line = " ".join(f"{b:02X}" for b in data)
    print(f"{prefix}{len(data)}B: {line}")


def main() -> None:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.2)
    mc = MotorControl(ser)

    # Drain anything the adapter spat out at startup.
    time.sleep(0.3)
    startup = ser.read_all()
    hexdump("startup bytes: ", startup)

    # Send a broadcast refresh (to CAN ID 0x7FF) for a plausibly-valid motor.
    m = Motor(DM_Motor_Type.DM6006, 0x01, 0x11)
    mc.addMotor(m)
    print("sending broadcast refresh_motor_status...")
    mc.refresh_motor_status(m)

    # Listen for whatever comes back over the next couple seconds.
    print(f"listening for {LISTEN_S:.1f}s...")
    t_end = time.monotonic() + LISTEN_S
    total = b""
    while time.monotonic() < t_end:
        chunk = ser.read_all()
        if chunk:
            total += chunk
        time.sleep(0.05)
    hexdump("received: ", total)

    # Second probe: try enabling then reading status a few times.
    print("\nnow trying enable + 5x refresh...")
    mc.enable(m)
    for _ in range(5):
        mc.refresh_motor_status(m)
        time.sleep(0.05)
    time.sleep(0.3)
    burst = ser.read_all()
    hexdump("after enable+refresh: ", burst)

    mc.disable(m)
    ser.close()


if __name__ == "__main__":
    main()
