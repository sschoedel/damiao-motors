#!/usr/bin/env python3
"""First-motion test: gentle sinusoidal position command in MIT mode.

Oscillates the motor by AMPL_RAD around its current position at FREQ_HZ.
Ctrl-C to stop early; the finally-block always disables the motor.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "DM_Control_Python"))

import serial  # noqa: E402
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type  # noqa: E402

SERIAL_PORT = "/dev/cu.usbmodem31404"
SERIAL_BAUD = 921600

MOTOR_CAN_ID = 0x01   # motor's slave ID (factory default is 0x01)
MASTER_ID    = 0x11   # host ID used for feedback frames — any nonzero, unique per bus

# --- motion params — intentionally tame for a first spin ---
KP        = 5.0     # spring stiffness  (SDK range 0-500)
KD        = 0.5     # damping           (SDK range 0-5)
AMPL_RAD  = 0.5     # ~28.6°
FREQ_HZ   = 0.25    # 4 s period
DURATION  = 20.0    # 5 full cycles
LOOP_HZ   = 200.0   # MIT wants fast updates; motor watchdog disables if starved
PRINT_HZ  = 10.0


def main() -> None:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.5)
    mc = MotorControl(ser)

    motor = Motor(DM_Motor_Type.DM6006, MOTOR_CAN_ID, MASTER_ID)
    mc.addMotor(motor)
    mc.switchControlMode(motor, Control_Type.MIT)
    mc.enable(motor)  # sleeps 100 ms internally

    # Read the starting position so we oscillate around wherever the motor sits,
    # rather than snapping to absolute zero on the first command.
    mc.refresh_motor_status(motor)
    q_center = motor.getPosition()
    print(f"enabled. starting position = {q_center:+.3f} rad")

    dt = 1.0 / LOOP_HZ
    print_every = max(1, int(LOOP_HZ / PRINT_HZ))
    t0 = time.monotonic()
    i = 0
    try:
        while True:
            t = time.monotonic() - t0
            if t > DURATION:
                break

            q_des = q_center + AMPL_RAD * math.sin(2 * math.pi * FREQ_HZ * t)
            mc.controlMIT(motor, KP, KD, q_des, 0.0, 0.0)
            mc.refresh_motor_status(motor)

            if i % print_every == 0:
                print(
                    f"t={t:5.2f}  q_des={q_des:+.3f}  q={motor.getPosition():+.3f}  "
                    f"dq={motor.getVelocity():+.3f}  tau={motor.getTorque():+.3f}"
                )
            i += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        # Send a zero-effort command before disabling to soften the handoff.
        try:
            mc.controlMIT(motor, 0.0, KD, motor.getPosition(), 0.0, 0.0)
        except Exception:
            pass
        try:
            mc.disable(motor)
        except Exception:
            pass
        ser.close()
        print("motor disabled, serial closed")


if __name__ == "__main__":
    main()
