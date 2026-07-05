#!/usr/bin/env python3
"""Scan CAN IDs 0x01..0x20 and report any motor that answers.

If exactly one ID answers -> that's your motor's slave ID.
If none answer -> CAN wiring, power, or bus baud rate is wrong.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "DM_Control_Python"))

import serial  # noqa: E402
from DM_CAN import Motor, MotorControl, DM_Motor_Type, DM_variable  # noqa: E402

SERIAL_PORT = "/dev/cu.usbmodem31404"
SERIAL_BAUD = 921600
MASTER_ID   = 0x11
SCAN_RANGE  = range(0x01, 0x21)


def main() -> None:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.5)
    mc = MotorControl(ser)

    print(f"scanning CAN IDs {hex(SCAN_RANGE.start)}..{hex(SCAN_RANGE.stop - 1)}...")
    found = []
    for slave_id in SCAN_RANGE:
        m = Motor(DM_Motor_Type.DM6006, slave_id, MASTER_ID)
        mc.addMotor(m)
        sw = mc.read_motor_param(m, DM_variable.sw_ver)
        if sw is not None:
            sn = mc.read_motor_param(m, DM_variable.SN)
            mst = mc.read_motor_param(m, DM_variable.MST_ID)
            print(f"  id={hex(slave_id)}  sw_ver={sw}  SN={sn}  MST_ID={mst}")
            found.append(slave_id)
        else:
            print(f"  id={hex(slave_id)}  (no reply)")

    ser.close()
    print()
    if not found:
        print("no motors answered. Check: CAN H/L not swapped, common GND wired,")
        print("motor powered (24 V), and CAN bus baud rate = 1 Mbps.")
    elif len(found) == 1:
        print(f"motor found at CAN ID {hex(found[0])}.")
        print(f"set MOTOR_CAN_ID = {hex(found[0])} in hello_spin.py")
    else:
        print(f"multiple motors answered: {[hex(x) for x in found]}")


if __name__ == "__main__":
    main()
