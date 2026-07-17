#!/usr/bin/env python3
"""Write an arbitrary Damiao register (RID) and save to flash.

Useful RIDs (from Damiao's register map):
    7  = MST_ID     (feedback CAN ID)
    8  = ESC_ID     (command CAN ID)
    9  = TIMEOUT    (CAN watchdog, 0 = disabled)
    10 = CTRL_MODE  (1=MIT, 2=Pos-Vel, 3=Vel)

Example — force MIT mode on motor 0x05:
    sudo -E env "PATH=$PATH" uv run set_param.py --esc 0x05 --rid 10 --value 1
"""

import argparse
import os
import struct
import time

from dmcan import Adapter, USB2CANFD
import damiao


BROADCAST = damiao.BROADCAST_ID


def param_write(slave_id: int, rid: int, value: int) -> bytes:
    lo = slave_id & 0xFF
    hi = (slave_id >> 8) & 0xFF
    vb = struct.pack("<I", value)
    return bytes([lo, hi, 0x55, rid, vb[0], vb[1], vb[2], vb[3]])


def save_flash(slave_id: int) -> bytes:
    lo = slave_id & 0xFF
    hi = (slave_id >> 8) & 0xFF
    return bytes([lo, hi, 0xAA, 0, 0, 0, 0, 0])


def main() -> None:
    p = argparse.ArgumentParser(description="Write a Damiao register and save to flash")
    p.add_argument("--esc", type=lambda x: int(x, 0), required=True,
                   help="motor's current ESC_ID")
    p.add_argument("--rid", type=lambda x: int(x, 0), required=True,
                   help="register ID to write")
    p.add_argument("--value", type=lambda x: int(x, 0), required=True,
                   help="value to write (uint32)")
    args = p.parse_args()

    a = Adapter()
    a.open(device_type=USB2CANFD, index=0)
    a.set_classic_can(channel=0, bitrate=1_000_000, sample_point=0.8)
    a.enable_channel(0)
    print("adapter open")

    print(f"writing RID {args.rid} = {args.value} on motor 0x{args.esc:02X} ...")
    a.drain()
    a.send(BROADCAST, param_write(args.esc, args.rid, args.value))
    time.sleep(0.2)
    for f in a.drain():
        print(f"  reply: id=0x{f.can_id:03X} data={f.data.hex(' ')}")

    print("saving to flash ...")
    a.send(BROADCAST, save_flash(args.esc))
    time.sleep(0.5)
    for f in a.drain():
        print(f"  reply: id=0x{f.can_id:03X} data={f.data.hex(' ')}")

    print("done — power-cycle the motor")
    a._shutting_down = True
    time.sleep(0.3)
    os._exit(0)


if __name__ == "__main__":
    main()
