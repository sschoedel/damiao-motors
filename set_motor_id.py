#!/usr/bin/env python3
"""Reassign a Damiao motor's ESC_ID and/or MST_ID, then save to flash.

Connect ONLY the motor you want to reassign. Power-cycle after flashing.

Usage:
    sudo -E env "PATH=$PATH" uv run set_motor_id.py \
        --current-esc 0x01 --new-esc 0x02 --new-mst 0x01
"""

import argparse
import os
import struct
import time

from dmcan import Adapter, USB2CANFD
import damiao


RID_MST = 7
RID_ESC = 8
BROADCAST = damiao.BROADCAST_ID


def param_write(slave_id: int, rid: int, value: int) -> bytes:
    lo = slave_id & 0xFF
    hi = (slave_id >> 8) & 0xFF
    val_bytes = struct.pack("<I", value)
    return bytes([lo, hi, 0x55, rid, val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3]])


def save_flash(slave_id: int) -> bytes:
    lo = slave_id & 0xFF
    hi = (slave_id >> 8) & 0xFF
    return bytes([lo, hi, 0xAA, 0, 0, 0, 0, 0])


def main() -> None:
    p = argparse.ArgumentParser(description="Reassign Damiao motor CAN IDs")
    p.add_argument("--current-esc", type=lambda x: int(x, 0), default=0x01,
                   help="Motor's current ESC_ID (default 0x01)")
    p.add_argument("--new-esc", type=lambda x: int(x, 0), default=None,
                   help="New ESC_ID to assign")
    p.add_argument("--new-mst", type=lambda x: int(x, 0), default=None,
                   help="New MST_ID to assign")
    args = p.parse_args()

    if args.new_esc is None and args.new_mst is None:
        p.error("specify at least one of --new-esc or --new-mst")

    a = Adapter()
    a.open(device_type=USB2CANFD, index=0)
    a.set_classic_can(channel=0, bitrate=1_000_000, sample_point=0.8)
    a.enable_channel(0)
    print("adapter open")

    esc = args.current_esc

    # verify motor is alive via enable + MIT query
    print(f"verifying motor 0x{esc:02X} is reachable ...")
    a.send(esc, damiao.ENABLE_CMD)
    time.sleep(0.1)
    a.drain()
    a.send(esc, damiao.pack_mit(0.0, 0.0, 0.0, 0.0, 0.0))
    time.sleep(0.3)
    frames = a.drain()
    alive = False
    for f in frames:
        try:
            fb = damiao.Feedback.parse(f.data)
            if fb.motor_id == (esc & 0x0F):
                print(f"  motor alive: pos={fb.pos:+.3f} err={fb.err_name}")
                alive = True
                break
        except ValueError:
            pass
    if not alive:
        print("  no feedback — check wiring, power, and --current-esc")
        a._shutting_down = True
        time.sleep(0.3)
        os._exit(1)

    # disable motor before param changes
    a.send(esc, damiao.DISABLE_CMD)
    time.sleep(0.1)

    # blind-write new IDs — motor may not ACK param writes but still accept them
    if args.new_mst is not None:
        print(f"writing MST_ID = 0x{args.new_mst:02X} ...")
        a.send(BROADCAST, param_write(esc, RID_MST, args.new_mst))
        time.sleep(0.2)
        frames = a.drain()
        if frames:
            for f in frames:
                print(f"  reply: id=0x{f.can_id:03X} data={f.data.hex(' ')}")
        else:
            print("  no ACK (normal for some firmware)")

    if args.new_esc is not None:
        print(f"writing ESC_ID = 0x{args.new_esc:02X} ...")
        a.send(BROADCAST, param_write(esc, RID_ESC, args.new_esc))
        time.sleep(0.2)
        frames = a.drain()
        if frames:
            for f in frames:
                print(f"  reply: id=0x{f.can_id:03X} data={f.data.hex(' ')}")
        else:
            print("  no ACK (normal for some firmware)")
        esc = args.new_esc

    print("saving to flash ...")
    a.send(BROADCAST, save_flash(esc))
    time.sleep(0.5)
    frames = a.drain()
    if frames:
        for f in frames:
            print(f"  reply: id=0x{f.can_id:03X} data={f.data.hex(' ')}")

    # verify by enabling at the NEW ID and checking for feedback
    print(f"verifying motor responds at new ESC_ID 0x{esc:02X} ...")
    a.send(esc, damiao.ENABLE_CMD)
    time.sleep(0.1)
    a.drain()
    a.send(esc, damiao.pack_mit(0.0, 0.0, 0.0, 0.0, 0.0))
    time.sleep(0.3)
    frames = a.drain()
    verified = False
    for f in frames:
        try:
            fb = damiao.Feedback.parse(f.data)
            if fb.motor_id == (esc & 0x0F):
                print(f"  confirmed at 0x{esc:02X}: pos={fb.pos:+.3f}")
                verified = True
                break
        except ValueError:
            pass

    a.send(esc, damiao.DISABLE_CMD)
    time.sleep(0.1)

    if verified:
        print("done — power-cycle the motor, reconnect both, re-scan")
    else:
        print("no reply at new ID — power-cycle and try scan_bus.py to see where it landed")

    a._shutting_down = True
    time.sleep(0.3)
    os._exit(0)


if __name__ == "__main__":
    main()
