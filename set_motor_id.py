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


def param_read(slave_id: int, rid: int) -> bytes:
    lo = slave_id & 0xFF
    hi = (slave_id >> 8) & 0xFF
    return bytes([lo, hi, 0x33, rid, 0, 0, 0, 0])


def save_flash(slave_id: int) -> bytes:
    lo = slave_id & 0xFF
    hi = (slave_id >> 8) & 0xFF
    return bytes([lo, hi, 0xAA, 0, 0, 0, 0, 0])


def read_register(a: Adapter, slave_id: int, rid: int, timeout: float = 0.3) -> int | None:
    a.drain()
    a.send(BROADCAST, param_read(slave_id, rid))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = a.recv(timeout=0.05)
        if frame is None:
            continue
        d = frame.data
        if len(d) >= 8 and d[2] == 0x33 and d[3] == rid:
            return struct.unpack_from("<I", d, 4)[0]
    return None


def write_register(a: Adapter, slave_id: int, rid: int, value: int, timeout: float = 0.3) -> bool:
    a.drain()
    a.send(BROADCAST, param_write(slave_id, rid, value))
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frame = a.recv(timeout=0.05)
        if frame is None:
            continue
        d = frame.data
        if len(d) >= 8 and d[2] == 0x55 and d[3] == rid:
            wrote = struct.unpack_from("<I", d, 4)[0]
            return wrote == value
    return False


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

    # first check the motor is alive at all
    print(f"pinging motor 0x{esc:02X} with refresh query ...")
    a.drain()
    a.send(BROADCAST, damiao.refresh_query(esc))
    time.sleep(0.3)
    frames = a.drain()
    if frames:
        for f in frames:
            print(f"  RX id=0x{f.can_id:03X} dlc={f.dlc} data={f.data.hex(' ')}")
        try:
            fb = damiao.Feedback.parse(frames[0].data)
            print(f"  motor alive: pos={fb.pos:+.3f} err={fb.err_name}")
        except ValueError:
            pass
    else:
        print("  no reply — motor not reachable, check wiring and power")
        a._shutting_down = True
        time.sleep(0.3)
        os._exit(1)

    # try enabling motor first — some firmware requires enable before param access
    print("enabling motor ...")
    a.send(esc, damiao.ENABLE_CMD)
    time.sleep(0.2)
    a.drain()

    print("reading current registers ...")
    a.drain()
    a.send(BROADCAST, param_read(esc, RID_ESC))
    time.sleep(0.3)
    frames = a.drain()
    print(f"  param_read ESC reply frames: {len(frames)}")
    for f in frames:
        print(f"    RX id=0x{f.can_id:03X} dlc={f.dlc} data={f.data.hex(' ')}")
    cur_esc_val = None
    for f in frames:
        d = f.data
        if len(d) >= 8 and d[2] == 0x33 and d[3] == RID_ESC:
            cur_esc_val = struct.unpack_from("<I", d, 4)[0]

    a.drain()
    a.send(BROADCAST, param_read(esc, RID_MST))
    time.sleep(0.3)
    frames = a.drain()
    print(f"  param_read MST reply frames: {len(frames)}")
    for f in frames:
        print(f"    RX id=0x{f.can_id:03X} dlc={f.dlc} data={f.data.hex(' ')}")
    cur_mst_val = None
    for f in frames:
        d = f.data
        if len(d) >= 8 and d[2] == 0x33 and d[3] == RID_MST:
            cur_mst_val = struct.unpack_from("<I", d, 4)[0]

    print(f"current ESC_ID (RID {RID_ESC}): {f'0x{cur_esc_val:02X}' if cur_esc_val is not None else 'no matching reply'}")
    print(f"current MST_ID (RID {RID_MST}): {f'0x{cur_mst_val:02X}' if cur_mst_val is not None else 'no matching reply'}")

    if args.new_mst is not None:
        print(f"writing MST_ID = 0x{args.new_mst:02X} ...")
        if write_register(a, esc, RID_MST, args.new_mst):
            print("  MST_ID write confirmed")
        else:
            print("  MST_ID write: no confirmation (may still have worked)")

    if args.new_esc is not None:
        print(f"writing ESC_ID = 0x{args.new_esc:02X} ...")
        if write_register(a, esc, RID_ESC, args.new_esc):
            print("  ESC_ID write confirmed")
        else:
            print("  ESC_ID write: no confirmation (may still have worked)")
        esc = args.new_esc

    print("saving to flash ...")
    a.drain()
    a.send(BROADCAST, save_flash(esc))
    time.sleep(0.3)

    ver_esc = read_register(a, esc, RID_ESC)
    ver_mst = read_register(a, esc, RID_MST)
    print(f"verify ESC_ID: {f'0x{ver_esc:02X}' if ver_esc is not None else 'no reply'}")
    print(f"verify MST_ID: {f'0x{ver_mst:02X}' if ver_mst is not None else 'no reply'}")

    if ver_esc is not None and ver_mst is not None:
        print("done — power-cycle the motor, then reconnect both and re-scan")
    else:
        print("verify failed — try power-cycling and re-running scan_bus.py")

    a._shutting_down = True
    time.sleep(0.3)
    os._exit(0)


if __name__ == "__main__":
    main()
