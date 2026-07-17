#!/usr/bin/env python3
"""Read ExpressLRS/CRSF joystick channels from the XR1 Nano and log to console.

Wiring (Pi 5): XR1 TX -> GPIO15/pin 10, XR1 RX -> GPIO14/pin 8,
5V -> pin 2, G -> pin 6. Requires dtparam=uart0=on in /boot/firmware/config.txt
and the serial login console disabled.

Usage:
    python3 crsf_read.py                    # /dev/ttyAMA0 @ 420000
    python3 crsf_read.py --port /dev/ttyAMA0 --print-hz 20
"""

import argparse
import time

import serial

CRSF_SYNC = 0xC8
FRAMETYPE_RC_CHANNELS = 0x16
FRAMETYPE_LINK_STATS = 0x14

# Raw 11-bit channel values ELRS actually emits (988us..2012us sticks)
TICKS_MIN, TICKS_MID, TICKS_MAX = 172, 992, 1811


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_channels(payload: bytes) -> list[int]:
    """22 bytes -> 16 channels of 11 bits, LSB first."""
    bits = int.from_bytes(payload, "little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


def normalize(ticks: int) -> float:
    """Map raw ticks to -1.0 .. +1.0 (0 at stick center)."""
    return max(-1.0, min(1.0, (ticks - TICKS_MID) / ((TICKS_MAX - TICKS_MIN) / 2)))


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", default="/dev/ttyAMA0")
    p.add_argument("--baud", type=int, default=420000)
    p.add_argument("--channels", type=int, default=8, help="how many channels to print")
    p.add_argument("--print-hz", type=float, default=20.0)
    p.add_argument("--raw", action="store_true", help="print raw ticks instead of -1..+1")
    args = p.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.02)
    print(f"listening on {args.port} @ {args.baud} baud (ctrl-c to stop)")

    buf = bytearray()
    channels: list[int] | None = None
    rssi_dbm, lq = None, None
    frames = 0
    last_print = 0.0
    last_frame = time.monotonic()

    try:
        while True:
            buf += ser.read(256)

            # scan for complete frames: [sync][len][type][payload...][crc]
            while True:
                start = buf.find(bytes([CRSF_SYNC]))
                if start < 0:
                    buf.clear()
                    break
                if start > 0:
                    del buf[:start]
                if len(buf) < 2:
                    break
                frame_len = buf[1]  # bytes after the len byte (type+payload+crc)
                if not 2 <= frame_len <= 62:
                    del buf[0]  # bogus length; resync
                    continue
                if len(buf) < 2 + frame_len:
                    break
                frame = bytes(buf[: 2 + frame_len])
                del buf[: 2 + frame_len]

                if crc8_dvb_s2(frame[2:-1]) != frame[-1]:
                    continue

                ftype = frame[2]
                if ftype == FRAMETYPE_RC_CHANNELS and frame_len == 24:
                    channels = unpack_channels(frame[3:-1])
                    frames += 1
                    last_frame = time.monotonic()
                elif ftype == FRAMETYPE_LINK_STATS:
                    rssi_dbm, lq = -frame[3], frame[5]

            now = time.monotonic()
            if now - last_print >= 1.0 / args.print_hz:
                last_print = now
                if channels is None:
                    print("waiting for CRSF frames... (is the TX on and bound?)")
                elif now - last_frame > 0.5:
                    print(f"LINK LOST ({now - last_frame:.1f}s since last frame)")
                else:
                    vals = channels[: args.channels]
                    if args.raw:
                        chans = "  ".join(f"ch{i+1}={v:4d}" for i, v in enumerate(vals))
                    else:
                        chans = "  ".join(f"ch{i+1}={normalize(v):+.2f}" for i, v in enumerate(vals))
                    link = f"rssi={rssi_dbm}dBm lq={lq}%" if lq is not None else ""
                    print(f"{chans}  [{frames} frames] {link}")
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
