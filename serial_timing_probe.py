#!/usr/bin/env python3
"""Timing probe for the DAMIAO adapter's CDC-ACM serial transport.

Sends refresh queries to the 4 motors of one bus at 100 Hz for 6 s and
measures RX chunk arrival gaps. If the serial path is unbatched, gaps sit
near the 10 ms query cadence — unlike the libusb SDK path's ~100 ms bursts.

    uv run serial_timing_probe.py /dev/ttyACM0 [0x01 0x03 0x05 0x07]
"""

import os
import statistics
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "DM_Control_Python"))
import serial

# minimal frame builder (from DM_CAN.py's send_data_frame template)
FRAME = bytearray([0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a,
                   0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
                   0x00] + [0] * 8 + [0x00])


def refresh_frame(slave_id):
    f = FRAME.copy()
    can_id = 0x7FF
    f[13] = can_id & 0xFF
    f[14] = (can_id >> 8) & 0xFF
    f[21] = slave_id & 0xFF
    f[22] = (slave_id >> 8) & 0xFF
    f[23] = 0xCC
    return bytes(f)


def main(port, slaves):
    ser = serial.Serial(port, 921600, timeout=0.005)
    try:
        ser.set_low_latency_mode(True)
        print("low latency mode set")
    except Exception as e:
        print(f"(low latency not settable: {e})")
    time.sleep(0.2)
    ser.reset_input_buffer()
    chunks = []
    t_end = time.monotonic() + 6.0
    nbytes = 0
    while time.monotonic() < t_end:
        t0 = time.monotonic()
        for s in slaves:
            ser.write(refresh_frame(s))
        data = ser.read(4096)
        if data:
            chunks.append((time.monotonic(), len(data)))
            nbytes += len(data)
        dt = 0.01 - (time.monotonic() - t0)
        if dt > 0:
            time.sleep(dt)
    ser.close()
    if len(chunks) < 5:
        print(f"almost nothing received ({nbytes} bytes) — wrong port/framing?")
        return
    gaps = sorted(chunks[i + 1][0] - chunks[i][0] for i in range(len(chunks) - 1))
    print(f"{nbytes} bytes in {len(chunks)} chunks over 6 s")
    print(f"chunk gap med {statistics.median(gaps)*1e3:.1f} ms  "
          f"p95 {gaps[int(0.95*len(gaps))]*1e3:.1f} ms  max {gaps[-1]*1e3:.1f} ms")


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    slaves = [int(s, 16) for s in sys.argv[2:]] or [0x01, 0x03, 0x05, 0x07]
    main(port, slaves)
