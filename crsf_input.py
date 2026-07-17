"""Threaded CRSF/ELRS joystick receiver for the robot command panel.

Wraps the frame parsing from crsf_read.py (XR1 Nano on the Pi's UART,
/dev/ttyAMA0 @ 420000) in a background reader with staleness tracking.

    rx = CrsfReceiver.create()          # None-safe: returns NoJoystick if absent
    ch = rx.read()                      # dict or None if stale/no link
    ch["ch1"], ch["ch2"]                # -1..+1 normalized sticks
    rx.stats                            # frames, rssi_dbm, lq

Channel map (mode-2 transmitter): ch1 = right stick horizontal,
ch2 = right stick vertical. Sticks are only trusted while frames are
arriving (<= STALE_S old); a powered-off transmitter reads as None.
"""

from __future__ import annotations

import threading
import time

try:
    import serial
except ImportError:
    serial = None

CRSF_SYNC = 0xC8
FRAMETYPE_RC_CHANNELS = 0x16
FRAMETYPE_LINK_STATS = 0x14
TICKS_MIN, TICKS_MID, TICKS_MAX = 172, 992, 1811
STALE_S = 0.3
DEFAULT_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 420000


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def unpack_channels(payload: bytes) -> list[int]:
    bits = int.from_bytes(payload, "little")
    return [(bits >> (11 * i)) & 0x7FF for i in range(16)]


def normalize(ticks: int) -> float:
    return max(-1.0, min(1.0, (ticks - TICKS_MID) / ((TICKS_MAX - TICKS_MIN) / 2)))


class CrsfParser:
    """Incremental frame parser; feed() bytes, yields (ftype, payload)."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf += data
        out = []
        while True:
            start = self._buf.find(bytes([CRSF_SYNC]))
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            if len(self._buf) < 2:
                break
            frame_len = self._buf[1]
            if not 2 <= frame_len <= 62:
                del self._buf[0]
                continue
            if len(self._buf) < 2 + frame_len:
                break
            frame = bytes(self._buf[: 2 + frame_len])
            del self._buf[: 2 + frame_len]
            if crc8_dvb_s2(frame[2:-1]) != frame[-1]:
                del self._buf[:0]  # frame consumed; just skip it
                continue
            out.append((frame[2], frame[3:-1]))
        return out


class CrsfReceiver:
    available = True

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        if serial is None:
            raise RuntimeError("pyserial not installed")
        self.ser = serial.Serial(port, baud, timeout=0.02)
        self.port = port
        self._parser = CrsfParser()
        self._lock = threading.Lock()
        self._channels: list[float] | None = None
        self._t = 0.0
        self.stats = {"frames": 0, "rssi_dbm": None, "lq": None}
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while self._running:
            try:
                data = self.ser.read(256)
            except (OSError, serial.SerialException):
                time.sleep(0.2)
                continue
            if not data:
                continue
            for ftype, payload in self._parser.feed(data):
                if ftype == FRAMETYPE_RC_CHANNELS and len(payload) == 22:
                    ch = [normalize(t) for t in unpack_channels(payload)]
                    with self._lock:
                        self._channels = ch
                        self._t = time.monotonic()
                        self.stats["frames"] += 1
                elif ftype == FRAMETYPE_LINK_STATS and len(payload) >= 6:
                    self.stats["rssi_dbm"] = -payload[0]
                    self.stats["lq"] = payload[2]

    def read(self) -> dict | None:
        """Latest sticks as {ch1..ch16: -1..1}, or None if stale/no link."""
        with self._lock:
            if self._channels is None or time.monotonic() - self._t > STALE_S:
                return None
            return {f"ch{i+1}": v for i, v in enumerate(self._channels)}

    def close(self):
        self._running = False
        self._thread.join(timeout=0.5)
        try:
            self.ser.close()
        except Exception:
            pass


class NoJoystick:
    available = False
    stats = {"frames": 0, "rssi_dbm": None, "lq": None}

    def read(self):
        return None


def create(port: str = DEFAULT_PORT):
    try:
        rx = CrsfReceiver(port)
        print(f"CRSF joystick on {rx.port}")
        return rx
    except Exception as e:
        print(f"joystick unavailable: {e}")
        return NoJoystick()
