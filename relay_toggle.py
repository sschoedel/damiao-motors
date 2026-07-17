#!/usr/bin/env python3
"""Toggle the motor-power relay from the keyboard.

The relay's IN pin is on GPIO 17 (header pin 11), high-level trigger:
GPIO high = relay closed = motor power on.

Run on the pi:

    uv run relay_toggle.py

Keys:
    e      toggle relay
    q      quit (relay is switched OFF on exit)
"""

import sys
import termios
import tty

from gpiozero import DigitalOutputDevice

RELAY_GPIO = 17


def read_key() -> str:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    relay = DigitalOutputDevice(RELAY_GPIO, active_high=True,
                                initial_value=False)
    print("relay control on GPIO 17 — 'e' to toggle, 'q' to quit")
    print("state: OFF")
    try:
        while True:
            key = read_key()
            if key == "e":
                relay.toggle()
                print(f"state: {'ON' if relay.value else 'OFF'}")
            elif key in ("q", "\x03"):   # q or ctrl-c
                break
    finally:
        relay.off()
        relay.close()
        print("relay OFF, exiting")


if __name__ == "__main__":
    main()
