#!/usr/bin/env python3
"""Scan CAN IDs via motorbridge's dm-serial transport.

Adds a Damiao 6006 motor at each candidate ID, tries request_feedback + get_state,
and also tries enable+state to catch motors that only reply when enabled.
"""

import os
import time

os.environ.setdefault(
    "MOTOR_DM_DEVICE_LIB",
    os.path.join(os.path.dirname(__file__), "runtime/libdm_device.dylib"),
)

import motorbridge as mb  # noqa: E402

SERIAL_PORT = "/dev/cu.usbmodem31404"
BAUD        = 921600
FEEDBACK_ID = 0x11


def try_one(ctrl, slave_id):
    m = ctrl.add_damiao_motor(motor_id=slave_id, feedback_id=FEEDBACK_ID, model="6006")

    # pass 1: passive read
    m.request_feedback()
    time.sleep(0.15)
    state = m.get_state()
    if state is not None:
        return "passive", state

    # pass 2: enable then read
    try:
        m.enable()
    except Exception as e:
        return f"enable_err:{type(e).__name__}", None
    for _ in range(4):
        m.request_feedback()
        time.sleep(0.08)
        state = m.get_state()
        if state is not None:
            try:
                m.disable()
            except Exception:
                pass
            return "after_enable", state
    try:
        m.disable()
    except Exception:
        pass
    return None, None


def main():
    ctrl = mb.Controller.from_dm_serial(serial_port=SERIAL_PORT, baud=BAUD)
    print(f"scanning CAN IDs 0x01..0x10 via dm-serial on {SERIAL_PORT}\n")
    found = []
    for slave in range(0x01, 0x11):
        kind, state = try_one(ctrl, slave)
        if state is not None:
            found.append((slave, kind, state))
            print(f"  id={hex(slave)}  RESPONDED via {kind}: {state}")
        else:
            print(f"  id={hex(slave)}  no reply ({kind or 'silent'})")
    ctrl.close()
    print()
    if found:
        for slave, kind, state in found:
            print(f"motor at CAN ID {hex(slave)}")
    else:
        print("no motors responded to any ID.")


if __name__ == "__main__":
    main()
