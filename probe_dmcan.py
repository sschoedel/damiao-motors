#!/usr/bin/env python3
"""Direct-to-SDK probe. Bypasses motorbridge; calls libdm_device.dylib via ctypes.

Sequence:
  1. Enumerate + open USB2CANFD adapter
  2. Force channel 0 into CAN2.0 mode at 1 Mbps
  3. Enable the channel  ← if this succeeds, the adapter LED should change
                            from blue-slow to green-slow (handshake done)
  4. Send an enable command to motor at CAN ID 0x01
  5. Wait a moment for a feedback frame; print anything received
  6. Send disable, cleanly shut down
"""

import time

from dmcan import Adapter, USB2CANFD


MOTOR_CAN_ID = 0x01
MASTER_ID    = 0x11

ENABLE_CMD  = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
DISABLE_CMD = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
# Damiao broadcast "refresh status" — motor answers on its MST_ID
REFRESH_TO_BROADCAST = bytes([MOTOR_CAN_ID, 0x00, 0xCC, 0x00, 0x00, 0x00, 0x00, 0x00])


def main() -> None:
    with Adapter() as a:
        print("step 1: enumerate + open USB2CANFD adapter")
        a.open(device_type=USB2CANFD, index=0)
        print("  device opened OK")

        print("step 2: read current channel-0 config (before we change anything)")
        info = a.read_baudrate(0)
        print(f"  default: canfd={info.canfd} can_baud={info.can_baudrate} "
              f"canfd_baud={info.canfd_baudrate} can_sp={info.can_sp:.2f} "
              f"canfd_sp={info.canfd_sp:.2f}")

        print("step 3: force CAN2.0 @ 1 Mbps on channel 0")
        a.set_classic_can(channel=0, bitrate=1_000_000, sample_point=0.8)
        time.sleep(0.1)
        info = a.read_baudrate(0)
        print(f"  after set: canfd={info.canfd} can_baud={info.can_baudrate} "
              f"can_sp={info.can_sp:.2f}")

        print("step 4: enable channel 0")
        a.enable_channel(0)
        print("  channel enabled — check the adapter LED. If it changed from")
        print("  BLUE-SLOW to GREEN-SLOW, the handshake worked.")
        time.sleep(0.5)

        print("step 5: send enable to motor 0x01")
        a.send(MOTOR_CAN_ID, ENABLE_CMD)

        print("step 6: listen for 1.5 s...")
        deadline = time.monotonic() + 1.5
        got_any = False
        while time.monotonic() < deadline:
            frame = a.recv(timeout=0.1)
            if frame is None:
                continue
            got_any = True
            print(f"  RX  id=0x{frame.can_id:03X}  dlc={frame.dlc}  "
                  f"data={frame.data.hex(' ')}")

        # Also send the broadcast refresh in case enable-ack was lost
        print("  sending broadcast refresh...")
        a.send(0x7FF, REFRESH_TO_BROADCAST)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            frame = a.recv(timeout=0.1)
            if frame is None:
                continue
            got_any = True
            print(f"  RX  id=0x{frame.can_id:03X}  dlc={frame.dlc}  "
                  f"data={frame.data.hex(' ')}")

        if not got_any:
            print("  no frames received — bus is silent")
            print("  if the LED did go green, the handshake worked but the motor")
            print("  still isn't answering. If the LED stayed blue, the enable")
            print("  call didn't actually engage the CAN interface.")

        print("step 7: disable + close")
        a.send(MOTOR_CAN_ID, DISABLE_CMD)
        a.disable_channel(0)


if __name__ == "__main__":
    main()
