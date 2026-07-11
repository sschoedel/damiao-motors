# Damiao Motor Control

Control Damiao J6006-2EC motors from Linux via the Damiao USB-to-CANFD adapter.

## Setup

```bash
uv sync
python3 setup_runtime.py
```

This installs dependencies and downloads the platform-appropriate `libdm_device` runtime library.

### USB permissions

Commands require root access for USB. Either use `sudo`:

```bash
sudo -E env "PATH=$PATH" uv run <script.py>
```

Or install a udev rule for permanent access:

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", ATTR{idProduct}=="6877", MODE="0666"' | sudo tee /etc/udev/rules.d/99-damiao.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Then unplug/replug the adapter.

## Scanning the bus

Find all motors connected to the CAN bus:

```bash
sudo -E env "PATH=$PATH" uv run scan_bus.py
```

Reports each motor's ESC_ID (command address), MST_ID (feedback address), position, temperature, and error state. Warns if any IDs collide.

## Changing motor IDs

Each motor needs a unique ESC_ID (command address) and MST_ID (feedback address). Factory default is ESC_ID=0x01, MST_ID=0x00.

**Important:** connect only the motor you're changing — disconnect all others from the CAN bus.

```bash
sudo -E env "PATH=$PATH" uv run set_motor_id.py \
    --current-esc <current_esc_id> \
    --new-esc <new_esc_id> \
    --new-mst <new_mst_id>
```

`--new-esc` and `--new-mst` are each optional — set one or both.

Power-cycle the motor after changing IDs for the new values to take effect from flash.

### ID conventions

Use contiguous pairs — each motor gets two adjacent CAN IDs:

| Motor | ESC_ID | MST_ID |
|-------|--------|--------|
| 1     | 0x01   | 0x02   |
| 2     | 0x03   | 0x04   |
| 3     | 0x05   | 0x06   |
| 4     | 0x07   | 0x08   |

All IDs on the same CAN bus must be unique. Motors on separate CAN buses (separate adapters) can reuse the same IDs.

### Example: setting up two motors from factory defaults

Both motors start at ESC_ID=0x01, MST_ID=0x00.

Motor A — only MST needs changing:
```bash
sudo -E env "PATH=$PATH" uv run set_motor_id.py \
    --current-esc 0x01 --new-mst 0x02
```

Disconnect motor A, connect motor B:
```bash
sudo -E env "PATH=$PATH" uv run set_motor_id.py \
    --current-esc 0x01 --new-esc 0x03 --new-mst 0x04
```

Power-cycle both, reconnect both, then verify:
```bash
sudo -E env "PATH=$PATH" uv run scan_bus.py
```

## Testing with sinusoidal motion

### Single motor

```bash
sudo -E env "PATH=$PATH" uv run hello_spin.py
```

Runs a 0.5 rad amplitude sinusoid at 0.25 Hz for 20 seconds on motor 0x01.

### Two motors

```bash
sudo -E env "PATH=$PATH" uv run hello_spin_two.py
```

Runs anti-phase sinusoids on all motors listed in the `MOTORS` config at the top of the script. Edit the `slave_id` and `master_id` fields to match your motor IDs.
