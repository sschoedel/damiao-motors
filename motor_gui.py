#!/usr/bin/env python3
"""Viser-based command center for the Pineapple robot (Damiao MIT mode).

Run this on the machine with the CAN adapter(s) (e.g. the raspberry pi):

    sudo -E env "PATH=$PATH" uv run motor_gui.py

Then open http://<pi-hostname-or-ip>:8080 in a browser on any machine on
the network.

Two control modes (dropdown at the top):
  manual — the original per-motor panels: enable checkboxes, pos/vel/tau
           sliders, kp/kd, live feedback. For debugging individual motors.
  robot  — the LQR runtime (lqr_runtime.py + lqr_tables.npz) drives all
           eight motors: damp / stand / balance / sit buttons, v-w sliders,
           state-estimator telemetry. Per-motor sliders are ignored while
           in robot mode, but feedback readouts stay live.

Safety:
  - E-STOP button and DISABLE ALL always work in both modes.
  - Browser deadman: in robot mode, if NO viser client is connected for
    0.5 s the robot drops to damp (losing the laptop = robot goes limp).
  - Stale-feedback watchdog: any motor silent for 50 ms in robot mode
    trips to damp.
  - Balance is locked out until an IMU driver exists (imu_interface.py)
    and calibration.yaml has non-zero offsets.

Calibration (robot HOISTED, see joint_calibration.py):
  - direction: select a joint, "pulse selected joint", watch it, then
    answer the prompt buttons. Run once per robot.
  - range: "run range calibration" sweeps every leg joint to both stops
    (~2 Nm) and writes calibration.yaml zero offsets.

Original per-motor tool docs: motors start DISABLED; Kp=0 and tau=0 at
startup so enabling a motor holds it with light damping only; the pos
slider snaps to the motor's current position on enable.
"""

import os
import socket
import time
from dataclasses import dataclass, field

import numpy as np
import viser
import yaml

import dmcan
from dmcan import Adapter
import damiao
import robot_config
import crsf_input
import imu_interface
import joint_calibration
import robot_viz
from lqr_runtime import (
    Calibration,
    RobotRuntime,
    Snapshot,
    SpikeFilter,
    TableController,
    forward_accel,
)

# Motor power: the 24V relay is hardwired normally-ON; the physical E-STOP
# button breaks the line. No software power control (GPIO 17 retired).

SLAVE_RANGE = range(0x01, 0x10)
SCAN_REPLY_WAIT_S = 0.15

LOOP_HZ = 100.0        # command stream + feedback drain rate
GUI_HZ = 10.0          # feedback readout refresh rate

VEL_LIM = 5.0
TAU_LIM = 2.0
KP_LIM = 200.0
KD_LIM = 20.0          # protocol caps Kd at 5 — pack_mit clamps
KP_DEFAULT = 50.0
KD_DEFAULT = 2.0

# Anchored to this file's directory (not CWD) so sudo/uv launch dirs don't
# matter. calibration.yaml is GITIGNORED: it is Pi-local state written by
# the calibration routines and must survive sync_to_robot.sh --delete
# passes (same mechanism as runtime/libdm_device.so). Back it up manually
# if wanted: scp pi:~/motor_control_*/calibration.yaml .
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
# CRSF right stick -> drive commands (robot mode). Signs chosen so stick
# up = forward, stick right = turn right (negative yaw in our convention);
# flip here if the transmitter is reversed. Deadband kills center drift.
JOY_V_SIGN = 1.0    # ch2 (right stick vertical) -> v
JOY_W_SIGN = -1.0   # ch1 (right stick horizontal) -> w
JOY_DEADBAND = 0.06
# Aux-switch -> mode mapping, edge-triggered (fires only when the switch
# MOVES to the position, never on link acquire — a switch parked on
# "balance" cannot arm the robot at power-on). Positions: -1/0/+1 for
# lo/mid/hi. Flip switches while watching the ch5-ch8 readout in the
# joystick status line to discover your transmitter's channel order,
# then adjust here. Valid actions: damp / stand / balance / sit / enable
# ("enable" = clear errors + enable all motors, same as the GUI button).
JOY_MODE_SWITCHES = {
    ("ch5", 1): "enable",
    ("ch5", -1): "damp",
    ("ch7", 1): "stand",
    ("ch7", -1): "sit",
    ("ch6", 1): "balance",
    ("ch6", -1): "damp",
}


def switch_pos(v: float) -> int:
    return -1 if v < -0.5 else (1 if v > 0.5 else 0)

CAL_FILE = os.path.join(BASE_DIR, "calibration.yaml")
TABLES_FILE = os.path.join(BASE_DIR, "lqr_tables.npz")
TUNING_FILE = os.path.join(BASE_DIR, "tuning.yaml")  # gitignored, Pi-local

# robot_config ESC id -> controller joint index (JOINT_NAMES order:
# L hip, L thigh, L calf, L wheel, R hip, R thigh, R calf, R wheel)
ESC_TO_JOINT = {0x01: 0, 0x03: 1, 0x05: 2, 0x07: 3,
                0x09: 4, 0x0B: 5, 0x0D: 6, 0x0F: 7}
JOINT_LABELS = ["l_hip_aa", "l_hip_fe", "l_knee", "l_ankle",
                "r_hip_aa", "r_hip_fe", "r_knee", "r_ankle"]
STALE_FB_S = 0.05
DEADMAN_S = 0.5
V_MAX_TELEOP = 1.25    # joystick/slider full-scale (sim-verified to 1.5)
W_MAX_TELEOP = 1.25    # rad/s (sim-verified to 2.0 in place; governor
                       # still caps yaw while translating)


@dataclass
class MotorUI:
    adapter: dmcan.Bus
    adapter_idx: int
    slave_id: int
    label: str
    limits: damiao.Limits
    folder: viser.GuiFolderHandle
    enable_cb: viser.GuiInputHandle
    pos: viser.GuiInputHandle
    vel: viser.GuiInputHandle
    tau: viser.GuiInputHandle
    kp: viser.GuiInputHandle
    kd: viser.GuiInputHandle
    fb_q: viser.GuiInputHandle
    fb_dq: viser.GuiInputHandle
    fb_tau: viser.GuiInputHandle
    fb_temp: viser.GuiInputHandle
    fb_err: viser.GuiInputHandle
    fb_fresh: viser.GuiInputHandle
    last_fb: damiao.Feedback | None = field(default=None)
    last_fb_t: float = 0.0
    fb_count: int = 0
    prev_err: int | None = field(default=None)


def open_hardware() -> tuple[list[Adapter], list[dmcan.Bus]]:
    """Open every adapter and enumerate its CAN channels as buses.

    A USB2CANFD_DUAL exposes two channels (port CAN 1 = channel 0, CAN 2 =
    channel 1) and hosts both legs by itself; single-channel adapters
    contribute one bus each. Channel 1 is probed by simply trying it.
    """
    adapters = dmcan.open_all()
    buses: list[dmcan.Bus] = []
    for idx, a in enumerate(adapters):
        for ch in (0, 1):
            bus = dmcan.Bus(a, ch)
            try:
                bus.configure(bitrate=1_000_000, sample_point=0.8)
            except dmcan.DmcanError:
                if ch == 0:
                    raise   # channel 0 must work; 1 just means single-channel
                break
            buses.append(bus)
            print(f"adapter {idx} ch {ch}: CAN 1 Mbps")
    return adapters, buses


def scan_motors(a: dmcan.Bus) -> list[int]:
    """Return slave IDs that answer a broadcast refresh query."""
    found = []
    for slave in SLAVE_RANGE:
        a.drain()
        a.send(damiao.BROADCAST_ID, damiao.refresh_query(slave))
        deadline = time.monotonic() + SCAN_REPLY_WAIT_S
        while time.monotonic() < deadline:
            frame = a.recv(timeout=0.05)
            if frame is None:
                continue
            try:
                fb = damiao.Feedback.parse(frame.data)
            except ValueError:
                continue
            if fb.motor_id == slave:
                found.append(slave)
                break
    return found


def bus_side(slaves: list[int]) -> str | None:
    """Identify a bus as left/right from the ESC IDs that answered."""
    sides = {"left" if s < 0x09 else "right" for s in slaves}
    if len(sides) == 1:
        return sides.pop()
    if len(sides) > 1:
        return "MIXED"
    return None


def motor_frame_range(cfg, cal: Calibration, joint_idx: int | None):
    """Slider limits in the MOTOR frame, using calibration when known."""
    q_min, q_max = cfg.q_range if cfg is not None else (-3.14, 3.14)
    if joint_idx is None or np.all(cal.offsets == 0.0):
        return q_min, q_max
    s, o = cal.signs[joint_idx], cal.offsets[joint_idx]
    a, b = s * q_min + o, s * q_max + o
    return (a, b) if a <= b else (b, a)


def build_motor_panel(server: viser.ViserServer, a: dmcan.Bus,
                      adapter_idx: int, slave_id: int,
                      bus_name: str, cal: Calibration) -> MotorUI:
    cfg = robot_config.BY_ESC_ID.get(slave_id)
    joint_idx = ESC_TO_JOINT.get(slave_id)
    if cfg is not None:
        label = f"{bus_name} · {cfg.name} 0x{slave_id:02X} ({cfg.model})"
        limits = cfg.limits
        kp_init, kd_init = cfg.kp_default, cfg.kd_default
        q_min, q_max = motor_frame_range(cfg, cal, joint_idx)
    else:
        label = f"{bus_name} · motor 0x{slave_id:02X} (unknown)"
        limits = damiao.DEFAULT_LIMITS
        kp_init, kd_init = KP_DEFAULT, KD_DEFAULT
        q_min, q_max = -limits.p_max, limits.p_max

    folder = server.gui.add_folder(label)
    with folder:
        enable_cb = server.gui.add_checkbox("enable", initial_value=False)
        pos = server.gui.add_slider("pos [rad]", min=q_min, max=q_max,
                                    step=0.01,
                                    initial_value=min(max(0.0, q_min), q_max))
        vel = server.gui.add_slider("vel [rad/s]", min=-VEL_LIM, max=VEL_LIM,
                                    step=0.01, initial_value=0.0)
        tau = server.gui.add_slider("tau [Nm]", min=-TAU_LIM, max=TAU_LIM,
                                    step=0.01, initial_value=0.0)
        kp = server.gui.add_slider("Kp", min=0.0, max=KP_LIM,
                                   step=0.1, initial_value=kp_init)
        kd = server.gui.add_slider("Kd", min=0.0, max=KD_LIM,
                                   step=0.01, initial_value=kd_init)
        fb_q = server.gui.add_number("q [rad]", initial_value=0.0, disabled=True)
        fb_dq = server.gui.add_number("dq [rad/s]", initial_value=0.0, disabled=True)
        fb_tau = server.gui.add_number("tau [Nm]", initial_value=0.0, disabled=True)
        fb_temp = server.gui.add_text("temp [°C]", initial_value="—",
                                      disabled=True)
        fb_err = server.gui.add_text("status", initial_value="—",
                                     disabled=True)
        fb_fresh = server.gui.add_text("feedback", initial_value="—",
                                       disabled=True)

    m = MotorUI(adapter=a, adapter_idx=adapter_idx, slave_id=slave_id,
                label=label, limits=limits, folder=folder,
                enable_cb=enable_cb, pos=pos, vel=vel, tau=tau, kp=kp, kd=kd,
                fb_q=fb_q, fb_dq=fb_dq, fb_tau=fb_tau, fb_temp=fb_temp,
                fb_err=fb_err, fb_fresh=fb_fresh)

    @enable_cb.on_update
    def _(_event, m=m) -> None:
        if m.enable_cb.value:
            if m.last_fb is not None:
                m.pos.value = min(max(float(round(m.last_fb.pos, 2)),
                                      m.pos.min), m.pos.max)
            # clear any latched error first (the motors' CAN watchdog,
            # TIMEOUT=200 ms, latches CommLoss whenever the bus goes quiet
            # — e.g. after every GUI stop — and enable is refused until
            # the error is cleared)
            m.adapter.send(m.slave_id, damiao.CLEAR_ERR_CMD)
            m.adapter.send(m.slave_id, damiao.ENABLE_CMD)
            print(f"bus {m.adapter_idx} motor 0x{m.slave_id:02X}: enabled")
        else:
            m.adapter.send(m.slave_id, damiao.DISABLE_CMD)
            print(f"bus {m.adapter_idx} motor 0x{m.slave_id:02X}: disabled")

    return m


class TwitchLogger:
    """100 Hz ring buffer of raw per-motor feedback + sent commands; when a
    joint jumps faster than physically possible, dump the surrounding
    window to a log file so we can see what moved first (encoder, torque,
    error flag, or our own command)."""

    WINDOW = 40   # ticks kept before the event (0.4 s)
    TAIL = 20     # ticks captured after
    # sensitive triggers for standstill twitch-hunting (any of the three
    # fires a dump; DUMP_GAP_S rate-caps log volume so false positives
    # during active balancing are cheap):
    STEP_LIM = np.array([0.05, 0.05, 0.05, 0.3] * 2)   # rad per tick
    DQ_LIM = np.array([1.0, 1.0, 1.0, 8.0] * 2)        # rad/s
    DTAU_LIM = np.array([3.0, 3.0, 3.0, 2.0] * 2)      # Nm per tick
    DUMP_GAP_S = 2.0

    def __init__(self):
        import collections
        self.buf = collections.deque(maxlen=self.WINDOW)
        self.prev_q = None
        self.tail = 0
        self.event_rows = []
        self.count = 0
        self.last_dump_t = 0.0
        self.prev_tau = None
        self.trigger_reason = ""
        self.path = f"/tmp/twitch_{int(time.time())}.log"

    def record(self, t, q, dq, tau, errs, cmd_tau, stalled):
        row = (t, q.copy(), dq.copy(), tau.copy(), list(errs),
               cmd_tau.copy() if cmd_tau is not None else None, stalled)
        trigger = None
        reason = ""
        if self.prev_q is not None and self.prev_tau is not None:
            for name, over in (
                ("q-step", np.abs(q - self.prev_q) - self.STEP_LIM),
                ("dq", np.abs(dq) - self.DQ_LIM),
                ("dtau", np.abs(tau - self.prev_tau) - self.DTAU_LIM),
            ):
                j = int(np.argmax(over))
                if over[j] > 0:
                    trigger, reason = j, name
                    break
            if trigger is not None and t - self.last_dump_t < self.DUMP_GAP_S:
                trigger = None  # rate cap
        self.prev_q = q.copy()
        self.prev_tau = tau.copy()
        if self.tail > 0:
            self.event_rows.append(row)
            self.tail -= 1
            if self.tail == 0:
                self._dump()
        elif trigger is not None:
            self.count += 1
            self.event_rows = list(self.buf) + [row]
            self.tail = self.TAIL
            self.trigger_joint = trigger
            self.trigger_reason = reason
            self.last_dump_t = t
        self.buf.append(row)

    def _dump(self):
        with open(self.path, "a") as f:
            f.write(f"\n=== twitch #{self.count} joint {self.trigger_joint} "
                    f"({JOINT_LABELS[self.trigger_joint]}, "
                    f"trigger={self.trigger_reason}) ===\n")
            f.write("t q[8] dq[8] tau_est[8] err[8] cmd_tau[8] stalled\n")
            for (t, q, dq, tau, errs, ct, st) in self.event_rows:
                f.write(f"{t:.4f} {np.round(q,4).tolist()} "
                        f"{np.round(dq,3).tolist()} {np.round(tau,2).tolist()} "
                        f"{errs} "
                        f"{np.round(ct,2).tolist() if ct is not None else None} "
                        f"{int(st)}\n")
        self.event_rows = []


def load_cal() -> Calibration:
    return Calibration.load(CAL_FILE)


def save_cal(cal: Calibration) -> None:
    with open(CAL_FILE, "w") as f:
        yaml.safe_dump(
            {"signs": [float(s) for s in cal.signs],
             "offsets": [float(o) for o in cal.offsets],
             "joint_order": JOINT_LABELS},
            f, sort_keys=False)
    print(f"wrote {CAL_FILE}")


def main() -> None:
    adapters, buses = open_hardware()
    if not buses:
        raise SystemExit("no USB-CANFD adapters found (need sudo or udev rule?)")

    motors: list[MotorUI] = []
    server = viser.ViserServer(host="0.0.0.0", port=8080)

    cal = load_cal()
    imu = imu_interface.create()
    joystick = crsf_input.create()
    runtime: RobotRuntime | None = None
    tables_err = ""
    try:
        runtime = RobotRuntime(TableController(TABLES_FILE), dt=1.0 / LOOP_HZ)
    except Exception as e:  # tables missing/corrupt — robot mode unavailable
        tables_err = str(e)

    estop_btn = server.gui.add_button("DAMP ALL", color="red")
    disable_btn = server.gui.add_button("DISABLE ALL")
    rescan_btn = server.gui.add_button("RESCAN MOTORS")
    mode_dd = server.gui.add_dropdown("control mode", ("manual", "robot"),
                                      initial_value="robot")
    rescan_requested = [True]
    estop_requested = [False]

    # ---- robot-mode panel --------------------------------------------------
    with server.gui.add_folder("robot control"):
        robot_status = server.gui.add_markdown("**mode:** manual")
        trip_md = server.gui.add_markdown("")
        b_enable_all = server.gui.add_button("enable all motors")
        b_damp = server.gui.add_button("damp")
        b_stand = server.gui.add_button("stand")
        b_balance = server.gui.add_button("balance")
        b_sit = server.gui.add_button("sit")
        cb_joy = server.gui.add_checkbox(
            "joystick control (right stick)", initial_value=True)
        joy_md = server.gui.add_markdown("joystick: —")
        s_v = server.gui.add_slider("v [m/s]", min=-V_MAX_TELEOP,
                                    max=V_MAX_TELEOP, step=0.05,
                                    initial_value=0.0)
        s_w = server.gui.add_slider("w [rad/s]", min=-W_MAX_TELEOP,
                                    max=W_MAX_TELEOP, step=0.05,
                                    initial_value=0.0)
        b_zero = server.gui.add_button("zero v/w")

    with server.gui.add_folder("controller tuning"):
        # applied live (also mid-balance — move in small steps). See the
        # wheel-wiggle tuning procedure in the repo README.
        tn0 = {}
        try:
            with open(TUNING_FILE) as f:
                tn0 = yaml.safe_load(f) or {}
        except FileNotFoundError:
            pass
        if runtime is not None:
            defaults = dict(runtime.ctrl.tune)
            defaults.update({k: v for k, v in tn0.items() if k in defaults})
            runtime.ctrl.set_tuning(**defaults)
        else:
            defaults = {"wheel_kd": 1.5, "wheel_gain": 1.0,
                        "vx_gain": 1.0, "vi_gain": 1.0,
                        "hip_kp_extra": 0.0}
        defaults.setdefault("hip_kp_extra", 0.0)
        s_wkd = server.gui.add_slider("wheel kd [Nm s]", min=0.3, max=4.0,
                                      step=0.1,
                                      initial_value=defaults["wheel_kd"])
        s_wg = server.gui.add_slider("wheel gain", min=0.3, max=1.2,
                                     step=0.05,
                                     initial_value=defaults["wheel_gain"])
        s_vxg = server.gui.add_slider("vx gain", min=0.3, max=1.2, step=0.05,
                                      initial_value=defaults["vx_gain"])
        s_vig = server.gui.add_slider("vel integrator", min=0.0, max=1.5,
                                      step=0.05,
                                      initial_value=defaults["vi_gain"])
        s_hkp = server.gui.add_slider("hip kp extra [Nm/rad]", min=0.0,
                                      max=80.0, step=2.0,
                                      initial_value=defaults["hip_kp_extra"])
        cb_spike = server.gui.add_checkbox(
            "spike rejector (leave OFF while hunting twitches)",
            initial_value=False)
        b_tune_save = server.gui.add_button("save tuning")
        tune_md = server.gui.add_markdown("")

        def _apply_tuning(_e=None):
            if runtime is None:
                return
            runtime.ctrl.set_tuning(
                wheel_kd=s_wkd.value, wheel_gain=s_wg.value,
                vx_gain=s_vxg.value, vi_gain=s_vig.value,
                hip_kp_extra=s_hkp.value)

        for _s in (s_wkd, s_wg, s_vxg, s_vig, s_hkp):
            _s.on_update(_apply_tuning)

        @b_tune_save.on_click
        def _(_e) -> None:
            with open(TUNING_FILE, "w") as f:
                yaml.safe_dump(
                    {"wheel_kd": float(s_wkd.value),
                     "wheel_gain": float(s_wg.value),
                     "vx_gain": float(s_vxg.value),
                     "vi_gain": float(s_vig.value),
                     "hip_kp_extra": float(s_hkp.value)}, f)
            tune_md.content = f"saved to {os.path.basename(TUNING_FILE)}"

    with server.gui.add_folder("state estimator"):
        est_md = server.gui.add_markdown(
            "_(needs all 8 motors + IMU; live in any mode)_")
        imu_md = server.gui.add_markdown("imu: —")
        imu_prev = [dict(getattr(imu, "stats", {})), time.monotonic()]
    viz = robot_viz.RobotViz(server)
    viz_state = [None]  # (quat, z_err, q_joints) for the 3D view

    with server.gui.add_folder("calibration (robot HOISTED)"):
        cal_md = server.gui.add_markdown(
            f"signs: {cal.signs.astype(int).tolist()}\n\n"
            f"offsets: {np.round(cal.offsets, 3).tolist()}"
        )
        dir_joint_dd = server.gui.add_dropdown("joint", tuple(JOINT_LABELS))
        s_pulse_tau = server.gui.add_slider("pulse torque [Nm]", min=0.5,
                                            max=8.0, step=0.5,
                                            initial_value=2.0)
        b_pulse = server.gui.add_button("pulse selected joint (+/-)")
        dir_q_md = server.gui.add_markdown("")
        b_dir_yes = server.gui.add_button("motion matched sim +dir")
        b_dir_no = server.gui.add_button("motion was OPPOSITE")
        b_range = server.gui.add_button("run range calibration (all legs)")
        range_md = server.gui.add_markdown("")

    spike_filter = SpikeFilter(1.0 / LOOP_HZ)
    twitch_log = TwitchLogger()
    last_cmd_tau = [None]

    mode_req: list = []        # mode-name strings, applied in the loop
    pulse_req: list = []       # joint indices to pulse
    range_req = [False]
    dir_answer: list = []      # (joint_idx, matched: bool)
    calibrator = [None]        # active joint_calibration state machine
    cal_kind = [""]

    @estop_btn.on_click
    def _(_e) -> None:
        estop_requested[0] = True

    @disable_btn.on_click
    def _(_event) -> None:
        for m in motors:
            m.enable_cb.value = False

    @rescan_btn.on_click
    def _(_event) -> None:
        rescan_requested[0] = True

    def do_enable_all() -> None:
        n = 0
        for m in motors:
            if not m.enable_cb.value:
                m.enable_cb.value = True   # triggers the on_update enable path
                n += 1
        print(f"enable all: {n} motor(s) newly enabled, "
              f"{len(motors)} panel(s) present")

    @b_enable_all.on_click
    def _(_e) -> None:
        do_enable_all()

    b_damp.on_click(lambda _e: mode_req.append("damp"))
    b_stand.on_click(lambda _e: mode_req.append("stand"))
    b_balance.on_click(lambda _e: mode_req.append("balance"))
    b_sit.on_click(lambda _e: mode_req.append("sit"))

    def _zero(_e):
        s_v.value = 0.0
        s_w.value = 0.0

    b_zero.on_click(_zero)
    b_pulse.on_click(
        lambda _e: pulse_req.append(JOINT_LABELS.index(dir_joint_dd.value)))
    b_range.on_click(lambda _e: range_req.__setitem__(0, True))
    b_dir_yes.on_click(
        lambda _e: dir_answer.append((JOINT_LABELS.index(dir_joint_dd.value), True)))
    b_dir_no.on_click(
        lambda _e: dir_answer.append((JOINT_LABELS.index(dir_joint_dd.value), False)))

    def do_rescan() -> None:
        for m in motors:
            try:
                m.adapter.send(m.slave_id, damiao.DISABLE_CMD)
            except Exception:
                pass
            m.folder.remove()
        motors.clear()
        for idx, b in enumerate(buses):
            b.drain()
            slaves = scan_motors(b)
            side = bus_side(slaves)
            if side == "MIXED":
                print(f"WARNING: bus {idx} (adapter ch {b.channel}) sees "
                      f"motors from BOTH legs ({[hex(s) for s in slaves]}) "
                      f"— CAN chains miswired?")
            bus_name = side if side in ("left", "right") else f"bus {idx}"
            print(f"bus {idx} (ch {b.channel}) -> {bus_name}: "
                  f"motors {[hex(s) for s in slaves] or 'none'}")
            for slave in slaves:
                motors.append(build_motor_panel(server, b, idx, slave,
                                                bus_name, cal))

    def joint_map() -> dict[int, MotorUI]:
        return {ESC_TO_JOINT[m.slave_id]: m for m in motors
                if m.slave_id in ESC_TO_JOINT}

    def robot_sensing(jm: dict[int, MotorUI], now: float):
        """(q, dq, tau) sim-frame arrays, or (None, reason)."""
        if len(jm) != 8:
            return None, f"only {len(jm)}/8 motors present"
        q_m = np.zeros(8)
        dq_m = np.zeros(8)
        tau_m = np.zeros(8)
        for j, m in jm.items():
            if m.last_fb is None or now - m.last_fb_t > STALE_FB_S:
                return None, f"stale feedback: {m.label}"
            q_m[j] = m.last_fb.pos
            dq_m[j] = m.last_fb.vel
            tau_m[j] = m.last_fb.tau
        return cal.to_sim(q_m, dq_m, tau_m), ""

    def send_robot_cmd(jm: dict[int, MotorUI], cmd_sim) -> None:
        cmd = cal.cmd_to_motor(cmd_sim)
        for j, m in jm.items():
            m.adapter.send(m.slave_id, damiao.pack_mit(
                float(cmd.q[j]), float(cmd.dq[j]),
                float(cmd.kp[j]), float(cmd.kd[j]), float(cmd.tau[j]),
                m.limits))

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            lan_ip = s.getsockname()[0]
    except OSError:
        lan_ip = socket.gethostbyname(socket.gethostname())
    print(f"\nserving GUI on http://{lan_ip}:8080 — open this from your mac")

    dt = 1.0 / LOOP_HZ
    gui_every = max(1, int(LOOP_HZ / GUI_HZ))
    viz_every = max(1, int(LOOP_HZ / 30.0))  # 3D view at ~30 Hz
    i = 0
    banner_t = [0.0]
    prev_mode = [mode_dd.value]
    loop_prev = [time.monotonic()]
    stale_streak = [0]
    overrun_count = [0]
    fb_prev: dict = {}
    fb_prev_t = [time.monotonic()]
    joy_was_active = [False]
    joy_switch_state: dict = {}
    bus_err_total = [0, 0]
    try:
        while True:
            if rescan_requested[0]:
                rescan_requested[0] = False
                do_rescan()
                if not motors:
                    print("no motors found — power them on and hit RESCAN")

            loop_start = time.monotonic()
            now = loop_start
            # a late iteration start means the HOST stalled (GC, viser,
            # USB backpressure) — feedback ages during the stall through
            # no fault of the motors, so staleness must not trip on it
            loop_stalled = (now - loop_prev[0]) > 3 * dt
            if loop_stalled:
                overrun_count[0] += 1
            loop_prev[0] = now

            for a in adapters:
                for frame in a.drain():
                    if len(frame.data) < 8:
                        continue
                    motor_id = frame.data[0] & 0x0F
                    for m in motors:
                        # motor ids are unique robot-wide, so id alone routes
                        # correctly even when a dual adapter's two channels
                        # share one rx queue
                        if m.slave_id == motor_id:
                            m.last_fb = damiao.Feedback.parse(frame.data,
                                                              m.limits)
                            m.last_fb_t = now
                            m.fb_count += 1
                            if m.last_fb.err_code != m.prev_err:
                                if m.prev_err is not None:
                                    print(f"[{now:.3f}] {m.label}: "
                                          f"status -> {m.last_fb.err_name}")
                                m.prev_err = m.last_fb.err_code
                            break

            if estop_requested[0]:
                estop_requested[0] = False
                mode_dd.value = "robot"
                if runtime is not None:
                    runtime.trip("DAMP ALL PRESSED")
                calibrator[0] = None
                print(f"[{now:.3f}] DAMP ALL -> damp")

            jm = joint_map()
            if len(jm) == 8 and all(m.last_fb is not None for m in jm.values()):
                twitch_log.record(
                    now,
                    np.array([jm[j].last_fb.pos for j in range(8)]),
                    np.array([jm[j].last_fb.vel for j in range(8)]),
                    np.array([jm[j].last_fb.tau for j in range(8)]),
                    [jm[j].last_fb.err_code for j in range(8)],
                    last_cmd_tau[0], loop_stalled)

            # ---- calibration requests (need all 8 motors, any mode) -------
            all_enabled = len(jm) == 8 and all(
                m.enable_cb.value for m in jm.values())
            if pulse_req and calibrator[0] is None:
                j = pulse_req.pop(0)
                if not all_enabled:
                    dir_q_md.content = ("🔴 **motors not enabled/present — "
                                        "check panels**")
                else:
                    c = joint_calibration.DirectionCalibrator(dt, j)
                    c.PULSE_TAU = float(s_pulse_tau.value)
                    calibrator[0] = c
                    cal_kind[0] = "direction"
                    dir_q_md.content = (f"pulsing **{JOINT_LABELS[j]}** at "
                                        f"±{c.PULSE_TAU:.1f} Nm: watch it!")
            if range_req[0] and calibrator[0] is None:
                range_req[0] = False
                if not all_enabled:
                    range_md.content = ("🔴 **motors not enabled — press "
                                        "'enable all motors' first**")
                    calibrator[0] = None
                else:
                    calibrator[0] = joint_calibration.RangeCalibrator(
                        np.array([robot_config.BY_ESC_ID[e].q_range
                                  for e in sorted(ESC_TO_JOINT,
                                                  key=ESC_TO_JOINT.get)]),
                        dt, cal.signs)
                    cal_kind[0] = "range"
                    range_md.content = "range calibration running..."
            for j, matched in ([dir_answer.pop(0)] if dir_answer else []):
                resp = getattr(calibrator[0], "response_sign", 1.0) \
                    if cal_kind[0] == "direction" else 1.0
                cal.signs[j] = resp if matched else -resp
                save_cal(cal)
                cal_md.content = (
                    f"signs: {cal.signs.astype(int).tolist()}\n\n"
                    f"offsets: {np.round(cal.offsets, 3).tolist()}")

            mode = mode_dd.value
            if mode != prev_mode[0]:
                # mode switches must never carry stale commands across:
                # -> manual: snap pos sliders to actual positions so the
                #    slider stream doesn't yank enabled motors
                # -> robot: runtime always starts parked in damp
                if mode == "manual":
                    for m in motors:
                        if m.last_fb is not None:
                            m.pos.value = min(max(float(round(m.last_fb.pos, 2)),
                                                  m.pos.min), m.pos.max)
                if runtime is not None:
                    runtime.mode = "damp"
                    runtime.v_cmd = runtime.w_cmd = 0.0
                s_v.value = 0.0
                s_w.value = 0.0
                print(f"[{now:.3f}] control mode -> {mode} (robot parked in damp)")
                prev_mode[0] = mode

            if calibrator[0] is not None and len(jm) == 8:
                # calibration overrides both modes; raw motor frame I/O
                c = calibrator[0]
                q_m = np.array([jm[j].last_fb.pos if jm[j].last_fb else 0.0
                                for j in range(8)])
                dq_m = np.array([jm[j].last_fb.vel if jm[j].last_fb else 0.0
                                 for j in range(8)])
                cmd = c.tick(q_m, dq_m)
                for j, m in jm.items():
                    m.adapter.send(m.slave_id, damiao.pack_mit(
                        float(cmd.q[j]), float(cmd.dq[j]),
                        float(cmd.kp[j]), float(cmd.kd[j]), float(cmd.tau[j]),
                        m.limits))
                if c.done:
                    if cal_kind[0] == "direction":
                        diff = c._peak_pos - c._peak_neg
                        if c.moved:
                            dir_q_md.content = (
                                f"pulse done: response {diff:+.2f} rad/s "
                                f"(peaks {c._peak_pos:+.2f}/{c._peak_neg:+.2f})."
                                " Did it move in the sim + direction? answer below")
                        else:
                            dir_q_md.content = (
                                f"🔴 **no motion detected** — response "
                                f"{diff:+.3f} rad/s (threshold "
                                f"{c.MIN_MOTION}). If it truly didn't move, "
                                "raise the pulse torque; if it visibly moved, "
                                "check this joint's feedback readout.")
                    else:
                        cal.offsets = c.result.offsets
                        save_cal(cal)
                        we = np.round(c.result.width_error, 3).tolist()
                        range_md.content = (
                            f"done. width errors: {we}"
                            + (f" — TIMED OUT: {c.timed_out}" if c.timed_out
                               else "") + " — offsets saved; RESCAN to "
                            "refresh slider limits")
                        cal_md.content = (
                            f"signs: {cal.signs.astype(int).tolist()}\n\n"
                            f"offsets: {np.round(cal.offsets, 3).tolist()}")
                    calibrator[0] = None
            elif mode == "robot" and runtime is not None:
                # ---- robot mode: LQR runtime drives everything ----------
                if any(len(server.get_clients()) > 0 for _ in (0,)):
                    runtime.note_heartbeat(now)
                runtime.check_deadman(now)
                for req in mode_req:
                    if req == "enable":
                        do_enable_all()
                        print(f"[{now:.3f}] switch: enable all motors")
                        continue
                    if req == "damp":
                        runtime.set_mode("damp", Snapshot(
                            q=np.zeros(8), dq=np.zeros(8),
                            quat=np.array([1.0, 0, 0, 0]),
                            gyro=np.zeros(3), accel_x=0.0))
                        print(f"[{now:.3f}] robot mode -> damp")
                        continue
                    if not all_enabled:
                        missing = sorted(set(range(8)) - set(jm))
                        unchecked = [JOINT_LABELS[j] for j, m in jm.items()
                                     if not m.enable_cb.value]
                        detail = []
                        if missing:
                            detail.append("missing from scan: "
                                          + ", ".join(JOINT_LABELS[j] for j in missing))
                        if unchecked:
                            detail.append("not enabled: " + ", ".join(unchecked))
                        trip_md.content = ("🔴 **cannot arm — "
                                           + "; ".join(detail) + "**")
                        continue
                    if req == "balance" and (imu is None or
                                             not getattr(imu, "available", False)):
                        trip_md.content = "🔴 **no IMU driver — balance locked out**"
                        continue
                    # stand/sit/balance all track sim-frame joint targets:
                    # without calibrated zero offsets the motor-frame
                    # targets are wrong and motors go to unexpected places
                    if np.all(cal.offsets == 0.0):
                        trip_md.content = ("🔴 **calibration.yaml has zero "
                                           f"offsets — calibrate before {req}**")
                        continue
                    sensing, why = robot_sensing(jm, now)
                    if sensing is None:
                        trip_md.content = f"🔴 **cannot arm: {why}**"
                        continue
                    q, dq, _ = sensing
                    imu_s = imu.read() if imu is not None else None
                    quat = imu_s[0] if imu_s else np.array([1.0, 0, 0, 0])
                    gyro = imu_s[1] if imu_s else np.zeros(3)
                    runtime.set_mode(req, Snapshot(q=q, dq=dq, quat=quat,
                                                   gyro=gyro, accel_x=0.0))
                    print(f"[{now:.3f}] robot mode -> {req}")
                mode_req.clear()
                if runtime.mode == "damp" and (s_v.value or s_w.value):
                    s_v.value = 0.0
                    s_w.value = 0.0
                joy = joystick.read() if cb_joy.value else None
                if joy is None and joy_was_active[0]:
                    # transmitter dropped out while driving: zero everything
                    # (the sliders were mirroring the stick, so they must
                    # not keep commanding its last value)
                    joy_was_active[0] = False
                    s_v.value = 0.0
                    s_w.value = 0.0
                    print(f"[{now:.3f}] joystick link lost — commands zeroed")
                if joy is not None:
                    if not joy_was_active[0]:
                        # link (re)acquired: prime switch state silently
                        joy_switch_state.clear()
                        for (chn, _pos) in JOY_MODE_SWITCHES:
                            joy_switch_state[chn] = switch_pos(joy[chn])
                    for (chn, pos), action in JOY_MODE_SWITCHES.items():
                        cur = switch_pos(joy[chn])
                        if cur != joy_switch_state.get(chn) and cur == pos:
                            mode_req.append(action)
                            print(f"[{now:.3f}] joystick switch {chn} -> {action}")
                    for chn in {c for (c, _p) in JOY_MODE_SWITCHES}:
                        joy_switch_state[chn] = switch_pos(joy[chn])
                if joy is not None:
                    joy_was_active[0] = True
                    def _db(x):
                        return 0.0 if abs(x) < JOY_DEADBAND else x
                    v_joy = JOY_V_SIGN * _db(joy["ch2"]) * V_MAX_TELEOP
                    w_joy = JOY_W_SIGN * _db(joy["ch1"]) * W_MAX_TELEOP
                    runtime.set_command(v_joy, w_joy,
                                        V_MAX_TELEOP, W_MAX_TELEOP)
                else:
                    # no link (or disabled): sliders drive; a transmitter
                    # dropping out mid-drive therefore commands zero via
                    # the slider zeroing in damp or their last value —
                    # zero the sliders too when the stick WAS in control
                    runtime.set_command(s_v.value, s_w.value,
                                        V_MAX_TELEOP, W_MAX_TELEOP)

                sensing, why = robot_sensing(jm, now)
                if sensing is None:
                    # distinguish real motor silence from host stalls: only
                    # trip after 3 consecutive stale ticks with a healthy
                    # loop (a stall resets nothing — we just hold and let
                    # the next tick re-check with fresh drains)
                    if not loop_stalled:
                        stale_streak[0] += 1
                    if stale_streak[0] >= 3 and runtime.mode != "damp":
                        runtime.trip(f"SENSING LOST ({why})")
                    # Bootstrap/keep-alive: DAMIAO motors only reply when
                    # spoken to, and we only send MIT frames when sensing
                    # is fresh — so poke every mapped motor with a passive
                    # status query at 50 Hz until feedback recovers.
                    if i % 2 == 0:
                        for m in jm.values():
                            try:
                                m.adapter.send(damiao.BROADCAST_ID,
                                               damiao.refresh_query(m.slave_id))
                            except Exception:
                                pass
                else:
                    stale_streak[0] = 0
                    q, dq, _ = sensing
                    if cb_spike.value:
                        q, dq = spike_filter.apply(q, dq)
                    imu_s = imu.read() if imu is not None else None
                    if imu_s is None and runtime.mode == "balance":
                        runtime.trip("IMU LOST")
                    quat = imu_s[0] if imu_s else np.array([1.0, 0, 0, 0])
                    gyro = imu_s[1] if imu_s else np.zeros(3)
                    ax = forward_accel(quat, imu_s[2]) if imu_s else 0.0
                    cmd = runtime.tick(Snapshot(q=q, dq=dq, quat=quat,
                                                gyro=gyro, accel_x=ax))
                    send_robot_cmd(jm, cmd)
                    last_cmd_tau[0] = cmd.tau
                    x = getattr(runtime.ctrl, "last_x", None)
                    if x is not None:
                        viz_state[0] = (quat,
                                        x[runtime.ctrl._idx["z"]], q)
                if runtime.tripped and now - banner_t[0] > 1.0:
                    banner_t[0] = now
                    print(f"[{now:.3f}] !! TRIP: {runtime.trip_reason} — damp")
            else:
                mode_req.clear()
                # ---- manual mode: original per-motor slider streaming ----
                for m in motors:
                    if m.enable_cb.value:
                        m.adapter.send(m.slave_id, damiao.pack_mit(
                            m.pos.value, m.vel.value,
                            m.kp.value, m.kd.value, m.tau.value,
                            m.limits))

            if i % viz_every == 0:
                # estimator preview: keep the panel + 3D view live in ANY
                # mode (e.g. hoisted IMU tilt-sign checks in manual mode).
                # Skipped while balancing — the control tick refreshes it.
                if (runtime is not None
                        and not (mode == "robot" and runtime.mode == "balance")
                        and calibrator[0] is None):
                    jm_prev = joint_map()
                    imu_s = imu.read() if imu is not None else None
                    if len(jm_prev) == 8 and all(
                            m.last_fb is not None for m in jm_prev.values()):
                        q_m = np.array([jm_prev[j].last_fb.pos for j in range(8)])
                        dq_m = np.array([jm_prev[j].last_fb.vel for j in range(8)])
                        q, dq, _ = cal.to_sim(q_m, dq_m, np.zeros(8))
                        quat = imu_s[0] if imu_s else np.array([1.0, 0, 0, 0])
                        gyro = imu_s[1] if imu_s else np.zeros(3)
                        ax = forward_accel(quat, imu_s[2]) if imu_s else 0.0
                        x = runtime.ctrl.estimated_state(
                            Snapshot(q=q, dq=dq, quat=quat,
                                     gyro=gyro, accel_x=ax),
                            viz_every * dt)
                        viz_state[0] = (quat,
                                        x[runtime.ctrl._idx["z"]], q)
                    if imu_s is None:
                        est_md.content = ("_(needs IMU — "
                                          f"{type(imu).__name__} — 3D view "
                                          "shows joints, zero tilt)_")
                if viz_state[0] is not None:
                    viz.update(*viz_state[0])
                    viz_state[0] = None

            if i % gui_every == 0:
                fb_rates_done = fb_prev_t[0]
                stats = getattr(imu, "stats", None)
                if stats and now - imu_prev[1] >= 1.0:
                    span = now - imu_prev[1]
                    p0 = imu_prev[0]
                    rq = (stats["n_quat"] - p0.get("n_quat", 0)) / span
                    rg = (stats["n_gyro"] - p0.get("n_gyro", 0)) / span
                    bad = stats["bad_quat"] - p0.get("bad_quat", 0)
                    coord = {0: "ENU", 1: "NED→zup", 2: "NWU"}.get(
                        stats["coord"], "?")
                    warn = " 🔴" if (rq < 50 or bad > 0) else ""
                    imu_md.content = (f"imu: quat {rq:.0f} Hz, gyro {rg:.0f} "
                                      f"Hz, bad {bad}/s, {coord}{warn}")
                    imu_prev[0] = dict(stats)
                    imu_prev[1] = now
                for m in motors:
                    if not m.enable_cb.value and mode != "robot":
                        try:
                            m.adapter.send(damiao.BROADCAST_ID,
                                           damiao.refresh_query(m.slave_id))
                        except Exception:
                            pass
                    if m.last_fb is not None:
                        age_ms = (now - m.last_fb_t) * 1e3
                        rate = (m.fb_count - fb_prev.get(m.slave_id, 0)) / \
                            max(now - fb_prev_t[0], 1e-3)
                        fb_prev[m.slave_id] = m.fb_count
                        m.fb_fresh.value = f"{age_ms:.0f} ms old, {rate:.0f} Hz"
                        m.fb_q.value = round(m.last_fb.pos, 3)
                        m.fb_dq.value = round(m.last_fb.vel, 3)
                        m.fb_tau.value = round(m.last_fb.tau, 3)
                        m.fb_temp.value = (f"mos {m.last_fb.t_mos}  "
                                           f"rotor {m.last_fb.t_rotor}")
                        m.fb_err.value = m.last_fb.err_name
                fb_prev_t[0] = now
                joy = joystick.read()
                st = joystick.stats
                if joy is not None:
                    joy_md.content = (
                        f"joystick: **ch1 {joy['ch1']:+.2f} "
                        f"ch2 {joy['ch2']:+.2f}**  |  aux "
                        f"ch5 {switch_pos(joy['ch5']):+d} "
                        f"ch6 {switch_pos(joy['ch6']):+d} "
                        f"ch7 {switch_pos(joy['ch7']):+d} "
                        f"ch8 {switch_pos(joy['ch8']):+d}  |  "
                        f"lq {st['lq']}  rssi {st['rssi_dbm']} dBm")
                    if cb_joy.value and mode == "robot" and runtime is not None:
                        s_v.value = round(runtime.v_cmd, 2)
                        s_w.value = round(runtime.w_cmd, 2)
                elif getattr(joystick, "available", False):
                    joy_md.content = "joystick: 🔴 **no link** (transmitter off?)"
                else:
                    joy_md.content = "joystick: not connected"
                if runtime is not None:
                    tel = runtime.telemetry()
                    robot_status.content = (
                        f"**mode:** {mode} / {tel['mode']}  |  "
                        f"v={tel['v_cmd']:+.2f} w={tel['w_cmd']:+.2f}  |  "
                        f"loop stalls: {overrun_count[0]}  |  "
                        f"CAN errs: {bus_err_total[0]}/{bus_err_total[1]}  |  "
                        f"twitches: {twitch_log.count}  "
                        f"spikes rej: {spike_filter.reject_count}")
                    if tel.get("tripped"):
                        trip_md.content = (
                            f"🔴 **TRIPPED: {tel['reason']}** — motors "
                            "damped; press a mode button to re-arm")
                    if "roll" in tel:
                        est_md.content = (
                            f"roll **{tel['roll']:+.3f}** pitch "
                            f"**{tel['pitch']:+.3f}** rad\n\n"
                            f"vx **{tel['vx']:+.2f}** m/s  yaw rate "
                            f"**{tel['dyaw']:+.2f}** rad/s\n\n"
                            f"z err {tel['z']:+.3f} m | wheels "
                            f"{tel['wheel_l']:+.1f}/{tel['wheel_r']:+.1f} rad/s"
                            f"\n\nintegrators {np.round(tel['integ'], 3).tolist()}")
                elif tables_err:
                    robot_status.content = f"**robot mode unavailable:** {tables_err}"

            for idx, a in enumerate(adapters):
                if a.err_count:
                    if idx < 2:
                        bus_err_total[idx] += a.err_count
                    print(f"[{now:.3f}] bus {idx}: "
                          f"{a.err_count} CAN error frame(s)")
                    a.err_count = 0

            elapsed = time.monotonic() - loop_start
            if elapsed > 3 * dt:
                print(f"[{now:.3f}] loop overrun: "
                      f"{elapsed * 1000:.1f} ms (target {dt * 1000:.0f} ms)")
            i += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        for m in motors:
            try:
                m.adapter.send(m.slave_id, damiao.DISABLE_CMD)
            except Exception:
                pass
        for a in adapters:
            a._shutting_down = True
        time.sleep(0.3)
        os._exit(0)


if __name__ == "__main__":
    main()
