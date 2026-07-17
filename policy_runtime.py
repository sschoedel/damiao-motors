"""RL locomotion policy runtime for the robot (mjlab-exported ONNX).

Mirrors pineapple_rl_deploy/mjlab_deploy.py's observation/action contract
exactly, adapted to the motor_control stack:

  obs (one step, metadata joint order = RIGHT leg first!):
    [base_ang_vel*0.25, projected_gravity, cmd_vel, height_cmd*3.0,
     leg joint_pos_rel, joint_vel*0.05, last_action]
  history: 6 steps, flattened per feature group, clipped to +-100
  actions: 6 leg position targets (default + a*0.5, kp 40/25/25) then
           2 wheel velocity targets (a*5.0, kd 0.3), at 50 Hz

The ONNX metadata (joint names, defaults, leg action scale) is the source
of truth; note the metadata order is hip_r..wheel_r, hip_l..wheel_l while
this stack uses left-first ESC order — MD_TO_OURS maps between them.
Verified against 2026-07-15_12-55-03_overnight_r3_robust_hard.onnx
(Robust-Hard: 35 ms delay, kp +-30%, friction 0.4-2.5x, pushes, mass DR).
"""

from __future__ import annotations

import os

import numpy as np

from lqr_runtime import MitCommand, Snapshot

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_POLICY = os.path.join(BASE_DIR, "policy_r3_robust_hard.onnx")

# metadata joint order (right leg first) -> our left-first joint index
MD_TO_OURS = np.array([4, 5, 6, 7, 0, 1, 2, 3])
OURS_TO_MD = np.argsort(MD_TO_OURS)

# deploy config values (pineapple_v3_rl_deploy.yaml), metadata joint order
KPS_MD = np.array([40.0, 25.0, 25.0, 0.0, 40.0, 25.0, 25.0, 0.0])
KDS_MD = np.array([1.0, 0.5, 0.5, 0.3, 1.0, 0.5, 0.5, 0.3])
WHEEL_ACTION_SCALE = 5.0
ANG_VEL_SCALE = 0.25
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
CMD_SCALE = np.array([1.0, 1.0, 1.0])
HEIGHT_SCALE = 3.0
OBS_HISTORY = 6
POLICY_HZ = 50.0
# command limits the policy was trained/configured for
V_LIMIT = 1.0
W_LIMIT = 0.5
HEIGHT_CMD = 0.38  # fixed default height (trained range 0.33-0.43)


def projected_gravity(quat: np.ndarray) -> np.ndarray:
    """Unit gravity ("down") vector in the base frame, mjlab convention."""
    qw, qx, qy, qz = quat
    return np.array([
        2.0 * (-qz * qx + qw * qy),
        -2.0 * (qz * qy + qw * qx),
        1.0 - 2.0 * (qw * qw + qz * qz),
    ])


def diamond_constraint(v: float, w: float) -> tuple[float, float]:
    """|v|/V + |w|/W <= 1 (deploy-stack teleop constraint)."""
    ratio = abs(v) / V_LIMIT + abs(w) / W_LIMIT
    if ratio > 1.0:
        v /= ratio
        w /= ratio
    return v, w


class PolicyController:
    """ONNX policy stepping at 50 Hz inside the 100 Hz control loop."""

    def __init__(self, onnx_path: str = DEFAULT_POLICY, loop_hz: float = 100.0):
        import onnxruntime as ort

        # errors only — device discovery logs harmless GPU-probe warnings
        # on the Pi (/sys/class/drm has no vendor files there)
        ort.set_default_logger_severity(3)
        self.session = ort.InferenceSession(
            onnx_path, providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        md = self.session.get_modelmeta().custom_metadata_map
        names = md["joint_names"].split(",")
        assert names[0] == "hip_r_joint" and names[4] == "hip_l_joint", names
        self.default_md = np.array(
            [float(x) for x in md["default_joint_pos"].split(",")])
        scale = [float(x) for x in md["action_scale"].split(",")]
        self.leg_action_scale = np.full(6, scale[0]) if len(scale) == 1 \
            else np.array(scale)
        self.leg_idx_md = np.flatnonzero(KPS_MD > 0.0)   # [0,1,2,4,5,6]
        self.wheel_idx_md = np.flatnonzero(KPS_MD == 0.0)  # [3,7]
        one_step = 3 + 3 + 3 + 1 + 6 + 8 + 8
        expected = one_step * OBS_HISTORY
        got = self.session.get_inputs()[0].shape[1]
        assert got == expected, f"obs size mismatch: model {got} != {expected}"
        self._divider = max(1, int(round(loop_hz / POLICY_HZ)))
        self.reset()

    def reset(self):
        self._hist = None  # backfilled with the first obs, like mjlab reset
        self._action = np.zeros(8, dtype=np.float32)  # legs(6)+wheels(2)
        self._tick = 0
        self._cmd = None  # cached MitCommand between policy steps

    def mit_command(self, snap: Snapshot, v_cmd: float, w_cmd: float) -> MitCommand:
        """Call at the loop rate; runs the network every `_divider` ticks
        and holds targets in between (the motor board PD keeps closing the
        loop at its own rate, exactly like the 500 Hz deploy stack)."""
        if self._cmd is None or self._tick % self._divider == 0:
            self._cmd = self._step(snap, v_cmd, w_cmd)
        self._tick += 1
        return self._cmd

    def _step(self, snap: Snapshot, v_cmd: float, w_cmd: float) -> MitCommand:
        v_cmd, w_cmd = diamond_constraint(
            float(np.clip(v_cmd, -V_LIMIT, V_LIMIT)),
            float(np.clip(w_cmd, -W_LIMIT, W_LIMIT)))
        q_md = snap.q[MD_TO_OURS]
        dq_md = snap.dq[MD_TO_OURS]
        leg_pos_rel = (q_md[self.leg_idx_md]
                       - self.default_md[self.leg_idx_md]) * DOF_POS_SCALE
        obs = np.concatenate([
            snap.gyro * ANG_VEL_SCALE,
            projected_gravity(snap.quat),
            np.array([v_cmd, 0.0, w_cmd]) * CMD_SCALE,
            [HEIGHT_CMD * HEIGHT_SCALE],
            leg_pos_rel,
            dq_md * DOF_VEL_SCALE,
            self._action,
        ]).astype(np.float32)
        # per-feature-group flattened history (mjlab convention)
        if self._hist is None:
            # mjlab's CircularBuffer backfills the whole history with the
            # first frame after reset — match it (the DDS deploy stack
            # zero-fills instead, which is a startup transient we avoid)
            self._hist = np.tile(obs, (OBS_HISTORY, 1))
        else:
            self._hist = np.roll(self._hist, shift=-1, axis=0)
            self._hist[-1] = obs
        sizes = [3, 3, 3, 1, 6, 8, 8]
        splits = np.cumsum(sizes)[:-1]
        groups = np.split(self._hist, splits, axis=1)
        obs_vec = np.clip(
            np.concatenate([g.flatten() for g in groups]), -100.0, 100.0)

        (action,) = self.session.run(
            [self.output_name],
            {self.input_name: obs_vec.reshape(1, -1).astype(np.float32)})
        self._action = np.asarray(action, dtype=np.float32).squeeze(0)

        # actions -> per-joint targets in metadata order
        q_target_md = self.default_md.copy()
        dq_target_md = np.zeros(8)
        for i, j in enumerate(self.leg_idx_md):
            q_target_md[j] = (self.default_md[j]
                              + self._action[i] * self.leg_action_scale[i])
        for k, j in enumerate(self.wheel_idx_md):
            dq_target_md[j] = self._action[6 + k] * WHEEL_ACTION_SCALE

        # back to our left-first order for the MIT command
        return MitCommand(
            q=q_target_md[OURS_TO_MD],
            dq=dq_target_md[OURS_TO_MD],
            kp=KPS_MD[OURS_TO_MD],
            kd=KDS_MD[OURS_TO_MD],
            tau=np.zeros(8),
        )


def create(loop_hz: float = 100.0):
    try:
        pc = PolicyController(loop_hz=loop_hz)
        print(f"policy loaded: {os.path.basename(DEFAULT_POLICY)}")
        return pc
    except Exception as e:
        print(f"policy unavailable: {e}")
        return None
