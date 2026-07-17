"""Joint calibration state machines for the CAN stack (numpy-only).

Port of pineapple_lqr/lqr/calibration.py (validated there in MuJoCo with
injected sign flips/offsets) onto robot_config joint ranges. Consumes
motor-frame (q, dq) snapshots at the loop rate and emits sim-frame-agnostic
MIT commands; drive it from the motor_gui control loop with the robot
HOISTED.

Direction calibration (run once): differential +/- torque pulses per joint
(gravity bias cancels); the operator confirms the observed motion against
the sim convention in the GUI. Range calibration (rerun after re-zeroes):
slow position-servo sweep to both stops per leg joint — a stop is tracking
error while the joint is stationary (contact torque bounded ~ SWEEP_KP *
ERR_STOP ≈ 2 Nm) — then offsets come from aligning range midpoints with
the XML ranges. Wheels are continuous: no zero needed.
"""

from __future__ import annotations

import dataclasses

import numpy as np

from lqr_runtime import MitCommand

LEG_IDX = [0, 1, 2, 4, 5, 6]
WHEEL_IDX = [3, 7]


@dataclasses.dataclass
class DirectionResult:
    signs: np.ndarray
    moved: np.ndarray


class DirectionCalibrator:
    PULSE_TAU = 2.0
    PULSE_S = 0.35
    SETTLE_S = 0.8
    HOLD_KP = 15.0
    HOLD_KD = 1.0
    MIN_MOTION = 0.2

    def __init__(self, dt: float, joint: int):
        """Calibrate a single joint (GUI drives one at a time)."""
        self.dt = dt
        self.joint = joint
        self._phase = 0
        self._t = 0.0
        self._peak_pos = 0.0
        self._peak_neg = 0.0
        self._hold_q: np.ndarray | None = None
        self.done = False
        self.moved = False
        self.response_sign = 1.0

    def tick(self, q: np.ndarray, dq: np.ndarray) -> MitCommand:
        if self._hold_q is None:
            self._hold_q = q.copy()
        kp = np.full(8, self.HOLD_KP)
        kd = np.full(8, self.HOLD_KD)
        kp[WHEEL_IDX] = 0.0
        tau = np.zeros(8)
        cmd_q = self._hold_q.copy()
        if self.done:
            return MitCommand(q=cmd_q, dq=np.zeros(8), kp=kp, kd=kd, tau=tau)
        j = self.joint
        self._t += self.dt
        if self._phase in (0, 2):
            kp[j] = 0.0
            kd[j] = 0.0
            sgn = 1.0 if self._phase == 0 else -1.0
            tau[j] = sgn * self.PULSE_TAU
            if self._phase == 0:
                if abs(dq[j]) > abs(self._peak_pos):
                    self._peak_pos = dq[j]
            else:
                if abs(dq[j]) > abs(self._peak_neg):
                    self._peak_neg = dq[j]
            if self._t >= self.PULSE_S:
                self._phase += 1
                self._t = 0.0
        else:
            if self._t >= self.SETTLE_S:
                if self._phase == 3:
                    diff = self._peak_pos - self._peak_neg
                    self.moved = abs(diff) > self.MIN_MOTION
                    self.response_sign = 1.0 if diff >= 0 else -1.0
                    self.done = True
                else:
                    self._phase = 2
                self._t = 0.0
        return MitCommand(q=cmd_q, dq=np.zeros(8), kp=kp, kd=kd, tau=tau)


@dataclasses.dataclass
class RangeResult:
    q_min: np.ndarray
    q_max: np.ndarray
    offsets: np.ndarray
    width_error: np.ndarray


class RangeCalibrator:
    SWEEP_KP = 20.0
    SWEEP_KD = 1.0
    SWEEP_RATE = 0.5
    ERR_STOP = 0.10
    STALL_VEL = 0.08
    ERR_HOLD_S = 0.25
    BACKOFF_S = 0.8
    TIMEOUT_S = 15.0
    HOLD_KP = 15.0
    HOLD_KD = 1.0

    def __init__(self, ranges: np.ndarray, dt: float, signs: np.ndarray):
        """ranges: (8, 2) sim-frame joint ranges (robot_config q_range)."""
        self.dt = dt
        self.ranges = np.asarray(ranges, float)
        self.signs = np.asarray(signs, float)
        self._legpos = 0
        self._phase = 0
        self._t = 0.0
        self._hold = 0.0
        self._target: float | None = None
        self._hold_q: np.ndarray | None = None
        self.done = False
        self.timed_out: list[int] = []
        n = len(LEG_IDX)
        self.result = RangeResult(
            q_min=np.full(n, np.nan), q_max=np.full(n, np.nan),
            offsets=np.zeros(8), width_error=np.full(n, np.nan),
        )

    @property
    def active_joint(self) -> int:
        return LEG_IDX[min(self._legpos, len(LEG_IDX) - 1)]

    def _finish_joint(self, q: np.ndarray):
        k = self._legpos
        j = LEG_IDX[k]
        lo, hi = self.ranges[j]
        mid_meas = 0.5 * (self.result.q_min[k] + self.result.q_max[k])
        self.result.offsets[j] = mid_meas - self.signs[j] * 0.5 * (lo + hi)
        self.result.width_error[k] = (
            self.result.q_max[k] - self.result.q_min[k]
        ) - (hi - lo)
        self._legpos += 1
        self._phase = 0
        self._t = 0.0
        self._hold = 0.0
        self._target = None
        self._hold_q = q.copy()
        if self._legpos >= len(LEG_IDX):
            self.done = True

    def tick(self, q: np.ndarray, dq: np.ndarray) -> MitCommand:
        if self._hold_q is None:
            self._hold_q = q.copy()
        kp = np.full(8, self.HOLD_KP)
        kd = np.full(8, self.HOLD_KD)
        kp[WHEEL_IDX] = 0.0
        cmd_q = self._hold_q.copy()
        z = np.zeros(8)
        if self.done:
            return MitCommand(q=cmd_q, dq=z, kp=kp, kd=kd, tau=z.copy())
        k = self._legpos
        j = LEG_IDX[k]
        self._t += self.dt
        if self._phase in (0, 2):
            direction = -1.0 if self._phase == 0 else 1.0
            if self._target is None:
                self._target = q[j]
            self._target += direction * self.SWEEP_RATE * self.dt
            kp[j] = self.SWEEP_KP
            kd[j] = self.SWEEP_KD
            cmd_q[j] = self._target
            err = direction * (self._target - q[j])
            if err > self.ERR_STOP and abs(dq[j]) < self.STALL_VEL:
                self._hold += self.dt
            else:
                self._hold = 0.0
            if self._hold >= self.ERR_HOLD_S:
                if self._phase == 0:
                    self.result.q_min[k] = q[j]
                else:
                    self.result.q_max[k] = q[j]
                self._phase += 1
                self._t = 0.0
                self._hold = 0.0
                self._target = None
                self._hold_q = q.copy()
            elif self._t > self.TIMEOUT_S:
                self.timed_out.append(j)
                self._finish_joint(q)
        else:
            if self._t >= self.BACKOFF_S:
                if self._phase == 3:
                    self._finish_joint(q)
                else:
                    self._phase = 2
                self._t = 0.0
        return MitCommand(q=cmd_q, dq=z, kp=kp, kd=kd, tau=z.copy())
