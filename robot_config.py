"""Robot actuator layout.

Two legs, four actuators each. IDs are globally unique across the robot:
each actuator takes a contiguous (ESC_ID, MST_ID) pair, left leg first.

    l_hip_aa  0x01/0x02   r_hip_aa  0x09/0x0A
    l_hip_fe  0x03/0x04   r_hip_fe  0x0B/0x0C
    l_knee    0x05/0x06   r_knee    0x0D/0x0E
    l_ankle   0x07/0x08   r_ankle   0x0F/0x10
"""

from __future__ import annotations

from dataclasses import dataclass

import damiao


@dataclass(frozen=True)
class ActuatorCfg:
    name: str
    model: str      # key into damiao.MODEL_LIMITS
    esc_id: int     # command CAN ID
    mst_id: int     # feedback CAN ID
    kp_default: float
    kd_default: float
    q_min: float | None = None   # joint limit [rad]; None = protocol limit
    q_max: float | None = None

    @property
    def limits(self) -> damiao.Limits:
        return damiao.MODEL_LIMITS[self.model]

    @property
    def q_range(self) -> tuple[float, float]:
        """Joint range clamped to what the MIT frame can encode."""
        p = self.limits.p_max
        lo = -p if self.q_min is None else max(self.q_min, -p)
        hi = p if self.q_max is None else min(self.q_max, p)
        return lo, hi


# Joint ranges from pineappleV3_mjcf/pineappleV3_armless.xml. hip_aa is a
# roll joint so its range mirrors between legs; the ankle drives a wheel
# (continuous — no limit). Assumes motor zero = MJCF zero configuration.
#
# joint order within a leg:
# (joint, model, kp_default, kd_default, {side: (q_min, q_max)})
_LEG_JOINTS = [
    ("hip_aa", "J4340P", 50.0, 2.0, {"l": (-0.6108, 1.0472),
                                     "r": (-1.0472, 0.6108)}),
    ("hip_fe", "J4340",  50.0, 2.0, {"l": (0.0, 1.74533),
                                     "r": (0.0, 1.74533)}),
    ("knee",   "J6248P",  9.0, 2.0, {"l": (-3.229, 0.0),
                                     "r": (-3.229, 0.0)}),
    ("ankle",  "J6006",  10.0, 1.0, {"l": (None, None),
                                     "r": (None, None)}),
]

ACTUATORS: list[ActuatorCfg] = [
    ActuatorCfg(f"{side}_{joint}", model,
                esc_id=base + 2 * i, mst_id=base + 2 * i + 1,
                kp_default=kp, kd_default=kd,
                q_min=ranges[side][0], q_max=ranges[side][1])
    for side, base in (("l", 0x01), ("r", 0x09))
    for i, (joint, model, kp, kd, ranges) in enumerate(_LEG_JOINTS)
]

BY_ESC_ID: dict[int, ActuatorCfg] = {a.esc_id: a for a in ACTUATORS}
