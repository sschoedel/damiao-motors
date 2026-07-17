"""Damiao motor protocol helpers on top of dmcan.py.

Frame formats per J6006-2EC User Manual V1.0:
- Enable/disable/set-zero: 7x 0xFF + terminator byte (0xFC/0xFD/0xFE)
- MIT control frame: 8-byte packed [pos:16, vel:12, kp:12, kd:12, tau:12]
- Feedback frame: 8-byte packed [id|err:8, pos:16, vel:12, tau:12, t_mos:8, t_rotor:8]

Motor CAN bus is fixed at 1 Mbps standard 11-bit frames.
"""

from __future__ import annotations

from dataclasses import dataclass


# MIT-frame scaling limits are per motor model (cmjang DM_CAN.py Limit_Param
# + motorbridge motor_vendors/damiao/src/motor.rs). Kp/Kd ranges are the
# same for every model.
@dataclass(frozen=True)
class Limits:
    p_max: float   # rad
    v_max: float   # rad/s
    t_max: float   # Nm


MODEL_LIMITS = {
    "J4340":  Limits(12.5, 10.0, 28.0),
    "J4340P": Limits(12.5, 10.0, 28.0),
    "J6006":  Limits(12.5, 45.0, 20.0),
    "J6248P": Limits(12.566, 20.0, 120.0),
}

DEFAULT_LIMITS = MODEL_LIMITS["J6006"]

# J6006 values kept as module constants for backwards compatibility
P_MAX  = DEFAULT_LIMITS.p_max
V_MAX  = DEFAULT_LIMITS.v_max
T_MAX  = DEFAULT_LIMITS.t_max
KP_MAX = 500.0
KD_MAX = 5.0


ENABLE_CMD   = bytes([0xFF] * 7 + [0xFC])
DISABLE_CMD  = bytes([0xFF] * 7 + [0xFD])
CLEAR_ERR_CMD = bytes([0xFF] * 7 + [0xFB])   # clears latched errors (CommLoss etc.)
SET_ZERO_CMD = bytes([0xFF] * 7 + [0xFE])

# Broadcast "refresh status" — send to CAN ID 0x7FF with this 8-byte payload
# to query a single motor by slave_id; motor answers via its MST_ID.
BROADCAST_ID = 0x7FF


def refresh_query(slave_id: int) -> bytes:
    return bytes([slave_id & 0xFF, (slave_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])


ERR_NAMES = {
    0x0: "OK/disabled",
    0x1: "enabled",
    0x8: "OverVoltage",
    0x9: "UnderVoltage",
    0xA: "OverCurrent",
    0xB: "MOS OverTemp",
    0xC: "Motor OverTemp",
    0xD: "CommLoss",
    0xE: "OverLoad",
}


def _f_to_u(x: float, lo: float, hi: float, bits: int) -> int:
    x = max(lo, min(hi, x))
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def _u_to_f(u: int, lo: float, hi: float, bits: int) -> float:
    return u * (hi - lo) / ((1 << bits) - 1) + lo


def pack_mit(pos: float, vel: float, kp: float, kd: float, tau: float,
             limits: Limits = DEFAULT_LIMITS) -> bytes:
    """Pack an MIT-mode control frame. `limits` must match the motor model."""
    q  = _f_to_u(pos, -limits.p_max, limits.p_max, 16)
    dq = _f_to_u(vel, -limits.v_max, limits.v_max, 12)
    kp_u  = _f_to_u(kp,  0.0, KP_MAX, 12)
    kd_u  = _f_to_u(kd,  0.0, KD_MAX, 12)
    tau_u = _f_to_u(tau, -limits.t_max, limits.t_max, 12)
    return bytes([
        (q >> 8) & 0xFF,                                           # D[0] p_des[15:8]
        q & 0xFF,                                                  # D[1] p_des[7:0]
        (dq >> 4) & 0xFF,                                          # D[2] v_des[11:4]
        ((dq & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),                 # D[3] v_des[3:0]|Kp[11:8]
        kp_u & 0xFF,                                               # D[4] Kp[7:0]
        (kd_u >> 4) & 0xFF,                                        # D[5] Kd[11:4]
        ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F),              # D[6] Kd[3:0]|t_ff[11:8]
        tau_u & 0xFF,                                              # D[7] t_ff[7:0]
    ])


@dataclass
class Feedback:
    motor_id: int
    err_code: int
    pos: float           # radians
    vel: float           # rad/s
    tau: float           # Nm
    t_mos: int           # °C
    t_rotor: int         # °C

    @property
    def err_name(self) -> str:
        return ERR_NAMES.get(self.err_code, f"code=0x{self.err_code:X}")

    @classmethod
    def parse(cls, data: bytes, limits: Limits = DEFAULT_LIMITS) -> "Feedback":
        if len(data) < 8:
            raise ValueError(f"feedback frame too short: {len(data)} bytes")
        motor_id = data[0] & 0x0F
        err      = (data[0] >> 4) & 0x0F
        pos_u = (data[1] << 8) | data[2]
        vel_u = (data[3] << 4) | ((data[4] >> 4) & 0x0F)
        tau_u = ((data[4] & 0x0F) << 8) | data[5]
        return cls(
            motor_id=motor_id,
            err_code=err,
            pos=_u_to_f(pos_u, -limits.p_max, limits.p_max, 16),
            vel=_u_to_f(vel_u, -limits.v_max, limits.v_max, 12),
            tau=_u_to_f(tau_u, -limits.t_max, limits.t_max, 12),
            t_mos=data[6],
            t_rotor=data[7],
        )
