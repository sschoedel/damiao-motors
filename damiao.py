"""Damiao motor protocol helpers on top of dmcan.py.

Frame formats per J6006-2EC User Manual V1.0:
- Enable/disable/set-zero: 7x 0xFF + terminator byte (0xFC/0xFD/0xFE)
- MIT control frame: 8-byte packed [pos:16, vel:12, kp:12, kd:12, tau:12]
- Feedback frame: 8-byte packed [id|err:8, pos:16, vel:12, tau:12, t_mos:8, t_rotor:8]

Motor CAN bus is fixed at 1 Mbps standard 11-bit frames.
"""

from __future__ import annotations

from dataclasses import dataclass


# J6006-2EC parameter limits (datasheet page 8 + cmjang DM_CAN.py Limit_Param[4])
P_MAX  = 12.5    # rad
V_MAX  = 45.0    # rad/s
T_MAX  = 20.0    # Nm
KP_MAX = 500.0
KD_MAX = 5.0


ENABLE_CMD   = bytes([0xFF] * 7 + [0xFC])
DISABLE_CMD  = bytes([0xFF] * 7 + [0xFD])
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


def pack_mit(pos: float, vel: float, kp: float, kd: float, tau: float) -> bytes:
    """Pack an MIT-mode control frame."""
    q  = _f_to_u(pos, -P_MAX, P_MAX, 16)
    dq = _f_to_u(vel, -V_MAX, V_MAX, 12)
    kp_u  = _f_to_u(kp,  0.0, KP_MAX, 12)
    kd_u  = _f_to_u(kd,  0.0, KD_MAX, 12)
    tau_u = _f_to_u(tau, -T_MAX, T_MAX, 12)
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
    def parse(cls, data: bytes) -> "Feedback":
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
            pos=_u_to_f(pos_u, -P_MAX, P_MAX, 16),
            vel=_u_to_f(vel_u, -V_MAX, V_MAX, 12),
            tau=_u_to_f(tau_u, -T_MAX, T_MAX, 12),
            t_mos=data[6],
            t_rotor=data[7],
        )
