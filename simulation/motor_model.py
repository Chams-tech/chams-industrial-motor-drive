"""SPMSM mathematical model (Phase 1 equations) with RK4 integration.

State (peak-amplitude dq convention):
    id    : d-axis current [A]
    iq    : q-axis current [A]
    speed : mechanical speed omega_m [rad/s]
    theta : electrical rotor angle [rad]

Equations:
    di_d/dt = (v_d - R_s*i_d + w_e*L_q*i_q) / L_d
    di_q/dt = (v_q - R_s*i_q - w_e*(L_d*i_d + lambda_pm)) / L_q
    T_e     = 1.5 * p * lambda_pm * iq            (SPMSM: L_d = L_q)
    J dw/dt = T_e - T_L - B*w - T_c*sign(w)
    w_e     = p * w
    dtheta/dt = w_e
"""

from __future__ import annotations

import math


def calculate_electrical_speed(mechanical_speed: float, pole_pairs: int) -> float:
    """Convert mechanical speed [rad/s] to electrical speed [rad/s]."""
    return pole_pairs * mechanical_speed


def calculate_electromagnetic_torque(id_current: float, iq_current: float,
                                     parameters: dict) -> float:
    """Electromagnetic torque [N.m].

    SPMSM (L_d = L_q): reluctance term is zero, torque depends on i_q only.
    The i_d argument is kept for interface compatibility with an IPMSM model.
    """
    return 1.5 * parameters["p"] * parameters["lambda_pm"] * iq_current


def calculate_current_derivatives(vd: float, vq: float,
                                  id_current: float, iq_current: float,
                                  electrical_speed: float,
                                  parameters: dict) -> tuple[float, float]:
    """Return (di_d/dt, di_q/dt) [A/s]."""
    r = parameters["R_s"]
    ld = parameters["L_d"]
    lq = parameters["L_q"]
    lam = parameters["lambda_pm"]
    we = electrical_speed

    did = (vd - r * id_current + we * lq * iq_current) / ld
    diq = (vq - r * iq_current - we * (ld * id_current + lam)) / lq
    return did, diq


def calculate_mechanical_acceleration(torque: float, load_torque: float,
                                      mechanical_speed: float,
                                      parameters: dict) -> float:
    """Return d(omega_m)/dt [rad/s^2]."""
    j = parameters["J"]
    b = parameters["B"]
    tc = parameters["T_c"]
    return (torque - load_torque - b * mechanical_speed
            - tc * math.copysign(1.0, mechanical_speed)) / j


def _derivatives(state: tuple[float, float, float, float],
                 vd: float, vq: float, load_torque: float,
                 parameters: dict) -> tuple[float, float, float, float]:
    id_c, iq_c, speed, _theta = state
    we = calculate_electrical_speed(speed, parameters["p"])
    did, diq = calculate_current_derivatives(vd, vq, id_c, iq_c, we, parameters)
    torque = calculate_electromagnetic_torque(id_c, iq_c, parameters)
    acc = calculate_mechanical_acceleration(torque, load_torque, speed, parameters)
    dtheta = we
    return did, diq, acc, dtheta


def update_motor_state(state: dict, vd: float, vq: float,
                       load_torque: float, dt: float,
                       parameters: dict) -> dict:
    """Advance the motor state by one time step using classical RK4.

    ``state`` keys: id, iq, speed, theta. Returns a new state dict.
    """
    s0 = (state["id"], state["iq"], state["speed"], state["theta"])

    def pack(deriv, scale):
        return tuple(s0[i] + scale * deriv[i] for i in range(4))

    k1 = _derivatives(s0, vd, vq, load_torque, parameters)
    k2 = _derivatives(pack(k1, dt / 2.0), vd, vq, load_torque, parameters)
    k3 = _derivatives(pack(k2, dt / 2.0), vd, vq, load_torque, parameters)
    k4 = _derivatives(pack(k3, dt), vd, vq, load_torque, parameters)

    new = tuple(
        s0[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
        for i in range(4)
    )
    return {"id": new[0], "iq": new[1], "speed": new[2], "theta": new[3]}
