"""FOC inner current controllers (d and q axes) with decoupling.

    e_d = id* - id ,  e_q = iq* - iq
    v_d = PI_d(e_d) - w_e * L_q * i_q
    v_q = PI_q(e_q) + w_e * (L_d * i_d + lambda_pm)

The signs match the motor model in motor_model.py.
Voltage commands are limited to the linear SVPWM range V_dc/sqrt(3).
"""

from __future__ import annotations

import math

from .pi_controller import PIController


class CurrentController:
    def __init__(self, parameters: dict, gains: dict | None = None):
        gains = gains or {}
        kp = gains.get("kp", 0.6)
        ki = gains.get("ki", 150.0)
        v_limit = parameters["V_dc"] / math.sqrt(3.0)
        self.parameters = parameters
        self.v_limit = v_limit
        self.pi_d = PIController(kp, ki, -v_limit, v_limit)
        self.pi_q = PIController(kp, ki, -v_limit, v_limit)

    def update(self, id_reference: float, iq_reference: float,
               id_measurement: float, iq_measurement: float,
               electrical_speed: float, dt: float) -> tuple[float, float]:
        """Return the voltage commands (v_d, v_q) [V]."""
        p = self.parameters
        vd_pi = self.pi_d.update(id_reference, id_measurement, dt)
        vq_pi = self.pi_q.update(iq_reference, iq_measurement, dt)

        # Decoupling compensation (signs per the SPMSM model)
        vd = vd_pi - electrical_speed * p["L_q"] * iq_measurement
        vq = vq_pi + electrical_speed * (p["L_d"] * id_measurement
                                         + p["lambda_pm"])

        # Limit the voltage vector to the linear modulation range
        magnitude = math.hypot(vd, vq)
        if magnitude > self.v_limit and magnitude > 0.0:
            scale = self.v_limit / magnitude
            vd *= scale
            vq *= scale
        return vd, vq

    def reset(self) -> None:
        self.pi_d.reset()
        self.pi_q.reset()
