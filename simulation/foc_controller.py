"""Integrated FOC controller.

Sequence per control tick (20 kHz):
    1. read three-phase currents
    2. Clarke transform        abc -> alpha-beta
    3. Park transform          alpha-beta -> dq (rotor angle)
    4. speed loop (decimated)  speed error -> i_q* (i_d* = 0)
    5. current errors + d/q PI
    6. decoupling compensation
    7. inverse Park            dq -> alpha-beta
    8. SVPWM                   alpha-beta -> duty cycles

The speed loop runs at 1 kHz (every ``speed_divider`` ticks); the current
loop runs every tick.
"""

from __future__ import annotations

from .current_controller import CurrentController
from .speed_controller import SpeedController
from .svpwm import calculate_svpwm_duties, limit_voltage_vector
from .transformations import (clarke_transform, inverse_park_transform,
                              park_transform)


class FOCController:
    def __init__(self, motor_parameters: dict, control_parameters: dict):
        self.motor = motor_parameters
        self.speed_divider = int(control_parameters.get("speed_divider", 20))
        gains = {"kp": control_parameters.get("current_kp", 0.6),
                 "ki": control_parameters.get("current_ki", 150.0)}
        self.current_controller = CurrentController(motor_parameters, gains)
        self.speed_controller = SpeedController(
            kp=control_parameters.get("speed_kp", 0.027),
            ki=control_parameters.get("speed_ki", 1.55),
            current_limit=motor_parameters["I_peak_max"],
        )
        self._tick = 0
        self._id_ref = 0.0
        self._iq_ref = 0.0
        # Last outputs (for logging/introspection)
        self.v_d = 0.0
        self.v_q = 0.0
        self.duty = (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)
        self.id_meas = 0.0
        self.iq_meas = 0.0

    def reset(self) -> None:
        self.current_controller.reset()
        self.speed_controller.reset()
        self._tick = 0
        self._id_ref = 0.0
        self._iq_ref = 0.0
        self.v_d = 0.0
        self.v_q = 0.0
        self.duty = (1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0)

    def update(self, phase_currents: tuple[float, float, float],
               electrical_angle: float,
               speed_reference: float, measured_speed: float,
               dt: float) -> tuple[float, float, float]:
        """Run one FOC tick. Returns inverter duty cycles (d_a, d_b, d_c)."""
        # --- Outer speed loop (decimated) ---------------------------- #
        if self._tick % self.speed_divider == 0:
            self._iq_ref = self.speed_controller.update(
                speed_reference, measured_speed, dt * self.speed_divider)
        self._tick += 1
        self._id_ref = self.speed_controller.id_reference

        # --- Current loop -------------------------------------------- #
        ia, ib, ic = phase_currents
        ialpha, ibeta = clarke_transform(ia, ib, ic)
        self.id_meas, self.iq_meas = park_transform(
            ialpha, ibeta, electrical_angle)

        electrical_speed = self.motor["p"] * measured_speed
        vd, vq = self.current_controller.update(
            self._id_ref, self._iq_ref,
            self.id_meas, self.iq_meas,
            electrical_speed, dt)
        self.v_d, self.v_q = vd, vq

        # --- Modulation ---------------------------------------------- #
        valpha, vbeta = inverse_park_transform(vd, vq, electrical_angle)
        self.duty = calculate_svpwm_duties(valpha, vbeta,
                                           self.motor["V_dc"])
        return self.duty
