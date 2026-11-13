"""FOC outer speed controller.

    e_w   = speed_reference - measured_speed
    iq*   = PI_w(e_w), limited to +-current_limit
    id*   = 0

The output limit acts as the current limit; anti-windup is inherited from
the PI controller (conditional integration).
"""

from __future__ import annotations

from .pi_controller import PIController


class SpeedController:
    def __init__(self, kp: float, ki: float, current_limit: float):
        self.current_limit = float(current_limit)
        self.pi = PIController(kp, ki, -self.current_limit, self.current_limit)

    def update(self, speed_reference: float, measured_speed: float,
               dt: float) -> float:
        """Return the q-axis current reference [A peak]."""
        return self.pi.update(speed_reference, measured_speed, dt)

    @property
    def id_reference(self) -> float:
        """d-axis current reference: i_d* = 0 (SPMSM, no field weakening)."""
        return 0.0

    def reset(self) -> None:
        self.pi.reset()
