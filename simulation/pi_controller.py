"""Discrete PI controller with output saturation and anti-windup.

    e(k)   = reference - measurement
    u(k)   = Kp * e(k) + Ki * integral
    u_sat  = clamp(u, output_min, output_max)

Anti-windup: conditional integration — the integral term only accumulates
when the unsaturated output is within limits (or when the error drives the
output back into the linear region).
"""

from __future__ import annotations


class PIController:
    def __init__(self, kp: float, ki: float,
                 output_min: float, output_max: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.output_min = float(output_min)
        self.output_max = float(output_max)
        self.integral = 0.0
        self.output = 0.0

    def update(self, reference: float, measurement: float, dt: float) -> float:
        error = reference - measurement

        # Candidate unsaturated output
        tentative = self.kp * error + self.ki * self.integral

        # Anti-windup: only integrate if not saturating away from the error
        if not (tentative >= self.output_max and error > 0.0) and not (
                tentative <= self.output_min and error < 0.0):
            self.integral += error * dt

        self.output = self.kp * error + self.ki * self.integral
        self.output = min(max(self.output, self.output_min), self.output_max)
        return self.output

    def reset(self) -> None:
        self.integral = 0.0
        self.output = 0.0
