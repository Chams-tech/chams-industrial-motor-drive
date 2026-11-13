"""Unit tests for the PI controller."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.pi_controller import PIController


def test_step_response_converges():
    """PI on a first-order plant drives the error toward zero."""
    kp, ki = 2.0, 50.0
    pi = PIController(kp, ki, -10.0, 10.0)
    dt = 0.001
    x = 0.0          # plant: dx/dt = u
    ref = 5.0
    for _ in range(4000):  # 4 s
        u = pi.update(ref, x, dt)
        x += u * dt
    assert abs(ref - x) < 0.05


def test_output_saturation():
    pi = PIController(1.0, 10.0, -2.0, 2.0)
    out = pi.update(100.0, 0.0, 0.001)
    assert out == 2.0
    out = pi.update(-100.0, 0.0, 0.001)
    assert out == -2.0


def test_anti_windup_limits_integral():
    """While saturated in one direction the integral must not run away."""
    pi = PIController(kp=0.0, ki=100.0, output_min=-1.0, output_max=1.0)
    for _ in range(10_000):
        pi.update(10.0, 0.0, 0.001)
    # Output pinned at max; integral kept small by anti-windup
    assert pi.output == 1.0
    assert pi.integral * pi.ki <= 1.0 + 1e-9


def test_reset():
    pi = PIController(1.0, 1.0, -5.0, 5.0)
    pi.update(1.0, 0.0, 0.1)
    pi.update(1.0, 0.0, 0.1)
    assert pi.integral != 0.0
    pi.reset()
    assert pi.integral == 0.0
    assert pi.output == 0.0
