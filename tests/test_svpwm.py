"""Unit tests for SVPWM."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.svpwm import (calculate_sector, calculate_svpwm_duties,
                              limit_voltage_vector)

VDC = 48.0


def test_duties_within_bounds():
    limit = VDC / math.sqrt(3.0)
    for mag in (0.0, 5.0, limit * 0.5, limit):
        for angle in [i * math.pi / 12.0 for i in range(24)]:
            va = mag * math.cos(angle)
            vb = mag * math.sin(angle)
            da, db, dc = calculate_svpwm_duties(va, vb, VDC)
            assert 0.0 <= da <= 1.0
            assert 0.0 <= db <= 1.0
            assert 0.0 <= dc <= 1.0


def test_zero_vector_gives_centered_duties():
    """Zero voltage vector -> all legs at 50% (average phase voltages = 0)."""
    da, db, dc = calculate_svpwm_duties(0.0, 0.0, VDC)
    assert abs(da - 0.5) < 1e-9
    assert abs(db - 0.5) < 1e-9
    assert abs(dc - 0.5) < 1e-9
    mean_d = (da + db + dc) / 3.0
    for d in (da, db, dc):
        assert abs(VDC * (d - mean_d)) < 1e-9


def test_limit_voltage_vector():
    limit = VDC / math.sqrt(3.0)
    # Inside the limit: unchanged
    va, vb = limit_voltage_vector(10.0, 0.0, VDC)
    assert (va, vb) == (10.0, 0.0)
    # Outside: magnitude clamped to V_dc/sqrt(3)
    va, vb = limit_voltage_vector(1000.0, 500.0, VDC)
    assert abs(math.hypot(va, vb) - limit) < 1e-9


def test_reconstructed_phase_voltages_match_command():
    """In the linear range the average inverter output must reproduce the
    commanded voltage vector (after zero-sequence removal)."""
    limit = VDC / math.sqrt(3.0)
    for angle in [i * math.pi / 9.0 for i in range(18)]:
        mag = 0.7 * limit
        va_cmd = mag * math.cos(angle)
        vb_cmd = mag * math.sin(angle)
        da, db, dc = calculate_svpwm_duties(va_cmd, vb_cmd, VDC)
        mean_d = (da + db + dc) / 3.0
        va_out = VDC * (da - mean_d)
        vb_out = VDC * (db - mean_d)
        vc_out = VDC * (dc - mean_d)
        # Clarke of the output
        alpha = va_out
        beta = (va_out + 2.0 * vb_out) / math.sqrt(3.0)
        assert abs(alpha - va_cmd) < 1e-9
        assert abs(beta - vb_cmd) < 1e-9
        assert abs(va_out + vb_out + vc_out) < 1e-9


def test_all_six_sectors_covered():
    sectors = set()
    limit = VDC / math.sqrt(3.0)
    for i in range(72):
        angle = i * 2.0 * math.pi / 72.0
        sectors.add(calculate_sector(limit * math.cos(angle),
                                     limit * math.sin(angle)))
    assert sectors == {1, 2, 3, 4, 5, 6}


def test_duties_are_centered():
    """SVPWM centering property: min(d) + max(d) = 1 (linear range)."""
    limit = VDC / math.sqrt(3.0)
    for i in range(24):
        angle = i * 2.0 * math.pi / 24.0
        da, db, dc = calculate_svpwm_duties(0.9 * limit * math.cos(angle),
                                            0.9 * limit * math.sin(angle), VDC)
        assert abs(min(da, db, dc) + max(da, db, dc) - 1.0) < 1e-9
