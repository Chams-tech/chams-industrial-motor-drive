"""Space Vector Pulse Width Modulation (SVPWM).

Converts a voltage vector (v_alpha, v_beta) into three duty cycles in [0, 1]
for a two-level three-phase inverter.

Linear modulation range:
    |v_alpha-beta| <= V_dc / sqrt(3)

Duty cycles satisfy d_a + d_b + d_c = 1 (the SVPWM zero sequence).
"""

from __future__ import annotations

import math

SQRT3 = math.sqrt(3.0)


def limit_voltage_vector(valpha: float, vbeta: float,
                         dc_bus_voltage: float) -> tuple[float, float]:
    """Clamp the voltage vector to the linear SVPWM range V_dc/sqrt(3)."""
    limit = dc_bus_voltage / SQRT3
    magnitude = math.hypot(valpha, vbeta)
    if magnitude > limit and magnitude > 0.0:
        scale = limit / magnitude
        return valpha * scale, vbeta * scale
    return valpha, vbeta


def calculate_sector(valpha: float, vbeta: float) -> int:
    """Return the SVPWM sector index in [1, 6] (angle-based, 30 deg offset).

    Sector 1: -30 deg..30 deg (centered on the alpha axis), etc.
    Returns 1..6; a zero vector is assigned to sector 1.
    """
    angle = math.atan2(vbeta, valpha)          # [-pi, pi]
    sector = int(math.floor((angle + math.pi / 6.0) / (math.pi / 3.0))) + 1
    if sector == 7:
        sector = 1
    if sector < 1:
        sector += 6
    return sector


def calculate_svpwm_duties(valpha: float, vbeta: float,
                           dc_bus_voltage: float) -> tuple[float, float, float]:
    """Calculate (d_a, d_b, d_c) duty cycles in [0, 1].

    Sector-based SVPWM with min-max injection (equivalent to the
    space-vector zero-sequence distribution).
    """
    valpha, vbeta = limit_voltage_vector(valpha, vbeta, dc_bus_voltage)

    # Phase voltages from inverse Clarke (reference potentials)
    va = valpha
    vb = -0.5 * valpha + (SQRT3 / 2.0) * vbeta
    vc = -0.5 * valpha - (SQRT3 / 2.0) * vbeta

    # Duty = 1/2 + v_phase / V_dc  (centered PWM), then inject the
    # SVPWM zero sequence so that min(d) = (1 - |max-min|/V_dc...)/2 style
    da = 0.5 + va / dc_bus_voltage
    db = 0.5 + vb / dc_bus_voltage
    dc = 0.5 + vc / dc_bus_voltage

    # SVPWM zero-sequence injection: shift so the span is centered in [0, 1]
    d_min = min(da, db, dc)
    d_max = max(da, db, dc)
    offset = 0.5 - 0.5 * (d_min + d_max)
    da += offset
    db += offset
    dc += offset

    # Numerical guard
    da = min(max(da, 0.0), 1.0)
    db = min(max(db, 0.0), 1.0)
    dc = min(max(dc, 0.0), 1.0)
    return da, db, dc
