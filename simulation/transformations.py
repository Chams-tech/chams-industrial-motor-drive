"""Clarke and Park transformations for FOC (amplitude-invariant).

Conventions
-----------
- abc: balanced three-phase quantities (zero sequence neglected).
- alpha-beta: stationary frame, amplitude-invariant (i_alpha = i_a).
- dq: rotating frame aligned with the rotor electrical angle theta_e.
"""

from __future__ import annotations

import math


def clarke_transform(ia: float, ib: float, ic: float) -> tuple[float, float]:
    """abc -> alpha-beta.

    i_alpha = i_a
    i_beta  = (i_a + 2*i_b) / sqrt(3)     (valid for ia+ib+ic = 0)
    """
    ialpha = ia
    ibeta = (ia + 2.0 * ib) / math.sqrt(3.0)
    return ialpha, ibeta


def park_transform(ialpha: float, ibeta: float,
                   electrical_angle: float) -> tuple[float, float]:
    """alpha-beta -> dq."""
    c = math.cos(electrical_angle)
    s = math.sin(electrical_angle)
    id_current = ialpha * c + ibeta * s
    iq_current = -ialpha * s + ibeta * c
    return id_current, iq_current


def inverse_park_transform(vd: float, vq: float,
                           electrical_angle: float) -> tuple[float, float]:
    """dq -> alpha-beta."""
    c = math.cos(electrical_angle)
    s = math.sin(electrical_angle)
    valpha = vd * c - vq * s
    vbeta = vd * s + vq * c
    return valpha, vbeta


def inverse_clarke_transform(valpha: float, vbeta: float) -> tuple[float, float, float]:
    """alpha-beta -> abc (balanced, zero sequence = 0)."""
    va = valpha
    vb = -0.5 * valpha + (math.sqrt(3.0) / 2.0) * vbeta
    vc = -0.5 * valpha - (math.sqrt(3.0) / 2.0) * vbeta
    return va, vb, vc
