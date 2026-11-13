"""Unit tests for Clarke/Park transformations."""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from simulation.transformations import (clarke_transform,
                                        inverse_clarke_transform,
                                        inverse_park_transform,
                                        park_transform)

TOL = 1e-9


def _balanced(amplitude, theta):
    """Balanced three-phase set from a peak amplitude and phase angle."""
    ia = amplitude * math.cos(theta)
    ib = amplitude * math.cos(theta - 2.0 * math.pi / 3.0)
    ic = amplitude * math.cos(theta + 2.0 * math.pi / 3.0)
    return ia, ib, ic


def test_clarke_of_balanced_set():
    ia, ib, ic = _balanced(5.0, 0.0)
    ialpha, ibeta = clarke_transform(ia, ib, ic)
    assert abs(ialpha - 5.0) < TOL
    assert abs(ibeta - 0.0) < TOL


def test_abc_ab_abc_roundtrip():
    for angle in (0.0, 0.7, 2.5, -1.9):
        ia, ib, ic = _balanced(7.3, angle)
        ialpha, ibeta = clarke_transform(ia, ib, ic)
        ra, rb, rc = inverse_clarke_transform(ialpha, ibeta)
        assert abs(ra - ia) < 1e-9
        assert abs(rb - ib) < 1e-9
        assert abs(rc - ic) < 1e-9


def test_ab_dq_ab_roundtrip():
    for theta in (0.0, 1.0, 3.7, -2.2):
        va, vb = 12.0, -4.5
        vd, vq = park_transform(va, vb, theta)
        rva, rvb = inverse_park_transform(vd, vq, theta)
        assert abs(rva - va) < 1e-9
        assert abs(rvb - vb) < 1e-9


def test_park_alignment():
    """A vector aligned with theta maps entirely onto the d axis."""
    theta = 1.234
    mag = 9.0
    va = mag * math.cos(theta)
    vb = mag * math.sin(theta)
    vd, vq = park_transform(va, vb, theta)
    assert abs(vd - mag) < TOL
    assert abs(vq) < TOL


def test_full_chain_roundtrip():
    for angle in (0.3, 1.9, -2.8):
        ia, ib, ic = _balanced(3.21, angle)
        ialpha, ibeta = clarke_transform(ia, ib, ic)
        # Park at the same angle -> pure d current (in phase with phase-a)
        idc, iqc = park_transform(ialpha, ibeta, angle)
        # Inverse chain
        va, vb = inverse_park_transform(idc, iqc, angle)
        ra, rb, rc = inverse_clarke_transform(va, vb)
        assert abs(ra - ia) < 1e-9
        assert abs(rb - ib) < 1e-9
        assert abs(rc - ic) < 1e-9
