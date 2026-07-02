"""
Validation of the CORDIC golden model against NumPy/math ground truth.

Run:  pytest model/ -v -s
(-s so the error-statistics printouts are visible)

Error budgets (calibrated against the 3-guard-bit architecture; see the
guard-bit sweep note in cordic_model.py header):
  * rotation cos/sin:       <= 2 LSB     (measured max ~1.5)
  * vectoring magnitude:    <= 4 LSB     (K-scaled)
  * vectoring angle:        <= 12 brads at |v| >= 1000 counts
    (angle error at small |v| is input quantization, not CORDIC error —
     see model header; tests respect the documented input range)
"""

import math
import numpy as np
import pytest

from cordic_model import (
    cordic_rotate, cordic_vector,
    fixed_to_float, brads_to_rad, rad_to_brads,
    ONE, K,
)

RNG = np.random.default_rng(seed=328)  # reproducible

ROT_TOL_LSB = 2         # cos/sin tolerance, Q1.14 LSBs
VEC_MAG_TOL_LSB = 4     # magnitude tolerance, Q1.14 LSBs (K-scaled)
VEC_ANG_TOL_BRAD = 12   # atan2 tolerance, brads (1 brad = 0.0055 deg)

VEC_MIN_MAG = 1000      # documented input floor for angle accuracy
VEC_MAX_MAG = 19000     # documented input ceiling (K-scaling headroom)


def _stats(name, errs):
    errs = np.asarray(errs, dtype=float)
    print(f"\n[{name}] n={len(errs)}  max|err|={np.max(np.abs(errs)):.2f}  "
          f"mean={np.mean(errs):+.3f}  rms={np.sqrt(np.mean(errs**2)):.3f}")


def _wrapped_brad_delta(a, b):
    """Signed shortest distance between two 16-bit angles, in brads."""
    return ((a - b + 32768) & 0xFFFF) - 32768


# ----------------------------------------------------------------------
# Rotation mode
# ----------------------------------------------------------------------

def test_rotation_cardinals():
    """Cardinal angles should be near-exact."""
    cases = {
        0x0000: (+1.0, 0.0),
        0x4000: (0.0, +1.0),
        0x8000: (-1.0, 0.0),
        0xC000: (0.0, -1.0),
    }
    for a, (c_true, s_true) in cases.items():
        c, s = cordic_rotate(a)
        assert abs(c - c_true * ONE) <= ROT_TOL_LSB, hex(a)
        assert abs(s - s_true * ONE) <= ROT_TOL_LSB, hex(a)


def test_rotation_sweep():
    """Dense sweep: every 16 brads (4096 points) around the circle."""
    errs_c, errs_s = [], []
    for step in range(4096):
        a = step * 16
        c, s = cordic_rotate(a)
        rad = a / 65536 * 2 * math.pi
        errs_c.append(c - math.cos(rad) * ONE)
        errs_s.append(s - math.sin(rad) * ONE)
    _stats("rotation sweep cos", errs_c)
    _stats("rotation sweep sin", errs_s)
    assert np.max(np.abs(errs_c)) <= ROT_TOL_LSB
    assert np.max(np.abs(errs_s)) <= ROT_TOL_LSB


def test_rotation_random():
    """10k random angles across the full 16-bit range."""
    angles = RNG.integers(0, 65536, size=10_000)
    errs_c, errs_s = [], []
    for a in angles:
        c, s = cordic_rotate(int(a))
        rad = int(a) / 65536 * 2 * math.pi
        errs_c.append(c - math.cos(rad) * ONE)
        errs_s.append(s - math.sin(rad) * ONE)
    _stats("rotation random cos", errs_c)
    _stats("rotation random sin", errs_s)
    assert np.max(np.abs(errs_c)) <= ROT_TOL_LSB
    assert np.max(np.abs(errs_s)) <= ROT_TOL_LSB


def test_rotation_pythagorean():
    """cos^2 + sin^2 == 1 (catches gain-seed errors independent of
    angle accuracy)."""
    for a in RNG.integers(0, 65536, size=1_000):
        c, s = cordic_rotate(int(a))
        mag = math.hypot(fixed_to_float(c), fixed_to_float(s))
        assert abs(mag - 1.0) < 4 / ONE, f"angle {int(a):#06x}: |v|={mag}"


# ----------------------------------------------------------------------
# Vectoring mode
# ----------------------------------------------------------------------

def _random_vectors(n, min_mag=VEC_MIN_MAG, max_mag=VEC_MAX_MAG):
    mags = RNG.uniform(min_mag, max_mag, size=n)
    angs = RNG.uniform(-math.pi, math.pi, size=n)
    xs = np.round(mags * np.cos(angs)).astype(int)
    ys = np.round(mags * np.sin(angs)).astype(int)
    return xs, ys


def test_vectoring_axes():
    """Axis-aligned vectors: exact angles expected."""
    cases = [
        ((10000, 0), 0x0000),
        ((0, 10000), 0x4000),
        ((-10000, 0), 0x8000),
        ((0, -10000), 0xC000),
    ]
    for (x, y), a_true in cases:
        mag, ang = cordic_vector(x, y)
        assert abs(_wrapped_brad_delta(ang, a_true)) <= VEC_ANG_TOL_BRAD, \
            f"({x},{y})"
        assert abs(mag - K * 10000) <= VEC_MAG_TOL_LSB, f"({x},{y})"


def test_vectoring_random():
    """10k random vectors within documented input range."""
    xs, ys = _random_vectors(10_000)
    errs_a, errs_m = [], []
    for x, y in zip(xs, ys):
        mag, ang = cordic_vector(int(x), int(y))
        a_true = rad_to_brads(math.atan2(y, x))
        errs_a.append(_wrapped_brad_delta(ang, a_true))
        errs_m.append(mag - K * math.hypot(x, y))
    _stats("vectoring angle (brads)", errs_a)
    _stats("vectoring magnitude (LSB, K-scaled)", errs_m)
    assert np.max(np.abs(errs_a)) <= VEC_ANG_TOL_BRAD
    assert np.max(np.abs(errs_m)) <= VEC_MAG_TOL_LSB


def test_vectoring_small_vectors_documented_behavior():
    """Below the documented floor, angle error grows ~ 1/|v|. Verify it
    stays within the physics bound (input quantization), so we know the
    degradation is graceful, not broken."""
    for mag in (200, 400, 700):
        worst = 0
        for _ in range(500):
            ang = RNG.uniform(-math.pi, math.pi)
            x = round(mag * math.cos(ang))
            y = round(mag * math.sin(ang))
            _m, a = cordic_vector(x, y)
            a_true = rad_to_brads(math.atan2(y, x))
            worst = max(worst, abs(_wrapped_brad_delta(a, a_true)))
        # bound: input quantization (~0.5/mag rad) + CORDIC floor, x2 margin
        bound = 2 * (0.5 / mag / (2 * math.pi) * 65536 + VEC_ANG_TOL_BRAD)
        print(f"\n[small vectors |v|={mag}] worst angle err = {worst} brads "
              f"(bound {bound:.0f})")
        assert worst <= bound


def test_roundtrip():
    """rotate(angle) -> vector(cos, sin) recovers the angle: end-to-end
    consistency of both modes through the same datapath conventions."""
    for a in RNG.integers(0, 65536, size=2_000):
        c, s = cordic_rotate(int(a))
        _mag, ang = cordic_vector(c, s)
        assert abs(_wrapped_brad_delta(ang, int(a))) <= VEC_ANG_TOL_BRAD, \
            f"angle {int(a):#06x}"
