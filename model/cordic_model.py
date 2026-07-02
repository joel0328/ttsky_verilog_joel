"""
Fixed-point CORDIC golden model — the bit-exact spec for the RTL.

Conventions (these ARE the design decisions; the Verilog must match):

  * Result width:   16-bit signed two's complement.
  * Internal width: 19-bit (16 + GUARD guard bits). Guard bits absorb
                    the truncation error of the arithmetic right-shifts;
                    without them worst-case error is ~8 LSB, with 3 it
                    is ~1.5 LSB (measured). RTL cost: 9 extra flops.
  * Iterations:     16 (N_ITER). ~1 bit of precision per iteration.
  * Angle unit:     binary angular measure ("brads"):
                      full circle = 2^16 counts
                      0x0000 =   0 deg      0x4000 =  90 deg
                      0x8000 = 180 deg      0xC000 = 270 deg
                    Quadrant = top two bits => quadrant fold is pure bit
                    inspection, no comparators.
  * Fixed point:    external x/y are Q1.14 (int / 2^14; +1.0 = 16384).
                    Headroom for CORDIC gain K ~= 1.6468 (max ~1.65 < 2).
                    Internally everything carries GUARD extra LSBs.
  * Gain:           rotation mode pre-seeds x0 = (2^14 / K) so cos/sin
                    come out unscaled. Vectoring-mode magnitude is
                    returned K-scaled (documented; cheap to correct
                    downstream if needed).
  * Rounding:       guard bits are removed with round-half-up at output
                    (add 1<<(GUARD-1), then shift). One adder in RTL.
  * All datapath arithmetic is integer shifts/adds — exactly what the
    silicon will do. No floats anywhere below the API surface.

Vectoring input limits (document in the datasheet):
  * |v| <= ~19000 counts (K-scaled magnitude must fit 16 bits signed).
  * Angle accuracy degrades for tiny vectors — a +/-0.5-count input
    quantization on a 200-count vector is inherently ~50 brads. Keep
    |v| >= ~1000 counts for ~10-brad angle accuracy. This is input
    quantization physics, not CORDIC error.
"""

import math

# ----------------------------------------------------------------------
# Parameters (single source of truth — RTL constants derive from these)
# ----------------------------------------------------------------------

WIDTH = 16                    # external result width, bits (signed)
GUARD = 3                     # internal guard bits (see header)
IWIDTH = WIDTH + GUARD        # internal datapath width = 19
N_ITER = 16                   # CORDIC iterations
FRAC = 14                     # Q1.14 external fixed point
ONE = 1 << FRAC               # fixed-point 1.0 = 16384

ANGLE_BITS = 16                          # full circle = 2**16 brads
HALF_TURN = 1 << (ANGLE_BITS - 1)        # 180 deg = 32768

# CORDIC gain K = prod(sqrt(1 + 2^-2i)), i = 0..N-1
K = math.prod(math.sqrt(1.0 + 2.0 ** (-2 * i)) for i in range(N_ITER))

# Rotation-mode x seed: (1/K) in internal (guard-extended) units
INV_K_SEED = round((ONE << GUARD) / K)

# atan(2^-i) LUT in guard-extended brads. These constants go verbatim
# into the RTL case statement (19-bit values).
ATAN_TABLE = [
    round(math.atan(2.0 ** -i) / (2 * math.pi) * (1 << (ANGLE_BITS + GUARD)))
    for i in range(N_ITER)
]

_ROUND = 1 << (GUARD - 1)     # rounding constant for guard-bit removal


def _wrap(v: int, width: int = IWIDTH) -> int:
    """Wrap into signed two's-complement range of `width` bits, exactly
    as a register of that width would."""
    v &= (1 << width) - 1
    if v >= (1 << (width - 1)):
        v -= 1 << width
    return v


def _round_out(v: int) -> int:
    """Remove guard bits with round-half-up, wrap to external width."""
    return _wrap((v + _ROUND) >> GUARD, WIDTH)


# ----------------------------------------------------------------------
# The shared microrotation core (this loop IS the folded datapath)
# ----------------------------------------------------------------------

def _iterate(x: int, y: int, z: int, mode: str):
    """Run N_ITER CORDIC microrotations on guard-extended values.

    mode == 'rot': drive z toward 0   (d = sign of z)
    mode == 'vec': drive y toward 0   (d = -sign of y)

    Every update is one arithmetic shift and one add/sub per variable,
    on IWIDTH-bit two's complement — nothing else. Python's >> on
    negative ints is arithmetic, matching Verilog's >>> on signed.
    """
    for i in range(N_ITER):
        if mode == 'rot':
            d = 1 if z >= 0 else -1
        else:  # 'vec'
            d = 1 if y < 0 else -1

        x, y, z = (
            _wrap(x - d * (y >> i)),
            _wrap(y + d * (x >> i)),
            _wrap(z - d * ATAN_TABLE[i]),
        )

    return x, y, z


# ----------------------------------------------------------------------
# Rotation mode: angle in -> (cos, sin) out
# ----------------------------------------------------------------------

def cordic_rotate(angle_brads: int):
    """(cos, sin) of an angle in brads. Only low 16 bits of the input
    matter — angles wrap like real angles.

    Returns (cos_fixed, sin_fixed) as Q1.14 signed 16-bit integers.

    Quadrant fold: the core converges only over ~+/-99.9 deg, so angles
    in the left half-plane (90..270 deg) are rotated by 180 first and
    both outputs negated after. Left half-plane test is bit15 ^ bit14 —
    the exact bit test the RTL will use.
    """
    a = angle_brads & 0xFFFF

    fold = ((a >> 15) ^ (a >> 14)) & 1
    if fold:
        a = (a + HALF_TURN) & 0xFFFF   # rotate 180 into right half-plane

    # Signed 16-bit angle (now within +/-90 deg), guard-extended
    z0 = _wrap(a, ANGLE_BITS) << GUARD

    x, y, _z = _iterate(INV_K_SEED, 0, z0, mode='rot')

    if fold:
        x, y = _wrap(-x), _wrap(-y)

    return _round_out(x), _round_out(y)   # (cos, sin) in Q1.14


# ----------------------------------------------------------------------
# Vectoring mode: (x, y) in -> (K*magnitude, atan2) out
# ----------------------------------------------------------------------

def cordic_vector(x_in: int, y_in: int):
    """Magnitude and angle of vector (x_in, y_in), Q1.14 signed 16-bit.

    Returns (mag_fixed, angle_brads):
      mag_fixed   = K * sqrt(x^2 + y^2) in Q1.14 (K-scaled by design)
      angle_brads = atan2(y, x), signed 16-bit brads

    Convergence fold: core requires x >= 0. If x < 0, pre-rotate 180
    (negate both components), then add/subtract a half turn to the
    resulting angle so it lands in (-180, 180].
    """
    x = _wrap(x_in, WIDTH) << GUARD
    y = _wrap(y_in, WIDTH) << GUARD

    fold = x < 0
    if fold:
        x, y = _wrap(-x), _wrap(-y)

    xr, _yr, z = _iterate(x, y, 0, mode='vec')

    if fold:
        z = _wrap(z - (HALF_TURN << GUARD)) if z >= 0 \
            else _wrap(z + (HALF_TURN << GUARD))

    return _round_out(xr), _round_out(z)


# ----------------------------------------------------------------------
# Float helpers for tests / humans (NOT part of the hardware spec)
# ----------------------------------------------------------------------

def fixed_to_float(v: int) -> float:
    """Q1.14 -> float."""
    return v / ONE


def brads_to_rad(b: int) -> float:
    """Signed brads -> radians."""
    return b * (2 * math.pi) / (1 << ANGLE_BITS)


def rad_to_brads(r: float) -> int:
    """Radians -> brads (wrapped, unsigned 16-bit representation)."""
    return round(r / (2 * math.pi) * (1 << ANGLE_BITS)) & 0xFFFF


if __name__ == "__main__":
    print(f"K = {K:.6f}   INV_K_SEED = {INV_K_SEED} (Q1.14 << {GUARD})")
    print(f"ATAN_TABLE ({IWIDTH}-bit) = {ATAN_TABLE}")
    for deg in (0, 30, 45, 60, 90, 135, 180, 270, 315):
        a = round(deg / 360 * 65536) & 0xFFFF
        c, s = cordic_rotate(a)
        print(f"{deg:5.1f} deg: cos={fixed_to_float(c):+.5f} "
              f"(true {math.cos(math.radians(deg)):+.5f})  "
              f"sin={fixed_to_float(s):+.5f} "
              f"(true {math.sin(math.radians(deg)):+.5f})")
