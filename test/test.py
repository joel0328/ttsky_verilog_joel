# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

"""
Committed verification for the CORDIC project: cordic_core driven directly
(bypassing the pin wrapper), and tt_um_joel0328 driven through the real
register-map pin protocol. Both are checked for EXACT bit-for-bit agreement
with model/cordic_model.py, the golden spec.

Earlier verification of cordic_core lived only in throwaway /tmp
testbenches; that gap is what this file closes.

Sampling discipline: every helper below reads DUT outputs only from the
cocotb ReadOnly phase of the cycle where the relevant done/valid flag was
just observed, then leaves the simulation at a clean post-RisingEdge point
before any further writes. This avoids a real trap in this design:
x_out/y_out/z_out sit two combinational levels below the state register
(state -> fold/unfold mux -> round_out) while `done` sits one level below
it, so sampling both in the same delta cycle (e.g. a bare `wait(done)` in
raw Verilog) can catch outputs a delta before they finish settling. ReadOnly
guarantees the full combinational settle for the current time step.
"""

import os
import random
import sys

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "model"))
from cordic_model import cordic_rotate, cordic_vector  # noqa: E402

MODE_ROT = 0
MODE_VEC = 1

# Register-map addresses (must match src/project.v)
ADDR_X_LO, ADDR_X_HI = 0x0, 0x1
ADDR_Y_LO, ADDR_Y_HI = 0x2, 0x3
ADDR_Z_LO, ADDR_Z_HI = 0x4, 0x5
ADDR_CTRL, ADDR_START = 0x6, 0x7
ADDR_XOUT_LO, ADDR_XOUT_HI = 0x8, 0x9
ADDR_YOUT_LO, ADDR_YOUT_HI = 0xA, 0xB
ADDR_ZOUT_LO, ADDR_ZOUT_HI = 0xC, 0xD
ADDR_STATUS = 0xE

WE_BIT = 1 << 4

# Gate-level netlists flatten cordic_core into tt_um_joel0328 during
# synthesis, so the standalone cordic_core_dut instance in tb.v (and its
# feeder regs/wires) is compiled out under `GL_TEST (see tb.v). Core-level
# tests that poke that instance directly can only run in RTL sim; the
# wrapper-level tests still exercise the same logic through the real pins
# and must run in both modes.
GATES = os.environ.get("GATES") == "yes"


def to_signed16(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


# ----------------------------------------------------------------------
# Shared setup
# ----------------------------------------------------------------------

async def start_clock(dut):
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())


async def reset(dut):
    dut.rst_n.value = 0
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    if not GATES:
        dut.core_start.value = 0
        dut.core_mode.value = 0
        dut.core_x_in.value = 0
        dut.core_y_in.value = 0
        dut.core_z_in.value = 0
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


# ----------------------------------------------------------------------
# Core-level helpers: drive cordic_core_dut (the standalone instance in
# tb.v) directly through its parallel interface.
# ----------------------------------------------------------------------

async def core_pulse_start(dut):
    await RisingEdge(dut.clk)  # let mode/x_in/y_in/z_in settle
    dut.core_start.value = 1
    await RisingEdge(dut.clk)  # core samples start here
    dut.core_start.value = 0


async def core_wait_done(dut):
    while True:
        await RisingEdge(dut.clk)
        await ReadOnly()
        if dut.core_done.value:
            x = dut.core_x_out.value.to_signed()
            y = dut.core_y_out.value.to_signed()
            z = dut.core_z_out.value.to_signed()
            await RisingEdge(dut.clk)  # leave ReadOnly before any new write
            return x, y, z


async def core_rotate(dut, angle):
    dut.core_mode.value = MODE_ROT
    dut.core_z_in.value = angle & 0xFFFF
    dut.core_x_in.value = 0
    dut.core_y_in.value = 0
    await core_pulse_start(dut)
    x, y, _z = await core_wait_done(dut)
    return x, y  # cos, sin


async def core_vector(dut, x_in, y_in):
    dut.core_mode.value = MODE_VEC
    dut.core_x_in.value = x_in & 0xFFFF
    dut.core_y_in.value = y_in & 0xFFFF
    dut.core_z_in.value = 0
    await core_pulse_start(dut)
    x, _y, z = await core_wait_done(dut)
    return x, z  # magnitude, angle


# ----------------------------------------------------------------------
# Wrapper-level helpers: drive tt_um_joel0328 through the real
# ui_in/uio_in register-map pin protocol.
# ----------------------------------------------------------------------

async def write_reg(dut, addr, value):
    dut.ui_in.value = value & 0xFF
    dut.uio_in.value = (addr & 0xF) | WE_BIT
    await RisingEdge(dut.clk)  # WE-rising edge latched here
    dut.uio_in.value = addr & 0xF  # deassert WE
    await RisingEdge(dut.clk)  # settle before the next write/read


async def read_reg(dut, addr):
    dut.uio_in.value = addr & 0xF
    await ReadOnly()
    val = int(dut.uo_out.value)
    await RisingEdge(dut.clk)  # leave ReadOnly cleanly before any new write
    return val


async def wait_result_valid(dut, max_polls=200):
    for _ in range(max_polls):
        status = await read_reg(dut, ADDR_STATUS)
        if status & 1:
            return
    raise AssertionError("timed out waiting for STATUS.result_valid")


async def read_result(dut):
    x_lo = await read_reg(dut, ADDR_XOUT_LO)
    x_hi = await read_reg(dut, ADDR_XOUT_HI)
    y_lo = await read_reg(dut, ADDR_YOUT_LO)
    y_hi = await read_reg(dut, ADDR_YOUT_HI)
    z_lo = await read_reg(dut, ADDR_ZOUT_LO)
    z_hi = await read_reg(dut, ADDR_ZOUT_HI)
    x = to_signed16((x_hi << 8) | x_lo)
    y = to_signed16((y_hi << 8) | y_lo)
    z = to_signed16((z_hi << 8) | z_lo)
    return x, y, z


async def wrapper_compute(dut, mode, x=0, y=0, z=0):
    await write_reg(dut, ADDR_X_LO, x & 0xFF)
    await write_reg(dut, ADDR_X_HI, (x >> 8) & 0xFF)
    await write_reg(dut, ADDR_Y_LO, y & 0xFF)
    await write_reg(dut, ADDR_Y_HI, (y >> 8) & 0xFF)
    await write_reg(dut, ADDR_Z_LO, z & 0xFF)
    await write_reg(dut, ADDR_Z_HI, (z >> 8) & 0xFF)
    await write_reg(dut, ADDR_CTRL, mode & 0x1)
    await write_reg(dut, ADDR_START, 0x01)  # any value fires the start pulse
    await wait_result_valid(dut)
    return await read_result(dut)


# ----------------------------------------------------------------------
# Edge / boundary vectors
# ----------------------------------------------------------------------

# Fold boundary is bit15 ^ bit14: flips at 0x4000 (+90 deg) and 0xC000
# (-90 deg / 270 deg). Bracket both boundaries on either side, plus the
# angle-wrap extremes.
ROT_EDGE_ANGLES = [
    0x0000, 0x7FFF, 0x8000, 0xFFFF,               # wrap extremes
    0x3FFE, 0x3FFF, 0x4000, 0x4001,               # +90 deg boundary
    0xBFFE, 0xBFFF, 0xC000, 0xC001,                # -90 deg boundary
]

# x = 0 (vectoring fold boundary is sign of x), plus operand min/max/
# saturation combinations at the raw 16-bit extremes.
VEC_EDGE_VECTORS = [
    (0, 10000), (0, -10000), (0, 1), (0, -1), (0, 0),
    (32767, 0), (-32768, 0), (1, 0), (-1, 0),
    (32767, 32767), (-32768, -32768),
    (-32768, 32767), (32767, -32768),
]


# ----------------------------------------------------------------------
# 1. Core-level tests
# ----------------------------------------------------------------------

@cocotb.test(skip=GATES)
async def test_core_rotation_matches_model(dut):
    """cordic_core, driven directly: rotate mode must match
    cordic_rotate() exactly, not just within tolerance."""
    await start_clock(dut)
    await reset(dut)

    rng = random.Random(1234)
    angles = ROT_EDGE_ANGLES + [rng.randint(0, 0xFFFF) for _ in range(60)]

    for a in angles:
        cos_rtl, sin_rtl = await core_rotate(dut, a)
        cos_m, sin_m = cordic_rotate(a & 0xFFFF)
        assert (cos_rtl, sin_rtl) == (cos_m, sin_m), (
            f"angle={a:#06x}: rtl=({cos_rtl},{sin_rtl}) "
            f"model=({cos_m},{sin_m})"
        )


@cocotb.test(skip=GATES)
async def test_core_vectoring_matches_model(dut):
    """cordic_core, driven directly: vector mode must match
    cordic_vector() exactly, including x=0 and operand-extreme edges."""
    await start_clock(dut)
    await reset(dut)

    rng = random.Random(5678)
    vectors = VEC_EDGE_VECTORS + [
        (rng.randint(-32768, 32767), rng.randint(-32768, 32767))
        for _ in range(60)
    ]

    for x, y in vectors:
        mag_rtl, ang_rtl = await core_vector(dut, x, y)
        mag_m, ang_m = cordic_vector(x, y)
        assert (mag_rtl, ang_rtl) == (mag_m, ang_m), (
            f"x={x} y={y}: rtl=({mag_rtl},{ang_rtl}) model=({mag_m},{ang_m})"
        )


# ----------------------------------------------------------------------
# 2. Wrapper end-to-end tests (real pin protocol)
# ----------------------------------------------------------------------

@cocotb.test()
async def test_wrapper_rotation_matches_model(dut):
    """Full register-map path: write X/Y/Z_IN + CTRL, pulse START via WE,
    poll STATUS, read back X/Y_OUT — reassembled result must match
    cordic_rotate() exactly."""
    await start_clock(dut)
    await reset(dut)

    rng = random.Random(4321)
    angles = ROT_EDGE_ANGLES + [rng.randint(0, 0xFFFF) for _ in range(20)]

    for a in angles:
        cos_rtl, sin_rtl, _z_rtl = await wrapper_compute(dut, MODE_ROT, z=a)
        cos_m, sin_m = cordic_rotate(a & 0xFFFF)
        assert (cos_rtl, sin_rtl) == (cos_m, sin_m), (
            f"angle={a:#06x}: rtl=({cos_rtl},{sin_rtl}) "
            f"model=({cos_m},{sin_m})"
        )


@cocotb.test()
async def test_wrapper_vectoring_matches_model(dut):
    """Full register-map path for vector mode; reassembled X_OUT/Z_OUT
    must match cordic_vector() exactly."""
    await start_clock(dut)
    await reset(dut)

    rng = random.Random(8765)
    vectors = VEC_EDGE_VECTORS + [
        (rng.randint(-32768, 32767), rng.randint(-32768, 32767))
        for _ in range(20)
    ]

    for x, y in vectors:
        mag_rtl, _y_rtl, ang_rtl = await wrapper_compute(dut, MODE_VEC, x=x, y=y)
        mag_m, ang_m = cordic_vector(x, y)
        assert (mag_rtl, ang_rtl) == (mag_m, ang_m), (
            f"x={x} y={y}: rtl=({mag_rtl},{ang_rtl}) model=({mag_m},{ang_m})"
        )


@cocotb.test()
async def test_wrapper_status_sticky_and_clears_on_start(dut):
    """result_valid must come up only after done (not immediately after
    START) and must clear the moment a new START is issued, per the
    sticky-flag contract in project.v."""
    await start_clock(dut)
    await reset(dut)

    status = await read_reg(dut, ADDR_STATUS)
    assert status & 1 == 0, "result_valid should be clear after reset"

    await write_reg(dut, ADDR_Z_LO, 0x00)
    await write_reg(dut, ADDR_Z_HI, 0x00)
    await write_reg(dut, ADDR_CTRL, MODE_ROT)
    await write_reg(dut, ADDR_START, 0x01)

    status = await read_reg(dut, ADDR_STATUS)
    assert status & 1 == 0, "result_valid must not jump high before done"

    await wait_result_valid(dut)
    x0, y0, _z0 = await read_result(dut)

    # Re-arm with a different angle; result_valid must drop immediately.
    await write_reg(dut, ADDR_Z_LO, 0x00)
    await write_reg(dut, ADDR_Z_HI, 0x40)  # 0x4000 -> 90 deg
    await write_reg(dut, ADDR_CTRL, MODE_ROT)
    await write_reg(dut, ADDR_START, 0x01)

    status = await read_reg(dut, ADDR_STATUS)
    assert status & 1 == 0, "result_valid must clear when START is re-issued"

    await wait_result_valid(dut)
    x1, y1, _z1 = await read_result(dut)
    assert (x1, y1) != (x0, y0), "second result should differ from the first"
