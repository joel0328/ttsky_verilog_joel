# CLAUDE.md — Dual-Mode CORDIC (Tiny Tapeout TTSKY26c)

Context and standing decisions for this repo. Read this first, then treat
`cordic_model.py` as the authoritative spec for all numeric behavior.

## What this project is

A dual-mode CORDIC engine being submitted to Tiny Tapeout, shuttle **TTSKY26c**,
deadline **September 7, 2026**. "Dual-mode" = both rotation (angle → sin/cos) and
vectoring (x,y → magnitude, atan2). Iterative, not pipelined — right for a TT tile
budget and for serializing operands through the 8-bit ports.

## Division of labor (important)

- **You (Claude Code)** generate: the RTL core, the cocotb testbench, and the TT
  pin wrapper. You run the local sim/verification loop and iterate the RTL against
  the golden model until the suite passes.
- **The human (Joel)** does: hardware testing, implementation, TT hardening/GDS,
  and the TT submission. He is a senior electrical engineer and **will read and
  understand every line** to build Verilog experience — so:
  - Write clear, conventional FSMD Verilog. No clever one-liners that obscure intent.
  - Comment the *why*, especially around fold/unfold, sign steering, guard-bit
    rounding, and the iteration datapath.
  - When you make a design choice, state the reasoning in a comment or in chat so it
    can be learned, not just accepted.

## The spec is the golden model — do not trust this file for constants

`cordic_model.py` (validated by `test_model.py`, 8 passing tests) fixes the exact
numeric contract. **Take the exact atan LUT values, `INV_K_SEED`, and rounding from
the model file itself — do not hardcode any constant from memory or from this brief.**
The RTL must be bit-exact to the model.

## Locked architectural decisions (intent, not exact values)

- **Fixed point:** 16-bit external, **Q1.14**.
- **Guard bits:** **3 guard bits → 19-bit internal datapath.** This was chosen by a
  measured parameter sweep in the model: naive 16-bit gave ~8 LSB worst-case error;
  3 guard bits dropped it to **~1.76 LSB worst-case** for the cost of 9 flip-flops.
  That error bound is a datasheet claim — the RTL must hold it.
- **Angle format:** **brads (binary radians).** Full circle = natural 16-bit
  wraparound; 0x4000 = 90°; quadrant = top two bits. This makes a phase-accumulator
  NCO plug in directly. Do NOT switch to radians.
- **atan LUT:** 16-entry, implemented as a synthesizable `case` (not a real ROM).
  Values come from the model.
- **Gain pre-scale:** seed with the model's `INV_K_SEED` (read exact value from file).
- **Iterations:** 16.
- **FSM:** `IDLE → LOAD (fold, load seeds) → ITERATE ×16 → DONE (unfold, round)`.
- **Output rounding:** round-half-up, `(+4) >> 3`, dropping the 3 guard bits.
- **Signed shifts:** use `>>>` on signed regs for the micro-rotations.

## Synthesis constraints (this hardens through OpenLane on sky130)

The RTL feeds the TT GDS flow, so keep it clean from the start:
- Single clock domain (TT provides one `clk`, `rst_n`, `ena`). No gated/derived clocks.
- No latches — every reg assigned on every path; full `case`/`if` coverage.
- No combinational loops, no multi-driven nets.
- Synchronous reset off `rst_n` per TT convention.

## Pin protocol — WORKING ASSUMPTION: register-map only

TT gives 8 dedicated in (`ui_in`), 8 dedicated out (`uo_out`), 8 bidirectional
(`uio_*`). We need two 16-bit operands + a mode bit in, and two 16-bit results out,
so operands are loaded a byte at a time via a small register map:
- address/select on `uio`, data byte on `ui_in`, a write strobe to load;
- a start strobe to begin compute; a `done` flag out; results read back byte-wise.

Keep the core's interface clean and internal (parallel 16-bit ports, start/done);
put ALL pin-packing in the `tt_um_*` wrapper as a **separate module**. Do not
entangle the core with pin scarcity.

> Standalone "demo mode" (drive an angle on `ui_in`, watch sin on `uo_out`, no host)
> is a **stretch goal only** — add it after the core + wrapper verify and only if the
> September timeline has slack. Do not build it preemptively.

## Files to produce

- `src/cordic_core.v` — datapath (x/y/z regs, iteration counter, atan `case`,
  sign-steered add/sub) + FSM. Clean internal interface.
- `src/tt_um_*.v` — TT wrapper: instantiates the core, implements the register-map
  pin protocol, ties off unused pins per TT rules.
- cocotb bench (in the repo's test dir) — imports `cordic_model.py`, drives the RTL,
  diffs RTL output vs. model across the model's own vector set, and asserts the
  worst-case error stays at/under the 1.76 LSB bar in **both** modes. Add edge
  vectors: angle wrap, operand min/max/saturation, mode boundaries.

## Verification gate (nothing proceeds past this)

The existing `test_model.py` must keep passing, and the new cocotb bench must be
green locally (Icarus 12.0 + cocotb venv, already set up in WSL2) and in CI before
moving to hardening. If RTL disagrees with the model, fix the RTL — the model is
right by definition.

## Then (human's lane)

Push to trigger TT CI (gds/test/docs are independent workflows). Read the
timing/utilization reports. If timing doesn't close, the RTL levers are: deepen the
FSM (fewer combinational ops per cycle) or trim the LUT path. `info.yaml` pinout +
`docs/info.md` (How it works / How to test) still need filling for the docs workflow.
