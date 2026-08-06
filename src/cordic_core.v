/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * cordic_core — CORDIC datapath + FSM only (no pin mapping; see the
 * tt_um wrapper for that). Bit-exact RTL image of model/cordic_model.py.
 *
 * Both operating modes share one shift-add iteration core:
 *
 *   MODE_ROT: angle in (z_in)        -> (cos, sin) out (x_out, y_out)
 *   MODE_VEC: vector in (x_in, y_in) -> (K*|v|, atan2) out (x_out, z_out)
 *
 * All numeric constants (WIDTH, GUARD, N_ITER, INV_K_SEED, ATAN_TABLE,
 * HALF_TURN_EXT) are copied verbatim from model/cordic_model.py — they
 * are not re-derived here. If WIDTH/GUARD/N_ITER ever change, rerun
 * `python3 model/cordic_model.py` and copy the printed values in.
 *
 * Handshake: pulse `start` for one cycle while idle (done == 0). The
 * core latches mode/x_in/y_in/z_in on that edge and moves through:
 *
 *   IDLE -(start)-> LOAD (fold, seed x0/y0/z0) -> ITERATE x N_ITER
 *        -> DONE (unfold, round; done=1 for 1 cycle) -> IDLE
 *
 * LOAD and DONE are each their own single-cycle state (rather than
 * folding fold/seed into IDLE's exit, or unfold/round into ITERATE's
 * last lap) so every state has one clear job and a short combinational
 * path — easier to read, and kinder to synthesis timing than a single
 * state doing double duty.
 */

`default_nettype none

module cordic_core (
    input  wire        clk,
    input  wire        rst_n,       // active-low sync reset

    input  wire        start,       // pulse 1 cycle while idle to launch
    input  wire        mode,        // 0 = MODE_ROT, 1 = MODE_VEC
    input  wire signed [15:0] x_in, // MODE_VEC: x component (Q1.14)
    input  wire signed [15:0] y_in, // MODE_VEC: y component (Q1.14)
    input  wire signed [15:0] z_in, // MODE_ROT: angle (brads)

    output wire                done,  // 1-cycle pulse, outputs valid
    output wire signed [15:0]  x_out, // MODE_ROT: cos   | MODE_VEC: K*|v|
    output wire signed [15:0]  y_out, // MODE_ROT: sin   | MODE_VEC: residual
    output wire signed [15:0]  z_out  // MODE_ROT: residual | MODE_VEC: angle
);

  // ------------------------------------------------------------------
  // Constants (model/cordic_model.py is the source of truth)
  // ------------------------------------------------------------------
  localparam WIDTH  = 16;             // external result width
  localparam GUARD  = 3;              // internal guard bits
  localparam IWIDTH = WIDTH + GUARD;  // internal datapath width = 19
  localparam N_ITER = 16;             // CORDIC iterations
  localparam ITER_W = 4;              // ceil(log2(N_ITER)), counts 0..15

  localparam MODE_ROT = 1'b0;
  localparam MODE_VEC = 1'b1;

  // Rotation-mode x seed: round((2^14 << GUARD) / K), K = CORDIC gain
  localparam signed [IWIDTH-1:0] INV_K_SEED = 19'sd79594;

  // HALF_TURN (=2^15 brads) << GUARD == 2^18 exactly, which lands right
  // on the negative boundary of a 19-bit two's-complement range, so its
  // wrapped value is just the minimum representable value (sign bit
  // set, everything else clear).
  localparam signed [IWIDTH-1:0] HALF_TURN_EXT = {1'b1, {(IWIDTH-1){1'b0}}};

  // Guard-bit rounding constant: round-half-up before truncation.
  localparam ROUND_CONST = 1 << (GUARD - 1);  // 4

  // atan(2^-i) in guard-extended brads, i = 0..15 (ATAN_TABLE in the
  // model). Values go verbatim into this case, per the model's header.
  function automatic signed [IWIDTH-1:0] atan_lut(input [ITER_W-1:0] i);
    case (i)
      4'd0:  atan_lut = 19'sd65536;
      4'd1:  atan_lut = 19'sd38688;
      4'd2:  atan_lut = 19'sd20442;
      4'd3:  atan_lut = 19'sd10377;
      4'd4:  atan_lut = 19'sd5208;
      4'd5:  atan_lut = 19'sd2607;
      4'd6:  atan_lut = 19'sd1304;
      4'd7:  atan_lut = 19'sd652;
      4'd8:  atan_lut = 19'sd326;
      4'd9:  atan_lut = 19'sd163;
      4'd10: atan_lut = 19'sd81;
      4'd11: atan_lut = 19'sd41;
      4'd12: atan_lut = 19'sd20;
      4'd13: atan_lut = 19'sd10;
      4'd14: atan_lut = 19'sd5;
      default: atan_lut = 19'sd3;  // i == 15
    endcase
  endfunction

  // Strip guard bits: round-half-up then truncate to WIDTH bits.
  // Mirrors _round_out(): _wrap((v + ROUND) >> GUARD, WIDTH). The extra
  // bit on `sum` avoids overflowing before the shift, matching Python's
  // unbounded-int add.
  function automatic signed [WIDTH-1:0] round_out(input signed [IWIDTH-1:0] v);
    reg signed [IWIDTH:0] sum;
    begin
      sum       = {v[IWIDTH-1], v} + ROUND_CONST;
      round_out = sum >>> GUARD;  // truncated to WIDTH bits on assignment
    end
  endfunction

  // ------------------------------------------------------------------
  // FSM state
  // ------------------------------------------------------------------
  localparam S_IDLE = 2'd0;
  localparam S_LOAD = 2'd1;
  localparam S_ITER = 2'd2;
  localparam S_DONE = 2'd3;

  reg [1:0]        state;
  reg [ITER_W-1:0] iter_cnt;

  // Latched in IDLE on `start`, held stable for LOAD's fold/seed math —
  // decouples the datapath from whatever the caller does with
  // x_in/y_in/z_in/mode after the start pulse.
  reg               mode_reg;
  reg signed [15:0] raw_x, raw_y, raw_z;

  reg                     fold_reg;
  reg signed [IWIDTH-1:0] x_reg, y_reg, z_reg;

  // ------------------------------------------------------------------
  // Fold + seed (combinational; registered into x_reg/y_reg/z_reg on
  // the LOAD -> ITER transition). This is the pre-iteration half of
  // cordic_rotate()/cordic_vector(): reduce the input into the range
  // the shared core actually converges over, then set up x0/y0/z0.
  //
  //   MODE_ROT: the core only converges over ~+/-90 deg, so angles in
  //   the left half-plane (quadrant test = bit15 ^ bit14) get rotated
  //   180 deg first; the fold is undone by negating the outputs later.
  //
  //   MODE_VEC: the core requires x >= 0, so a negative x pre-rotates
  //   180 deg (negate both components); the fold is undone by adding/
  //   subtracting a half turn from the resulting angle later.
  // ------------------------------------------------------------------
  wire               rot_fold  = raw_z[15] ^ raw_z[14];
  wire signed [15:0] rot_angle = rot_fold ? (raw_z - 16'sd32768) : raw_z;

  wire                     vec_fold  = raw_x[15];  // pre-fold x < 0
  wire signed [IWIDTH-1:0] vec_x_pre = {raw_x, {GUARD{1'b0}}};
  wire signed [IWIDTH-1:0] vec_y_pre = {raw_y, {GUARD{1'b0}}};

  wire fold_next = (mode_reg == MODE_ROT) ? rot_fold : vec_fold;

  wire signed [IWIDTH-1:0] x0 =
      (mode_reg == MODE_ROT) ? INV_K_SEED :
      (vec_fold              ? -vec_x_pre : vec_x_pre);

  wire signed [IWIDTH-1:0] y0 =
      (mode_reg == MODE_ROT) ? {IWIDTH{1'b0}} :
      (vec_fold              ? -vec_y_pre : vec_y_pre);

  wire signed [IWIDTH-1:0] z0 =
      (mode_reg == MODE_ROT) ? {rot_angle, {GUARD{1'b0}}} : {IWIDTH{1'b0}};

  // ------------------------------------------------------------------
  // Iteration: one micro-rotation per cycle, shared by both modes —
  // the RTL image of _iterate()'s loop body. Rotate mode drives z
  // toward 0 (d = sign(z)); vector mode drives y toward 0
  // (d = -sign(y)); everything else is the same shift/add/sub.
  // ------------------------------------------------------------------
  wire dsel = (mode_reg == MODE_ROT) ? (z_reg >= 0) : (y_reg < 0);  // d=+1 iff set

  wire signed [IWIDTH-1:0] y_shift = y_reg >>> iter_cnt;
  wire signed [IWIDTH-1:0] x_shift = x_reg >>> iter_cnt;
  wire signed [IWIDTH-1:0] atan_i  = atan_lut(iter_cnt);

  wire signed [IWIDTH-1:0] x_next = dsel ? (x_reg - y_shift) : (x_reg + y_shift);
  wire signed [IWIDTH-1:0] y_next = dsel ? (y_reg + x_shift) : (y_reg - x_shift);
  wire signed [IWIDTH-1:0] z_next = dsel ? (z_reg - atan_i)  : (z_reg + atan_i);

  // ------------------------------------------------------------------
  // Unfold + round (combinational; valid while state == S_DONE). Tail
  // half of cordic_rotate()/cordic_vector(): undo the pre-fold, then
  // strip guard bits with round-half-up.
  // ------------------------------------------------------------------
  wire signed [IWIDTH-1:0] rot_x_final = fold_reg ? -x_reg : x_reg;
  wire signed [IWIDTH-1:0] rot_y_final = fold_reg ? -y_reg : y_reg;

  wire signed [IWIDTH-1:0] vec_z_final = fold_reg ?
      ((z_reg >= 0) ? (z_reg - HALF_TURN_EXT) : (z_reg + HALF_TURN_EXT)) :
      z_reg;

  wire signed [IWIDTH-1:0] x_final = (mode_reg == MODE_ROT) ? rot_x_final : x_reg;
  wire signed [IWIDTH-1:0] y_final = (mode_reg == MODE_ROT) ? rot_y_final : y_reg;
  wire signed [IWIDTH-1:0] z_final = (mode_reg == MODE_ROT) ? z_reg       : vec_z_final;

  assign done  = (state == S_DONE);
  assign x_out = round_out(x_final);
  assign y_out = round_out(y_final);
  assign z_out = round_out(z_final);

  // ------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state    <= S_IDLE;
      iter_cnt <= {ITER_W{1'b0}};
      mode_reg <= 1'b0;
      raw_x    <= 16'sd0;
      raw_y    <= 16'sd0;
      raw_z    <= 16'sd0;
      fold_reg <= 1'b0;
      x_reg    <= {IWIDTH{1'b0}};
      y_reg    <= {IWIDTH{1'b0}};
      z_reg    <= {IWIDTH{1'b0}};
    end else begin
      case (state)
        S_IDLE: begin
          if (start) begin
            mode_reg <= mode;
            raw_x    <= x_in;
            raw_y    <= y_in;
            raw_z    <= z_in;
            state    <= S_LOAD;
          end
        end

        S_LOAD: begin
          fold_reg <= fold_next;
          x_reg    <= x0;
          y_reg    <= y0;
          z_reg    <= z0;
          iter_cnt <= {ITER_W{1'b0}};
          state    <= S_ITER;
        end

        S_ITER: begin
          x_reg <= x_next;
          y_reg <= y_next;
          z_reg <= z_next;
          if (iter_cnt == N_ITER - 1) begin
            state <= S_DONE;
          end else begin
            iter_cnt <= iter_cnt + 1'b1;
          end
        end

        S_DONE: begin
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
