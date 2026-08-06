/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * tt_um_joel0328 — Tiny Tapeout pin wrapper around cordic_core.
 *
 * cordic_core has a clean parallel 16-bit interface, but TT only gives us
 * 8 dedicated in, 8 dedicated out, and 8 bidirectional pins. This wrapper
 * is purely a byte-serialized register-map adapter around the core — all
 * pin-packing lives here, none of it leaks into cordic_core.v.
 *
 * Pins:
 *   ui_in[7:0]  = write data byte (host -> chip)
 *   uo_out[7:0] = read data byte (chip -> host), combinational mux by addr
 *   uio_in[3:0] = register address
 *   uio_in[4]   = write enable (WE)
 *   uio_in[7:5] = unused (reserved for a future demo-mode jumper)
 *   uio_out/uio_oe = 0 (uio is input-only here; no direction flips)
 *   ena         = ignored (always asserted when the tile is powered)
 *
 * Register map (uio_in[3:0]):
 *   Writable, latched on the rising edge of WE while addressed:
 *     0x0 X_IN[7:0]    0x1 X_IN[15:8]
 *     0x2 Y_IN[7:0]    0x3 Y_IN[15:8]
 *     0x4 Z_IN[7:0]    0x5 Z_IN[15:8]
 *     0x6 CTRL   bit0 = mode (0 = rotate, 1 = vector)
 *     0x7 START  any write fires exactly one start pulse to the core
 *   Readable, combinational mux, no strobe needed:
 *     0x8 X_OUT[7:0]   0x9 X_OUT[15:8]
 *     0xA Y_OUT[7:0]   0xB Y_OUT[15:8]
 *     0xC Z_OUT[7:0]   0xD Z_OUT[15:8]
 *     0xE STATUS bit0 = result_valid, other bits 0
 *
 * WE is level-driven by the host and may stay high for multiple chip
 * clocks, so every write is edge-detected (we_rising = WE & ~WE_prev) to
 * turn it into a single discrete event regardless of how long WE is held.
 *
 * result_valid is a STICKY flag, not a passthrough of the core's `done`:
 * `done` is a single-cycle pulse that a byte-at-a-time polling host would
 * likely miss entirely, so we latch it on `done` and clear it when a new
 * START is issued. cordic_core holds x_out/y_out/z_out stable from `done`
 * until the next start (see cordic_core.v), so the result bytes stay
 * correctly readable for as long as the host takes to poll STATUS and
 * read them back.
 */

`default_nettype none

module tt_um_joel0328 (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // ------------------------------------------------------------------
  // Register-map addresses
  // ------------------------------------------------------------------
  localparam ADDR_X_LO   = 4'h0;
  localparam ADDR_X_HI   = 4'h1;
  localparam ADDR_Y_LO   = 4'h2;
  localparam ADDR_Y_HI   = 4'h3;
  localparam ADDR_Z_LO   = 4'h4;
  localparam ADDR_Z_HI   = 4'h5;
  localparam ADDR_CTRL   = 4'h6;
  localparam ADDR_START  = 4'h7;
  localparam ADDR_XOUT_LO = 4'h8;
  localparam ADDR_XOUT_HI = 4'h9;
  localparam ADDR_YOUT_LO = 4'hA;
  localparam ADDR_YOUT_HI = 4'hB;
  localparam ADDR_ZOUT_LO = 4'hC;
  localparam ADDR_ZOUT_HI = 4'hD;
  localparam ADDR_STATUS  = 4'hE;

  wire [3:0] addr = uio_in[3:0];
  wire       we   = uio_in[4];

  // ------------------------------------------------------------------
  // Host-facing registers
  // ------------------------------------------------------------------
  reg        we_prev;
  reg signed [15:0] x_in_reg, y_in_reg, z_in_reg;
  reg        mode_reg;
  reg        start_pulse;
  reg        result_valid;

  wire we_rising = we & ~we_prev;

  // ------------------------------------------------------------------
  // cordic_core instance — unmodified, unchanged interface
  // ------------------------------------------------------------------
  wire                core_done;
  wire signed [15:0]  core_x_out, core_y_out, core_z_out;

  cordic_core cordic (
      .clk    (clk),
      .rst_n  (rst_n),
      .start  (start_pulse),
      .mode   (mode_reg),
      .x_in   (x_in_reg),
      .y_in   (y_in_reg),
      .z_in   (z_in_reg),
      .done   (core_done),
      .x_out  (core_x_out),
      .y_out  (core_y_out),
      .z_out  (core_z_out)
  );

  // ------------------------------------------------------------------
  // Writes: edge-detected so a WE held high for many clocks still only
  // fires one write. start_pulse is registered here too, so it comes out
  // exactly one cycle after the START write's we_rising edge — one cycle
  // after every earlier X/Y/Z_IN and CTRL write has already landed.
  // ------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      we_prev      <= 1'b0;
      x_in_reg     <= 16'sd0;
      y_in_reg     <= 16'sd0;
      z_in_reg     <= 16'sd0;
      mode_reg     <= 1'b0;
      start_pulse  <= 1'b0;
      result_valid <= 1'b0;
    end else begin
      we_prev     <= we;
      start_pulse <= we_rising && (addr == ADDR_START);

      if (we_rising) begin
        case (addr)
          ADDR_X_LO: x_in_reg[7:0]  <= ui_in;
          ADDR_X_HI: x_in_reg[15:8] <= ui_in;
          ADDR_Y_LO: y_in_reg[7:0]  <= ui_in;
          ADDR_Y_HI: y_in_reg[15:8] <= ui_in;
          ADDR_Z_LO: z_in_reg[7:0]  <= ui_in;
          ADDR_Z_HI: z_in_reg[15:8] <= ui_in;
          ADDR_CTRL: mode_reg       <= ui_in[0];
          default:   ;  // START and read-only addresses: no register write here
        endcase
      end

      if (core_done)
        result_valid <= 1'b1;
      else if (we_rising && addr == ADDR_START)
        result_valid <= 1'b0;
    end
  end

  // ------------------------------------------------------------------
  // Reads: pure combinational mux, no strobe needed. cordic_core keeps
  // x_out/y_out/z_out stable across states until the next start, so
  // these bytes stay valid for as long as the host takes to read them.
  // ------------------------------------------------------------------
  reg [7:0] uo_out_r;

  always @(*) begin
    case (addr)
      ADDR_XOUT_LO: uo_out_r = core_x_out[7:0];
      ADDR_XOUT_HI: uo_out_r = core_x_out[15:8];
      ADDR_YOUT_LO: uo_out_r = core_y_out[7:0];
      ADDR_YOUT_HI: uo_out_r = core_y_out[15:8];
      ADDR_ZOUT_LO: uo_out_r = core_z_out[7:0];
      ADDR_ZOUT_HI: uo_out_r = core_z_out[15:8];
      ADDR_STATUS:  uo_out_r = {7'b0, result_valid};
      default:      uo_out_r = 8'h00;
    endcase
  end

  assign uo_out  = uo_out_r;
  assign uio_out = 8'h00;
  assign uio_oe  = 8'h00;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, uio_in[7:5], 1'b0};

endmodule
