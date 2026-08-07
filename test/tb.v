`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb ();

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`endif

  // Replace tt_um_example with your module name:
  tt_um_joel0328 user_project (

      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

  // Second, independent instance of cordic_core (same unmodified
  // cordic_core.v) wired straight to its own top-level pins, for
  // core-level cocotb tests that drive the datapath directly instead of
  // through the tt_um_joel0328 register-map wrapper. This instance is
  // test-only scaffolding — it has no bearing on the tapeout netlist,
  // which only instantiates cordic_core once, inside user_project.
  //
  // Gate-level sim compiles a flattened netlist for tt_um_joel0328 where
  // cordic_core no longer exists as a separate module (synthesis folds it
  // into the top level), so this standalone instance — and the regs/wires
  // that feed it — must be excluded from GL_TEST builds or elaboration
  // fails with "Unknown module type: cordic_core".
`ifndef GL_TEST
  reg               core_start;
  reg               core_mode;
  reg  signed [15:0] core_x_in, core_y_in, core_z_in;
  wire               core_done;
  wire signed [15:0] core_x_out, core_y_out, core_z_out;

  cordic_core cordic_core_dut (
      .clk    (clk),
      .rst_n  (rst_n),
      .start  (core_start),
      .mode   (core_mode),
      .x_in   (core_x_in),
      .y_in   (core_y_in),
      .z_in   (core_z_in),
      .done   (core_done),
      .x_out  (core_x_out),
      .y_out  (core_y_out),
      .z_out  (core_z_out)
  );
`endif

endmodule
