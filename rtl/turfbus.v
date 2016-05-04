`timescale 1ns / 1ps
`include "wishbone.vh"
// TURFbus-to-WISHBONE bridge. 
// TURFbus is an extremely simple high-ish speed serial
// link. Right now it operates 1-bit per cycle but we'll
// see how high we can push it.
// SREQ is the return path.
module turfbus( input wbm_clk_i,
					 input wbm_rst_i,
					 `WBM_NAMED_PORT(wbm, 32, 20, 4),
					 input TCLK_P,
					 input TCLK_N,
					 output SREQ_neg,
					 input TREQ_neg,
					 output [7:0] TD,
					 output [7:0] TD_oe
    );

	wire TCLK;
	IBUFDS u_tclk_ibuf(.I(TCLK_P),.IB(TCLK_N),.O(TCLK));
	assign TD = {8{1'b0}};
	assign TD_oe = {8{1'b1}};
	assign SREQ_neg = 1;
	`WB_KILL(wbm, 32, 20, 4);
endmodule
