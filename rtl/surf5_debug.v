`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// This file is a part of the Antarctic Impulsive Transient Antenna (ANITA)
// project, a collaborative scientific effort between multiple institutions. For
// more information, contact Peter Gorham (gorham@phys.hawaii.edu).
//
// All rights reserved.
//
// Author: Patrick Allison, Ohio State University (allison.122@osu.edu)
// Author:
// Author:
////////////////////////////////////////////////////////////////////////////////
`include "wishbone.vh"
module surf5_debug(
		input wbc_clk_i,
		input clk0_i,
		input clk1_i,
		input clk_big_i,
		`WBM_NAMED_PORT(wbvio, 32, 20, 4),
		input [70:0] clk0_debug0_i,
		input [70:0] clk0_debug1_i,
		input [70:0] clk0_debug2_i,
		input [70:0] clk0_debug3_i,
		input [70:0] clk1_debug_i,
		input [14:0] clk_big_debug_i,
		output [7:0] global_debug_o
    );

	wire [70:0] ila0_debug_vec[3:0];
	assign ila0_debug_vec[0] = clk0_debug0_i;
	assign ila0_debug_vec[1] = clk0_debug1_i;
	assign ila0_debug_vec[2] = clk0_debug2_i;
	assign ila0_debug_vec[3] = clk0_debug3_i;
	
	reg [70:0] ila0_debug = {71{1'b0}};
	reg [70:0] ila1_debug = {71{1'b0}};
	wire [63:0] vio_sync_in;
	wire [47:0] vio_sync_out;

	wire [31:0] bridge_dat_o = vio_sync_in[0 +: 32];
	wire [19:0] bridge_adr_o = vio_sync_in[32 +: 20];
	wire bridge_we_o = vio_sync_in[52];
	wire bridge_go_o = vio_sync_in[53];
	wire bridge_lock_o = vio_sync_in[54];
	// vio_sync_in[63:54] are available for debug multiplexing
	wire [1:0] ila0_sel = vio_sync_in[56:55];

	wire [31:0] bridge_dat_i;
	wire bridge_done_i;
	wire bridge_err_i;
	assign vio_sync_out[0 +: 32] = bridge_dat_i;
	assign vio_sync_out[32] = bridge_done_i;
	assign vio_sync_out[33] = bridge_err_i;
	// vio_sync_out[47:34] are available for... something
	
	wbvio_bridge u_bridge(.clk_i(wbc_clk_i),.rst_i(1'b0),
								 .wbvio_dat_i(bridge_dat_o),
								 .wbvio_adr_i(bridge_adr_o),
								 .wbvio_we_i(bridge_we_o),
								 .wbvio_go_i(bridge_go_o),
								 .wbvio_lock_i(bridge_lock_o),
								 .wbvio_dat_o(bridge_dat_i),
								 .wbvio_done_o(bridge_done_i),
								 .wbvio_err_o(bridge_err_i),
								 .cyc_o(wbvio_cyc_o),
								 .stb_o(wbvio_stb_o),
								 .we_o(wbvio_we_o),
								 .dat_i(wbvio_dat_i),
								 .dat_o(wbvio_dat_o),
								 .adr_o(wbvio_adr_o),
								 .ack_i(wbvio_ack_i),
								 .err_i(wbvio_err_i),
								 .rty_i(wbvio_rty_i)
								 );
	always @(posedge clk0_i) begin
		ila0_debug <= ila0_debug_vec[ila0_sel[1:0]];
	end
	always @(posedge clk1_i) begin
		ila1_debug <= clk1_debug_i;
	end
	wire [35:0] vio_control;
	wire [35:0] ila0_control;
	wire [35:0] ila1_control;
	wire [35:0] big_ila_control;
	surf5_icon u_icon(.CONTROL0(vio_control),.CONTROL1(ila0_control),.CONTROL2(ila1_control),.CONTROL3(big_ila_control));
	surf5_ila u_ila0(.CONTROL(ila0_control),.CLK(clk0_i),.TRIG0(ila0_debug));
	surf5_ila u_ila1(.CONTROL(ila1_control),.CLK(clk1_i),.TRIG0(ila1_debug));
	surf5_big_ila u_big_ila(.CONTROL(big_ila_control),.CLK(clk_big_i),.TRIG0(clk_big_debug_i));
	surf5_vio u_vio(.CONTROL(vio_control),.CLK(wbc_clk_i),.SYNC_IN(vio_sync_out),.SYNC_OUT(vio_sync_in),
						 .ASYNC_OUT(global_debug_o));
						 	
endmodule
