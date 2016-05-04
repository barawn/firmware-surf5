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
		`WBM_NAMED_PORT(wbvio, 32, 20, 4),
		input [70:0] wbc_debug_i,
		input [70:0] ice_debug_i,
		input [70:0] lab4_i2c_debug_i,
		input [23:0] i2c_debug_i,
		input [70:0] rfp_debug_i,
		output [7:0] global_debug_o
    );

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
		if (ila0_sel[1:0] == 2'b01) ila0_debug <= rfp_debug_i;
		else if (ila0_sel[1:0] == 2'b11) begin
			ila0_debug[50:0] <= lab4_i2c_debug_i[50:0];
			ila0_debug[51] <= i2c_debug_i[0];
			ila0_debug[52] <= i2c_debug_i[12];
			ila0_debug[53] <= i2c_debug_i[1];
			ila0_debug[54] <= i2c_debug_i[13];
		end
		else ila0_debug <= wbc_debug_i;
	end
	always @(posedge clk1_i) begin
		ila1_debug <= ice_debug_i;
	end
	wire [35:0] vio_control;
	wire [35:0] ila0_control;
	wire [35:0] ila1_control;

	surf5_icon u_icon(.CONTROL0(vio_control),.CONTROL1(ila0_control),.CONTROL2(ila1_control));
	surf5_ila u_ila0(.CONTROL(ila0_control),.CLK(clk0_i),.TRIG0(ila0_debug));
	surf5_ila u_ila1(.CONTROL(ila1_control),.CLK(clk1_i),.TRIG0(ila1_debug));
	surf5_vio u_vio(.CONTROL(vio_control),.CLK(wbc_clk_i),.SYNC_IN(vio_sync_out),.SYNC_OUT(vio_sync_in),
						 .ASYNC_OUT(global_debug_o));
						 	
endmodule
