`include "wishbone.vh"
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:20:26 06/06/2016 
// Design Name: 
// Module Name:    lab4d_ram 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module lab4d_ram(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),
		input sys_clk_i,
		input readout_i,
		input [15:0] prescale_i,
		output complete_o,
		input [11:0] DOE_LVDS_P,
		input [11:0] DOE_LVDS_N,
		output [11:0] SS_INCR,
		output [11:0] SRCLK_P,
		output [11:0] SRCLK_N
    );

	reg wb_ack = 0;
	always @(posedge clk_i) begin
		wb_ack <= wb_cyc_i && wb_stb_i;
	end
	
	assign wb_ack_o = wb_ack;
endmodule
