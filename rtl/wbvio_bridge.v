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
module wbvio_bridge(
		input clk_i,
		input rst_i,
		// VIO bridge.
		input [31:0] wbvio_dat_i,
		input [19:0] wbvio_adr_i,
		input wbvio_we_i,
		input wbvio_go_i,
		input wbvio_lock_i,
		output [31:0] wbvio_dat_o,
		output wbvio_done_o,
		output wbvio_err_o,
		// WISHBONE side.
		output [31:0] dat_o,
		output [19:0] adr_o,
		output cyc_o,
		output stb_o,
		output we_o,
		input [31:0] dat_i,
		input ack_i,
		input err_i,
		input rty_i
    );
	 
	 reg cyc = 0;
	 reg stb = 0;
	 reg we = 0;
	 reg [31:0] wb_dat_o = {32{1'b0}};
	 reg [31:0] wb_dat_i = {32{1'b0}};
	 reg [19:0] wb_adr_o = {20{1'b0}};
	 reg done = 0;
	 reg err = 0;
	 wire go_rising;
	 SYNCEDGE #(.LATENCY(0),.EDGE("RISING")) u_go_edge_det(.I(wbvio_go_i),.O(go_rising),.CLK(clk_i));

	 always @(posedge clk_i) begin
		// cyc is asserted if we are trying to hold the bus, or if we want to access the bus
		// in a single shot.
		// It is only cleared if an acknowledge comes, and we don't want to keep the bus,
		// or in a reset.
		if (rst_i || ((ack_i || err_i || rty_i) && !wbvio_lock_i)) cyc <= 0;
		else if (wbvio_lock_i || go_rising) cyc <= 1;
		
		// stb is asserted on a rising edge of wbvio_go_i.
		if (rst_i || ack_i || err_i || rty_i) stb <= 0;
		else if (go_rising) stb <= 1;
		
		// The data outputs is captured on go_rising.
		if (go_rising) begin
			wb_dat_o <= wbvio_dat_i;
			we <= wbvio_we_i;
			wb_adr_o <= wbvio_adr_i;
		end
		
		// Data inputs are captured on ack.
		if (ack_i) wb_dat_i <= dat_i;
		
		if (!wbvio_go_i) err <= 0;
		else if (err_i || rty_i) err <= 1;
		
		if (!wbvio_go_i) done <= 0;
		else if (ack_i) done <= 1;
	end

	assign dat_o = wb_dat_o;
	assign adr_o = wb_adr_o;
	assign cyc_o = cyc;
	assign stb_o = stb;
	assign we_o = we;
	
	assign wbvio_dat_o = wb_dat_i;
	assign wbvio_done_o = done;
	assign wbvio_err_o = err;

endmodule
