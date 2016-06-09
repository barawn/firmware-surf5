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
module lab4d_wilkinson_ramp(
		input clk_i,
		input wclk_i,
		input update_i,
		input [15:0] ramp_to_wclk_i,
		input [15:0] wclk_stop_count_i,
		input do_ramp_i,
		output ramp_done_o,
		output [11:0] RAMP,
		output [11:0] WCLK_P,
		output [11:0] WCLK_N,
		output dbg_ramp_o
    );

	parameter [15:0] RAMP_TO_WCLK_DEFAULT = {16{1'b0}};
	parameter [15:0] WCLK_STOP_COUNT_DEFAULT = 16'd4096;
	parameter [11:0] WCLK_POLARITY = 12'b010000000111;

	wire update_wclk;
	wire do_ramp_wclk;
	wire ramp_done_wclk;
	wire ramp_out_wclk;
	wire wilk_out_wclk;
	wire [11:0] wilk_to_obuf;
	reg [15:0] ramp_to_wclk_wclk = RAMP_TO_WCLK_DEFAULT;
	reg [15:0] wclk_stop_count_wclk = WCLK_STOP_COUNT_DEFAULT;
	reg [15:0] counter_wclk = {16{1'b0}};

	reg ramping = 0;

	flag_sync u_update_sync(.in_clkA(update_i),.clkA(clk_i),.out_clkB(update_wclk),.clkB(wclk_i));
	flag_sync u_do_ramp_sync(.in_clkA(do_ramp_i),.clkA(clk_i),.out_clkB(do_ramp_wclk),.clkB(wclk_i));
	flag_sync u_ramp_done_sync(.in_clkA(ramp_done_wclk),.clkA(wclk_i),.out_clkB(ramp_done_o),.clkB(clk_i));
	signal_sync u_dbg_ramp_sync(.in_clkA(ramping),.clkA(wclk_i),.out_clkB(dbg_ramp_o),.clkB(clk_i));
	
	localparam FSM_BITS = 2;
	localparam [FSM_BITS-1:0] IDLE 	 = 0;
	localparam [FSM_BITS-1:0] RAMPING = 1;
	localparam [FSM_BITS-1:0] WCLK 	 = 2;
	localparam [FSM_BITS-1:0] DONE    = 3;
	reg [FSM_BITS-1:0] state_wclk = IDLE;
	
	always @(posedge wclk_i) begin
		if (update_wclk) begin
			ramp_to_wclk_wclk <= ramp_to_wclk_i;
			wclk_stop_count_wclk <= wclk_stop_count_i;
		end
		case (state_wclk)
			IDLE: if (do_ramp_wclk) state_wclk <= RAMPING;
			RAMPING: if (counter_wclk == ramp_to_wclk_wclk) state_wclk <= WCLK;
			WCLK: if (counter_wclk == wclk_stop_count_wclk) state_wclk <= DONE;
			DONE: state_wclk <= IDLE;
		endcase
		if (state_wclk == RAMPING) begin
			if (counter_wclk == ramp_to_wclk_wclk) counter_wclk <= {16{1'b0}};
			else counter_wclk <= counter_wclk + 1;
		end else if (state_wclk == WCLK) begin
			if (counter_wclk == wclk_stop_count_wclk) counter_wclk <= {16{1'b0}};
			else counter_wclk <= counter_wclk + 1;
		end else begin
			counter_wclk <= {16{1'b0}};
		end
		
		ramping <= ramp_out_wclk;
	end
	assign ramp_out_wclk = (state_wclk == RAMPING || state_wclk == WCLK);
	assign wilk_out_wclk = (state_wclk == WCLK);
	assign ramp_done_wclk = (state_wclk == DONE);
	
	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : LOOP
			if (WCLK_POLARITY[i] == 0) begin : POS
				ODDR #(.INIT(1'b0)) u_ff_p(.D1(wilk_out_wclk),
												 .D2(1'b0),
												 .CE(1'b1),
												 .C(wclk_i),
												 .Q(wilk_to_obuf[i]),
												 .S(1'b0),.R(1'b0));
				OBUFDS u_obuf_p(.I(wilk_to_obuf[i]),.O(WCLK_P[i]),.OB(WCLK_N[i]));
			end else begin : NEG
				ODDR #(.INIT(1'b1)) u_ff_n(.D1(~wilk_out_wclk),
													.D2(1'b1),
													.CE(1'b1),
													.C(wclk_i),
													.Q(wilk_to_obuf[i]),
													.S(1'b0),.R(1'b0));
				OBUFDS u_obuf_n(.I(wilk_to_obuf[i]),.O(WCLK_N[i]),.OB(WCLK_P[i]));
			end
			(* IOB = "TRUE" *)
			FDRE u_rampfd(.D(ramp_out_wclk),.CE(1'b1),.C(wclk_i),.R(1'b0),.Q(RAMP[i]));
		end
	endgenerate
	
endmodule
