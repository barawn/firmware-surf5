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
module lab4d_data_shift_register_x12(
		input sys_clk_i,
		input readout_i,
		output done_o,
		output [143:0] dat_o,
		output dat_wr_o,
		input [3:0] prescale_i,

		input [11:0] DOE,
		output [11:0] SRCLK,
		output [11:0] SS_INCR
    );
	reg [3:0] bit_counter = {4{1'b0}};
	reg [6:0] sample_counter = {7{1'b0}};
	wire [7:0] sample_counter_plus_one = sample_counter + 1;
	reg [3:0] prescale_counter = {4{1'b0}};
	reg srclk = 0;
	reg dat_wr = 0;
	
	localparam FSM_BITS=3;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] SHIFT_LOW = 1;
	localparam [FSM_BITS-1:0] SHIFT_HIGH = 2;
	localparam [FSM_BITS-1:0] INCREMENT = 3;
	localparam [FSM_BITS-1:0] DONE = 4;
	reg [FSM_BITS-1:0] state = IDLE;
	
	wire srclk_ce = (prescale_counter == prescale_i);
	wire srclk_d = (state == SHIFT_LOW && (bit_counter != 11));
	// SS_INCR goes low entering SHIFT_LOW from INCREMENT or IDLE and goes high entering INCREMENT.
	wire ss_incr_ce = (state == IDLE && readout_i) || (state == SHIFT_LOW && prescale_counter == prescale_i && bit_counter == 11)
							|| (state == INCREMENT && prescale_counter == prescale_i && !sample_counter_plus_one[7]);
	wire ss_incr_d = (state == SHIFT_LOW);
	
	wire shreg_ce = (state == SHIFT_LOW && prescale_counter == prescale_i);
	
	// SS_INCR fall (SHIFT_LOW) 	 -> clock bit 0  : bit_counter 0->1
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 1  : bit_counter 1->2
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 2  : bit_counter 2->3
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 3  : bit_counter 3->4
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 4  : bit_counter 4->5
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 5  : bit_counter 5->6
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 6  : bit_counter 6->7
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 7  : bit_counter 7->8
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 8  : bit_counter 8->9
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 9  : bit_counter 9->10
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 10 : bit_counter 10->11
	// SHIFT_HIGH, SHIFT_LOW       -> clock bit 11 : bit_counter=11, exit
	// INCREMENT, then back to SS_INCR falling edge
	
	always @(posedge sys_clk_i) begin
		if (state == INCREMENT || state == DONE) bit_counter <= {4{1'b0}};
		else if (shreg_ce) bit_counter <= bit_counter + 1;

		if (state == INCREMENT && prescale_counter == prescale_i) sample_counter <= sample_counter_plus_one;
		else if (state == DONE) sample_counter <= {7{1'b0}};

		case (state)
			IDLE: if (readout_i) state <= SHIFT_LOW;
			SHIFT_LOW: if (prescale_counter == prescale_i) begin
				if (bit_counter == 11) state <= INCREMENT;
				else state <= SHIFT_HIGH;
			end
			SHIFT_HIGH: if (prescale_counter == prescale_i) state <= SHIFT_LOW;
			INCREMENT: if (prescale_counter == prescale_i) begin
				if (sample_counter_plus_one[7]) state <= DONE;
				else state <= SHIFT_LOW;
			end
			DONE: state <= IDLE;
		endcase
		
		if (state == INCREMENT && prescale_counter == prescale_i) dat_wr <= 1;
		else dat_wr <= 0;		
	end

	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : LAB
			wire shreg_msb;
			reg [10:0] data_shreg = {11{1'b0}};
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b0)) u_shreg(.D(DOE[i]),.CE(shreg_ce),.C(sys_clk_i),.R(1'b0),.Q(shreg_msb));
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b1)) u_ss_incr(.D(ss_incr_d),.CE(ss_incr_ce),.C(sys_clk_i),.R(1'b0),.Q(SS_INCR[i]));
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b0)) u_srclk(.D(srclk_d),.CE(srclk_ce),.C(sys_clk_i),.R(1'b0),.Q(SRCLK[i]));
			always @(posedge sys_clk_i) begin : SHREG
				if (shreg_ce) data_shreg <= {shreg_msb,data_shreg[10:1]};
			end
			assign dat_o[12*i +: 12] = {shreg_msb, data_shreg};
		end
	endgenerate

	assign dat_wr_o = dat_wr;

endmodule
