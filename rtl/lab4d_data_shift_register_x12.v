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
		output srclk_o,
		output ss_incr_o,
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
	
	reg dbg_srclk = 0;
	reg dbg_ss_incr = 0;
	
	localparam FSM_BITS=3;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] LOAD_LOW = 1;
	localparam [FSM_BITS-1:0] LOAD_HIGH = 2;
	localparam [FSM_BITS-1:0] SHIFT_LOW = 3;
	localparam [FSM_BITS-1:0] SHIFT_HIGH = 4;
	localparam [FSM_BITS-1:0] DONE = 4;
	reg [FSM_BITS-1:0] state = IDLE;
		
	// SRCLK needs to run the entire transfer.
	// We keep SS_INCR high most of the time, and then drop it at the beginning of each readout.
	// So we go IDLE, LOAD_HIGH, LOAD_LOW, SHIFT_HIGH, SHIFT_LOW, SHIFT_HIGH, SHIFT_LOW, etc.
	// clk		state			ss_incr		sr_clk	data at lab4		Q		bit_count		sample_count	shreg_ce
	// 0			IDLE			1				0			X						X		0					0					0
	//	1			LOAD_HIGH	0				1			X						X		0					0					0
	// 2			LOAD_LOW		0				0			X						X		0					0					0
	// 3			SHIFT_HIGH  1				1			A0						X		0					0					0
	// 4			SHIFT_LOW	1				0			A0						A0		0					0					1
	// 5			SHIFT_HIGH	1				1			A1						A0		1					0					0
	// 6			SHIFT_LOW	1				0			A1						A1		1					0					1
	// .... etc. At the end of the transfer we want to do
	// n        SHIFT_HIGH	1				1			A10					A9	   10					0					0
	// n+1		SHIFT_LOW	1				0			A10					A10	10					0					1
	// n+2		SHIFT_HIGH	0				1			A11					A10  	11					0					0
	// n+3		SHIFT_LOW	0				0			A11					A11	11					0					1
	// n+4		SHIFT_HIGH	1				1			B0						A11	0					1
	// n+5		SHIFT_LOW	1				0			B0						B0		0					1
	// So our stop occurs when the last sample is read out (when sample_count_plus_one[7] = 1, bit_count == 11, and state == SHIFT_LOW)

	// SRCLK goes HIGH entering LOAD_HIGH (state == IDLE && readout_i)
	// SRCLK goes HIGH entering SHIFT_HIGH (prescale_counter == prescale_i) && (state == LOAD_LOW || state == SHIFT_LOW && !(bit_count == 11 && sample_count_plus_one[7]))
	// SRCLK goes LOW entering SHIFT_LOW or LOAD_LOW (prescale_counter == prescale_i && (state == LOAD_HIGH || state == SHIFT_HIGH))	
	wire srclk_ce = (prescale_counter == prescale_i) || (state == IDLE && readout_i);
	wire srclk_d = (state == SHIFT_LOW && !((bit_counter == 11) && sample_counter_plus_one[7])) || (state == IDLE && readout_i) || (state == LOAD_LOW);
	// SS_INCR goes low entering LOAD_HIGH (state == IDLE && readout_i)
	// SS_INCR goes low entering SHIFT_HIGH when bit_counter == 10
	// (prescale_counter == prescale_i) && (state == SHIFT_LOW) && (bit_counter == 10)
	// SS_INCR goes high exiting LOAD_LOW (prescale_counter == prescale_i) && (state == LOAD_LOW)
	// We can just make SS_INCR's CE be
	wire ss_incr_ce = (state == IDLE && readout_i) || (prescale_counter == prescale_i && (state == LOAD_LOW || state == SHIFT_LOW));
	wire ss_incr_d = !(state == IDLE && readout_i) && !(state == SHIFT_LOW && bit_counter == 10);
	// Shreg gets clocked in SHIFT_LOW.
	wire shreg_ce = (state == SHIFT_LOW && prescale_counter == prescale_i);

	always @(posedge sys_clk_i) begin	
		if (prescale_counter == prescale_i) prescale_counter <= {4{1'b0}};
		else if (state != IDLE && state != DONE) prescale_counter <= prescale_counter + 1;
		else prescale_counter <= {4{1'b0}};
	
		if (state == IDLE) bit_counter <= {4{1'b0}};
		else if (state == SHIFT_LOW) begin
			if (bit_counter == 11) bit_counter <= {4{1'b0}};
			else bit_counter <= bit_counter + 1;
		end
		
		if (state == IDLE || state == DONE) sample_counter <= {7{1'b0}};
		else if (state == SHIFT_LOW && bit_counter == 11) sample_counter <= sample_counter_plus_one;

		case (state)
			IDLE: if (readout_i) state <= LOAD_HIGH;
			LOAD_HIGH: if (prescale_counter == prescale_i) state <= LOAD_LOW;
			LOAD_LOW: if (prescale_counter == prescale_i) state <= SHIFT_HIGH;
			SHIFT_HIGH: if (prescale_counter == prescale_i) state <= SHIFT_LOW;
			SHIFT_LOW: if (prescale_counter == prescale_i) begin
				if (bit_counter == 11 && sample_counter_plus_one[7]) state <= DONE;
				else state <= SHIFT_HIGH;
			end
			DONE: state <= IDLE;
		endcase
		
		// dat_wr can go upon entry to SHIFT_LOW.
		if (state == SHIFT_HIGH && prescale_counter == prescale_i && bit_counter == 11) dat_wr <= 1;
		else dat_wr <= 0;
		
		if (srclk_ce) dbg_srclk <= srclk_d;
		if (ss_incr_ce) dbg_ss_incr <= ss_incr_d;
	end

	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : LAB
			wire shreg_msb;
			reg [10:0] data_shreg = {11{1'b0}};
			// Always have DOE clock in constantly. It just only gets added to the shreg occasionally.
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b0)) u_shreg(.D(DOE[i]),.CE(1'b1),.C(sys_clk_i),.R(1'b0),.Q(shreg_msb));
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
	assign srclk_o = dbg_srclk;
	assign ss_incr_o = dbg_ss_incr;
	assign dat_wr_o = dat_wr;
	assign done_o = (state == DONE);
endmodule
