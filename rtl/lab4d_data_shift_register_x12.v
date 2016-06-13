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
		input readout_rst_i,
		output done_o,
		output [143:0] dat_o,
		output dat_wr_o,
		input [3:0] prescale_i,
		output srclk_o,
		output ss_incr_o,
		output [6:0] sample_counter_o,
		output [3:0] bit_counter_o,
		input [11:0] DOE,
		output [11:0] SRCLK,
		output [11:0] SS_INCR
    );
	// Readout shift register in the LAB4D:
	// Basically it's just clock out 12 bits and have SS_INCR low in the last clock cycle.
	// However, there's a quirk: REGCLR, and presumedly power-on, both have the SS_INCR
	// negative-edge detection screwed up since it puts them both in the 0 state.
	// So we need to clock in SS_INCR at reset.

	//% Number of bits clocked out each sample.
	localparam [3:0] MAX_BITS = 12;
	//% Number of clocks output at reset.
	localparam [3:0] RESET_CLOCKS = 2;
	
	//% Current number of bits in sample.
	reg [3:0] bit_counter = {4{1'b0}};
	//% Current number of samples.
	reg [6:0] sample_counter = {7{1'b0}};
	//% Next counter value (used for terminal count detection)
	wire [7:0] sample_counter_plus_one = sample_counter + 1;

	//% The prescale counter slows down everything internal to this block.
	reg [3:0] prescale_counter = {4{1'b0}};

	//% Goes high when readout_i goes high, and then goes low when state exits IDLE.
	reg readout_request_seen = 0;
	
	//% Flag to capture the shift register data.
	reg dat_wr = 0;
	
	//% Indicates that the first bit is being loaded, which means we don't increment the bit counter.
	reg loading_first_bit = 0;
	
	//% Debug equal to the SRCLK output
	reg dbg_srclk = 0;
	//% Debug equal to the SS_INCR output.
	reg dbg_ss_incr = 1;
	
	localparam FSM_BITS=3;
	localparam [FSM_BITS-1:0] RESET_LOW = 0;
	localparam [FSM_BITS-1:0] RESET_HIGH = 1;
	localparam [FSM_BITS-1:0] RESET_LOW_EXIT = 2;
	localparam [FSM_BITS-1:0] IDLE = 3;
	localparam [FSM_BITS-1:0] SHIFT_HIGH = 4;
	localparam [FSM_BITS-1:0] SHIFT_LOW = 5;
	localparam [FSM_BITS-1:0] SHIFT_LOW_EXIT = 6;
	localparam [FSM_BITS-1:0] DONE = 7;
	//% FSM state.
	reg [FSM_BITS-1:0] state = RESET_LOW;
		
	//% Overall clock enable for this entire block.
	wire ce = (prescale_counter == prescale_i);

	//% Clock enable for SRCLK.
	wire srclk_ce = ce;
	//% Data input for SRCLK. Goes high entering RESET_HIGH or SHIFT_HIGH.
	wire srclk_d = (state == SHIFT_LOW) || (state == IDLE && readout_request_seen) || (state == RESET_LOW);
	
	//% Clock enable for SS_INCR.
	wire ss_incr_ce = (ce) && ((state == IDLE && readout_request_seen) || (state == SHIFT_LOW) || (state == SHIFT_LOW_EXIT));
	//% Data input for SS_INCR. Goes low at first entry to SHIFT_HIGH, and also at next-to-last bit entry into SHIFT_HIGH, unless last sample.
	wire ss_incr_d = !(state == IDLE && readout_request_seen) && !(state == SHIFT_LOW && bit_counter == MAX_BITS-2 && !sample_counter_plus_one[7]);
	
	//% Clock enable for the shift register.
	wire shreg_ce = ce && (state == SHIFT_LOW);

	always @(posedge sys_clk_i) begin	
		// Prescale counter runs all the time.
		if (readout_rst_i) prescale_counter <= {4{1'b0}};
		else if (prescale_counter == prescale_i) prescale_counter <= {4{1'b0}};
		else prescale_counter <= {4{1'b0}};		
	
		// Bit counter increments in SHIFT_LOW or RESET_HIGH.
		if (readout_rst_i) bit_counter <= {4{1'b0}};
		else if (state == IDLE || loading_first_bit) bit_counter <= {4{1'b0}};
		else if (ce) begin
			if (state == SHIFT_LOW) begin
				if (bit_counter == MAX_BITS-1) bit_counter <= {4{1'b0}};
				else bit_counter <= bit_counter + 1;
			end else if (state == RESET_HIGH) begin
				bit_counter <= bit_counter + 1;
			end
		end
		
		// Sample counter increments in SHIFT_LOW when bit_counter is at maximum.
		if (readout_rst_i) sample_counter <= {7{1'b0}};
		else if (state == IDLE || state == DONE) sample_counter <= {7{1'b0}};
		else if (state == SHIFT_LOW && bit_counter == MAX_BITS-1) sample_counter <= sample_counter_plus_one;

		// Readout request seen goes high when seeing readout_i, and cleared at SHIFT_HIGH.
		if (readout_rst_i) readout_request_seen <= 0;
		else if (readout_i) readout_request_seen <= 1;
		else if (state == SHIFT_HIGH) readout_request_seen <= 0;
		
		if (readout_rst_i) state <= RESET_LOW;
		else if (ce) begin
			case (state)
				// Move to RESET_HIGH
				RESET_LOW: state <= RESET_HIGH;
				// If we've counted the max number of clocks, move to exit. Otherwise keep going.
				RESET_HIGH: if (bit_counter == RESET_CLOCKS-1) state <= RESET_LOW_EXIT;
								else state <= RESET_LOW;
				// At exit, go to IDLE.
				RESET_LOW_EXIT: state <= IDLE;
				// At idle, if we're waiting for a readout_i, move to SHIFT_HIGH.
				// Our problem here is the first bit. 
				IDLE: if (readout_request_seen) state <= SHIFT_HIGH;
				// At SHIFT_HIGH, if we've clocked in 12 bits (bit_counter increments in SHIFT_LOW)
				// and we've clocked in 127 samples (127+1 sets sample_counter_plus_one[7]) move to SHIFT_LOW_EXIT.
				// Otherwise keep going.
				SHIFT_HIGH: if (bit_counter == MAX_BITS-1 && sample_counter_plus_one[7]) state <= SHIFT_LOW_EXIT;
								else state <= SHIFT_LOW;
				// Move to SHIFT_HIGH.
				SHIFT_LOW: state <= SHIFT_HIGH;
				// We're done, so exit.
				SHIFT_LOW_EXIT: state <= DONE;
				// Back to IDLE.
				DONE: state <= IDLE;
			endcase
		end
		
		if (ce) begin
			if (state == IDLE && readout_request_seen) loading_first_bit <= 1;
			else if (state == SHIFT_LOW) loading_first_bit <= 0;
		end
		
		// dat_wr can go upon entry to SHIFT_LOW when we've clocked in 12 bits.
		dat_wr <= ce && (state == SHIFT_HIGH) && (bit_counter == MAX_BITS-1);
		
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
			FDSE #(.INIT(1'b1)) u_ss_incr(.D(ss_incr_d),.CE(ss_incr_ce),.C(sys_clk_i),.S(readout_rst_i),.Q(SS_INCR[i]));
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b0)) u_srclk(.D(srclk_d),.CE(srclk_ce),.C(sys_clk_i),.R(1'b0),.Q(SRCLK[i]));
			always @(posedge sys_clk_i) begin : SHREG
				if (shreg_ce) data_shreg <= {shreg_msb,data_shreg[10:1]};
			end
			assign dat_o[12*i +: 12] = {shreg_msb, data_shreg};
		end
	endgenerate
	assign sample_counter_o = sample_counter;
	assign bit_counter_o = bit_counter;
	assign srclk_o = dbg_srclk;
	assign ss_incr_o = dbg_ss_incr;
	assign dat_wr_o = dat_wr;
	assign done_o = (state == DONE);
endmodule
