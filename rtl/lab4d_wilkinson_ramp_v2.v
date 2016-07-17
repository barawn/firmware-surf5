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
module lab4d_wilkinson_ramp_v2(
		input clk_i,
		input sys_clk_i,
		input wclk_i,
		input rst_i,
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
	parameter [15:0] WCLK_STOP_COUNT_DEFAULT = 16'd1024;
	parameter [11:0] WCLK_POLARITY = 12'b010000000111;

	wire update_sysclk;
	wire do_ramp_sysclk;
	wire ramp_done_sysclk;
	wire ramp_out_sysclk;
	wire wilk_out_sysclk;
	wire [11:0] wilk_to_obuf;
	reg [15:0] ramp_to_wclk_sysclk = RAMP_TO_WCLK_DEFAULT;
	reg [15:0] wclk_stop_count_sysclk = WCLK_STOP_COUNT_DEFAULT;
	reg [15:0] counter_sysclk = {16{1'b0}};

	wire reset_sysclk;
	
	reg ramping = 0;

	flag_sync u_reset_sync(.in_clkA(rst_i),.clkA(clk_i),.out_clkB(reset_sysclk),.clkB(sys_clk_i));
	flag_sync u_update_sync(.in_clkA(update_i),.clkA(clk_i),.out_clkB(update_sysclk),.clkB(sys_clk_i));
	flag_sync u_do_ramp_sync(.in_clkA(do_ramp_i),.clkA(clk_i),.out_clkB(do_ramp_sysclk),.clkB(sys_clk_i));
	flag_sync u_ramp_done_sync(.in_clkA(ramp_done_sysclk),.clkA(sys_clk_i),.out_clkB(ramp_done_o),.clkB(clk_i));
	signal_sync u_dbg_ramp_sync(.in_clkA(ramping),.clkA(sys_clk_i),.out_clkB(dbg_ramp_o),.clkB(clk_i));
	
	localparam FSM_BITS = 2;
	localparam [FSM_BITS-1:0] IDLE 	 = 0;
	localparam [FSM_BITS-1:0] RAMPING = 1;
	localparam [FSM_BITS-1:0] WCLK 	 = 2;
	localparam [FSM_BITS-1:0] DONE    = 3;
	reg [FSM_BITS-1:0] state_sysclk = IDLE;
	
	always @(posedge sys_clk_i) begin
		if (update_sysclk) begin
			ramp_to_wclk_sysclk <= ramp_to_wclk_i;
			wclk_stop_count_sysclk <= wclk_stop_count_i;
		end
		case (state_sysclk)
			IDLE: if (do_ramp_sysclk) state_sysclk <= RAMPING;
			RAMPING: if (counter_sysclk == ramp_to_wclk_sysclk) state_sysclk <= WCLK;
			WCLK: if (counter_sysclk == wclk_stop_count_sysclk) state_sysclk <= DONE;
			DONE: state_sysclk <= IDLE;
		endcase
		if (state_sysclk == RAMPING) begin
			if (counter_sysclk == ramp_to_wclk_sysclk) counter_sysclk <= {16{1'b0}};
			else counter_sysclk <= counter_sysclk + 1;
		end else if (state_sysclk == WCLK) begin
			if (counter_sysclk == wclk_stop_count_sysclk) counter_sysclk <= {16{1'b0}};
			else counter_sysclk <= counter_sysclk + 1;
		end else begin
			counter_sysclk <= {16{1'b0}};
		end		
		ramping <= ramp_out_sysclk;
	end
	assign ramp_out_sysclk = (state_sysclk == RAMPING || state_sysclk == WCLK);
	assign wilk_out_sysclk = (state_sysclk == WCLK);
	assign ramp_done_sysclk = (state_sysclk == DONE);
	
	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : LOOP
			reg [7:0] reset_delay = {8{1'b0}};
			reg oserdes_reset = 1;
			(* EQUIVALENT_REGISTER_REMOVAL = "FALSE" *)
			(* KEEP = "TRUE" *)
			reg ce_sysclk = 0;
			(* EQUIVALENT_REGISTER_REMOVAL = "FALSE" *)
			(* KEEP = "TRUE" *)
			reg ce_sysclk_to_wclk = 0;
			(* EQUIVALENT_REGISTER_REMOVAL = "FALSE" *)
			(* KEEP = "TRUE" *)
			reg ce_wclk = 0;
			always @(posedge sys_clk_i) begin : RESET
				if (reset_delay[2]) oserdes_reset <= 0;
				else if (reset_sysclk) oserdes_reset <= 1;
				reset_delay <= {reset_delay[6:0],reset_sysclk};
				if (oserdes_reset) ce_sysclk <= 0;
				else if (reset_delay[7]) ce_sysclk <= 1;
				
				ce_sysclk_to_wclk <= ce_sysclk;
			end
			always @(posedge wclk_i) begin : CE
				ce_wclk <= ce_sysclk_to_wclk;
			end
			if (WCLK_POLARITY[i] == 0) begin : POS
				OSERDESE2 #(.DATA_RATE_OQ("DDR"),
								.DATA_RATE_TQ("SDR"),
								.DATA_WIDTH(4),
								.INIT_OQ(1'b0),
								.SRVAL_OQ(1'b0),
								.TRISTATE_WIDTH(1))
					u_oserdes_p(.D1(0),.D2(wilk_out_sysclk),.D3(0),.D4(wilk_out_sysclk),.D5(0),.D6(wilk_out_sysclk),.D7(0),.D8(wilk_out_sysclk),
									.OQ(wilk_to_obuf[i]),
									.CLK(wclk_i),.CLKDIV(sys_clk_i),.RST(oserdes_reset),.OCE(ce_wclk));
				OBUFDS u_obuf_p(.I(wilk_to_obuf[i]),.O(WCLK_P[i]),.OB(WCLK_N[i]));
			end else begin : NEG
				OSERDESE2 #(.DATA_RATE_OQ("DDR"),
								.DATA_RATE_TQ("SDR"),
								.DATA_WIDTH(4),
								.INIT_OQ(1'b1),
								.SRVAL_OQ(1'b1),
								.TRISTATE_WIDTH(1))
					u_oserdes_n(.D1(1),.D2(!wilk_out_sysclk),.D3(1),.D4(!wilk_out_sysclk),.D5(1),.D6(!wilk_out_sysclk),.D7(1),.D8(!wilk_out_sysclk),
									.OQ(wilk_to_obuf[i]),
									.CLK(wclk_i),.CLKDIV(sys_clk_i),.RST(oserdes_reset),.OCE(ce_wclk));
				OBUFDS u_obuf_n(.I(wilk_to_obuf[i]),.O(WCLK_N[i]),.OB(WCLK_P[i]));
			end
			(* IOB = "TRUE" *)
			FDRE u_rampfd(.D(ramp_out_sysclk),.CE(1'b1),.C(sys_clk_i),.R(1'b0),.Q(RAMP[i]));
		end
	endgenerate
	
endmodule
