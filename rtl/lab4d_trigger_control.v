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
module lab4d_trigger_control(
		input clk_i,
		input sys_clk_i,
		input sys_clk_div4_flag_i,
		input enable_i,
		output ready_o,
		input trigger_i,
		input force_trigger_i,
		output trigger_busy_o,
		output [5:0] trigger_address_o,
		input trigger_clear_i,
		output [59:0] WR
    );

	reg [1:0] buffer_wr_ptr = {2{1'b0}};
	wire [1:0] next_buffer_wr_ptr = buffer_wr_ptr + 1;
	reg [1:0] buffer_rd_ptr = {2{1'b0}};
	wire enable_next_buffer;
	reg [2:0] bank = {3{1'b0}};
	wire [3:0] bank_plus_one = bank + 1;
	reg [2:0] buffer_bank[3:0];
	
	wire enable_sysclk;
	wire force_trigger_sysclk;	
	reg enabled_sysclk = 0;
	wire trigger_clear_sysclk;
	reg [1:0] trigger_is_busy_sysclk = 0;
	reg trigger_flag_sysclk = 0;
	wire trigger_flag_clk;
	reg trigger_ready_clk = 0;
	reg [5:0] trigger_address_clk = {6{1'b0}};
	
	signal_sync u_enable_sync(.in_clkA(enable_i),.clkA(clk_i),.out_clkB(enable_sysclk),.clkB(sys_clk_i));
	signal_sync u_enabled_sync(.in_clkA(enabled_sysclk),.clkA(sys_clk_i),.out_clkB(ready_o),.clkB(clk_i));
	flag_sync u_force_sync(.in_clkA(force_trigger_i),.clkA(clk_i),.out_clkB(force_trigger_sysclk),.clkB(sys_clk_i));
	flag_sync u_clear_sync(.in_clkA(trigger_clear_i),.clkA(clk_i),.out_clkB(trigger_clear_sysclk),.clkB(sys_clk_i));
	flag_sync u_trigger_flag_sync(.in_clkA(trigger_flag_sysclk),.clkA(sys_clk_i),.out_clkB(trigger_flag_clk),.clkB(clk_i));
	
	initial begin
		buffer_bank[0] <= {3{1'b0}};
		buffer_bank[1] <= {3{1'b0}};
		buffer_bank[2] <= {3{1'b0}};
		buffer_bank[3] <= {3{1'b0}};
	end
	assign enable_next_buffer = (trigger_i || force_trigger_sysclk) && !(next_buffer_wr_ptr == buffer_rd_ptr);
	always @(posedge sys_clk_i) begin
		enabled_sysclk <= enable_sysclk;

		if (enabled_sysclk) begin
			if (sys_clk_div4_flag_i) bank <= bank_plus_one[2:0];
		end
			else bank <= {3{1'b0}};
		
		if (enable_next_buffer) begin
			buffer_bank[buffer_wr_ptr] <= bank;
			buffer_wr_ptr <= buffer_wr_ptr + 1;
		end else if (!enabled_sysclk) begin
			buffer_wr_ptr <= 2'b00;
		end
		if (trigger_clear_sysclk && (buffer_rd_ptr != buffer_wr_ptr)) begin
			buffer_rd_ptr <= buffer_rd_ptr + 1;
		end else if (!enabled_sysclk) begin
			buffer_rd_ptr <= 2'b00;
		end
		
		trigger_is_busy_sysclk <= {trigger_is_busy_sysclk[0],(buffer_rd_ptr != buffer_wr_ptr) && !trigger_clear_sysclk};
		trigger_flag_sysclk <= {trigger_is_busy_sysclk[0] && !trigger_is_busy_sysclk[1]};
	end
	always @(posedge clk_i) begin
		if (trigger_flag_clk) trigger_address_clk <= {buffer_rd_ptr,buffer_bank[buffer_rd_ptr]};

		if (trigger_clear_i || !enable_i) trigger_ready_clk <= 0;
		else if (trigger_flag_clk) trigger_ready_clk <= 1;
	end
	
	generate
		genvar i,j;
		for (i=0;i<12;i=i+1) begin : LAB
			(* IOB = "TRUE" *)
			FDRE u_wr4(.D(next_buffer_wr_ptr[1]),
						  .CE(enable_next_buffer),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk),
						  .Q(WR[5*i+4]));
			FDRE u_wr3(.D(next_buffer_wr_ptr[0]),
						  .CE(enable_next_buffer),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk),
						  .Q(WR[5*i+3]));
			for (j=0;j<3;j=j+1) begin : BIT
				(* IOB = "TRUE" *)
				FDRE u_wr(.D(bank_plus_one[j]),
							 .CE(enabled_sysclk),
							 .C(sys_clk_i),
							 .R(!enabled_sysclk),
							 .Q(WR[5*i+j]));
			end
		end
	endgenerate

	assign trigger_address_o = trigger_address_clk;
	assign trigger_busy_o = trigger_ready_clk;
	
endmodule
