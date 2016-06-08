`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:51:41 06/06/2016
// Design Name:   lab4d_wilkinson_ramp
// Module Name:   C:/cygwin/home/barawn/repositories/github/firmware-surf5/sim//lab4d_wilkinson_ramp.v
// Project Name:  SURF5
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: lab4d_wilkinson_ramp
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module lab4d_wilkinson_ramp_tb;

	// Inputs
	reg clk_i;
	reg wclk_i;
	reg update_i;
	reg [15:0] ramp_to_wclk_i;
	reg [15:0] wclk_stop_count_i;
	reg do_ramp_i;

	// Outputs
	wire ramp_done_o;
	wire [11:0] RAMP;
	wire [11:0] WCLK_P;
	wire [11:0] WCLK_N;

	// Instantiate the Unit Under Test (UUT)
	lab4d_wilkinson_ramp uut (
		.clk_i(clk_i), 
		.wclk_i(wclk_i), 
		.update_i(update_i), 
		.ramp_to_wclk_i(ramp_to_wclk_i), 
		.wclk_stop_count_i(wclk_stop_count_i), 
		.do_ramp_i(do_ramp_i), 
		.ramp_done_o(ramp_done_o), 
		.RAMP(RAMP), 
		.WCLK_P(WCLK_P), 
		.WCLK_N(WCLK_N)
	);
	
	always begin
		#15 clk_i = ~clk_i;
	end
	always begin
		#2.5 wclk_i = ~wclk_i;
	end
	
	initial begin
		// Initialize Inputs
		clk_i = 0;
		wclk_i = 0;
		update_i = 0;
		ramp_to_wclk_i = 0;
		wclk_stop_count_i = 0;
		do_ramp_i = 0;

		// Wait 100 ns for global reset to finish
		#100;
      @(posedge clk_i);
		// Add stimulus here
		wclk_stop_count_i = 2047;
		update_i = 1;
		@(posedge clk_i);
		update_i = 0;
		@(posedge clk_i);
		@(posedge clk_i);
		do_ramp_i = 1;
		@(posedge clk_i);
		do_ramp_i = 0;
	end
      
endmodule

