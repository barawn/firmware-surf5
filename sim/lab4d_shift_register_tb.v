`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:33:45 06/06/2016
// Design Name:   lab4d_shift_register
// Module Name:   C:/cygwin/home/barawn/repositories/github/firmware-surf5/sim//lab4d_shift_register_tb.v
// Project Name:  SURF5
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: lab4d_shift_register
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module lab4d_shift_register_tb;

	// Inputs
	reg clk_i;
	reg go_i;
	reg [23:0] dat_i;
	reg [3:0] sel_i;
	reg [7:0] prescale_i;

	// Outputs
	wire busy_o;
	wire [11:0] SIN;
	wire [11:0] SCLK;
	wire [11:0] PCLK;

	// Instantiate the Unit Under Test (UUT)
	lab4d_shift_register uut (
		.clk_i(clk_i), 
		.go_i(go_i), 
		.dat_i(dat_i), 
		.sel_i(sel_i), 
		.prescale_i(prescale_i), 
		.busy_o(busy_o), 
		.SIN(SIN), 
		.SCLK(SCLK), 
		.PCLK(PCLK)
	);

	always begin
		#15 clk_i = ~clk_i;
	end

	initial begin
		// Initialize Inputs
		clk_i = 0;
		go_i = 0;
		dat_i = 0;
		sel_i = 0;
		prescale_i = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
		@(posedge clk_i);
		prescale_i = 10;
		@(posedge clk_i);
		#100;
		@(posedge clk_i);
		dat_i = 24'h123456;
		sel_i = 0;
		go_i = 1;
		@(posedge clk_i);
		go_i = 0;
	end
      
endmodule

