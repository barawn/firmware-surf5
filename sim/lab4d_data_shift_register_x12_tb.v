`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   11:48:09 06/13/2016
// Design Name:   lab4d_data_shift_register_x12
// Module Name:   C:/cygwin/home/barawn/repositories/github/firmware-surf5/sim/lab4d_data_shift_register_x12.v
// Project Name:  SURF5
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: lab4d_data_shift_register_x12
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module lab4d_data_shift_register_x12_tb;

	// Inputs
	reg sys_clk_i;
	reg readout_i;
	reg readout_rst_i;
	reg [3:0] prescale_i;
	reg [11:0] DOE;

	// Outputs
	wire done_o;
	wire [143:0] dat_o;
	wire dat_wr_o;
	wire srclk_o;
	wire ss_incr_o;
	wire [6:0] sample_counter_o;
	wire [3:0] bit_counter_o;
	wire [11:0] SRCLK;
	wire [11:0] SS_INCR;

	// Instantiate the Unit Under Test (UUT)
	lab4d_data_shift_register_x12 uut (
		.sys_clk_i(sys_clk_i), 
		.readout_i(readout_i), 
		.readout_rst_i(readout_rst_i), 
		.done_o(done_o), 
		.dat_o(dat_o), 
		.dat_wr_o(dat_wr_o), 
		.prescale_i(prescale_i), 
		.srclk_o(srclk_o), 
		.ss_incr_o(ss_incr_o), 
		.sample_counter_o(sample_counter_o), 
		.bit_counter_o(bit_counter_o), 
		.DOE(DOE), 
		.SRCLK(SRCLK), 
		.SS_INCR(SS_INCR)
	);

	always begin
		#5 sys_clk_i <= ~sys_clk_i;
	end

	initial begin
		// Initialize Inputs
		sys_clk_i = 0;
		readout_i = 0;
		readout_rst_i = 0;
		prescale_i = 0;
		DOE = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
		@(posedge sys_clk_i);
		readout_rst_i = 1;
		@(posedge sys_clk_i);
		readout_rst_i = 0;
		#500;
		@(posedge sys_clk_i);
		readout_i = 1;
		@(posedge sys_clk_i);
		readout_i = 0;
	end
      
endmodule

