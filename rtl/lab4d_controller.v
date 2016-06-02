`timescale 1ns / 1ps
`include "wishbone.vh"
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
module lab4d_controller(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),
		input sys_clk_i,
		input sys_clk_div4_flag_i,
		input wclk_i,
		input trig_i,
		
		output readout_o,
		output [5:0] readout_address_o,
		output [3:0] prescale_o,
		input complete_i,
		
		output [11:0] SIN,
		output [11:0] SCLK,
		output [11:0] PCLK,
		output [11:0] REGCLR;
		output [11:0] RAMP,
		output [11:0] WCLK_P,
		output [11:0] WCLK_N,
		input [11:0] SHOUT,
		output [59:0] WR		
    );
	
	localparam [3:0] READOUT_PRESCALE_DEFAULT = 4'h0;
	localparam [7:0] SHIFT_PRESCALE_DEFAULT = 8'h00;
	localparam [15:0] RAMP_TO_WILKINSON_DEFAULT = 16'h0000;
	localparam [15:0] WCLK_STOP_COUNT_DEFAULT = 16'd4096;

	// registers:
	// 0: resets/control/runmode
	// 1: shift register prescale
	// 2: readout prescale
	// 3: ramp to wilkinson delay
	// 4: wilkinson max count
	// 5: test pattern control
	// 6: LAB4 serial write
	// 7: LAB4_0 SHOUT readback
	// 8: LAB4_1 SHOUT readback
	// 9: LAB4_2 SHOUT readback
	// 10: LAB4_3 SHOUT readback
	// 11: LAB4_4 SHOUT readback
	// 12: LAB4_5 SHOUT readback
	// 13: LAB4_6 SHOUT readback
	// 14: LAB4_7 SHOUT readback
	// 15: LAB4_8 SHOUT readback
	// 16: LAB4_9 SHOUT readback
	// 17: LAB4_10 SHOUT readback
	// 18: LAB4_11 SHOUT readback
	// 19: current buffer
	// 20: last trigger buffer
	// 21: trigger control
	// 22-29: reserved
	// 30: picoblaze control
	// 31: picoblaze bram
	
	wire [31:0] register_mux[31:0];
	assign wb_dat_o = register_mux[wb_adr_i[4:0]];

	reg lab4_control_reset_request = 0;
	reg lab4_runmode_request = 0;
	reg lab4_runmode = 0;
	wire [31:0] lab4_control_register = {{29{1'b0}},lab4_runmode,lab4_runmode_request,lab4_reset_request};

	reg [3:0] readout_prescale = READOUT_PRESCALE_DEFAULT;
	wire [31:0] readout_prescale_register = {{28{1'b0}},readout_prescale};

	reg [7:0] shift_prescale = SHIFT_PRESCALE_DEFAULT;
	wire [31:0] shift_prescale_register = {{24{1'b0}},shift_prescale_register};

	reg [15:0] ramp_to_wilkinson = RAMP_TO_WILKINSON_DEFAULT;
	wire [31:0] ramp_to_wilkinson_register = {{16{1'b0}},ramp_to_wilkinson};

	reg [15:0] wclk_stop_count = WCLK_STOP_COUNT_DEFAULT;
	wire [31:0] wclk_stop_count_register = {{16{1'b0}},wclk_stop_count};
	
	reg do_ramp = 0;
	wire ramp_done;
	
	reg [23:0] lab4_user_write = {24{1'b0}};
	reg [3:0] lab4_user_select = {4{1'b0}};
	reg lab4_user_write_busy = 0;
	reg lab4_user_write_request = 0;
	wire [31:0] lab4_user_write_register = 
		{lab4_user_write_busy,lab4_user_write_request,2'b00,lab4_user_select,lab4_user_write};

	reg [23:0] lab4_serial_register = {24{1'b0}};
	reg lab4_serial_busy = 0;
	reg [3:0] lab4_serial_select = {4{1'b0}};
	reg [4:0] lab4_serial_counter = {5{1'b0}};
	
	reg [11:0] test_pattern_data = {12{1'b0}};
	reg test_pattern_request = 0;
	reg test_pattern_busy = 0;
	wire [31:0] test_pattern_register = {test_pattern_busy,test_pattern_request,{14{1'b0}},{4{1'b0}},test_pattern_data};

	wire trigger_busy;
	wire [5:0] trigger_address;
	wire trigger_clear;
	reg force_trigger = 0;
	
	
	wire [7:0] pb_outport;
	wire [7:0] pb_port;
	wire [7:0] pb_inport[31:0];
	wire [7:0] pb_inport_mux = pb_inport[pb_port[4:0]];
	// PicoBlaze ports
	// 00-03 : runmode/reset
	// 04/06 : test pattern control 0
	// 05/07 : test pattern control 1
	// 08/0C : user write 0
	// 09/0D : user write 1
	// 0A/0E : user write 2
	// 0B/0F : user write 3
	// 10    : serial     0
	// 11    : serial     1
	// 12    : serial     2
	// 13    : serial     3
	// 14    : trigger    0
	// 15    : trigger    1
	// 16    : readout
	// 17    : ramp
	assign pb_inport[0] = lab4_control_register[2:0];
	assign pb_inport[1] = lab4_control_register[2:0];
	assign pb_inport[2] = lab4_control_register[2:0];
	assign pb_inport[3] = lab4_control_register[2:0];
	assign pb_inport[4] = test_pattern_register[7:0];
	assign pb_inport[6] = test_pattern_register[7:0];
	assign pb_inport[5] = {1'b0,test_pattern_request,2'b00,test_pattern_register[11:8]};
	assign pb_inport[7] = {1'b0,test_pattern_request,2'b00,test_pattern_register[11:8]};
	assign pb_inport[8] = lab4_user_write_register[7:0];
	assign pb_inport[12] = lab4_user_write_register[7:0];
	assign pb_inport[9] = lab4_user_write_register[15:8];
	assign pb_inport[13] = lab4_user_write_register[15:8];
	assign pb_inport[10] = lab4_user_write_register[23:16];
	assign pb_inport[14] = lab4_user_write_register[23:16];
	assign pb_inport[11] = lab4_user_write_register[31:24];
	assign pb_inport[15] = lab4_user_write_register[31:24];
	assign pb_inport[16] = lab4_serial_register[7:0];
	assign pb_inport[17] = lab4_serial_register[15:8];
	assign pb_inport[18] = lab4_serial_register[23:16];
	assign pb_inport[19] = {lab4_serial_busy,3'b000,lab4_serial_select};
	
endmodule
