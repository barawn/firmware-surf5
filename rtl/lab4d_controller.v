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
		output [11:0] REGCLR,
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
	// 22: readout
	// 30: picoblaze control
	// 31: picoblaze bram
	
	wire [31:0] register_mux[31:0];
	assign wb_dat_o = register_mux[wb_adr_i[4:0]];

        reg ack = 0;

	reg lab4_control_reset_request = 0;
	reg lab4_runmode_request = 0;
	reg lab4_runmode = 0;
	wire lab4_running;
	wire [31:0] lab4_control_register = {{28{1'b0}},lab4_running,lab4_runmode,lab4_runmode_request,lab4_control_reset_request};

	reg [3:0] readout_prescale = READOUT_PRESCALE_DEFAULT;
	wire [31:0] readout_prescale_register = {{28{1'b0}},readout_prescale};

	reg [7:0] shift_prescale = SHIFT_PRESCALE_DEFAULT;
	wire [31:0] shift_prescale_register = {{24{1'b0}},shift_prescale_register};

	reg update_wilkinson = 0;

	reg [15:0] ramp_to_wilkinson = RAMP_TO_WILKINSON_DEFAULT;
	wire [31:0] ramp_to_wilkinson_register = {{16{1'b0}},ramp_to_wilkinson};

	reg [15:0] wclk_stop_count = WCLK_STOP_COUNT_DEFAULT;
	wire [31:0] wclk_stop_count_register = {{16{1'b0}},wclk_stop_count};
	
	reg do_ramp = 0;
	reg ramp_pending = 0;
	wire ramp_done;
	
	// user-side serial register. PicoBlaze interacts with
	// it and takes it over.
	reg [23:0] lab4_user_write = {24{1'b0}};
	reg [3:0] lab4_user_select = {4{1'b0}};
	reg lab4_user_write_request = 0;
	wire [31:0] lab4_user_write_register = 
		{lab4_user_write_request,3'b000,lab4_user_select,lab4_user_write};

	// actual serial register
	reg [23:0] lab4_serial_register = {24{1'b0}};
	wire lab4_serial_busy;
	reg [3:0] lab4_serial_select = {4{1'b0}};
	wire lab4_serial_go;
	
	// test pattern interface
	reg [11:0] test_pattern_data = {12{1'b0}};
	reg test_pattern_request = 0;
	wire [31:0] test_pattern_register = {test_pattern_request,{15{1'b0}},{4{1'b0}},test_pattern_data};

	// this trigger stuff sucks, we need to fix it
	wire trigger_busy;
	wire [5:0] trigger_address;
	reg trigger_clear = 0;
	reg force_trigger = 0;
	wire [31:0] trigger_register = {{25{1'b0}},trigger_busy,trigger_address};
	
	reg readout_pending = 0;
	reg readout_done = 0;
	wire [31:0] readout_register = {{29{1'b0}},readout_done,readout_pending};

	//% Holds PicoBlaze in reset.
	reg processor_reset = 0;
	//% Enables writes to BRAM.
	reg bram_we_enable = 0;
	//% Address register for BRAM.
	reg [9:0] bram_address_reg = {10{1'b0}};
	//% Data register for BRAM.
	reg [17:0] bram_data_reg = {18{1'b0}};
	//% Write flag to BRAM.
	reg bram_we = 0;
	//% Readback data from BRAM.
	wire [17:0] bram_readback;
	//% Outbound data to userside.
	assign pb_bram_data = {processor_reset,bram_we_enable,{2{1'b0}},bram_address_reg,bram_readback};
	
	//% PicoBlaze instruction bus.
	wire [17:0] pbInstruction;
	//% PicoBlaze address bus.
	wire [11:0] pbAddress;
	//% PicoBlaze ROM (well, ROM from PicoBlaze at least) read enable.
	wire pbRomEnable;
	//% PicoBlaze port specifier.
	wire [7:0] pb_port;
	//% PicoBlaze output port data.
	wire [7:0] pb_outport;
	//% PicoBlaze input port data.
	wire [7:0] pb_inport[31:0];
	//% Multiplexed PicoBlaze input port data.
	wire [7:0] pb_inport_mux = pb_inport[pb_port[4:0]];
	//% PicoBlaze write flag.
	wire pb_write;
	//% PicoBlaze read flag.
	wire pb_read;
	
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
	assign pb_inport[20] = trigger_register[7:0];
	assign pb_inport[21] = {8{1'b0}};
	assign pb_inport[22] = {readout_pending,{7{1'b0}}};
	assign pb_inport[23] = {ramp_pending,{7{1'b0}}};
	assign pb_inport[24] = pb_inport[16];
	assign pb_inport[25] = pb_inport[17];
	assign pb_inport[26] = pb_inport[18];
	assign pb_inport[27] = pb_inport[19];
	assign pb_inport[28] = pb_inport[20];
	assign pb_inport[29] = pb_inport[21];
	assign pb_inport[30] = pb_inport[22];
	assign pb_inport[31] = pb_inport[23];
	
	assign lab4_serial_go = (pb_port[4:0] == 19) && pb_write && pb_port[6];
	
	always @(posedge clk_i) begin
		if (ramp_done) ramp_pending <= 0;
		else if (do_ramp) ramp_pending <= 1;
	
		if (complete_i) readout_pending <= 0;
		else if (readout_o) readout_pending <= 1;
				
		if (pb_port[4:0] == 19 && pb_write) begin
			lab4_serial_select <= pb_outport[3:0];
		end
		if (pb_port[4:0] == 18 && pb_write) begin
			lab4_serial_register[16 +: 8] <= pb_outport;
		end
		if (pb_port[4:0] == 17 && pb_write) begin
			lab4_serial_register[0 +: 8] <= pb_outport;
		end
		
		if (pb_port[4:0] == 0 && pb_write) begin
			lab4_runmode <= pb_outport[2];
			lab4_control_reset_request <= pb_outport[0];
		end
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[4:0] == 5))
			test_pattern_request <= wb_dat_i[31];
		else if (pb_port[4:0] == 5 && pb_write) begin
			test_pattern_request <= pb_outport[7];
		end
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[4:0] == 6))
			lab4_user_write_request <= wb_dat_i[31];
		else if (pb_port[4:0] == 11 && pb_write) begin
			lab4_user_write_request <= pb_outport[7];
		end

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[4:0] == 3 || wb_adr_i[4:0] == 4)) begin
			update_wilkinson <= 1;
			if (wb_adr_i[4:0] == 3) ramp_to_wilkinson <= wb_dat_i[15:0];
			if (wb_adr_i[4:0] == 4) wclk_stop_count <= wb_dat_i[15:0];
		end

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[4:0] == 21)) begin
			trigger_clear <= wb_dat_i[0];
			force_trigger <= wb_dat_i[1];
		end		

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[4:0] == 31)) begin
			processor_reset <= wb_dat_i[31];
			bram_we_enable <= wb_dat_i[30];
			bram_data_reg <= wb_dat_i[0 +: 18];
			bram_address_reg <= wb_dat_i[18 +: 10];
		end
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[4:0] == 31)) bram_we <= 1;
		else bram_we <= 0;

                ack <= wb_cyc_i && wb_sel_i;
	end
			
	lab4d_shift_register u_shift_reg(.clk_i(clk_i),
												.go_i(lab4_serial_go),
												.dat_i(lab4_serial_register),
												.sel_i(lab4_serial_select),
												.prescale_i(shift_prescale),
												.busy_o(lab4_serial_busy),
												.SIN(SIN),
												.SCLK(SCLK),
												.PCLK(PCLK));
	lab4d_trigger_control u_trigger(.clk_i(clk_i),
											  .sys_clk_i(sys_clk_i),
											  .sys_clk_div4_flag_i(sys_clk_div4_flag_i),
											  .enable_i(lab4_runmode),
											  .ready_o(lab4_running),
											  .trigger_i(trig_i),
											  .force_trigger_i(force_trigger),
											  .trigger_busy_o(trigger_busy),
											  .trigger_address_o(trigger_address),
											  .trigger_clear_i(trigger_clear),
											  .WR(WR));
	lab4d_wilkinson_ramp u_ramp(.clk_i(clk_i),
										 .wclk_i(wclk_i),
										 .update_i(update_wilkinson),
										 .ramp_to_wclk_i(ramp_to_wilkinson),
										 .wclk_stop_count_i(wclk_stop_count),
										 .do_ramp_i(do_ramp),
										 .ramp_done_o(ramp_done),
										 .RAMP(RAMP),
										 .WCLK_P(WCLK_P),
										 .WCLK_N(WCLK_N));

	kcpsm6 processor(.address(pbAddress),.instruction(pbInstruction),
														  .bram_enable(pbRomEnable),.in_port(pb_inport_mux),
														  .out_port(pb_outport),.port_id(pb_port),
														  .write_strobe(pb_write),.read_strobe(pb_read),
														  .interrupt(1'b0), .sleep(1'b0),
														  .reset(processor_reset),.clk(user_clk_i));

	lab4_controller_rom rom(.address(pbAddress),.instruction(pbInstruction),
											 .enable(pbRomEnable),
											 .bram_we_i(bram_we && bram_we_enable),.bram_adr_i(bram_address_reg),
											 .bram_dat_i(bram_data_reg),.bram_dat_o(bram_readback),
											 .bram_rd_i(1'b1),.clk(user_clk_i));
	
	assign register_mux[0] = lab4_control_register;
	assign register_mux[1] = shift_prescale_register;
	assign register_mux[2] = readout_prescale_register;
	assign register_mux[3] = ramp_to_wilkinson_register;
	assign register_mux[4] = wclk_stop_count_register;
	assign register_mux[5] = test_pattern_register;
	assign register_mux[6] = {32{1'b0}};
	assign register_mux[7] = {32{1'b0}};
	assign register_mux[8] = {32{1'b0}};
	assign register_mux[9] = {32{1'b0}};
	assign register_mux[10] = {32{1'b0}};
	assign register_mux[11] = {32{1'b0}};
	assign register_mux[12] = {32{1'b0}};
	assign register_mux[13] = {32{1'b0}};
	assign register_mux[14] = {32{1'b0}};
	assign register_mux[15] = {32{1'b0}};
	assign register_mux[16] = {32{1'b0}};
	assign register_mux[17] = {32{1'b0}};
	assign register_mux[18] = {32{1'b0}};
	assign register_mux[19] = {32{1'b0}};
	assign register_mux[20] = {32{1'b0}};
	assign register_mux[21] = trigger_register;
	assign register_mux[22] = {32{1'b0}};
	assign register_mux[23] = {32{1'b0}};
	assign register_mux[24] = {32{1'b0}};
	assign register_mux[25] = {32{1'b0}};
	assign register_mux[26] = {32{1'b0}};
	assign register_mux[27] = {32{1'b0}};
	assign register_mux[28] = {32{1'b0}};
	assign register_mux[29] = {32{1'b0}};
	assign register_mux[30] = {32{1'b0}};
	assign register_mux[31] = pb_bram_data;
	
        assign ack_o = ack;
endmodule
