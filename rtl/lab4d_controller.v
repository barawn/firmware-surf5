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
		input sync_i,
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
		output [59:0] WR,
		output [70:0] debug_o,
		output [15:0] trigger_debug_o
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
	assign wb_dat_o = register_mux[wb_adr_i[6:2]];

        reg ack = 0;

	reg lab4_control_reset_request = 0;
	reg lab4_runmode_request = 0;
	reg lab4_runmode = 0;
	reg [11:0] lab4_regclear = {12{1'b0}};
	wire lab4_running;
	wire [31:0] lab4_control_register = {{4{1'b0}},lab4_regclear,{12{1'b0}},lab4_running,lab4_runmode,lab4_runmode_request,lab4_control_reset_request};

	reg [3:0] readout_prescale = READOUT_PRESCALE_DEFAULT;
	wire [31:0] readout_prescale_register = {{28{1'b0}},readout_prescale};

	reg [7:0] shift_prescale = SHIFT_PRESCALE_DEFAULT;
	wire [31:0] shift_prescale_register = {{24{1'b0}},shift_prescale_register};

	reg update_wilkinson = 0;

	reg [15:0] ramp_to_wilkinson = RAMP_TO_WILKINSON_DEFAULT;
	wire [31:0] ramp_to_wilkinson_register = {{16{1'b0}},ramp_to_wilkinson};

	reg [15:0] wclk_stop_count = WCLK_STOP_COUNT_DEFAULT;
	wire [31:0] wclk_stop_count_register = {{16{1'b0}},wclk_stop_count};
	
	wire do_ramp;
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

	// still sucks, not as much
	wire trigger_empty;
	wire trigger_start;
	wire trigger_stop;
	wire [5:0] trigger_address;
	reg trigger_clear = 0;
	reg force_trigger = 0;
	reg [2:0] post_trigger = {3{1'b0}};
	reg post_trigger_wr = 0;
	reg trigger_reset = 0;
	wire [31:0] trigger_register = {{13{1'b0}},post_trigger,{8{1'b0}},trigger_empty,1'b0,trigger_address};

	reg readout_pending = 0;
	reg readout_done = 0;
	reg readout_data_not_test_pattern = 0;
	wire [31:0] readout_register = {{24{1'b0}},readout_pending,{2{1'b0}},readout_data_not_test_pattern,{3{1'b0}},readout_done};
	
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
	assign pb_inport[22] = readout_register[7:0];
	assign pb_inport[23] = {ramp_pending,{7{1'b0}}};
	assign pb_inport[24] = pb_inport[16];
	assign pb_inport[25] = pb_inport[17];
	assign pb_inport[26] = pb_inport[18];
	assign pb_inport[27] = pb_inport[19];
	assign pb_inport[28] = pb_inport[20];
	assign pb_inport[29] = pb_inport[21];
	assign pb_inport[30] = pb_inport[22];
	assign pb_inport[31] = pb_inport[23];
	
	assign lab4_serial_go = (pb_port[4:0] == 19) && pb_write && pb_outport[6];
	assign do_ramp = (pb_port[4:0] == 23) && pb_write && pb_outport[6];
	wire do_readout;
	wire readout_complete;
	assign do_readout = (pb_port[4:0] == 22) && pb_write && pb_outport[6];
	flag_sync u_readout_flag(.in_clkA(do_readout),.clkA(clk_i),.out_clkB(readout_o),.clkB(sys_clk_i));
	flag_sync u_complete_flag(.in_clkA(complete_i),.clkA(sys_clk_i),.out_clkB(readout_complete),.clkB(clk_i));

	assign trigger_start = (pb_port[4:0] == 20) && pb_write && pb_outport[0];
	assign trigger_stop = (pb_port[4:0] == 20) && pb_write && pb_outport[1];
	assign trigger_read = (pb_port[4:0] == 20) && pb_write && pb_outport[6];
	
	always @(posedge clk_i) begin
		if (ramp_done) ramp_pending <= 0;
		else if (do_ramp) ramp_pending <= 1;
	
		if (readout_complete) readout_pending <= 0;
		else if (do_readout) readout_pending <= 1;

		if (pb_port[4:0] == 22 && pb_write) readout_done <= pb_outport[0];
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[6:0] == 7'h58) readout_data_not_test_pattern <= wb_dat_i[4];
				
		if (pb_port[4:0] == 19 && pb_write) begin
			lab4_serial_select <= pb_outport[3:0];
		end
		if (pb_port[4:0] == 18 && pb_write) begin
			lab4_serial_register[16 +: 8] <= pb_outport;
		end
		if (pb_port[4:0] == 17 && pb_write) begin
			lab4_serial_register[8 +: 8] <= pb_outport;
		end
		if (pb_port[4:0] == 16 && pb_write) begin
			lab4_serial_register[0 +: 8] <= pb_outport;
		end
		
		if (pb_port[4:0] == 0 && pb_write) begin
			lab4_runmode <= pb_outport[2];
			lab4_control_reset_request <= pb_outport[0];
		end else if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h00))
			lab4_control_reset_request <= wb_dat_i[0];
	
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h00)) begin
			lab4_runmode_request <= wb_dat_i[1];
			lab4_regclear <= wb_dat_i[16 +: 12];
		end
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h14))
			test_pattern_request <= wb_dat_i[31];
		else if (pb_port[4:0] == 5 && pb_write) begin
			test_pattern_request <= pb_outport[7];
		end
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h18)) begin
			lab4_user_write <= wb_dat_i[0 +: 24];
			lab4_user_select <= wb_dat_i[24 +: 4];
		end
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h18))
			lab4_user_write_request <= wb_dat_i[31];
		else if (pb_port[4:0] == 11 && pb_write) begin
			lab4_user_write_request <= pb_outport[7];
		end

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h0C || wb_adr_i[6:0] == 7'h10)) begin
			update_wilkinson <= 1;
			if (wb_adr_i[6:0] == 7'h0C) ramp_to_wilkinson <= wb_dat_i[15:0];
			if (wb_adr_i[6:0] == 7'h10) wclk_stop_count <= wb_dat_i[15:0];
		end

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h54)) begin
			trigger_clear <= wb_dat_i[0];
			force_trigger <= wb_dat_i[1];
			trigger_reset <= wb_dat_i[2];
			if (wb_dat_i[31]) begin 
				post_trigger_wr <= 1;
				post_trigger <= wb_dat_i[18:16];
			end
		end else begin
			trigger_clear <= 0;
			force_trigger <= 0;
			trigger_reset <= 0;
			post_trigger_wr <= 0;
		end

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h7C)) begin
			processor_reset <= wb_dat_i[31];
			bram_we_enable <= wb_dat_i[30];
			bram_data_reg <= wb_dat_i[0 +: 18];
			bram_address_reg <= wb_dat_i[18 +: 10];
		end
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h7C)) bram_we <= 1;
		else bram_we <= 0;

      ack <= wb_cyc_i && wb_sel_i;
	end
	wire dbg_sin;
	wire dbg_sclk;
	wire dbg_pclk;
	lab4d_shift_register u_shift_reg(.clk_i(clk_i),
												.go_i(lab4_serial_go),
												.dat_i(lab4_serial_register),
												.sel_i(lab4_serial_select),
												.prescale_i(shift_prescale),
												.busy_o(lab4_serial_busy),
												.dbg_sin_o(dbg_sin),
												.dbg_sclk_o(dbg_sclk),
												.dbg_pclk_o(dbg_pclk),
												.SIN(SIN),
												.SCLK(SCLK),
												.PCLK(PCLK));
	lab4d_trigger_control u_trigger(.clk_i(clk_i),
											  .sys_clk_i(sys_clk_i),
											  .sys_clk_div4_flag_i(sys_clk_div4_flag_i),
											  .sync_i(sync_i),
											  .start_i(trigger_start),
											  .stop_i(trigger_stop),											  
											  .ready_o(lab4_running),
											  .trigger_i(trig_i),
											  .force_trigger_i(force_trigger),
											  
											  .rst_i(trigger_reset),
											  .post_trigger_i(post_trigger),
											  .post_trigger_wr_i(post_trigger_wr),
											  											  
											  .trigger_empty_o(trigger_empty),
											  .trigger_rd_i(trigger_read),
											  .trigger_address_o(trigger_address),
											  .trigger_clear_i(trigger_clear),
											  
											  .trigger_debug_o(trigger_debug_o),
											  
											  .WR(WR));
	wire dbg_ramp;
	lab4d_wilkinson_ramp u_ramp(.clk_i(clk_i),
										 .wclk_i(wclk_i),
										 .update_i(update_wilkinson),
										 .ramp_to_wclk_i(ramp_to_wilkinson),
										 .wclk_stop_count_i(wclk_stop_count),
										 .do_ramp_i(do_ramp),
										 .ramp_done_o(ramp_done),
										 .dbg_ramp_o(dbg_ramp),
										 .RAMP(RAMP),
										 .WCLK_P(WCLK_P),
										 .WCLK_N(WCLK_N));

	kcpsm6 processor(.address(pbAddress),.instruction(pbInstruction),
														  .bram_enable(pbRomEnable),.in_port(pb_inport_mux),
														  .out_port(pb_outport),.port_id(pb_port),
														  .write_strobe(pb_write),.read_strobe(pb_read),
														  .interrupt(1'b0), .sleep(1'b0),
														  .reset(processor_reset),.clk(clk_i));

	lab4_controller_rom rom(.address(pbAddress),.instruction(pbInstruction),
											 .enable(pbRomEnable),
											 .bram_we_i(bram_we && bram_we_enable),.bram_adr_i(bram_address_reg),
											 .bram_dat_i(bram_data_reg),.bram_dat_o(bram_readback),
											 .bram_rd_i(1'b1),.clk(clk_i));
	
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

	assign REGCLR = lab4_regclear;

	assign debug_o[0 +: 12] = pbAddress;
	assign debug_o[12 +: 8] = (pb_write) ? pb_outport : pb_inport_mux;
	assign debug_o[20] = pb_write;
	assign debug_o[21] = pb_read;
	assign debug_o[22 +: 18] = pbInstruction;
	assign debug_o[40] = lab4_serial_go;
	assign debug_o[41] = lab4_serial_busy;
	assign debug_o[42] = dbg_sin;
	assign debug_o[43] = dbg_sclk;
	assign debug_o[44] = dbg_pclk;
	assign debug_o[45] = processor_reset;
	assign debug_o[46] = bram_we_enable;
	assign debug_o[47 +: 8] = pb_port;
	assign debug_o[55] = dbg_ramp;
        assign wb_ack_o = ack;
        assign wb_err_o = 1'b0;
        assign wb_rty_o = 0;
endmodule
