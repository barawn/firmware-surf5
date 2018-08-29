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
		
		input clk_ps_i,
		output ps_clk_o,
		output ps_en_o,
		output ps_incdec_o,
		input  ps_done_i,
		input  [11:0] MONTIMING_B,
		inout  sync_mon_io,
		
		output readout_o,
		output [3:0] readout_header_o,
		output readout_test_pattern_o,
		output readout_fifo_rst_o,
		output readout_rst_o,
		input [11:0] readout_fifo_empty_i,
		output [9:0] readout_empty_size_o,
		output [3:0] prescale_o,
		input complete_i,
		
		input [11:0] montiming_i,
		
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
		output [70:0] debug2_o,
		output [15:0] trigger_debug_o,
		output [14:0] phase_scanner_debug_o
    );
	
	localparam [3:0] READOUT_PRESCALE_DEFAULT = 4'h0;
	localparam [7:0] SHIFT_PRESCALE_DEFAULT = 8'h00;
	localparam [15:0] RAMP_TO_WILKINSON_DEFAULT = 16'h0000;
	localparam [15:0] WCLK_STOP_COUNT_DEFAULT = 16'd1024;

	// registers:
	// 0: resets/control/runmode
	// 1: shift register prescale
	// 2: readout prescale
	// 3: ramp to wilkinson delay
	// 4: wilkinson max count
	// 5: phase reset register
	// 6: LAB4 serial write
	// 7: shadow 3
	// 8: phase scanner reg 0
	// 9: phase scanner reg 1
	// 10: phase scanner reg 2
	// 11: phase scanner reg 3
	// 12: phase scanner reg 4
	// 13: phase scanner reg 5
	// 14: phase scanner reg 6
	// 15: phase scanner reg 7
	// 16: shadow 0
	// 17: shadow 1
	// 18: shadow 2
	// 19: current buffer
	// 20: last trigger buffer
	// 21: trigger control
	// 22: readout
	// 23: readout empty threshold (event size - 1)
	// 30: picoblaze control
	// 31: picoblaze bram
	
	wire [31:0] register_mux[31:0];
	
	wire [31:0] lab4_control_register;
	wire [31:0] readout_prescale_register;
	wire [31:0] shift_prescale_register;
	wire [31:0] ramp_to_wilkinson_register;
	wire [31:0] wclk_stop_count_register;
	wire [31:0] phase_reset_register;
	wire [31:0] lab4_user_write_register;
	
	wire [31:0] phase_scanner_dat_o;

	wire [31:0] trigger_register;
	wire [31:0] readout_register;

	wire [31:0] pb_bram_data;

	reg [9:0] readout_empty_threshold = {10{1'b0}};


	assign register_mux[0] = lab4_control_register;
	assign register_mux[1] = shift_prescale_register;
	assign register_mux[2] = readout_prescale_register;
	assign register_mux[3] = ramp_to_wilkinson_register;
	assign register_mux[4] = wclk_stop_count_register;
	assign register_mux[5] = phase_reset_register;
	assign register_mux[6] = lab4_user_write_register;
	assign register_mux[7] = register_mux[3];						// reserved
	assign register_mux[8] = phase_scanner_dat_o;				// phase scanner gets 01xxx xx (e.g. [6:5] == 01)
	assign register_mux[9] = phase_scanner_dat_o;
	assign register_mux[10] = phase_scanner_dat_o;
	assign register_mux[11] = phase_scanner_dat_o;
	assign register_mux[12] = phase_scanner_dat_o;
	assign register_mux[13] = phase_scanner_dat_o;
	assign register_mux[14] = phase_scanner_dat_o;
	assign register_mux[15] = phase_scanner_dat_o;
	assign register_mux[16] = register_mux[0];
	assign register_mux[17] = register_mux[1];
	assign register_mux[18] = register_mux[2];
	assign register_mux[19] = {32{1'b0}};
	assign register_mux[20] = {32{1'b0}};
	assign register_mux[21] = trigger_register;
	assign register_mux[22] = readout_register;
	assign register_mux[23] = {{22{1'b0}},readout_empty_threshold};
	assign register_mux[24] = phase_scanner_dat_o;
	assign register_mux[25] = phase_scanner_dat_o;
	assign register_mux[26] = phase_scanner_dat_o;
	assign register_mux[27] = phase_scanner_dat_o;
	assign register_mux[28] = phase_scanner_dat_o;
	assign register_mux[29] = phase_scanner_dat_o;
	assign register_mux[30] = phase_scanner_dat_o;
	assign register_mux[31] = pb_bram_data;
	assign wb_dat_o = register_mux[wb_adr_i[6:2]];




   reg ack = 0;

	// lab4 register:
	// bit 0 : reset request
	// bit 1: runmode request
	// bit 2: runmode
	// bit 3: running
	// bit 7:4:  unused
	// bit 8: wilk reset
	// bit 14:9 unused
	// bit 15: *readout* test pattern enable (not LAB4 test pattern enable! reads a counter into RAM. to test DMA)
	// bit 27:16: regclear
	reg lab4_control_reset_request = 0;
	reg lab4_runmode_request = 0;
	reg lab4_runmode = 0;
	reg lab4_wilk_reset = 0;
	reg [11:0] lab4_regclear = {12{1'b0}};
	reg lab4_readout_test_pattern_enable = 0;
	wire lab4_running;
	assign lab4_control_register = {{4{1'b0}},lab4_regclear,lab4_readout_test_pattern_enable,{11{1'b0}},lab4_running,lab4_runmode,lab4_runmode_request,lab4_control_reset_request};

	reg [3:0] readout_prescale = READOUT_PRESCALE_DEFAULT;
	assign readout_prescale_register = {{28{1'b0}},readout_prescale};

	reg [7:0] shift_prescale = SHIFT_PRESCALE_DEFAULT;
	assign shift_prescale_register = {{24{1'b0}},shift_prescale_register};

	reg update_wilkinson = 0;

	reg [15:0] ramp_to_wilkinson = RAMP_TO_WILKINSON_DEFAULT;
	assign ramp_to_wilkinson_register = {{16{1'b0}},ramp_to_wilkinson};

	reg [15:0] wclk_stop_count = WCLK_STOP_COUNT_DEFAULT;
	assign wclk_stop_count_register = {{16{1'b0}},wclk_stop_count};
	
	wire do_ramp;
	reg ramp_pending = 0;
	wire ramp_done;
	
	// user-side serial register. PicoBlaze interacts with
	// it and takes it over.
	reg [23:0] lab4_user_write = {24{1'b0}};
	reg [3:0] lab4_user_select = {4{1'b0}};
	reg lab4_user_write_request = 0;
	assign lab4_user_write_register = 
		{lab4_user_write_request,3'b000,lab4_user_select,lab4_user_write};

	// actual serial register
	reg [23:0] lab4_serial_register = {24{1'b0}};
	wire lab4_serial_busy;
	reg [3:0] lab4_serial_select = {4{1'b0}};
	wire lab4_serial_go;
	
	// PHAB reset interface
	reg [11:0] phab_at_latch = {12{1'b0}};
	reg [1:0] phab_phase_select = {2{1'b0}};
	reg phab_target_value = 0;
	assign phase_reset_register = {{4{1'b0}},phab_at_latch,{7{1'b0}},phab_target_value,{6{1'b0}},phab_phase_select};
	
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
	assign trigger_register = {{13{1'b0}},post_trigger,{8{1'b0}},trigger_empty,1'b0,trigger_address};


	// Readout Register:
	// bit 0: readout complete
	// bit 1: readout fifo reset
	// bit 2: readout reset
	// bit 3: data available (any readout fifo is not empty)
	// bit 4: readout data, not test pattern
	// bit 5: unused
	// bit 6: begin readout (from PicoBlaze only)
	// bit 7: readout pending
	// bits 15-8: unused
	// bits 27-16: readout fifo empty 0-11
	reg readout_pending = 0;	
	reg readout_done = 0;
	reg readout_data_not_test_pattern = 0;
	reg readout_fifo_reset = 0;
	reg readout_reset = 0;
	wire data_available = (readout_fifo_empty_i != {12{1'b1}});
	assign readout_register = {{4{1'b0}},readout_fifo_empty_i,{8{1'b0}},readout_pending,{2{1'b0}},readout_data_not_test_pattern,data_available,2'b00,readout_done};
	
	reg [3:0] readout_header_clk = {4{1'b0}};
	reg [3:0] readout_header_sysclk = {4{1'b0}};
	wire readout_header_write_sysclk;
	reg readout_header_write = 0;
	flag_sync u_readout_header(.in_clkA(readout_header_write),.clkA(clk_i),.out_clkB(readout_header_write_sysclk),.clkB(sys_clk_i));
	
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
	assign pb_inport[4] = 8'h00;
	assign pb_inport[6] = 8'h00;
	assign pb_inport[5] = 8'h00;
	assign pb_inport[7] = 8'h00;
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
	assign pb_inport[21] = {{4{1'b0}},readout_header_clk};
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
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[6:0] == 7'h58) begin
			readout_data_not_test_pattern <= wb_dat_i[4];
			readout_fifo_reset <= wb_dat_i[1];
			readout_reset <= wb_dat_i[2];
		end else begin
			readout_fifo_reset <= 0;
			readout_reset <= 0;
		end
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[6:0] == 7'h5C) begin
			readout_empty_threshold <= wb_dat_i[9:0];
		end
		
		if (pb_port[4:0] == 21 && pb_write) begin
			readout_header_write <= 1;
			readout_header_clk <= pb_outport[3:0];
		end else begin
			readout_header_write <= 0;
		end
				
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
			lab4_wilk_reset <= wb_dat_i[8];
			lab4_readout_test_pattern_enable <= wb_dat_i[15];
		end else begin
			lab4_wilk_reset <= 0;
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
		end else begin
			update_wilkinson <= 0;
		end

		if (pb_port[4:0] == 20 && pb_write) begin
			trigger_reset <= pb_outport[5];
		end else begin
			trigger_reset <= 0;
		end

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[6:0] == 7'h54)) begin
			trigger_clear <= wb_dat_i[0];
			force_trigger <= wb_dat_i[1];
			if (wb_dat_i[31]) begin 
				post_trigger_wr <= 1;
				post_trigger <= wb_dat_i[18:16];
			end
		end else begin
			trigger_clear <= 0;
			force_trigger <= 0;
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
	always @(posedge sys_clk_i) begin
		if (readout_header_write_sysclk) readout_header_sysclk <= readout_header_clk;
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
	lab4d_wilkinson_ramp_v2 u_ramp(.clk_i(clk_i),
										 .wclk_i(wclk_i),
										 .sys_clk_i(sys_clk_i),
										 .rst_i(lab4_wilk_reset),
										 .update_i(update_wilkinson),
										 .ramp_to_wclk_i(ramp_to_wilkinson),
										 .wclk_stop_count_i(wclk_stop_count),
										 .do_ramp_i(do_ramp),
										 .ramp_done_o(ramp_done),
										 .dbg_ramp_o(dbg_ramp),
										 .RAMP(RAMP),
										 .WCLK_P(WCLK_P),
										 .WCLK_N(WCLK_N));

	wire phase_scanner_ack;
	wire phase_scanner_err;
	wire phase_scanner_rty;
	surf5_phase_scanner_v2 u_phase_scanner(.clk_i(clk_i),.rst_i(rst_i),
														.wb_cyc_i(wb_cyc_i),.wb_stb_i(wb_stb_i && wb_adr_i[6:5] == 2'b01),
														.wb_adr_i(wb_adr_i[4:2]),
														.wb_we_i(wb_we_i),.wb_dat_i(wb_dat_i),
														.wb_dat_o(phase_scanner_dat_o),
														.wb_ack_o(phase_scanner_ack),
														.wb_err_o(phase_scanner_err),
														.wb_rty_o(phase_scanner_rty),
														.clk_ps_i(clk_ps_i),
														.ps_clk_o(ps_clk_o),
														.ps_en_o(ps_en_o),
														.ps_incdec_o(ps_incdec_o),
														.ps_done_i(ps_done_i),
														.MONTIMING_B(MONTIMING_B),
														.sync_i(sync_i),
														.sync_mon_io(sync_mon_io),
														.debug_o(phase_scanner_debug_o),
														.debug2_o(debug2_o));

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

	assign readout_header_o = readout_header_sysclk;
	assign readout_fifo_rst_o = readout_fifo_reset;
	assign readout_test_pattern_o = lab4_readout_test_pattern_enable;
	assign readout_empty_size_o = readout_empty_threshold;
	
	flag_sync u_readout_rst(.in_clkA(readout_reset),.clkA(clk_i),.out_clkB(readout_rst_o),.clkB(sys_clk_i));
	
        assign wb_ack_o = ack;
        assign wb_err_o = 1'b0;
        assign wb_rty_o = 0;
endmodule
