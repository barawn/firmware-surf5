`timescale 1ns / 1ps
`include "wishbone.vh"
module surf5_phase_scanner_v2(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 3, 4),
		input clk_ps_i,
		output ps_clk_o,
		output ps_en_o,
		output ps_incdec_o,
		input ps_done_i,
		input [11:0] MONTIMING_B,
		input	sync_i,
		inout sync_mon_io,
		output [14:0] debug_o,
		output [70:0] debug2_o
    );

	assign ps_clk_o = clk_i;

	wire do_ce_clk;
	wire done_ce_clk;
	reg latched_clkps = 0;
	wire ce_clkps;
	wire [11:0] montiming_b_q_clkps;
	wire sync_q_clkps;
	reg [11:0] montiming_q_clk = {12{1'b0}};
	reg sync_q_clk = 0;

	wire sync_mon_o = sync_i;
	wire sync_mon_i = sync_mon_io;
	reg sync_mon_disable = 0;
	assign sync_mon_io = (sync_mon_disable) ? 1'bZ : sync_mon_o;

	reg scan_valid = 0;
	reg scan_done = 0;
	
	reg ps_done_seen = 0;
	reg latched_clk_seen = 0;
	reg [7:0] command = {8{1'b0}};
	reg [31:0] argument = {32{1'b0}};
	reg [15:0] result = {16{1'b0}};
	reg [15:0] zero_step = {16{1'b0}};
	wire [31:0] pb_bram_data;

	reg ack = 0;
	wire [31:0] wb_dat[7:0];
	assign wb_dat[0] = {{24{1'b0}},command};
	assign wb_dat[1] = argument;
	assign wb_dat[2] = {{16{1'b0}},result};
	assign wb_dat[3] = {{16{1'b0}},zero_step};
	assign wb_dat[4] = wb_dat_o[0];
	assign wb_dat[5] = wb_dat_o[1];
	assign wb_dat[6] = wb_dat_o[2];
	assign wb_dat[7] = pb_bram_data;
	assign wb_dat_o = wb_dat[wb_adr_i];
	
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
	// PicoBlaze register 0: phase control. Bit 2 = phase increment, bit 3 = phase enable, bit 4 = latch.
	assign pb_inport[0] = {{6{1'b0}},latched_clk_seen,ps_done_seen};
	// PicoBlaze register 1: command
	assign pb_inport[1] = command;
	// PicoBlaze register 2: argument low
	assign pb_inport[2] = argument[7:0];
	// PicoBlaze register 3: argument midlow
	assign pb_inport[3] = argument[15:8];
	// PicoBlaze register 4: argument midhigh
	assign pb_inport[4] = argument[23:16];
	// PicoBlaze register 5: argument high
	assign pb_inport[5] = argument[31:24];
	// PicoBlaze register 6: result low
	assign pb_inport[6] = result[7:0];
	// PicoBlaze register 7: result high
	assign pb_inport[7] = result[15:8];
	// PicoBlaze register 8: zero low
	assign pb_inport[8] = zero_step[7:0];
	// PicoBlaze register 9: zero high
	assign pb_inport[9] = zero_step[15:8];

	assign pb_inport[10] = pb_inport[2];
	assign pb_inport[11] = pb_inport[3];
	assign pb_inport[12] = pb_inport[4];
	assign pb_inport[13] = pb_inport[5];
	assign pb_inport[14] = pb_inport[6];
	assign pb_inport[15] = pb_inport[7];
	
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

	(* IOB = "TRUE" *)
	FDRE #(.INIT(1'b0)) u_sync_q(.D(sync_mon_i),.CE(ce_clkps),.C(clk_ps_i),.Q(sync_q_clkps),.R(1'b0));
	assign pb_inport[28] = {{7{1'b0}},sync_q_clk};
	assign pb_inport[29] = pb_inport[21];
	assign pb_inport[30] = pb_inport[22];
	assign pb_inport[31] = pb_inport[23];
	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : MONTIMING
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b1)) u_montiming_b_q(.D(MONTIMING_B[i]),.CE(ce_clkps),.C(clk_ps_i),.Q(montiming_b_q_clkps[i]),.R(1'b0));

			assign pb_inport[16+i] = {{7{1'b0}},montiming_q_clk[i]};
		end
	endgenerate
	
	flag_sync u_do_ce(.in_clkA(do_ce_clk),.clkA(clk_i),.out_clkB(ce_clkps),.clkB(clk_ps_i));
	flag_sync u_done_ce(.in_clkA(latched_clkps),.clkA(clk_ps_i),.out_clkB(done_ce_clk),.clkB(clk_i));
	always @(posedge clk_ps_i) begin
		latched_clkps <= ce_clkps;
	end

	assign do_ce_clk = (pb_port[4:0] == 0) && pb_write && pb_outport[4];
	assign ps_en_o = (pb_port[4:0] == 0) && pb_write && pb_outport[3];
	assign ps_incdec_o = (pb_port[4:0] == 0) && pb_write && pb_outport[2];

	always @(posedge clk_i) begin
		if (pb_port[4:0] == 0 && pb_write) begin
			sync_mon_disable <= pb_outport[5];
		end

		if (do_ce_clk) latched_clk_seen <= 0;
		else if (done_ce_clk) latched_clk_seen <= 1; 

		if (done_ce_clk) begin
			montiming_q_clk <= ~montiming_b_q_clkps;
			sync_q_clk <= sync_q_clkps;
		end

		if (ps_en_o) ps_done_seen <= 0;
		else if (ps_done_i) ps_done_seen <= 1;
		
		if (pb_port[4:0] == 0 && pb_write) begin
			scan_valid <= pb_outport[6];
			scan_done <= pb_outport[7];
		end else begin
			scan_valid <= 0;
			scan_done <= 0;
		end
	
		if (wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[2:0] == 0) command <= wb_dat_i[7:0];
		else if (pb_port[4:0] == 1 && pb_write) command <= pb_outport[7:0];
				
		if (wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[2:0] == 1) argument <= wb_dat_i;
		
		if (pb_port[4:0] == 6 && pb_write) result[7:0] <= pb_outport;
		if (pb_port[4:0] == 7 && pb_write) result[15:8] <= pb_outport;
		
		if (wb_cyc_i && wb_stb_i && wb_we_i && wb_adr_i[2:0] == 3) zero_step <= wb_dat_i[15:0];

		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[2:0] == 7)) begin
			processor_reset <= wb_dat_i[31];
			bram_we_enable <= wb_dat_i[30];
			bram_data_reg <= wb_dat_i[0 +: 18];
			bram_address_reg <= wb_dat_i[18 +: 10];
		end
		if (wb_cyc_i && wb_stb_i && wb_we_i && (wb_adr_i[2:0] == 7)) bram_we <= 1;
		else bram_we <= 0;

      ack <= wb_cyc_i && wb_sel_i;		
	end
	
	kcpsm6 processor(.address(pbAddress),.instruction(pbInstruction),
														  .bram_enable(pbRomEnable),.in_port(pb_inport_mux),
														  .out_port(pb_outport),.port_id(pb_port),
														  .write_strobe(pb_write),.read_strobe(pb_read),
														  .interrupt(1'b0), .sleep(1'b0),
														  .reset(processor_reset),.clk(clk_i));

	surf5_phase_scanner_rom rom(.address(pbAddress),.instruction(pbInstruction),
											 .enable(pbRomEnable),
											 .bram_we_i(bram_we && bram_we_enable),.bram_adr_i(bram_address_reg),
											 .bram_dat_i(bram_data_reg),.bram_dat_o(bram_readback),
											 .bram_rd_i(1'b1),.clk(clk_i));	

	assign debug_o[0 +: 12] = montiming_q_clk;
	assign debug_o[12] = sync_q_clk;
	assign debug_o[13] = scan_valid;
	assign debug_o[14] = scan_done;

	assign debug2_o[0 +: 12] = pbAddress;
	assign debug2_o[12 +: 8] = (pb_write) ? pb_outport : pb_inport_mux;
	assign debug2_o[20] = pb_write;
	assign debug2_o[21] = pb_read;
	assign debug2_o[22 +: 18] = pbInstruction;
	assign debug2_o[40] = ps_en_o;
	assign debug2_o[41] = ps_incdec_o;
	assign debug2_o[42] = ps_done_i;
	assign debug2_o[43] = do_ce_clk;
	assign debug2_o[44] = done_ce_clk;
	assign debug2_o[45 +: 8] = pb_port;
	assign debug2_o[53 +: 8] = command;
	assign debug2_o[70:61] = {10{1'b0}};
	
	assign wb_ack_o = ack;
	assign wb_err_o = 0;
	assign wb_rty_o = 0;
endmodule
