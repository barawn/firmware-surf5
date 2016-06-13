`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:54:31 06/13/2016 
// Design Name: 
// Module Name:    surf5_phase_scanner 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`include "wishbone.vh"
module surf5_phase_scanner(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 1, 4),
		output ps_en_o,
		output ps_incdec_o,
		input ps_done_i,
		
		input sync_i,
		input clk_ps_i,
		inout sync_mon_io,
		input [11:0] MONTIMING_B,
		
		output [11:0] montiming_q_o,
		output sync_q_o,
		output scan_valid_o,
		output scan_done_o
	);
	// simple phase scanner, no picoblaze, just ChipScope
	// Our clock is 12.5 MHz, or 1/80th of the input clock of 1 GHz.
	// There are 56 steps, so that means we have 4480 steps.
	localparam [12:0] PHASE_SCAN_STEPS = 4480;
	reg [12:0] phase_step_count = {13{1'b0}};
	
	wire sync_mon_o = sync_i;
	wire sync_mon_i = sync_mon_io;
	reg sync_mon_disable = 0;
	assign sync_mon_io = (sync_mon_disable) ? 1'bZ : sync_mon_o;

	wire do_ce_clk;
	wire done_ce_clk;
	reg latched_clkps = 0;
	wire ce_clkps;
	wire [11:0] montiming_b_q_clkps;
	wire sync_q_clkps;
	reg [11:0] montiming_b_q_clk = {12{1'b0}};
	reg sync_q_clk = 0;
	
	(* IOB = "TRUE" *)
	FDRE #(.INIT(1'b0)) u_sync_q(.D(sync_mon_i),.CE(ce_clkps),.C(clk_ps_i),.Q(sync_q_clkps),.R(1'b0));
	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : MONTIMING
			(* IOB = "TRUE" *)
			FDRE #(.INIT(1'b1)) u_montiming_b_q(.D(MONTIMING_B[i]),.CE(ce_clkps),.C(clk_ps_i),.Q(montiming_b_q_clkps[i]),.R(1'b0));
		end
	endgenerate
	
	flag_sync u_do_ce(.in_clkA(do_ce_clk),.clkA(clk_i),.out_clkB(ce_clkps),.clkB(clk_ps_i));
	flag_sync u_done_ce(.in_clkA(latched_clkps),.clkA(clk_ps_i),.out_clkB(done_ce_clk),.clkB(clk_i));
	always @(posedge clk_ps_i) begin
		latched_clkps <= ce_clkps;
	end
	
	reg free_scan = 0;
	reg latched_clk = 0;
	localparam FSM_BITS = 3;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] PS_INC = 1;
	localparam [FSM_BITS-1:0] PS_WAIT = 2;
	localparam [FSM_BITS-1:0] DO_LATCH = 3;
	localparam [FSM_BITS-1:0] LATCH_WAIT = 4;
	localparam [FSM_BITS-1:0] DONE = 5;
	reg [FSM_BITS-1:0] state = IDLE;
	
	always @(posedge clk_i) begin
		latched_clk <= done_ce_clk;
		if (done_ce_clk) begin
			montiming_b_q_clk <= montiming_b_q_clkps;
			sync_q_clk <= sync_q_clkps;
		end
		if (wb_cyc_i && wb_stb_i && !wb_adr_i && wb_we_i) begin
			free_scan <= wb_dat_i[0];
			sync_mon_disable <= wb_dat_i[1];
		end
		case (state)
			IDLE: if (free_scan) state <= PS_INC;
			PS_INC: state <= PS_WAIT;
			PS_WAIT: if (ps_done_i) state <= DO_LATCH;
			DO_LATCH: state <= LATCH_WAIT;
			LATCH_WAIT: if (done_ce_clk) begin
				if (phase_step_count == PHASE_SCAN_STEPS-1) state <= DONE;
				else state <= PS_INC;
			end
			DONE: state <= IDLE;
		endcase

		if (state == DONE) phase_step_count <= {13{1'b0}};
		else if (state == LATCH_WAIT && done_ce_clk) phase_step_count <= phase_step_count + 1;		
	end
	
	assign scan_done_o = (state == DONE);
	assign scan_valid_o = (latched_clk);
	assign montiming_q_o = ~montiming_b_q_clk;
	assign sync_q_o = sync_q_clk;
	assign ps_en_o = (state == PS_INC);
	assign ps_incdec_o = (state == PS_INC);
	assign do_ce_clk = (state == DO_LATCH);

endmodule
