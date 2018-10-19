`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    11:56:17 08/01/2018 
// Design Name: 
// Module Name:    test_dma_controller 
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
module test_dma_controller(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wbs, 32, 16, 4),
		`WBM_NAMED_PORT(wbm, 32, 32, 4),
		`WBM_NAMED_PORT(dmad, 32, 20, 4),
		output dma_interrupt_o,
		output [2:0] wbm_cti_o,
		output [1:0] wbm_bte_o,
		output [70:0] debug_o
    );

	parameter [31:0] ID = "TEST";

	reg [31:0] target_begin_address = {32{1'b0}};
	reg [31:0] target_end_address = {32{1'b0}};
	reg [8:0] transfer_length = {9{1'b0}};
	wire [31:0] control_register;
	reg [19:0] rd_address = {20{1'b0}};
	wire [31:0] wishbone_registers[3:0];
	assign wishbone_registers[0] = rd_address;
	assign wishbone_registers[1] = target_begin_address;
	assign wishbone_registers[2] = transfer_length;
	assign wishbone_registers[3] = control_register;
	
	reg ack = 0;
	reg begin_dma = 0;
	reg abort_dma = 0;
	reg dma_complete_ok = 0;
	reg dma_err = 0;
	reg dma_active = 0;
	always @(posedge clk_i) begin
		ack <= wbs_cyc_i && wbs_stb_i && !ack;
		
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b00)) rd_address <= wbs_dat_i;
		
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b01)) target_begin_address <= wbs_dat_i;
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b10)) transfer_length <= wbs_dat_i;		

		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11)) begin_dma <= wbs_dat_i[0];
		else begin_dma <= 0;

		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11)) abort_dma <= wbs_dat_i[4];
		else abort_dma <= 0;

		
		if (begin_dma) dma_active <= 1;
		else if (state == DMA_IDLE) dma_active <= 0;
		
		if (begin_dma) dma_complete_ok <= 0;
		else if (state == DMA_ACK) dma_complete_ok <= 1;
		else if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && wbs_dat_i[5]) dma_complete_ok <= 0;

		if (begin_dma) dma_err <= 0;
		else if (wbm_err_i && state == DMA_ASSERT_CYC) dma_err <= 1;
		else if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && wbs_dat_i[5]) dma_err <= 0;
	end
	assign wbs_dat_o = wishbone_registers[wbs_adr_i[3:2]];
	assign wbs_rty_o = 0;
	assign wbs_err_o = 0;
	assign wbs_ack_o = ack;
	assign dma_interrupt_o = (dma_err || dma_complete_ok);
	//% All writeable control register bits are flags, so you can just write '1' to the specific bit to do what you want. Write 1 to cr[5] clears interrupt.
	assign control_register = {{13{1'b0}},state,{10{1'b0}},dma_interrupt_o, abort_dma, dma_err,dma_complete_ok,dma_active,begin_dma};


	// this is a one-way DMA. Two state machines: a reader and a writer.
	localparam RD_FSM_BITS=3;
	localparam [FSM_BITS-1:0] RD_DMA_IDLE = 0;
	localparam [FSM_BITS-1:0] RD_DMA_ASSERT_CYC = 1;
	localparam [FSM_BITS-1:0] RD_DMA_ACK = 2;
	localparam [FSM_BITS-1:0] RD_DMA_ERR = 3;
	localparam [FSM_BITS-1:0] RD_DMA_RTY = 4;
	reg [RD_FSM_BITS-1:0] rd_state = RD_DMA_IDLE;
	
	// our address needs to stay fixed.
	reg [9:0] rd_count = {10{1'b0}};

	// this should result in a 2-cycle transfer rate (ack, no ack, ack, no ack, etc.). We'll see.
	always @(posedge clk_i) begin
		if (begin_dma) rd_count <= {16{1'b0}};
		else if (rd_state == RD_DMA_ASSERT_CYC && dmad_ack_i) rd_count <= rd_count + 1;
		
		case (rd_state)
			RD_DMA_IDLE: if (begin_dma) rd_state <= RD_DMA_ASSERT_CYC;
			RD_DMA_ASSERT_CYC: if (dmad_rty_i) rd_state <= RD_DMA_RTY;
									 else if (dmad_err_i) rd_state <= RD_DMA_ERR;
									 else if (dmad_ack_i && (rd_count == {1'b0, transfer_length})) rd_state <= RD_DMA_ACK;
									 else if (abort_dma) rd_state <= RD_DMA_IDLE;
			RD_DMA_ACK: if (rd_count > {1'b0,transfer_length}) rd_state <= RD_DMA_IDLE;
							else rd_state <= RD_DMA_ASSERT_CYC;
			RD_DMA_RTY: rd_state <= RD_DMA_ASSERT_CYC;
			RD_DMA_ERR: rd_state <= RD_DMA_IDLE;
		endcase
	end
	// We might eventually change this to hold the path and do a faster read, but for now let's just see what happens.
	// This ends up being a safety mechanism too, since releasing cyc guarantees we can't deadlock things
	assign dmad_cyc_o = (rd_state == RD_DMA_ASSERT_CYC);
	assign dmad_stb_o = dmad_cyc_o;
	assign dmad_we_o = 0;
	assign dmad_adr_o = (rd_address);
	assign dmad_dat_o = {32{1'b0}};
	
	// DMA fifo holds the data between the read and write state machines.
	wire dma_data_read;
	wire dma_mostly_empty;
	wire dma_empty;
	wire dma_full;
	dma_fifo u_dma_fifo (
	  .clk(clk_i), // input clk
	  .srst(abort_dma), // input srst
	  .din(dmad_dat_i), // input [31 : 0] din
	  .wr_en(dmad_cyc_o && dmad_stb_o && dmad_ack_i), // input wr_en
	  .rd_en(wbm_cyc_o && wbm_stb_o && wbm_ack_i), // need to check this, might underflow at the end
	  .dout(wbm_dat_o), // output [31 : 0] dout
	  .full(dma_full), // output full
	  .empty(dma_empty), // output empty
	  .prog_empty(dma_mostly_empty) // output prog_empty
	);	
	localparam FSM_BITS=3;
	localparam [FSM_BITS-1:0] DMA_IDLE = 0;
	localparam [FSM_BITS-1:0] DMA_SETUP = 1;
	localparam [FSM_BITS-1:0] DMA_ASSERT_CYC = 2;
	localparam [FSM_BITS-1:0] DMA_RTY = 3;
	localparam [FSM_BITS-1:0] DMA_ACK = 4;
	reg [FSM_BITS-1:0] state = DMA_IDLE;
	
	// At DMA_ASSERT_CYC
	// check if current_address == target_end_address
	// if so, cti_o == 111
	// else cti_o == 010
	// at begin_dma
	// check if target_begin_address == target_end_address
	// if so, cti_o = 111
	// else cti_o == 010
	// ack_i means current_address <= current_address_plus_one	
	// let's give it a whirl
	reg [31:0] current_address = {32{1'b0}};
	reg [2:0] cycle_type_indicator = {3{1'b0}};
	wire [29:0] current_int_address_plus_one = current_address[31:2] + 1;
	wire [31:0] current_address_plus_four = {current_int_address_plus_one,2'b00};
	always @(posedge clk_i) begin
		if (begin_dma) current_address <= target_begin_address;
		else if (state == DMA_ASSERT_CYC && wbm_ack_i) current_address <= current_address_plus_four;
		
		// need to evaluate how slow this is
		if (begin_dma) target_end_address <= target_begin_address + {transfer_length,2'b00};
		
		cycle_type_indicator[2] <= (current_address == target_end_address);
		cycle_type_indicator[0] <= (current_address == target_end_address);
		cycle_type_indicator[1] <= 1;
		
		case (state)
			DMA_IDLE: if (begin_dma) state <= DMA_SETUP;
			DMA_SETUP: if (abort_dma) state <= DMA_IDLE;
						  else if(!dma_mostly_empty) state <= DMA_ASSERT_CYC;
			DMA_ASSERT_CYC: if (wbm_rty_i) state <= DMA_RTY;
								 else if (wbm_ack_i && cycle_type_indicator[2]) state <= DMA_ACK;
								 else if (wbm_err_i) state <= DMA_IDLE;
								 else if (abort_dma) state <= DMA_IDLE;
			DMA_RTY: state <= DMA_ASSERT_CYC;
			DMA_ACK: state <= DMA_IDLE;
		endcase	
	end
	
	assign wbm_cyc_o = (state == DMA_ASSERT_CYC);
	assign wbm_stb_o = (state == DMA_ASSERT_CYC);
	assign wbm_we_o = 1'b1;
	assign wbm_adr_o = current_address;
	assign wbm_sel_o = 4'hF;
	assign wbm_cti_o = cycle_type_indicator;
	assign wbm_bte_o = 2'b00;

	// debug is a lot harder now
	assign debug_o[0 +: 32] = dmad_dat_i;
	assign debug_o[32] = dmad_cyc_o;
	assign debug_o[33] = dmad_ack_i;
	assign debug_o[34] = dma_mostly_empty;
	assign debug_o[35] = wbm_cyc_o;
	assign debug_o[36] = wbm_ack_i;
	assign debug_o[37 +: 32] = wbm_dat_o;
	assign debug_o[69] = dma_active;
	assign debug_o[70] = begin_dma;

/*	assign debug_o[0 +: 32] = (wbm_we_o) ? wbm_dat_o : wbm_dat_i;
	assign debug_o[32 +: 20] = wbm_adr_o;
	assign debug_o[52] = wbm_cyc_o;
	assign debug_o[53] = wbm_stb_o;
	assign debug_o[54] = wbm_we_o;
	assign debug_o[55] = wbm_ack_i;
	assign debug_o[56] = wbm_rty_i;
	assign debug_o[57] = wbm_err_i;
	assign debug_o[59:58] = state;
	assign debug_o[60] = begin_dma;
	assign debug_o[61] = dma_active;
	assign debug_o[62] = dma_complete_ok;
	assign debug_o[63] = dma_err;
	assign debug_o[70:64] = {7{1'b0}};
	*/
endmodule
