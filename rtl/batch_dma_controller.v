`timescale 1ns / 1ps
// A simple batch DMA controller. This controller can do multiple sequences of DMAs 
// through the internal FIFO interface.
// FIFO is 30+18+9=57 bits wide, plus an additional bit for a "terminate sequence".

// So a multi-DMA sequence can be set up by
// 1: write write address (register 0, target_begin_address)
// 2: write read address (register 1, rd_address)
// 3: write transfer length (register 2)
// 4: write 0x40 to control register (register 3)
// repeat for all remaining DMA - for last, write 0xC1 instead of 0x40 (indicates termination and begins sequence)
// Interrupt is asserted when complete, and you write 0x20 to clear the interrupt after status has been read

`include "wishbone.vh"
module batch_dma_controller(
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

	parameter [31:0] ID = "BATC";

	//% 32-bit target begin address (memory destination).
	reg [29:0] target_begin_address = {30{1'b0}};
	//% Transfer length.
	reg [8:0] transfer_length = {9{1'b0}};
	//% And the 32-bit local read address.
	reg [17:0] rd_address = {18{1'b0}};
	reg sequence_complete = 0;

	wire [57:0] batch_input;
	assign batch_input[0 +: 30] = target_begin_address;
	assign batch_input[30 +: 18] = rd_address;
	assign batch_input[48 +: 9] = transfer_length;
	assign batch_input[57] = sequence_complete;

	wire [57:0] batch_output;
	wire [29:0] cur_target_begin_address = batch_output[0 +: 30];
	wire [17:0] cur_rd_address = batch_output[30 +: 18];
	wire [8:0]  cur_transfer_length = batch_output[48 +: 9];
	wire cur_sequence_complete = batch_output[57];	

	wire [31:0] control_register;
	wire [31:0] wishbone_registers[3:0];
	assign wishbone_registers[0] = {rd_address,2'b00};
	assign wishbone_registers[1] = {target_begin_address,2'b00};
	assign wishbone_registers[2] = transfer_length;
	assign wishbone_registers[3] = control_register;
	
	reg ack = 0;
	reg begin_dma = 0;
	reg begin_batch = 0;
	reg abort_batch = 0;
	reg batch_complete_ok = 0;
	reg batch_err = 0;
	reg batch_active = 0;	
	reg reset_batch = 0;
	reg write_batch = 0;
	wire abort_dma = abort_batch;
	wire dma_active;
	wire dma_complete_ok;
	wire dma_err;
	
	always @(posedge clk_i) begin
		ack <= wbs_cyc_i && wbs_stb_i && !ack;
		
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b00)) rd_address <= wbs_dat_i[2 +: 18];
		
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b01)) target_begin_address <= wbs_dat_i[2 +: 30];
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b10)) transfer_length <= wbs_dat_i;		


		// These are now batch controls.
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && ack) begin_batch <= wbs_dat_i[0];
		else begin_batch <= 0;

		if (rst_i) abort_batch <= 1;
		else if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && ack) abort_batch <= wbs_dat_i[4];
		else abort_batch <= 0;
		
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && ack) write_batch <= wbs_dat_i[6];
		else write_batch <= 0;
		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && ack) sequence_complete <= wbs_dat_i[7];
		else sequence_complete <= 0;

		if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && ack) reset_batch <= wbs_dat_i[8];
		else reset_batch <= 0;
		
		
		// These are now batch statuses.
		if (begin_batch) batch_active <= 1;
		else if (batch_state == BATCH_COMPLETE || batch_state == BATCH_ERR) batch_active <= 0;
		
		if (begin_batch) batch_complete_ok <= 0;
		else if (batch_state == BATCH_COMPLETE) batch_complete_ok <= 1;
		else if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && wbs_dat_i[5]) batch_complete_ok <= 0;
		
		if (begin_batch) batch_err <= 0;
		else if (batch_state == BATCH_ERR) batch_err <= 1;
		else if (wbs_cyc_i && wbs_stb_i && wbs_we_i && (wbs_adr_i[3:2] == 2'b11) && wbs_dat_i[5]) batch_err <= 0;


		// These are now controlled by the batch controller.

		if (batch_state == BATCH_DMA_BEGIN && batch_data_valid) begin_dma <= 1;
		else begin_dma <= 0;
		
	end
	assign wbs_dat_o = wishbone_registers[wbs_adr_i[3:2]];
	assign wbs_rty_o = 0;
	assign wbs_err_o = 0;
	assign wbs_ack_o = ack;
	assign dma_interrupt_o = (batch_err || batch_complete_ok);
	//% All writeable control register bits are flags, so you can just write '1' to the specific bit to do what you want. Write 1 to cr[5] clears interrupt.
	assign control_register = {{13{1'b0}},batch_state,{10{1'b0}},dma_interrupt_o, abort_batch, batch_err,batch_complete_ok,batch_active,begin_batch};

	wire batch_read;
	wire batch_data_valid;
	batch_dma_fifo u_fifo(.clk(clk_i),
								 .srst(rst_i || reset_batch),
								 .din(batch_input),
								 .wr_en(write_batch),
								 .dout(batch_output),
								 .rd_en(batch_read),
								 .valid(batch_data_valid));

	// Batch control FSM:
	// BATCH_IDLE: if (start_batch) batch_state <= BATCH_DMA_BEGIN;
	// BATCH_DMA_BEGIN: issue begin_dma, batch_state <= BATCH_DMA_WAIT;
	// BATCH_DMA_WAIT: if (dma_complete_ok && !cur_sequence_complete) state <= BATCH_READ_WAIT;
	//                 else if (dma_complete_ok) state <= BATCH_COMPLETE;
	//                 else if (dma_err) state <= BATCH_ERR;
	// BATCH_READ_WAIT: state <= BATCH_DMA_BEGIN;
	// BATCH_COMPLETE: state <= BATCH_IDLE;
	// BATCH_ERR: state <= BATCH_ERR;

	localparam FSM_BITS=3;
	localparam [FSM_BITS-1:0] BATCH_IDLE = 0;
	localparam [FSM_BITS-1:0] BATCH_DMA_BEGIN = 1;
	localparam [FSM_BITS-1:0] BATCH_DMA_WAIT = 2;
	localparam [FSM_BITS-1:0] BATCH_READ_WAIT = 3;
	localparam [FSM_BITS-1:0] BATCH_COMPLETE = 4;
	localparam [FSM_BITS-1:0] BATCH_ERR = 5;
	reg [FSM_BITS-1:0] batch_state = BATCH_IDLE;
	
	always @(posedge clk_i) begin
		if (rst_i || abort_batch) batch_state <= BATCH_IDLE;
		else begin
			case(batch_state)
				BATCH_IDLE: if (begin_batch) batch_state <= BATCH_DMA_BEGIN;
				BATCH_DMA_BEGIN: if (batch_data_valid) batch_state <= BATCH_DMA_WAIT;
				BATCH_DMA_WAIT: if (dma_complete_ok && dma_active) begin
											if (!cur_sequence_complete) batch_state <= BATCH_READ_WAIT;
											else batch_state <= BATCH_COMPLETE;
									 end else if (dma_err) batch_state <= BATCH_ERR;
				BATCH_READ_WAIT: batch_state <= BATCH_DMA_BEGIN;
				BATCH_COMPLETE: batch_state <= BATCH_IDLE;
				BATCH_ERR: batch_state <= BATCH_IDLE;
			endcase
		end
	end
	// Batch read goes at the end of each DMA, which means BATCH_DMA_WAIT or BATCH_COMPLETE.
	// (batch_err requires user reset)
	assign batch_read = (batch_state == BATCH_DMA_WAIT && dma_complete_ok && dma_active && !cur_sequence_complete) || (batch_state == BATCH_COMPLETE);
	// This contains the old 'test_dma_controller' DMA logic.
	basic_dma_engine u_engine(.target_begin_address({cur_target_begin_address,2'b00}),
									  .rd_address({cur_rd_address,2'b00}),
									  .transfer_length(cur_transfer_length),
									  .begin_dma(begin_dma),
									  .abort_dma(abort_dma),									  
									  .dma_complete_ok(dma_complete_ok),
									  .dma_active(dma_active),
									  .dma_err(dma_err),

									  `WBM_CONNECT(dmad, dmad),
									  `WBM_CONNECT(wbm, wbm),									  
									  
									  .clk_i(clk_i),
									  .rst_i(rst_i));


	// debug is a lot harder now
	assign debug_o[0 +: 32] = {cur_target_begin_address,2'b00};
	assign debug_o[32 +: 20] = {cur_rd_address, 2'b00};
	assign debug_o[52] = cur_sequence_complete;
	assign debug_o[53] = begin_dma;
	assign debug_o[54] = dma_active;
	assign debug_o[55] = dma_complete_ok;
	assign debug_o[56] = batch_read;
	assign debug_o[57] = begin_batch;
	assign debug_o[58 +: 3] = batch_state;
	/*
	assign debug_o[0 +: 32] = dmad_dat_i;
	assign debug_o[32] = dmad_cyc_o;
	assign debug_o[33] = dmad_ack_i;
	assign debug_o[34] = cur_sequence_complete;
	assign debug_o[35] = wbm_cyc_o;
	assign debug_o[36] = wbm_ack_i;
	assign debug_o[37 +: 32] = wbm_dat_o;
	assign debug_o[69] = dma_active;
	assign debug_o[70] = begin_dma;
	*/
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
