`include "wishbone.vh"
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:20:26 06/06/2016 
// Design Name: 
// Module Name:    lab4d_ram 
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
module lab4d_ram(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),
		input sys_clk_i,
		input readout_i,
		input [15:0] prescale_i,
		output complete_o,
		output [23:0] readout_debug_o,
		input [11:0] DOE_LVDS_P,
		input [11:0] DOE_LVDS_N,
		output [11:0] SS_INCR,
		output [11:0] SRCLK_P,
		output [11:0] SRCLK_N
    );
	parameter [11:0] SRCLK_POLARITY = 12'b110000000011;
	parameter [11:0] DOE_POLARITY = 12'b010000000100;
	wire [11:0] SRCLK;
	wire [11:0] DOE;
	wire [11:0] DOE_B;
	generate
		genvar kk;
		for (kk=0;kk<12;kk=kk+1) begin : DIFFLOOP
			if (SRCLK_POLARITY[kk] == 0) begin : POS
				OBUFDS u_srclk_p(.I(SRCLK[kk]),.O(SRCLK_P[kk]),.OB(SRCLK_N[kk]));
			end else begin : NEG
				OBUFDS u_srclk_n(.I(~SRCLK[kk]),.O(SRCLK_N[kk]),.OB(SRCLK_P[kk]));
			end
			if (DOE_POLARITY[kk] == 0) begin : DOEPOS
				IBUFDS_DIFF_OUT u_doe_p(.I(DOE_LVDS_P[kk]),.IB(DOE_LVDS_N[kk]),.O(DOE[kk]),.OB(DOE_B[kk]));
			end else begin : DOENEG
				IBUFDS_DIFF_OUT u_doe_n(.I(DOE_LVDS_N[kk]),.IB(DOE_LVDS_P[kk]),.O(DOE_B[kk]),.OB(DOE[kk]));
			end
		end
	endgenerate

	wire data_wr;
	wire [143:0] data;
	wire [31:0] data_out[15:0];
	wire [11:0] lab_read;


	localparam FSM_BITS=2;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] READ = 1;
	localparam [FSM_BITS-1:0] ACK = 2;
	reg [FSM_BITS-1:0] state = IDLE;
	always @(posedge clk_i) begin
		case (state)
			IDLE: if (wb_cyc_i && wb_stb_i && !wb_we_i) state <= READ;	// read is 1 here
			READ: state <= ACK;														// data is at output here
			ACK: state <= IDLE;														// data is at register here
		endcase
	end
	
	wire fifo_read = (wb_cyc_i && wb_stb_i) && (state == IDLE);
	assign wb_ack_o = (state == ACK);
	
	assign data_out[12] = data_out[4];
	assign data_out[13] = data_out[5];
	assign data_out[14] = data_out[6];
	assign data_out[15] = data_out[7];
	assign wb_dat_o = data_out[wb_adr_i[14:11]];
	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : FIFOS
			// Each LAB gets 512 entries of space, or 2048 bytes: so e.g. from 0000-07FF.
			// So the lab selection picks off bits [14:11].
			assign lab_read[i] = fifo_read && (wb_adr_i[14:11] == i);
			lab4d_fifo u_fifo(.wr_clk(sys_clk_i),.wr_en(data_wr),.din({{4{1'b0}},data[12*i +: 12]}),
									.rd_clk(clk_i),.rd_en(lab_read[i]),.dout(data_out[i]));
		end
	endgenerate
	
	wire dbg_ss_incr;
	wire dbg_srclk;
	lab4d_data_shift_register_x12 u_shreg(.sys_clk_i(sys_clk_i),.readout_i(readout_i),.done_o(complete_o),.dat_o(data),
													  .dat_wr_o(data_wr),.prescale_i(prescale_i),
													  .ss_incr_o(dbg_ss_incr),.srclk_o(dbg_srclk),.DOE(DOE),.SS_INCR(SS_INCR),.SRCLK(SRCLK));

	assign readout_debug_o[0 +: 12] = data[0 +: 12];
	assign readout_debug_o[12] = DOE[0];
	assign readout_debug_o[13] = data_wr;
	assign readout_debug_o[14] = readout_i;
	assign readout_debug_o[15] = dbg_ss_incr;
	assign readout_debug_o[16] = dbg_srclk;
	assign readout_debug_o[23:17] = {7{1'b0}};
endmodule
