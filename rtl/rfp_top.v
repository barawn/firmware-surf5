`include "wishbone.vh"
module rfp_top(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),
		inout RFP_SDA,
		inout RFP_SCL
);
	
	wire [7:0] i2c_dat;
	wire i2c_ack;
	wire i2c_err;
	wire i2c_rty;
	wire scl_in;
	wire sda_in;
	wire sda_out;
	wire scl_out;
	wire scl_oen;
	wire sda_oen;
	assign scl_in = RFP_SCL;
	assign sda_in = RFP_SDA;
	assign RFP_SCL = (scl_oen) ? 1'bZ : scl_out;
	assign RFP_SDA = (sda_oen) ? 1'bZ : sda_out;
	i2c_master_top u_top(.wb_clk_i(clk_i),.wb_rst_i(rst_i),.arst_i(1'b1),.wb_adr_i(wb_adr_i[2:0]),.wb_dat_i(wb_dat_i[7:0]),.wb_dat_o(i2c_dat),
								.wb_cyc_i(wb_cyc_i),.wb_stb_i(wb_adr_i[15] == 0 && wb_stb_i),.wb_ack_o(i2c_ack),.wb_err_o(i2c_err),.wb_rty_o(i2c_rty),
								.wb_sel_i(wb_stb_i),
								.scl_pad_i(scl_in),.scl_pad_o(scl_out),.scl_padoen_o(scl_oen),
								.sda_pad_i(sda_in),.sda_pad_o(sda_out),.sda_padoen_o(sda_oen));
	assign wb_dat_o = wb_adr_i[15] ? 32'h00000000 : {{24{1'b0}},i2c_dat};
	assign wb_ack_o = wb_adr_i[15] ? wb_cyc_i && wb_stb_i : i2c_ack;
	assign wb_err_o = wb_adr_i[15] ? 1'b0 : i2c_err;
	assign wb_rty_o = wb_adr_i[15] ? 1'b0 : i2c_rty;
endmodule
