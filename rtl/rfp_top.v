`include "wishbone.vh"
module rfp_top(
		input clk_i,
		input rst_i,
		`WBS_NAMED_PORT(wb, 32, 16, 4),
		inout RFP_DAC_SDA,
		inout RFP_DAC_SCL,
		inout [3:0] RFP_SDA,
		inout [3:0] RFP_SCL
);
	
	wire [7:0] i2c_dat[7:0];
	wire [7:0] i2c_ack;
	wire [7:0] i2c_err;
	wire [7:0] i2c_rty;
	wire [7:0] i2c_dat_muxed = i2c_dat[wb_adr_i[7:5]];
	wire i2c_ack_muxed = i2c_ack[wb_adr_i[7:5]];
	wire i2c_err_muxed = i2c_err[wb_adr_i[7:5]];
	wire i2c_rty_muxed = i2c_err[wb_adr_i[7:5]];
	
	assign i2c_dat[5] = i2c_dat[1];
	assign i2c_dat[6] = i2c_dat[2];
	assign i2c_dat[7] = i2c_dat[3];
	assign i2c_ack[7:5] = i2c_ack[3:1];
	assign i2c_err[7:5] = i2c_err[3:1];
	assign i2c_rty[7:5] = i2c_rty[3:1];
	
	wire i2c_stb[4:0];
	wire scl_in[4:0];
	wire sda_in[4:0];
	wire sda_out[4:0];
	wire scl_out[4:0];
	wire scl_oen[4:0];
	wire sda_oen[4:0];
	assign scl_in[0] = RFP_DAC_SCL;
	assign sda_in[0] = RFP_DAC_SDA;
	assign RFP_DAC_SCL = (scl_oen[0]) ? 1'bZ : scl_out[0];
	assign RFP_DAC_SDA = (sda_oen[0]) ? 1'bZ : sda_out[0];
	generate
		genvar i,j;
		for (i=0;i<4;i=i+1) begin : SCL_SDA_ASSIGN
			assign scl_in[i+1] = RFP_SCL[i];
			assign sda_in[i+1] = RFP_SDA[i];
			assign RFP_SCL[i] = (scl_oen[i+1]) ? 1'bZ : scl_out[i+1];
			assign RFP_SDA[i] = (sda_oen[i+1]) ? 1'bZ : sda_out[i+1];
		end
		for (j=0;j<5;j=j+1) begin : I2C_CORE_LOOP
			assign i2c_stb[j] = (!wb_adr_i[15] && wb_adr_i[7:5]==j) && wb_stb_i;
			i2c_master_top #(.WB_LATENCY(0),.ARST_LVL(1'b1)) u_top(.wb_clk_i(clk_i),.wb_rst_i(1'b0),.wb_adr_i(wb_adr_i[4:2]),.wb_dat_i(wb_dat_i[7:0]),.wb_dat_o(i2c_dat[j]),
										.wb_cyc_i(wb_cyc_i),.wb_stb_i(i2c_stb[j]),.wb_ack_o(i2c_ack[j]),.wb_err_o(i2c_err[j]),.wb_rty_o(i2c_rty[j]),
										.wb_we_i(wb_we_i),
										.wb_sel_i(wb_stb_i),
										.scl_pad_i(scl_in[j]),.scl_pad_o(scl_out[j]),.scl_padoen_o(scl_oen[j]),
										.sda_pad_i(sda_in[j]),.sda_pad_o(sda_out[j]),.sda_padoen_o(sda_oen[j]));
		end		
	endgenerate
	
	assign wb_dat_o = wb_adr_i[15] ? 32'h00000000 : {{24{1'b0}},i2c_dat_muxed};
	assign wb_ack_o = wb_adr_i[15] ? wb_cyc_i && wb_stb_i : i2c_ack_muxed;
	assign wb_err_o = wb_adr_i[15] ? 1'b0 : i2c_err_muxed;
	assign wb_rty_o = wb_adr_i[15] ? 1'b0 : i2c_rty_muxed;
endmodule
