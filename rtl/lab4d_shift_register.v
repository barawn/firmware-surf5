`timescale 1ns / 1ps
module lab4d_shift_register(
		input clk_i,
		input go_i,
		input [23:0] dat_i,
		input [3:0] sel_i,
		input [7:0] prescale_i,
		output busy_o,
		output [11:0] SIN,
		output [11:0] SCLK,
		output [11:0] PCLK
    );

	localparam NUM_BITS = 24;

	reg [7:0] prescale_counter = {8{1'b0}};
	reg [4:0] bit_counter = {5{1'b0}};
	reg [23:0] shift_reg = {24{1'b0}};
	reg sclk = 0;
	reg pclk = 0;
	wire [11:0] lab_is_selected;
	
	localparam FSM_BITS = 3;
	localparam [FSM_BITS-1:0] IDLE = 0;
	localparam [FSM_BITS-1:0] LOAD = 1;
	localparam [FSM_BITS-1:0] SCLK_HIGH = 2;
	localparam [FSM_BITS-1:0] SCLK_LOW = 3;
	localparam [FSM_BITS-1:0] PCLK_LOW = 4;
	localparam [FSM_BITS-1:0] PCLK_HIGH = 5;
	localparam [FSM_BITS-1:0] SIN_HIGH = 6;
	localparam [FSM_BITS-1:0] DONE = 7;
	reg [FSM_BITS-1:0] state = IDLE;
	
	always @(posedge clk_i) begin
		if (state == LOAD) shift_reg <= dat_i;
		else if (state == SCLK_HIGH && prescale_counter == prescale_i) shift_reg <= {shift_reg[22:0],1'b0};
		else if (state == PCLK_HIGH && prescale_counter == prescale_i) shift_reg[23] <= 1;
		else if (state == DONE) shift_reg[23] <= 0;
		
		sclk <= (state == SCLK_HIGH);
		pclk <= (state == PCLK_HIGH || state == SIN_HIGH);
		
		if (state == IDLE || prescale_counter == prescale_i) prescale_counter <= {8{1'b0}};
		else prescale_counter <= prescale_counter + 1;
		
		if (state == IDLE) bit_counter <= {5{1'b0}};
		if (state == SCLK_HIGH && prescale_counter == prescale_i) bit_counter <= bit_counter + 1;

		case (state)
			IDLE: if (go_i) state <= LOAD;
			LOAD: if (prescale_counter == prescale_i) state <= SCLK_HIGH;
			SCLK_HIGH: if (prescale_counter == prescale_i) state <= SCLK_LOW;
			SCLK_LOW: if (prescale_counter == prescale_i) begin
								if (bit_counter == NUM_BITS) state <= PCLK_LOW;
								else state <= SCLK_HIGH;
						 end
			PCLK_LOW: if (prescale_counter == prescale_i) state <= PCLK_HIGH;
			PCLK_HIGH: if (prescale_counter == prescale_i) state <= SIN_HIGH;
			SIN_HIGH: if (prescale_counter == prescale_i) state <= DONE;
			DONE: state <= IDLE;
		endcase
	end
	generate
		genvar i;
		for (i=0;i<12;i=i+1) begin : LABS
			assign lab_is_selected[i] = (sel_i == i) || (sel_i == 4'd15);
			(* IOB = "TRUE" *)
			FDRE u_sclk(.D(sclk),.C(clk_i),.R(!lab_is_selected[i]),.CE(1'b1),.Q(SCLK[i]));
			(* IOB = "TRUE" *)
			FDRE u_pclk(.D(pclk),.C(clk_i),.R(!lab_is_selected[i]),.CE(1'b1),.Q(PCLK[i]));
			(* IOB = "TRUE" *)
			FDRE u_sin(.D(shift_reg[23]),.C(clk_i),.R(!lab_is_selected[i]),.CE(1'b1),.Q(SIN[i]));
		end
	endgenerate
	
	assign busy_o = (state != IDLE);
endmodule
