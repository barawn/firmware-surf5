`timescale 1ns / 1ps
//Using Two Simple Priority Arbiters with a Mask - scalable
//author: dongjun_luo@hotmail.com
//https://github.com/freecores/round_robin_arbiter
//public domain
// altered for synchronous reset
module round_robin_arbiter (
	rst_n,
	clk,
	req,
	grant
);

parameter N = 4;

input		rst_n;
input		clk;
input	[N-1:0]	req;
output	[N-1:0]	grant;

reg	[N-1:0]	rotate_ptr = {N{1'b1}};
wire	[N-1:0]	mask_req;
wire	[N-1:0]	mask_grant;
wire	[N-1:0]	grant_comb;
reg	[N-1:0]	grant_reg = {N{1'b0}};
wire		no_mask_req;
wire	[N-1:0] nomask_grant;
wire		update_ptr;
genvar i;

// rotate pointer update logic
assign update_ptr = |grant[N-1:0];
always @ (posedge clk)
begin
	if (!rst_n)
// This won't work, it multiply-drives bits from 2 to N-1
//		rotate_ptr[N-1:0] <= {N{1'b1}};
	   rotate_ptr[1:0] <= 2'b11;
	else if (update_ptr)
	begin
		// note: N must be at least 2
		rotate_ptr[0] <= grant[N-1];
		rotate_ptr[1] <= grant[N-1] | grant[0];
	end
end

generate
for (i=2;i<N;i=i+1)
always @ (posedge clk) 
begin : ROTATE_LOOP
	if (!rst_n)
		rotate_ptr[i] <= 1'b1;
	else if (update_ptr)
		rotate_ptr[i] <= grant[N-1] | (|grant[i-1:0]);
end
endgenerate

// mask grant generation logic
assign mask_req[N-1:0] = req[N-1:0] & rotate_ptr[N-1:0];

assign mask_grant[0] = mask_req[0];
generate
for (i=1;i<N;i=i+1) begin : MASK_LOOP
	assign mask_grant[i] = (~|mask_req[i-1:0]) & mask_req[i];
end
endgenerate

// non-mask grant generation logic
assign nomask_grant[0] = req[0];
generate
for (i=1;i<N;i=i+1) begin : NOMASK_LOOP
	assign nomask_grant[i] = (~|req[i-1:0]) & req[i];
end
endgenerate

// grant generation logic
assign no_mask_req = ~|mask_req[N-1:0];
assign grant_comb[N-1:0] = mask_grant[N-1:0] | (nomask_grant[N-1:0] & {N{no_mask_req}});

always @ (posedge clk)
begin
	if (!rst_n)	grant_reg[N-1:0] <= {N{1'b0}};
	else		grant_reg[N-1:0] <= grant_comb[N-1:0] & ~grant_reg[N-1:0];
end

assign grant = grant_reg;
endmodule
