`timescale 1ns / 1ps
module signal_sync(
		input in_clkA,
		input clkA,
		output out_clkB,
		input clkB
    );

	reg [1:0] synced_in_B = {2{1'b0}};
	always @(posedge clkB) begin
		synced_in_B <= {synced_in_B[0],in_clkA};
	end
	assign out_clkB = synced_in_B[1];

endmodule
