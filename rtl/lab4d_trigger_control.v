`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// This file is a part of the Antarctic Impulsive Transient Antenna (ANITA)
// project, a collaborative scientific effort between multiple institutions. For
// more information, contact Peter Gorham (gorham@phys.hawaii.edu).
//
// All rights reserved.
//
// Author: Patrick Allison, Ohio State University (allison.122@osu.edu)
// Author:
// Author:
////////////////////////////////////////////////////////////////////////////////
module lab4d_trigger_control(
		input clk_i,
		input sys_clk_i,
		input sys_clk_div4_flag_i,
		input sync_i,
		input start_i,
		input stop_i,
		output ready_o,
		// Bit of status.
		output [1:0] current_bank_o,
		// Configuration interface
		input rst_i,
		input [2:0] post_trigger_i,
		input post_trigger_wr_i,
		input [1:0] trigger_repeat_i,
		input trigger_repeat_wr_i,
		
		// Triggering interface
		input trigger_i,
		input force_trigger_i,

		// Trigger FIFO read interface
		output trigger_empty_o,
		input trigger_rd_i,
		output [4:0] trigger_address_o,
		input trigger_clear_i,
		
		output [15:0] trigger_debug_o,
		output [59:0] WR
    );
	reg pre_sys_clk_div4_flag = 0;
	reg [1:0] sys_clk_counter = {2{1'b0}};

	reg [1:0] bank = {2{1'b0}};
	wire [2:0] bank_plus_one = bank + 1;
	wire enable_next_bank;
	
	reg [1:0] bank_full_counter = {2{1'b0}};
	
	reg [2:0] window = {3{1'b0}};
	wire [3:0] window_plus_one = window + 1;
	wire enable_next_window;
	
	reg [2:0] post_trigger_counter = {3{1'b0}};
	reg [2:0] post_trigger_limit = {3{1'b0}};
	
	reg [1:0] trigger_repeat = {2{1'b0}};
	reg [1:0] repeat_count = {2{1'b0}};
	reg do_repeat = 0;
	reg reset_post_trigger = 0;
	
	reg [5:0] trigger_address = {6{1'b0}};	
	reg triggering = 0;
	reg trigger_write = 0;
	
	reg start_seen = 0;
	wire start_sysclk;
	reg stop_seen = 0;
	wire stop_sysclk;

	reg enabled_sysclk = 0;

	wire force_trigger_sysclk;	
	wire trigger_clear_sysclk;
	wire post_trigger_wr_sysclk;
	wire trigger_repeat_wr_sysclk;
	
	flag_sync u_start_sync(.in_clkA(start_i),.clkA(clk_i),.out_clkB(start_sysclk),.clkB(sys_clk_i));
	flag_sync u_stop_sync(.in_clkA(stop_i),.clkA(clk_i),.out_clkB(stop_sysclk),.clkB(sys_clk_i));
	flag_sync u_force_sync(.in_clkA(force_trigger_i),.clkA(clk_i),.out_clkB(force_trigger_sysclk),.clkB(sys_clk_i));
	flag_sync u_clear_sync(.in_clkA(trigger_clear_i),.clkA(clk_i),.out_clkB(trigger_clear_sysclk),.clkB(sys_clk_i));
	signal_sync u_enabled_sync(.in_clkA(enabled_sysclk),.clkA(sys_clk_i),.out_clkB(ready_o),.clkB(clk_i));
	
	flag_sync u_post_trigger_sync(.in_clkA(post_trigger_wr_i),.clkA(clk_i),.out_clkB(post_trigger_wr_sysclk),.clkB(sys_clk_i));
	flag_sync u_trigger_repeat_sync(.in_clkA(trigger_repeat_wr_i),.clkA(clk_i),.out_clkB(trigger_repeat_wr_sysclk),.clkB(sys_clk_i));
	reg enabled_sysclk_reg = 0;
	reg update_bank_sysclk = 0;
	wire update_bank;
	reg [1:0] cur_bank = {2{1'b0}};
	flag_sync u_update_bank(.in_clkA(update_bank_sysclk),.clkA(sys_clk_i),.out_clkB(update_bank),.clkB(clk_i));
	
	always @(posedge clk_i) begin
		if (update_bank) cur_bank <= bank;
	end
	
	always @(posedge sys_clk_i) begin
		if (sys_clk_div4_flag_i) sys_clk_counter <= {2{1'b0}};
		else sys_clk_counter <= sys_clk_counter + 1;
	
		// clk div4_flag counter pre_div4_flag
		// 0   1         x       0
		// 1   0         0       0
		// 2   0         1       0
		// 3   0         2       1
		// 4   1         3       0
		// 5   0         0       0
	
		// flag when div4_flag is *about* to go
		pre_sys_clk_div4_flag <= (sys_clk_counter == 2'b01);
	
		enabled_sysclk_reg <= enabled_sysclk;
		// this goes high every time the bank changes or we start up.
		// OR is a rising edge detection on enabled_sysclk
		update_bank_sysclk <= (enable_next_bank) || (!enabled_sysclk_reg && enabled_sysclk);
	
		if (start_sysclk) start_seen <= 1;
		else if (enabled_sysclk) start_seen <= 0;
		
		if (stop_sysclk) stop_seen <= 1;
		else if (!enabled_sysclk) stop_seen <= 0;
		
		// Start up only when we're exiting sync=1. We then start with write address 0.
		// End at a boundary.
		if (start_seen && sys_clk_div4_flag_i && sync_i) enabled_sysclk <= 1;
		else if (stop_seen && sys_clk_div4_flag_i) enabled_sysclk <= 0;

		// Move forward a window at each sys_clk_div4_flag. Reset if not enabled.
		if (enabled_sysclk) begin
			if (sys_clk_div4_flag_i) window <= window + 1;
		end
			else window <= {3{1'b0}};
		
		// If we get a trigger, set to triggering state. Once we hit the post-trigger limit, exit that state.
		if (trigger_i || force_trigger_sysclk) triggering <= 1;
		else if (post_trigger_counter == post_trigger_limit && sys_clk_div4_flag_i && !do_repeat) triggering <= 0;

		// Capture address when we transition. Trigger address indicates LAST window.
		if (triggering && post_trigger_counter == post_trigger_limit && sys_clk_div4_flag_i) 
			trigger_address <= {window[0],bank,window[2:1]};

		// Move to next buffer when we hit post trigger limit.
		if (enable_next_bank) bank <= bank + 1;
		else if (!enabled_sysclk) bank <= {2{1'b0}};
		
		// Write the trigger into the FIFO when we hit the post trigger limit. trigger_write=1 and trigger_address latch happen at same time.
		if (triggering && post_trigger_counter == post_trigger_limit && sys_clk_div4_flag_i) trigger_write <= 1;
		else trigger_write <= 0;

		// Count the post trigger counter when triggering, but not when repeating. (do_repeat essentially acts as another trigger)
		if (!triggering || do_repeat) post_trigger_counter <= {3{1'b0}};
		else if (sys_clk_div4_flag_i) post_trigger_counter <= post_trigger_counter + 1;

		// Grab the limit from the control interface.
		if (post_trigger_wr_sysclk || reset_post_trigger) post_trigger_limit <= post_trigger_i;		
		else if (triggering && do_repeat) post_trigger_limit <= 7;

		if (trigger_repeat_wr_sysclk) trigger_repeat <= trigger_repeat_i;

		// this indicates that the trigger should be repeated (continued readout)
		// The logic here ensures that do_repeat is definitely high when triggering goes high.
		if ((triggering || trigger_i || force_trigger_sysclk) && (post_trigger_counter == post_trigger_limit) && pre_sys_clk_div4_flag && (repeat_count != trigger_repeat))
			do_repeat <= 1;
		else
			do_repeat <= 0;

		if (triggering && (post_trigger_counter == post_trigger_limit) && pre_sys_clk_div4_flag && (repeat_count == trigger_repeat))
			reset_post_trigger <= 1;
		else
			reset_post_trigger <= 0;

		if (triggering && (post_trigger_counter == post_trigger_limit) && sys_clk_div4_flag_i && do_repeat)
			repeat_count <= repeat_count + 1;
		else if (!triggering)
			repeat_count <= {2{1'b0}};
				
		// DO SOMETHING SMART HERE TO DISABLE TRIGGERS!!
		if (trigger_write) bank_full_counter <= bank_full_counter + 1;
		else if (trigger_clear_sysclk) bank_full_counter <= bank_full_counter - 1;

	end

	trigger_fifo u_fifo(.din(trigger_address),.dout(trigger_address_o),.rd_clk(clk_i),.wr_clk(sys_clk_i),
							  .wr_en(trigger_write),.rd_en(trigger_rd_i),.empty(trigger_empty_o),
							  .rst(rst_i));

	assign enable_next_bank = (triggering && post_trigger_counter == post_trigger_limit && sys_clk_div4_flag_i);
	assign enable_next_window = (sys_clk_div4_flag_i);

	// The WR[4:0] map here is to accomodate LAB4 weirdnesss.
	// WR[4] has to toggle every SST, so that makes *it* the LSB.
	// In the PicoBlaze code, what we have to do is mask off
	// 10 011 = 13 (e.g. address = address  0x13)
	// and then do
	// convert address
	// convert address | 0x10
	// convert address+1
	// convert address+1 | 0x10
	// convert address+2
	// convert address+2 | 0x10
	// convert address+3
	// convert address+3 | 0x10
	generate
		genvar i,j;
		for (i=0;i<12;i=i+1) begin : LAB
			(* IOB = "TRUE" *)
			FDRE u_wr4(.D(window_plus_one[0]),
						  .CE(enable_next_window),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk),
						  .Q(WR[5*i+4]));
			FDRE u_wr3(.D(bank_plus_one[1]),
						  .CE(enable_next_bank),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk),
						  .Q(WR[5*i+3]));
			FDRE u_wr2(.D(bank_plus_one[0]),
						  .CE(enable_next_bank),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk),
						  .Q(WR[5*i+2]));
			FDRE u_wr1(.D(window_plus_one[2]),
						  .CE(enable_next_window),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk),
						  .Q(WR[5*i+1]));
			FDRE u_wr0(.D(window_plus_one[1]),
						  .CE(enable_next_window),
						  .C(sys_clk_i),
						  .R(!enabled_sysclk),
						  .Q(WR[5*i+0]));
		end
	endgenerate
	assign trigger_debug_o[5:0] = {window[0],bank,window[2:1]};
	assign trigger_debug_o[6] = trigger_i;
	assign trigger_debug_o[7] = force_trigger_sysclk;
	assign trigger_debug_o[8] = trigger_write;
	assign trigger_debug_o[9] = triggering;
	assign trigger_debug_o[10] = rst_i;
	assign trigger_debug_o[11] = enabled_sysclk;
	assign trigger_debug_o[12] = start_sysclk;
	assign trigger_debug_o[13] = stop_sysclk;
	assign trigger_debug_o[14] = enable_next_bank;
	assign trigger_debug_o[15] = 0;

	assign current_bank_o = cur_bank;
endmodule
