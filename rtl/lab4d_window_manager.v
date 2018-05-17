`timescale 1ns / 1ps
module lab4d_window_manager(
		input [4:0] pretrigger_i,
		input [4:0] posttrigger_i,
		input [4:0] readout_i,

		input sys_clk_i,
		input sys_clk_div4_flag_i,
		input sync_i,
		input clk_i,

		input start_i,
		input stop_i,
		input rst_i,
		output rst_ack_o,
		output ready_o,
		
		output [4:0] window_o,
		
		// need to do something here to identify trigger boundaries.
		// ... why bother? Not really important. If you know how many
		// windows correspond to an event, it's fine.
		output [4:0] digitize_o,
		input digitize_rd_i,
		output digitize_empty_o,
		
		input [4:0] done_i,
		input done_wr_i,
		
		input trigger_i,
		output dead_o		
    );

	//% Window that we're going to push back into the available FIFO.
	reg [4:0] refill_window = {5{1'b0}};
	//% Refill window plus one.
	wire [5:0] refill_window_plus_one = refill_window + 1;
	
	//% Window to push into the available FIFO during the RESTORE phases. (Always in pairs).
	reg [4:0] restore_window = {5{1'b0}};
	//% Storage for the sampling window output (for using during restoration when dead).
	reg [4:0] sampling_window_store = {5{1'b0}};
	//% Storage for the done window output (to ensure they're restored in pairs).
	reg [4:0] done_window_store = {5{1'b0}};
	//% Phase (LSB) of the window that was stored in the last refill phase. 
	reg last_refill_phase = 0;

	//% Indicates that there are sampling restores pending.
	reg sampling_restore_pending = 0;
	//% Indicates that there are windows ready to be restored from the done FIFO.
	reg done_restore_pending = 0;
	//% Indicates that there are enough Done windows.
	reg done_has_enough = 0;
	
	//% Indicates that a restore will happen.
	reg do_restore = 0;
	//% Indicates that a refill will happen.
	reg do_refill = 0;
	//% Set when no trigger can occur due to lack of available windows.
	reg dead = 0;
	//% Set when we *will* go dead when the next trigger occurs.
	reg will_go_dead = 0;
	//% Set when we're recovering from being dead.
	reg coming_alive = 0;
	//% Set when windows are actively being moved from sampling to digitization.
	reg triggering = 0;
	//% Count of windows digitized for a trigger.
	reg [4:0] trigger_count = {5{1'b0}};
	//% Indicates that the window manager is running.
	reg running = 0;
	//% Set when a start flag has been seen, but window manager is waiting to start running.
	reg start_seen = 0;
	//% Set when a stop flag has been seen.
	reg stop_seen = 0;
	//% Set when an input trigger is seen.
	reg trig_seen = 0;
	//% Read from the done FIFO.
	reg done_rd = 0;
	//% Trigger is ending.
	reg trigger_ending = 0;
	//% Reset is done.
	reg reset_ack = 0;

	//% Number of pretrigger windows. Latched at run start.
	reg [4:0] pretrigger = {5{1'b0}};
	//% Number of readout windows.
	reg [4:0] readout = {5{1'b0}};
	
	//% Read enable for the available FIFO.
	reg read_from_available = 0;
	//% Clock enable for the sampling shift register
	reg clock_sampling = 0;
	//% sampling shift register
	reg [5:0] sampling_shift_register[31:0];
	integer i,j;
	initial for (i=0;i<31;i=i+1) sampling_shift_register[i] <= {6{1'b0}};
	
	wire [4:0] sampling_out = sampling_shift_register[pretrigger][4:0];
	wire sampling_valid = sampling_shift_register[pretrigger][5];
	
	wire [4:0] available_out;
	wire [4:0] done_out;
	wire done_almost_empty;
	
	//% Number of bits in the state vector.
	localparam FSM_BITS = 3;
	//% FSM is in reset state.
	localparam [FSM_BITS-1:0] RESET = 3'h0;
	//% FSM is initializing.
	localparam [FSM_BITS-1:0] INITIALIZE = 3'h1;
	//% FSM is idle, waiting for run start.
	localparam [FSM_BITS-1:0] IDLE = 3'h2;
	//% Running, clock 0. New digitization window pops off FIFO.
	localparam [FSM_BITS-1:0] RUN_READOUT = 3'h3;
	//% Running, clock 1. Push window back to available FIFO.
	localparam [FSM_BITS-1:0] RUN_REFILL = 3'h4;
	//% Running, clock 2. Restore first window in a window pair from either sampling (if dead) or done.
	localparam [FSM_BITS-1:0] RUN_RESTORE_0 = 3'h5;
	//% Running, clock 3. Restore the other window in the pair.
	localparam [FSM_BITS-1:0] RUN_RESTORE_1 = 3'h6;
	//% FSM is stopping.
	localparam [FSM_BITS-1:0] STOPPING = 3'h7;
	//% FSM state vector.
	reg [FSM_BITS-1:0] state = RESET;
	
	wire [4:0] available_window = (state == RUN_REFILL || state == INITIALIZE) ? refill_window : restore_window;
	wire [5:0] available_count;
	
	always @(posedge sys_clk_i) begin
		if (start_i) begin
			readout <= readout_i;
			pretrigger <= pretrigger_i;
		end
		reset_ack <= (state == INITIALIZE) && (refill_window_plus_one[5]);
		if (rst_i) state <= RESET;
		else begin case (state)
			RESET: state <= INITIALIZE;
			INITIALIZE: begin
				if (refill_window_plus_one[5]) state <= IDLE;
			end
			IDLE: if (start_seen && sys_clk_div4_flag_i && sync_i) state <= RUN_READOUT;
			RUN_READOUT: state <= RUN_REFILL;
			RUN_REFILL: state <= RUN_RESTORE_0;
			RUN_RESTORE_0: state <= RUN_RESTORE_1;
			RUN_RESTORE_1: if (stop_seen) state <= STOPPING; else state <= RUN_READOUT;
			// I dunno, do something here.
			STOPPING: state <= IDLE;
			endcase
		end
		if (start_i) start_seen <= 1;
		else if (state == RUN_READOUT) start_seen <= 0;
		
		// Determine the refill window. If we're initializing, we just count.
		if (rst_i) refill_window <= {5{1'b0}};
		else if (state == INITIALIZE) refill_window <= refill_window_plus_one;
		else if (state == RUN_READOUT) begin
			// If we're dead, we bypass the sampling shift register.
			if (dead) refill_window <= available_out;
			// If we're not triggering, then we refill the window 
			// that fell out of the sampling shift register.
			else if (!triggering) refill_window <= sampling_out;
		end
		// Determine if a refill happens.
		// This occurs initially (to reset the system), and also if we're either
		// not storing windows for readout (!triggering) or dead (in which case
		// we're bypassing the sampling shift register altogether).
		if (rst_i) do_refill <= 0;
		else if (state == RESET) do_refill <= 1;
		else if (state == INITIALIZE) begin
			do_refill <= !refill_window_plus_one[5];
		end else begin
			if (state == RUN_READOUT) begin
				if ((!triggering && sampling_valid) || (dead && !coming_alive)) do_refill <= 1;
				else do_refill <= 0;
			end else do_refill <= 0;
		end
		
		if (state == RUN_READOUT) begin
			if (dead) last_refill_phase <= available_out[0];
			else if (!triggering && sampling_valid) last_refill_phase <= sampling_out[0];
		end
		
		// Determine if there are sampling windows to restore.
		// This only happens when we're dead. Sampling windows take priority
		// over done windows.
		if (state == RUN_READOUT) begin
			if (dead && !triggering && sampling_valid) begin
				sampling_window_store <= sampling_out;
				sampling_restore_pending <= 1;
			end
		end else if (state == RUN_RESTORE_1) begin
			sampling_restore_pending <= 0;
		end
		// Determine if there are done windows to restore.
		// We need the done pending to be the same between RUN_RESTORE_1 and RUN_READOUT.
		// So we only check at a specific point in the cycle.
		if (state == RUN_RESTORE_0) begin
			if (!done_restore_pending) done_has_enough <= !done_almost_empty;
			else if (!sampling_restore_pending && done_restore_pending) done_has_enough <= 0;
		end
		// If there are done windows to restore, latch the first one, and indicate we're ready.
		// At run_restore_1, we'll clear the pending if we actually did it.
		if (state == RUN_READOUT) begin
			if (done_has_enough && !done_restore_pending) begin
				done_window_store <= done_out;
				done_restore_pending <= 1;
			end
		end else if (state == RUN_RESTORE_1) begin
			if (!sampling_restore_pending) done_restore_pending <= 0;
		end
		
		// Done windows get latched at RUN_READOUT, so we want done_rd to go high
		// then. So we do it one cycle early (RUN_RESTORE_1).
		// Then we obviously need to increment again once we know we're actually
		// going to restore, which we can know at RUN_RESTORE_0.
		if (state == RUN_RESTORE_1) begin
			if (!done_restore_pending && done_has_enough) done_rd <= 1;
			else done_rd <= 0;
		end else if (state == RUN_RESTORE_0) begin
			if (done_restore_pending && !sampling_restore_pending) done_rd <= 1;
			else done_rd <= 0;
		end else done_rd <= 0;

		// Restore windows if either the done FIFO or sampling shift register have windows to restore.
		if (state == RUN_RESTORE_1) do_restore <= 0;
		else if (state == RUN_REFILL) do_restore <= (done_restore_pending || sampling_restore_pending);		
		
		// Choose the restore windows from the sampling shift register or the done FIFO.
		if (state == RUN_REFILL) begin
			if (sampling_restore_pending) begin
				if (sampling_window_store[0] != last_refill_phase) restore_window <= sampling_window_store;
				else restore_window <= sampling_out;
			end else if (done_restore_pending) begin
				if (done_window_store[0] != last_refill_phase) restore_window <= done_window_store;
				else restore_window <= done_out;
			end
		end else if (state == RUN_RESTORE_0) begin
			if (sampling_restore_pending) begin
				if (sampling_window_store[0] != last_refill_phase) restore_window <= sampling_out;
				else restore_window <= sampling_window_store;
			end else if (done_restore_pending) begin
				if (done_window_store[0] != last_refill_phase) restore_window <= done_out;
				else restore_window <= done_window_store;
			end
		end

		if (start_seen && sys_clk_div4_flag_i && sync_i) running <= 1;
		else if (state == STOPPING) running <= 0;

		// no idea if this is going to work.
		if (clock_sampling) begin
			sampling_shift_register[0] <= {!dead && running,available_out};
			for (j=1;j<32;j=j+1) sampling_shift_register[j] <= sampling_shift_register[j-1];
		end
		// Shift register clocks 
		// Shift register clocks before the readout stage, so clock enable 2 clocks before.
		// It also clocks if we're not triggering and we're dead.
		// (e.g. we're emptying the sampling shift register). Also when we're initializing.
		clock_sampling <= (state == RUN_RESTORE_0) || (state == RUN_RESTORE_1 && dead && (!triggering || trigger_ending)) || (state == INITIALIZE);
		
		// triggering or something
		if (trigger_i && !dead) trig_seen <= 1;
		else if (state == RUN_RESTORE_1) trig_seen <= 0;

		// Trigger check. Happens right before a new window gets popped out.
		if (state == RUN_RESTORE_1) begin
			if (trig_seen) triggering <= 1;
			// do something about counting
			else if (trigger_ending) triggering <= 0;
		end
		if (state == RUN_RESTORE_1) begin
			if (triggering && !trigger_ending) trigger_count <= trigger_count + 1;
			else trigger_count <= {5{1'b0}};
		end
		// Trigger ending goes high 1 clock after RUN_RESTORE_1, but that's fine
		// as it won't be checked until next time anyway.
		trigger_ending <= (trigger_count == readout-1);

		// will_go_dead determination. Happens right before trigger determination.
		// RUN_RESTORE_0 is off by one (it's too big) so this is the reason for the
		// "readout + 3" here.
		if (state == RUN_RESTORE_0) begin
			if (available_count < readout+3)
				will_go_dead <= 1;
			else
				will_go_dead <= 0;
		end

		// End 'dead' 
		if (state == RUN_RESTORE_1 && trig_seen && will_go_dead) dead <= 1;
		else if (state == RUN_READOUT && coming_alive) dead <= 0;

		// bleh, we'll see what this does.
		// Check at RUN_READOUT because it has the correct count.
		if (!dead) coming_alive <= 0;
		else if (state == RUN_READOUT && (available_count >= pretrigger + 2)) coming_alive <= 1;
	end		

	lab4d_window_fifo u_available (
	  .clk(sys_clk_i), // input clk
	  .srst(rst_i), // input srst
	  .din(available_window), // input [4 : 0] din
	  .wr_en(do_refill||do_restore), // input wr_en
	  .rd_en(sys_clk_div4_flag_i && running), // input rd_en
	  .dout(available_out), // output [4 : 0] dout
	  .full(), // output full
	  .empty(), // output empty
	  .almost_empty(), // output almost_empty
	  .data_count(available_count) // output [5 : 0] data_count
	);
	// Asynchronous FIFO with almost empty for the done FIFO.
	lab4d_window_async_fifo u_done(
	  .rd_clk(sys_clk_i),
	  .wr_clk(clk_i),
	  .rst(rst_i),
	  .din(done_i),
	  .wr_en(done_wr_i),
	  .rd_en(done_rd),
	  .dout(done_out),
	  .almost_empty(done_almost_empty));
	// Asynchronous FIFO for the digitize FIFO.
	lab4d_window_async_fifo u_digitize(
		.rd_clk(clk_i),
		.rst(rst_i),
		.din(sampling_out),
		.wr_en(triggering && (state == RUN_READOUT)),
		.wr_clk(sys_clk_i),
		.rd_en(digitize_rd_i),
		.dout(digitize_o),
		.empty(digitize_empty_o));
	
	assign ready_o = running;
	assign dead_o = dead;
	assign rst_ack_o = reset_ack;
	assign window_o = available_out;
	
endmodule
