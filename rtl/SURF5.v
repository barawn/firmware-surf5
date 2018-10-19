`timescale 1ns / 1ps
`include "wishbone.vh"
`include "pci.vh"

module SURF5(
		//Local clocks
		input 	      LOCAL_CLK,
		output			LOCAL_OSC_EN,

		//External trigger - externally 50 ohm terminated - 2.5V bank
		input 	      EXT_TRIG,

		//Front Panel LED
		output 	      FP_LED,

		//SSTs and controls
		output 	      FPGA_SST_N,
		output 	      FPGA_SST_P,// Note inout: this is a 3-state pin.
		inout 	      FPGA_SST_SEL,
		//TURF_derived clock for LABs
		input 	      FPGA_TURF_SST_N,
		input 	      FPGA_TURF_SST_P, 
		
		//LEDs
		inout [3:0]   LED,
		
		//MONITORING PINS
		input				MON0,
		output			MON1,
		output			MON2,
		inout				MON3,
		input				MON4,

		//PCI SIGNALS
		// Directional.
		input 	      PCI_CLK,
		inout 	      pci_rst, 
		input 	      pci_idsel,
		input 	      pci_gnt, 
		output 	      pci_req,
		// *Always* bidirectional. Shared bus.
		inout [31:0]  	pci_ad,
		inout 	      pci_perr, 
		inout				pci_par,
		inout 	      pci_trdy, 
		inout 	      pci_devsel, 
		inout 	      pci_stop, 
		
		inout [3:0]   	pci_cbe,
		inout 	      pci_frame, 
		inout 	      pci_irdy, 

		inout 	      pci_inta, 
		inout 	      pci_serr, 


		//TURF interface - comments on directionality if not SURF outputs
		output [7:0]  TD_P,
		output [7:0]  TD_N,
		output 	      SCLK_N,
		output 	      SCLK_P,
		
		// TURFbus control interface. Figure something out here.
		output 	      SREQ_neg, 
		input 	      TREQ_neg, 
		input 	      TCLK_N, 
		input 	      TCLK_P, 

		// PPS fanout from TURF.
		input 	      PPS_N, 
		input 	      PPS_P, 

		// Lock buffer. Digitize request comes via TURFbus.
		input [3:0]   HOLD,

		// LAB4 interface
		output [11:0]	SCLK,
		output [11:0]  SIN,
		output [11:0] 	PCLK,
		input  [11:0] 	SHOUT,
		output [11:0]	REGCLR,
		
		output [11:0]  RAMP,
		output [11:0]  WCLK_P,
		output [11:0]  WCLK_N,
		
		output [11:0] SRCLK_P,
		output [11:0] SRCLK_N,
		input [11:0] DOE_LVDS_P,
		input [11:0] DOE_LVDS_N,
		output [11:0] SS_INCR,
		
		output [59:0] WR,

		input [11:0] MONTIMING_P,
		input [11:0] MONTIMING_N,
		
		//Alternate path to (and from) TURF using transceiver
		/*
		input 	      TMGT_CLK_N,
		input 	      TMGT_CLK_P,
		output 	      TMGT_TX_N,
		output 	      TMGT_TX_P,
		input 	      TMGT_RX_N,
		input 	      TMGT_RX_P,
		 */

		// Local I2C bus, and monitoring path from
		// microcontroller.
		inout 	      UC_SCL, 
		inout 	      UC_SDA,
		// RFP I2C bus
		inout				RFP_DAC_SCL,
		inout				RFP_DAC_SDA,
		inout	[3:0]		RFP_SCL,
		inout [3:0]		RFP_SDA,
		// SPI.
		output			SPI_CS_neg,
		output			SPI_CS_neg_alt,
		output			SPI_D0_MOSI,
		input 			SPI_D1_MISO,
		output			SPI_D2,
		output			SPI_D3
	 );
   
	localparam [3:0] BOARDREV = 4'h0;
	localparam [3:0] MONTH = 10;
	localparam [7:0] DAY = 19;
	localparam [3:0] MAJOR = 0;
	localparam [3:0] MINOR = 8;
	localparam [7:0] REVISION = 2;
	localparam [31:0] VERSION = {BOARDREV, MONTH, DAY, MAJOR, MINOR, REVISION };
	
	wire [7:0] TD = {8{1'b0}};
	generate
		genvar jj;
		for (jj=0;jj<8;jj=jj+1) begin : TURFBUS
			OBUFDS u_td_obuf(.I(TD[jj]),.OB(TD_N[jj]),.O(TD_P[jj]));
		end
	endgenerate
	reg [11:0] montiming_reg = {12{1'b0}};
	localparam [11:0] MONTIMING_POLARITY = 12'b100000000000;
	wire [11:0] MONTIMING;
	wire [11:0] MONTIMING_B;
	wire [11:0] SRCLK = {12{1'b0}};
	wire [11:0] DOE;
	generate
		genvar kk;
		for (kk=0;kk<12;kk=kk+1) begin : DIFFLOOP
			if (MONTIMING_POLARITY[kk] == 0) begin : POS
				IBUFDS_DIFF_OUT u_montiming(.I(MONTIMING_P[kk]),.IB(MONTIMING_N[kk]),.O(MONTIMING[kk]),.OB(MONTIMING_B[kk]));
			end else begin : NEG
				IBUFDS_DIFF_OUT u_montiming_b(.I(MONTIMING_N[kk]),.IB(MONTIMING_P[kk]),.O(MONTIMING_B[kk]),.OB(MONTIMING[kk]));
			end
		end
	endgenerate

	wire fpga_turf_sst_to_bufg;
	wire FPGA_TURF_SST;
	wire FPGA_SST;
	IBUFDS u_turf_ibuf(.I(FPGA_TURF_SST_P),.IB(FPGA_TURF_SST_N),.O(fpga_turf_sst_to_bufg));
	BUFG u_fpga_turf_sst(.I(fpga_turf_sst_to_bufg),.O(FPGA_TURF_SST));
	OBUFDS u_sst_obuf(.I(FPGA_SST),.O(FPGA_SST_P),.OB(FPGA_SST_N));
	OBUFDS u_sclk_obuf(.I(1'b0),.O(SCLK_P),.OB(SCLK_N));

	wire PPS;
	IBUFDS u_pps_ibuf(.I(PPS_P),.IB(PPS_N),.O(PPS));

	// Debugging. There are 2 debugging busses, both 71 bits wide (using 2 block RAMs each).
	// 2 because we have 2 main clock domains.
	//
	// The debugging busses are multiplexed inside the main debug module. Adding more debugging
	// just means adding more ports to that module (and more select lines on the VIO).
	
	// System clock debug.
	wire [70:0] wbc_debug;
	// LAB4 Controller debug
	wire [70:0] lab4_debug;
	// LAB4 second debug
	wire [70:0] lab4_debug2;
	// PCI clock debug.
	wire [70:0] pci_debug;
	
	// global_debug is an 8 bit output async output path (it controls any global behavior that has no clock).
	// global_debug[0] is used for the WISHBONE clock selection.
	wire [7:0] global_debug;
	   
   // Control WISHBONE bus clock. Probably the PCI clock.
   wire 		  wbc_clk;
   // Control WISHBONE bus reset.
   wire      wb_rst_out = 0;
   //% PPS. In WBC_CLK domain.
   wire      global_pps;
   //% PPS. In Sysclk domain.
   wire      global_pps_sysclk;
   //% External trigger (or whatever it's used for). In WBC_CLK domain.
   wire      global_ext_trig;
   //% External trigger. In Sysclk domain.
   wire      global_ext_trig_sysclk;   
      
   //% Internal LED control. Can be used by any module.
   wire [11:0] internal_led;
	assign internal_led = {12{1'b0}};
	
   //% Internal interrupts. Up to 31 can be used. 1 is used by SPI core.
   wire [30:0] 	    internal_interrupt;
	//% internal_interrupt[0] is used by the fifo below. 1 is used by the DMA engine. Others unused.
	assign internal_interrupt[30:2] = {29{1'b0}};
	
	wire pci_interrupt;
	
   //% System clock (100 MHz).
   wire 	    sys_clk;
	//% Phase shifted system clock divided by 8.
	wire		 sys_clk_div8_ps;
	//% Phase shift clock.
	wire		 ps_clk;
	//% Phase shift enable.
	wire		 ps_en;
	//% Phase shift incdec.
	wire		 ps_incdec;
	//% Phase shift done.
	wire		 ps_done;
	//% First clock of the 4 in an SST period.
	wire		 sys_clk_div4_flag;
	//% Overall phase (equiv. to PHAB in a LAB).
	wire 		 sync;
	//% Sync reset, coming from TURF.
	wire 		 sync_reset;
	//% Wilkinson clock (200 MHz).
	wire		 wclk;
	//% Local clock (25 MHz).
	wire 		 local_clk_int;
	wire		 local_osc_en_int;
	
	/*
	 * WISHBONE control bus. This accesses our main register space, which is 20-bits wide.
	 * Multiple ways to access it:
	 * 1) PCI
	 * 2) TURF
	 * 3) WBVIO bridge
	 */

	/*
	 * Masters.
	 */
	// pcic: PCI control master port WISHBONE bus.
   `WB_DEFINE( pcic, 32, 20, 4);
   // turfc: TURF control master port WISHBONE bus.
   `WB_DEFINE( turfc, 32, 20, 4);
	// wbvio: VIO master port WISHBONE bus.
	`WB_DEFINE(wbvio, 32, 20, 4);
	// dmad: DMA master port WISHBONE bus.
	`WB_DEFINE(dmad, 32, 20, 4);
	/*
	 * Slaves.
	 */
   // s5_id_ctrl: SURFv5 ID/Control slave port WISHBONE bus. 0x00000-0x0FFFF.
	// We don't need a lot here.
   `WB_DEFINE( s5_id_ctrl, 32, 16, 4);
	// LAB4 control. Also don't need a lot here. 0x10000-0x1FFFF.
	`WB_DEFINE( l4_ctrl, 32, 16, 4);
	// LAB4 RAM. Screw dense packing, waste the upper 4 bits. Make it 8x2x16-bitsx4k, which is 1 megabit.
	// On a 32-bit bus, this is 32K addresses. So 0x20000-0x2FFFF.
	`WB_DEFINE( l4_ram, 32, 16, 4);
	// Dummy slave 3.
	`WB_DEFINE( rfp, 32, 16, 4);
	  
	 // DMA slave connect.
	`WB_DEFINE( dmac, 32, 16, 4);
	  
	// WISHBONE data bus. These aren't merged anywhere yet. Still figuring out best methods.
	// pcid: PCI data slave port WISHBONE bus.
	`WB_DEFINE( pcid, 32, 32, 4);
	wire [2:0] pcid_cti;
	wire [1:0] pcid_bte;
	
	// turfd: TURF data slave port WISHBONE bus.
	`WB_DEFINE( turfd, 32, 32, 4);
	// Kill the PCID/TURFD busses. This just sets all the master signals to 0.
	//`WB_KILL( pcid , 32, 32, 4);
	`WB_KILL( turfd , 32, 32, 4);

	`PCI_TRIS(pci_rst);
	`PCI_TRIS(pci_inta);
	`PCI_TRIS(pci_req);
	`PCI_TRIS(pci_frame);
	`PCI_TRIS(pci_irdy);
	`PCI_TRIS(pci_devsel);
	`PCI_TRIS(pci_trdy);
	`PCI_TRIS(pci_stop);
	`PCI_TRIS(pci_par);
	`PCI_TRIS(pci_perr);
	`PCI_TRIS(pci_serr);
	`PCI_TRIS_VECTOR(pci_ad, 32);
	`PCI_TRIS_VECTOR(pci_cbe, 4);						

	// PCI bridge.
	pci_bridge32 u_pci(.pci_clk_i(PCI_CLK),
				`PCI_TRIS_CONNECT(pci_rst),
				.pci_req_o(pci_req_o),
				.pci_req_oe_o(pci_req_oe),
				.pci_gnt_i(pci_gnt),
				`PCI_TRIS_CONNECT(pci_inta),
				`PCI_TRIS_CONNECT(pci_frame),
				`PCI_TRIS_CONNECT(pci_irdy),
				.pci_idsel_i(pci_idsel),
				`PCI_TRIS_CONNECT(pci_devsel),
				`PCI_TRIS_CONNECT(pci_trdy),
				`PCI_TRIS_CONNECT(pci_stop),
				`PCI_TRIS_CONNECT(pci_ad),
				`PCI_TRIS_CONNECT(pci_cbe),
				`PCI_TRIS_CONNECT(pci_par),
				`PCI_TRIS_CONNECT(pci_perr),
				.pci_serr_o(pci_serr_o),
				.pci_serr_oe_o(pci_serr_oe),

				.wb_clk_i(wbc_clk),
				.wb_rst_o(wb_rst_in),
				.wb_rst_i(wb_rst_out),
				.wb_int_o(wb_int_in),
				.wb_int_i(pci_interrupt),

				`WBM_CONNECT(pcic, wbm),
				`WBS_CONNECT(pcid, wbs),
				.wbs_cti_i(pcid_cti),
				.wbs_bte_i(pcid_bte)				
				);

	reg [31:0] pci_debug_data = {32{1'b0}};
	reg [19:0] pci_debug_adr = {20{1'b0}};
	reg [3:0] pci_debug_sel = {4{1'b0}};
	reg pci_debug_cyc = 0;
	reg pci_debug_stb = 0;
	reg pci_debug_ack = 0;
	reg pci_debug_we = 0;
	reg pci_debug_err = 0;
	reg pci_debug_rty = 0;
	
	always @(posedge wbc_clk) begin
		if (pcic_we_o) pci_debug_data <= pcic_dat_o;
		else pci_debug_data <= pcic_dat_i;
		
		pci_debug_adr <= pcic_adr_o;
		pci_debug_cyc <= pcic_cyc_o;
		pci_debug_sel <= pcic_sel_o;
		pci_debug_stb <= pcic_stb_o;
		pci_debug_we <= pcic_we_o;
		pci_debug_ack <= pcic_ack_i;
		pci_debug_err <= pcic_err_i;
		pci_debug_rty <= pcic_rty_i;
	end
   
	assign pci_debug[0 +: 32] = pci_debug_data;
	assign pci_debug[32 +: 20] = pci_debug_adr;
	assign pci_debug[52 +: 4] = pci_debug_sel;
	assign pci_debug[56] = pci_debug_cyc;
	assign pci_debug[57] = pci_debug_stb;
	assign pci_debug[58] = pci_debug_we;
	assign pci_debug[59] = pci_debug_ack;
	assign pci_debug[60] = pci_debug_err;
	assign pci_debug[61] = pci_debug_rty;	
	
	BUFG u_local_clk_bufg(.I(LOCAL_CLK),.O(local_clk_int));
	
	BUFGCTRL u_wbc_clk_mux(.I0(PCI_CLK),
								  .I1(local_clk_int),
								  .S0(!global_debug[0]),
								  .S1(global_debug[0]),
								  .IGNORE0(1'b0),
								  .IGNORE1(1'b0),
								  .CE0(1'b1),
								  .CE1(1'b1),
								  .O(wbc_clk));
	// LOCAL_OSC_EN is positive-high, and we want that bit to be a *disable*
	assign LOCAL_OSC_EN = !global_debug[1] && !local_osc_en_int; 
	
   // WISHBONE Control bus interconnect. This is the first stupid version, which does not handle registered WISHBONE transfers,
   // and is just a shared bus interconnect.
   wbc_intercon u_wbc_intercon(	.clk_i(wbc_clk),.rst_i(wbc_rst),
				`WBS_CONNECT(pcic, pcic),
				`WBS_CONNECT(turfc, turfc),
				`WBS_CONNECT(hkmc, hkmc),
				`WBS_CONNECT(wbvio, wbvio),
				`WBS_CONNECT(dmad, dmad),
				`WBM_CONNECT(s5_id_ctrl, s5_id_ctrl),
				`WBM_CONNECT(l4_ctrl, l4_ctrl),
				`WBM_CONNECT(l4_ram, l4_ram),
				`WBM_CONNECT(rfp, rfp),
				`WBM_CONNECT(dmac, dma),
				.debug_o(wbc_debug));

	// LAB4 controller.
	// Handles buffer switching, serial control, and readout initialization.
	wire readout_begin_sysclk;
	wire [3:0] readout_header_sysclk;
	wire [3:0] readout_prescale_sysclk;
	wire readout_complete_sysclk;
	wire [15:0] trigger_debug;
	wire readout_fifo_rst;
	wire readout_rst;
	wire [11:0] readout_fifo_empty;
	wire [14:0] phase_scanner_dbg;
	wire readout_test_pattern_enable;
	wire [9:0] readout_empty_size;
	wire trigger_in = global_ext_trig_sysclk;
	lab4d_controller u_controller( .clk_i(wbc_clk),.rst_i(wbc_rst),
											 `WBS_CONNECT(l4_ctrl, wb),
											 .sys_clk_i(sys_clk),
											 .sys_clk_div4_flag_i(sys_clk_div4_flag),
											 .sync_i(sync),
											 .wclk_i(wclk),
											 .trig_i(trigger_in),
											 .readout_o(readout_begin_sysclk),
											 .readout_fifo_rst_o(readout_fifo_rst),
											 .readout_empty_size_o(readout_empty_size),
											 .readout_fifo_empty_i(readout_fifo_empty),
											 .readout_rst_o(readout_rst),
											 .readout_header_o(readout_header_sysclk),
											 .readout_test_pattern_o(readout_test_pattern_enable),
											 
											 .prescale_o(readout_prescale_sysclk),
											 .complete_i(readout_complete_sysclk),
											 
											 .montiming_i(montiming_reg),
	
											 .clk_ps_i(sys_clk_div8_ps),
											 .ps_clk_o(ps_clk),
											 .ps_en_o(ps_en),
											 .ps_incdec_o(ps_incdec),
											 .ps_done_i(ps_done),
											 .sync_mon_io(MON3),
											 
											 .MONTIMING_B(MONTIMING_B),											 
											 .SIN(SIN),
											 .SCLK(SCLK),
											 .PCLK(PCLK),
											 .REGCLR(REGCLR),
											 .RAMP(RAMP),
											 .WCLK_P(WCLK_P),
											 .WCLK_N(WCLK_N),
											 .SHOUT(SHOUT),
											 .WR(WR),
											 .debug_o(lab4_debug),
											 .debug2_o(lab4_debug2),
											 .trigger_debug_o(trigger_debug),
											 .phase_scanner_debug_o(phase_scanner_dbg)				 
											 );
	assign internal_interrupt[0] = !readout_fifo_empty;
	// LAB4 RAM and serial receiver.
	// The serial receiver just streams out 128x12 bits and writes them into
	// block RAM connected to the WISHBONE bus.
	//
	// The block RAM is only accessible when a DMA transaction is not occuring, however.
	// Otherwise it just returns zeros.
	//
	//
	wire dma_ram_lock;
	wire [31:0] readout_debug;
	wire [15:0] sample_debug;
	lab4d_ram u_ram( .clk_i(wbc_clk), .rst_i(wbc_rst),
						  `WBS_CONNECT(l4_ram, wb),
						  .dma_lock_i(dma_ram_lock),
						  `WBS_CONNECT(l4_dma, wbdma),
						  .sys_clk_i(sys_clk),
						  .wclk_i(wclk),
						  .readout_i(readout_begin_sysclk),
						  .readout_header_i(readout_header_sysclk),
						  .readout_test_pattern_i(readout_test_pattern_enable),
						  // FIX THIS
						  .readout_rst_i(readout_rst),
						  .readout_fifo_rst_i(readout_fifo_rst),
						  .readout_empty_size_i(readout_empty_size),
						  .readout_fifo_empty_o(readout_fifo_empty),
						  .prescale_i(readout_prescale_sysclk),
						  .complete_o(readout_complete_sysclk),
						  .readout_debug_o(readout_debug),
						  .DOE_LVDS_P(DOE_LVDS_P),
						  .DOE_LVDS_N(DOE_LVDS_N),
						  .SS_INCR(SS_INCR),
						  .SRCLK_P(SRCLK_P),
						  .SRCLK_N(SRCLK_N),
						  .sample_debug(sample_debug));

	rfp_top u_rfp(.clk_i(wbc_clk),.rst_i(wbc_rst),
					  `WBS_CONNECT(rfp, wb),
					  .RFP_DAC_SDA(RFP_DAC_SDA),
					  .RFP_DAC_SCL(RFP_DAC_SCL),
					  .RFP_SDA(RFP_SDA),
					  .RFP_SCL(RFP_SCL));

	// DMA controller.
	wire [70:0] dma_debug;
	wire dma_interrupt;
	batch_dma_controller u_dma_batch( .clk_i(wbc_clk), .rst_i(wbc_rst),.dma_interrupt_o(dma_interrupt),
					  `WBS_CONNECT(dmac, wbs),
					  `WBM_CONNECT(pcid, wbm),
					  `WBM_CONNECT(dmad, dmad),
					  .wbm_cti_o(pcid_cti),
					  .wbm_bte_o(pcid_bte),
					  .debug_o(dma_debug));
	assign internal_interrupt[1] = dma_interrupt;

   // TURFbus. This is the data path back to the TURF.
   // This also needs a slave port definition for the data side bus.
   // Also needs the top-level port connections to the TURFbus.
   turfbus u_turfbus( .wbm_clk_i(wbc_clk),
				.TCLK_P(TCLK_P),.TCLK_N(TCLK_N),
		      .wbm_rst_i(wbc_rst),		      
		      `WBM_CONNECT(turfc, wbm));
   
   // SURF5 ID and Control block. This allows for reading out device and firmware ID registers,
   // reprogramming the SPI flash, global ICE40 reset, LED control, and clock selection.
   // Also handles external trigger input/debounce.
   surf5_id_ctrl #(.VERSION(VERSION)) u_surf5_id_ctrl(.clk_i(wbc_clk),.rst_i(wbc_rst),
				 `WBS_CONNECT(s5_id_ctrl, wb),
				 // Interrupts.
				 .pci_interrupt_o(pci_interrupt),
				 .interrupt_i(internal_interrupt),
				 // Internal LEDs.
				 .internal_led_i(internal_led),
				 // System clock output.
				 .sys_clk_o(sys_clk),
				 .sys_clk_div4_flag_o(sys_clk_div4_flag),
				 .sys_clk_div8_ps_o(sys_clk_div8_ps),
				 .ps_clk_i(ps_clk),
				 .ps_en_i(ps_en),
				 .ps_incdec_i(ps_incdec),
				 .ps_done_o(ps_done),
				 .sync_o(sync),
				 .sync_reset_i(sync_reset),
				 .wclk_o(wclk),
				 // PPS generation, in both domains.
				 // Note that this may be a fake internal PPS
				 // if no external PPS has been detected.
				 .pps_o(global_pps),
				 .pps_sysclk_o(global_pps_sysclk),
				 // Ext trig generation, in both domains.
				 .ext_trig_o(global_ext_trig),
				 .ext_trig_sysclk_o(global_ext_trig_sysclk),
				 // Ext trig port
				 .EXT_TRIG(EXT_TRIG),
				 // PPS port
				 .PPS(PPS),
				 // SPI ports
				 .MOSI(SPI_D0_MOSI),
				 .MISO(SPI_D1_MISO),
				 .CS_B(SPI_CS_neg),
				 .CS_B_alt(SPI_CS_neg_alt),
				 // LED ports
				 .LED(LED),
				 .FP_LED(FP_LED),
				 // Clock ports.
				 .LOCAL_CLK(local_clk_int),
				 .LOCAL_OSC_EN(local_osc_en_int),
				 .FPGA_SST_SEL(FPGA_SST_SEL),
				 .FPGA_SST(FPGA_SST),
				 .FPGA_TURF_SST(FPGA_TURF_SST));						  

	wire [70:0] sysclk_debug;
	always @(posedge sys_clk) begin
		montiming_reg <= MONTIMING;
	end
	assign sysclk_debug[0 +: 12] = montiming_reg;
	assign sysclk_debug[12] = sys_clk_div4_flag;
	assign sysclk_debug[13] = sync;
	assign sysclk_debug[16 +: 16] = trigger_debug;
	assign sysclk_debug[32 +: 32] = readout_debug;
	
	wire [7:0] global_status;
	assign global_status[0] = pci_inta_debug;
	assign global_status[1] = 1'b0;
	assign global_status[2] = pci_interrupt;
	assign global_status[7:3] = {5{1'b0}};
	
	surf5_debug u_debug(.wbc_clk_i(wbc_clk),
							  .clk0_i(wbc_clk),
							  .clk1_i(wclk),
							  .clk_big_i(wbc_clk),
							  `WBM_CONNECT(wbvio, wbvio),
							  .clk0_debug0_i(pci_debug),
							  .clk0_debug1_i(wbc_debug),
							  .clk0_debug2_i(dma_debug),			// unused
							  .clk0_debug3_i(lab4_debug),	// unused
							  .clk1_debug_i(sysclk_debug),		// unused
							  .clk_big_debug_i(phase_scanner_dbg),
							  .global_debug_o(global_debug),
							  .global_debug_i(global_status));

	assign SREQ_neg = 1;
	assign SPI_D2 = 1;
	assign SPI_D3 = 1;

	reg [1:0] div4_delay = {2{1'b0}};
	reg internal_sst_copy = 0;
	
	always @(posedge sys_clk) begin
		div4_delay <= {div4_delay[0],sys_clk_div4_flag};
		if (sys_clk_div4_flag) internal_sst_copy <= 1;
		else if (div4_delay[1]) internal_sst_copy <= 0;
	end
	
	ODDR local_clk_ddr(.D1(1'b0),.D2(1'b1),.C(local_clk_int),.CE(1'b1),.S(1'b0),.R(1'b0),.Q(MON1));
	assign MON2 = internal_sst_copy;
endmodule
