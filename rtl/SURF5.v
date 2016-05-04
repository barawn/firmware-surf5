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
		output [4:0]  MON,

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
		
		// SPI.
		output			SPI_CS_neg,
		output			SPI_D0_MOSI,
		input 			SPI_D1_MISO
	 );
   
	localparam [3:0] BOARDREV = 4'h0;
	localparam [3:0] MONTH = 4;
	localparam [7:0] DAY = 20;
	localparam [3:0] MAJOR = 0;
	localparam [3:0] MINOR = 0;
	localparam [7:0] REVISION = 1;
	localparam [31:0] VERSION = {BOARDREV, MONTH, DAY, MAJOR, MINOR, REVISION };
	
	wire [7:0] TD = {8{1'b0}};
	generate
		genvar jj;
		for (jj=0;jj<8;jj=jj+1) begin : TURFBUS
			OBUFDS u_td_obuf(.I(TD[jj]),.OB(TD_N[jj]),.O(TD_P[jj]));
		end
	endgenerate

	wire FPGA_TURF_SST;
	wire FPGA_SST;
	IBUFDS u_turf_ibuf(.I(FPGA_TURF_SST_P),.IB(FPGA_TURF_SST_N),.O(FPGA_TURF_SST));
	OBUFDS u_sst_obuf(.I(FPGA_SST),.O(FPGA_SST_P),.OB(FPGA_SST_N));
	wire SCLK = 0;
	OBUFDS u_sclk_obuf(.I(SCLK),.O(SCLK_P),.OB(SCLK_N));

	wire PPS;
	IBUFDS u_pps_ibuf(.I(PPS_P),.IB(PPS_N),.O(PPS));

	// Debugging. There are 2 debugging busses, both 71 bits wide (using 2 block RAMs each).
	// 2 because we have 2 main clock domains.
	//
	// The debugging busses are multiplexed inside the main debug module. Adding more debugging
	// just means adding more ports to that module (and more select lines on the VIO).
	wire [70:0] wbc_debug;
	// global_debug is an 8 bit output async output path (it controls any global behavior that has no clock).
	// global_debug[0] is used for the WISHBONE clock selection.
	wire [7:0] global_debug;
	
	wire [70:0] pci_debug;
   // Internally there are three main busses: the 'control' WISHBONE bus, which has 3 masters and 4 slaves,
   // and the 'data' WISHBONE bus, which has 2 masters and 2 slaves, and the LAB4 I2C bus, which has
   // 12 slaves and 2 masters.
   // However, these are crossbared busses, so we have an utter bucket-ton of named wires here.
   // We will probably add a 4th master on the 'control' WISHBONE bus (an I2C-to-WISHBONE bridge to allow the uC
   // to pull out sensor data - an I2C to WISHBONE slave already exists).
   
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
	
	// Right now no one is using them, so 
   //% Internal interrupts. Up to 31 can be used. 1 is used by SPI core.
   wire [30:0] 	    internal_interrupt;
	assign internal_interrupt[30:0] = {31{1'b0}};
	wire pci_interrupt;
	
   //% System clock (100 MHz).
   wire 	    sys_clk;
	//% Local clock (25 MHz).
	wire 		 local_clk_int;
	wire		 local_osc_en_int;
   //% WCLK enable
	wire [11:0] wclk_en;
	
	// WISHBONE control bus. These are all merged into a common bus in the wbc_intercon module.
	// pcic: PCI control master port WISHBONE bus.
   `WB_DEFINE( pcic, 32, 20, 4);
   // turfc: TURF control master port WISHBONE bus.
   `WB_DEFINE( turfc, 32, 20, 4);
	// wbvio: VIO master port WISHBONE bus.
	`WB_DEFINE(wbvio, 32, 20, 4);
	
   // s5_id_ctrl: SURFv5 ID/Control slave port WISHBONE bus.
   `WB_DEFINE( s5_id_ctrl, 32, 16, 4);
	// Dummy slave 1.
	`WB_DEFINE( hksc, 32, 16, 4);
	// Dummy slave 2.
	`WB_DEFINE( lab4, 32, 19, 4);
	// Dummy slave 3.
	`WB_DEFINE( rfp, 32, 19, 4);
	
	// Kill the dummy busses.
	`WBM_KILL( hksc, 32 );
	`WBM_KILL( lab4, 32 );
	`WBM_KILL( rfp, 32 );
  
	// WISHBONE data bus. These aren't merged anywhere yet. Still figuring out best methods.
	// pcid: PCI data slave port WISHBONE bus.
	`WB_DEFINE( pcid, 32, 32, 4);
	// turfd: TURF data slave port WISHBONE bus.
	`WB_DEFINE( turfd, 32, 32, 4);
	// Kill the PCID/TURFD busses. This just sets all the master signals to 0.
	`WB_KILL( pcid , 32, 32, 4);
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
				`WBS_CONNECT(pcid, wbs)
//				.wbm_cti_o(wbm_cti),
//				.wbm_bte_o(wbm_bte)
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
	
	BUFGCTRL u_wbc_clk_mux(.I0(PCI_CLK),
								  .I1(local_clk_int),
								  .S0(!global_debug[0]),
								  .S1(global_debug[0]),
								  .IGNORE0(1'b0),
								  .IGNORE1(1'b0),
								  .CE0(1'b1),
								  .CE1(1'b1),
								  .O(wbc_clk));
	assign LOCAL_OSC_EN = global_debug[1]; //!(local_osc_en_int || global_debug[0]);

   // WISHBONE Control bus interconnect. This is the first stupid version, which does not handle registered WISHBONE transfers,
   // and is just a shared bus interconnect.
   wbc_intercon u_wbc_intercon(	.clk_i(wbc_clk),.rst_i(wbc_rst),
				`WBS_CONNECT(pcic, pcic),
				`WBS_CONNECT(turfc, turfc),
				`WBS_CONNECT(hkmc, hkmc),
				`WBS_CONNECT(wbvio, wbvio),
				`WBM_CONNECT(s5_id_ctrl, s5_id_ctrl),
				`WBM_CONNECT(hksc, hksc),
				`WBM_CONNECT(rfp, rfp),
				`WBM_CONNECT(lab4, lab4),
				.debug_o(wbc_debug));
   
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
				 // Local clock output (25 MHz).
				 .local_clk_o(local_clk_int),
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
				 // LED ports
				 .LED(LED),
				 .FP_LED(FP_LED),
				 // Clock ports.
				 .LOCAL_CLK(LOCAL_CLK),
				 .LOCAL_OSC_EN(local_osc_en_int),
				 .FPGA_SST_SEL(FPGA_SST_SEL),
				 .FPGA_SST(FPGA_SST),
				 .FPGA_TURF_SST(FPGA_TURF_SST));

	surf5_debug u_debug(.wbc_clk_i(wbc_clk),
							  .clk0_i(wbc_clk),
							  .clk1_i(sys_clk),
							  `WBM_CONNECT(wbvio, wbvio),
							  .wbc_debug_i(pci_debug),
							  .ice_debug_i(lab4_debug),
							  .i2c_debug_i(i2c_debug),
							  .lab4_i2c_debug_i(lab4_i2c_debug),
							  .rfp_debug_i(rfp_debug),
							  .global_debug_o(global_debug));

	assign MON = {5{1'b0}};
	assign SREQ_neg = 1;
endmodule
