// Directions here are always spec'd as "master to slave".

// This is for an interconnection of a master/slave.
// Clock and reset are never defined here at all.
//
// e.g.:
// `WB_DEFINE(this_wb_bus, 32, 32, 4);
// wire clk;
// wire rst;
// my_master u_master(WBM_CONNECT(this_wb_bus, wb), .clk_i(clk),.rst_i(rst));
// my_slave u_slave(WBS_CONNECT(this_wb_bus, wb), .clk_i(clk),.rst_i(rst));
//
// WBx_CONNECT(x, y) connects a WISHBONE bus prefixed with 'x'
// to a WISHBONE bus prefixed with 'y'.
//
// Then inside the master/slave, do
//
// module my_master( input clk_i, input rst_i, WBM_NAMED_PORT(wb, 32, 32, 4) );
// always @(clk_i) begin
//    -- do something
// end
// end module
// etc.

// Define a WISHBONE bus with prefix 'x',
// with data width 'y', address width 'z',
// and select width 'w'.
`define WB_DEFINE(x, y, z, w)	\
	wire [y``-1:0] x``_dat_i;		\
	wire [y``-1:0] x``_dat_o;		\
	wire [z``-1:0] x``_adr_o;		\
	wire			  x``_cyc_o;		\
	wire			  x``_we_o;		\
	wire			  x``_stb_o;		\
	wire			  x``_ack_i;		\
	wire			  x``_rty_i;		\
	wire			  x``_err_i;		\
	wire [w``-1:0]  x``_sel_o

// Define a vectorized WISHBONE bus with prefix 'x',
// data width 'y', address width 'z',
// select width 'w', and vector width 'n'.
`define WB_DEFINE_VECTOR(x, y, z, w, n)	\
	wire [y``-1:0] x``_dat_i[ n -1:0];		\
	wire [y``-1:0] x``_dat_o[ n -1:0]; 		\
	wire [z``-1:0] x``_adr_o[ n -1:0];		\
	wire [ n -1:0] x``_cyc_o;		\
	wire [ n -1:0] x``_we_o;		\
	wire [ n -1:0] x``_stb_o;		\
	wire [ n -1:0] x``_ack_i;		\
	wire [ n -1:0] x``_rty_i;		\
	wire [ n -1:0] x``_err_i;		\
	wire [w``-1:0]  x``_sel_o[ n -1:0]

// Create a series of named ports for
// the WISHBONE bus. Syntax is as for WB_DEFINE.
`define WBS_NAMED_PORT(x, y, z, w)	\
	output [y``-1:0] x``_dat_o,			\
	input [y``-1:0] x``_dat_i,			\
	input [z``-1:0] x``_adr_i,			\
	input 			x``_cyc_i,			\
	input				x``_we_i,				\
	input				x``_stb_i,			\
	output			x``_ack_o,			\
	output			x``_rty_o,			\
	output			x``_err_o,			\
	input	[w``-1:0] x``_sel_i

// Create a series of port definitions for
// the WISHBONE bus. Syntax is as for WB_DEFINE.
// This is used along with WBS_UNNAMED_PORT.
`define WBS_DEFINE_PORT(x, y, z, w) \
	output [y``-1:0] x``_dat_o;			\
	input [y``-1:0] x``_dat_i;			\
	input [z``-1:0] x``_adr_i;			\
	input 			x``_cyc_i;			\
	input				x``_we_i;				\
	input				x``_stb_i;			\
	output			x``_ack_o;			\
	output			x``_rty_o;			\
	output			x``_err_o;			\
	input	[w``-1:0] x``_sel_i

// Create a series of untyped ports for
// the WISHBONE bus. Syntax is as for WB_DEFINE.
// This is used along with WBS_DEFINE_PORT.
`define WBS_UNNAMED_PORT(x, y, z, w) 	\
	x``_dat_o,									\
	x``_dat_i,									\
	x``_adr_i,									\
	x``_cyc_i,									\
	x``_we_i,										\
	x``_stb_i,									\
	x``_ack_o,									\
	x``_rty_o,									\
	x``_err_o,									\
	x``_sel_i
	
// As with WBS_NAMED_PORT, just for a master port.
`define WBM_NAMED_PORT(x, y, z, w)	\
	input [y``-1:0] x``_dat_i,			\
	output [y``-1:0] x``_dat_o,			\
	output [z``-1:0] x``_adr_o,			\
	output 			x``_cyc_o,			\
	output			x``_we_o,				\
	output			x``_stb_o,			\
	input			x``_ack_i,				\
	input			x``_rty_i,				\
	input				x``_err_i,			\
	output [w``-1:0] x``_sel_o

// As with WBS_DEFINE_PORT, just for a master port.	
`define WBM_DEFINE_PORT(x, y, z, w)	\
	input [y``-1:0] x``_dat_i;			\
	output [y``-1:0] x``_dat_o;			\
	output [z``-1:0] x``_adr_o;			\
	output 			x``_cyc_o;			\
	output			x``_we_o;				\
	output			x``_stb_o;			\
	input			x``_ack_i;				\
	input			x``_rty_i;				\
	input				x``_err_i;			\
	output [w``-1:0] x``_sel_o

// As with WBS_UNNAMED_PORT, just for a master port.
`define WBM_UNNAMED_PORT(x, y, z, w)	\
	x``_dat_i,			\
	x``_dat_o,			\
	x``_adr_o,			\
	x``_cyc_o,			\
	x``_we_o,				\
	x``_stb_o,			\
	x``_ack_i,				\
	x``_rty_i,				\
	x``_err_i,			\
	x``_sel_o

// Connect a WISHBONE bus prefixed 'x'
// to a set of slave ports prefixed 'y'.
`define WBS_CONNECT(x, y)	\
	.y``_dat_i(x``_dat_o),					\
	.y``_dat_o(x``_dat_i),					\
	.y``_adr_i(x``_adr_o),					\
	.y``_cyc_i(x``_cyc_o),					\
	.y``_stb_i(x``_stb_o),					\
	.y``_we_i(x``_we_o),					\
	.y``_ack_o(x``_ack_i),					\
	.y``_rty_o(x``_rty_i),					\
	.y``_err_o(x``_err_i),					\
	.y``_sel_i(x``_sel_o)

// Connect a WISHBONE bus prefixed 'x'
// to a set of master ports prefixed 'y'
`define WBM_CONNECT(x, y)	\
	.y``_dat_o(x``_dat_o),					\
	.y``_dat_i(x``_dat_i),					\
	.y``_adr_o(x``_adr_o),					\
	.y``_cyc_o(x``_cyc_o),					\
	.y``_stb_o(x``_stb_o),					\
	.y``_we_o(x``_we_o),					\
	.y``_ack_i(x``_ack_i),					\
	.y``_rty_i(x``_rty_i),					\
	.y``_err_i(x``_err_i),					\
	.y``_sel_o(x``_sel_o)

// Connect a vectored WISHBONE bus at index 'n' prefixed by 'x'
// to a set of slave ports prefixed 'y'.
`define WBS_CONNECT_VECTOR(x, y, n)	\
	.y``_dat_i(x``_dat_o[ n ]),					\
	.y``_dat_o(x``_dat_i[ n ]),					\
	.y``_adr_i(x``_adr_o[ n ]),					\
	.y``_cyc_i(x``_cyc_o[ n ]),					\
	.y``_stb_i(x``_stb_o[ n ]),					\
	.y``_we_i(x``_we_o[ n ]),					\
	.y``_ack_o(x``_ack_i[ n ]),					\
	.y``_rty_o(x``_rty_i[ n ]),					\
	.y``_err_o(x``_err_i[ n ]),					\
	.y``_sel_i(x``_sel_o[ n ])

// Connect a vectored WISHBONE bus at index 'n' prefixed by 'x'
// to a set of master ports prefixed 'y'
`define WBM_CONNECT_VECTOR(x, y, n)	\
	.y``_dat_o(x``_dat_o[ n ]),					\
	.y``_dat_i(x``_dat_i[ n ]),					\
	.y``_adr_o(x``_adr_o[ n ]),					\
	.y``_cyc_o(x``_cyc_o[ n ]),					\
	.y``_stb_o(x``_stb_o[ n ]),					\
	.y``_we_o(x``_we_o[ n ]),					\
	.y``_ack_i(x``_ack_i[ n ]),					\
	.y``_rty_i(x``_rty_i[ n ]),					\
	.y``_err_i(x``_err_i[ n ]),					\
	.y``_sel_o(x``_sel_o[ n ])

// Kill a WISHBONE bus named x with data width y and addr width z and sel width w (set all signals to 0).
`define WB_KILL(x, y, z, w) \
	assign x``_cyc_o = 0;							\
	assign x``_stb_o = 0;							\
	assign x``_we_o = 0;								\
	assign x``_dat_o = {y{1'b0}};					\
	assign x``_adr_o = {z{1'b0}};					\
	assign x``_sel_o = {w{1'b0}}

// Kill a WISHBONE slave bus.
// This just acks everything immediately, and always returns 0.
`define WBM_KILL(x, y) \
	assign x``_ack_i = x``_cyc_o && x``_stb_o; \
	assign x``_err_i = 1'b0;                   \
	assign x``_rty_i = 1'b0;                   \
	assign x``_dat_i = {y{1'b0}}

