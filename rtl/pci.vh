`define PCI_TRIS( x ) \
wire x``_i; \
wire x``_o; \
wire x``_oe ; \
IOBUF x``_iobuf (.IO( x ), .I( x``_o ), .O( x``_i ), .T( x``_oe) ); \
wire x``_debug = x``_oe ? x``_i : x``_o



// Tristate bus. The 'dummy' debug is to end on a ';'-statement.
`define PCI_TRIS_VECTOR( x , y ) \
wire [ y - 1 : 0 ] x``_i; \
wire [ y - 1 : 0 ] x``_o; \
wire [ y - 1 : 0 ] x``_oe; \
wire [ y - 1 : 0 ] x``_debug; \
wire [ y - 1 : 0 ] x``_debug_dup; \
generate \
genvar x``_iter; \
for ( x``_iter = 0 ; x``_iter < y ; x``_iter = x``_iter + 1 ) begin : x``_IOBUF_LOOP \
IOBUF x``_iobuf(.IO( x [ x``_iter ] ), .I( x``_o[ x``_iter ]), .O( x``_i [ x``_iter ] ), .T( x``_oe [ x``_iter ] )); \
assign x``_debug_dup = ( x``_oe[ x``_iter ] ) ? x``_i [ x``_iter ] : x``_o [ x``_iter ] ; \
end \
endgenerate \
assign x``_debug = x``_debug_dup 

`define PCI_TRIS_CONNECT(x) \
.``x``_i ( ``x``_i ), \
.``x``_o ( ``x``_o ), \
.``x``_oe_o ( ``x``_oe )