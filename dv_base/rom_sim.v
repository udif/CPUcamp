// megafunction wizard: %ROM: 1-PORT%
// GENERATION: STANDARD

module rom # (
    parameter INSTR_WIDTH,
    parameter ROM_REGISTER_COUNT
) (
  	input	[$clog2(ROM_REGISTER_COUNT) - 1:0]  address,
	input	  clock,
	output	[INSTR_WIDTH - 1:0]  q
);

  	reg [INSTR_WIDTH-1:0]mem[0:ROM_REGISTER_COUNT-1];
  	reg [$clog2(ROM_REGISTER_COUNT) - 1:0]address_q;

  	initial
    	$readmemh("rom_plain.hex", mem);

  	always @(posedge clock)
   	begin  
    	address_q <= address;
    end
  
  	assign q = mem[address_q];

endmodule
