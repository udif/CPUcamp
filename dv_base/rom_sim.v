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
    string filename;

      initial
    begin
        if ($test$plusargs("rom") != 0)
            $value$plusargs("rom=%s", filename);
        else if (INSTR_WIDTH == 16)
            filename = "rom_plain16.hex";
        else if (INSTR_WIDTH == 32)
            filename = "rom_plain32.hex";
        else if (INSTR_WIDTH == 64)
            filename = "rom_plain64.hex";
        $readmemh(filename, mem);
        $display("reading %s into ROM", filename);
    end

      always @(posedge clock)
       begin  
        address_q <= address;
    end
  
      assign q = mem[address_q];

endmodule
