
module ram # (
    parameter DATA_WIDTH,
    parameter RAM_REGISTER_COUNT
) (
    input	[$clog2(RAM_REGISTER_COUNT) - 1:0]  address_a,
    input	[10 + (DATA_WIDTH/16 - 1) - 1:0]  address_b,
    input	  clock_a,
    input	  clock_b,
    input	[DATA_WIDTH-1:0]  data_a,
    input	[15:0]  data_b,
    input	  wren_a,
    input	  wren_b,
    output	[DATA_WIDTH-1:0]  q_a,
    output	[15:0]  q_b
  );
  	reg [DATA_WIDTH-1:0]mem[0:RAM_REGISTER_COUNT-1];
  	reg [$clog2(RAM_REGISTER_COUNT) - 1:0]address_a_q;
  	reg [$clog2(RAM_REGISTER_COUNT) - 1:0]address_b_q;

  	always @(posedge clock_a)
    begin  
    	address_a_q <= address_a;
    	address_b_q <= address_b;
      	if (wren_a)
        	mem[address_a] <= data_a;
      	if (wren_b)
        	mem[address_b] <= data_b;
    end
  
  	assign q_a = mem[address_a_q];
  	assign q_b = mem[address_b_q];
  
endmodule
