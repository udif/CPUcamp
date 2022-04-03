// synopsys translate_off
`timescale 1 ns / 1 ns
// synopsys translate_on
module dv;
  
  reg CLK_50;
  reg RESET;
  
  initial
  begin
    CLK_50 = 0;
    RESET = 0;
    #25
    RESET = 1;
    #1000
    $finish();
  end
  
  always
  begin    
    #10
    CLK_50 = ~CLK_50;
  end
  
  top top (
    .CLK_50(CLK_50),
    .SW(4'b0),
    .BUTTON(),
    .HEX0(),
    .HEX1(),
    .HEX2(),
    .LED(),
    .RED(),
    .GREEN(),
    .BLUE(),
    .h_sync(),
    .v_sync()
  );
endmodule
