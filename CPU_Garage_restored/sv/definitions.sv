`ifndef macros_vh
`define macros_vh

//=== PLL
// Uncomment the following line to use a PLL. Set the multiplication and division factors however you like.
`define USE_PLL
// max settings that worked for me:
// kiwi -       250/100
// de10-lite -  260/100

// *********************************************************************************
// *** don't forget to change PLL_MULTIPLY value in CPU_GARAGE.sdc file as well! ***
// *********************************************************************************
`define PLL_MULTIPLY 210
`define PLL_DIVIDE 100


//KIWI or DE10_LITE
`include "platform.sv"

`ifdef KIWI
    `define SEG7_ACTIVE_LOW
`endif

`endif