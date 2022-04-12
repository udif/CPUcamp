//
// Dummy VGA module for simulation
//
module vga(
        input logic CLK_50,
        input logic [DATA_WIDTH-1:0] pixel_in,
        input logic hex_drawing_request,
        input logic [7:0]  hex_rgb,
        input logic perf_drawing_request,
        input logic [7:0]  perf_rgb,

        output logic [3:0] RED,
        output logic [3:0] GREEN,
        output logic [3:0] BLUE,
        output logic h_sync,
        output logic v_sync,
        output logic [9:0] pixel_x,
        output logic [9:0] pixel_y
    );

    parameter DATA_WIDTH;
    parameter BITS_PER_MEMORY_PIXEL_X;
    parameter BITS_PER_MEMORY_PIXEL_Y;
    parameter HEX_START_X;
    parameter HEX_DIGIT_WIDTH;

endmodule
