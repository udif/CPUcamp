//
// Dummy Hex display module for simulation
//
module hex_display(
        input logic [9:0] pixel_x,
        input logic [9:0] pixel_y,
        input logic [DATA_WIDTH-1:0] word_value,

        output logic hex_drawing_request,
        output logic [7:0] hex_rgb
    );

    parameter DATA_WIDTH;
    parameter HEX_DIGITS_PER_LINE, HEX_DIGIT_WIDTH, HEX_START_X, HEX_PIXELS_PER_WORD;

endmodule
