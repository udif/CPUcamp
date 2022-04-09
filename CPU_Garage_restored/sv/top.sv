`include "definitions.sv"

module top(
        input     CLK_50,
        input  [3:0]  SW,
        input  [1:0] BUTTON,

        output    [6:0]    HEX0,
        output    [6:0]    HEX1,
        output    [6:0]    HEX2,

        output [9:0] LED,

        output [3:0]  RED,
        output [3:0]  GREEN,
        output [3:0]  BLUE,

        output      h_sync,
        output    v_sync
    );
    // Widths and Counts
    // You may change the width of the instruction (e.g. make a 32-bit cpu).
    // You may change the width of the data (e.g. read and write 32 bit data into the ram).
    parameter DATA_WIDTH = 16, INSTR_WIDTH = 64; // ram and instruction widths in bits (default is 16 for both).
    parameter ROM_REGISTER_COUNT = 2**8, RAM_REGISTER_COUNT = 2**10;

    // VGA Settings
    parameter BITS_PER_MEMORY_PIXEL_X = 4; //4
    parameter BITS_PER_MEMORY_PIXEL_Y = 5; //5
    parameter HEX_START_X = 512;
    parameter HEX_DIGIT_WIDTH = 16;
    parameter HEX_DIGIT_HEIGHT = 32;
    parameter RAM_SCREEN_OFFSET = {DATA_WIDTH{1'b0}};

    parameter logic [15:0] FINAL_PC = 16'(ROM_REGISTER_COUNT-1);
    parameter NUMBER_OF_DIGITS_PERF = 8;
    // The number of bits shown will be (2**(9-BITS_PER_MEMORY_PIXEL_X))*(384/(2**(BITS_PER_MEMORY_PIXEL_Y)))

    localparam HEX_DIGITS_PER_LINE = BITS_PER_MEMORY_PIXEL_X <= 4 ? 8 : 4;
    localparam WORDS_PER_LINE = (2**9) >> ($clog2(DATA_WIDTH)+BITS_PER_MEMORY_PIXEL_X);
    localparam PIXELS_PER_WORD = 2**($clog2(DATA_WIDTH)+BITS_PER_MEMORY_PIXEL_X);
    localparam BITS_PER_HEX_DIGIT = 4;
    localparam WORDS_PER_HEX_LINE = BITS_PER_HEX_DIGIT * HEX_DIGITS_PER_LINE / DATA_WIDTH;
    localparam HEX_PIXELS_PER_WORD = DATA_WIDTH / BITS_PER_HEX_DIGIT * HEX_DIGIT_WIDTH;

    //PLL
    logic cpu_clk; // The real clock driving the CPU.
    logic cpu_clk_temp; // Same clock but before gating with `finished`.

`ifdef USE_PLL

    pll #(.MULTIPLY(`PLL_MULTIPLY), .DIVIDE(`PLL_DIVIDE))
        pll_inst(
            .inclk0 ( CLK_50 ),
            .c0 ( cpu_clk_temp )
        );
`else
    assign cpu_clk_temp = CLK_50;
`endif

`ifndef NO_CLKCTRL
    clkctrl clkctrl (
                .inclk  (cpu_clk_temp),
                .ena    (!finished),
                .outclk (cpu_clk)
            );
`else
	assign cpu_clk = cpu_clk_temp; 
`endif

    // RAM
    logic write_m /*verilator public*/ ;
    logic [DATA_WIDTH-1:0] out_m /*verilator public*/ ;
    logic [14:0]read_data_addr /*verilator public*/ ;
    logic [14:0]read_data2_addr /*verilator public*/ ;
    logic [14:0]write_data_addr /*verilator public*/ ;
    logic [DATA_WIDTH-1:0] in_m /*verilator public*/ ;
    logic [DATA_WIDTH-1:0] in_m2 /*verilator public*/ ;
    // ROM
    logic [INSTR_WIDTH-1:0] instruction  /*verilator public*/ ;
    // instruction bus address width is driven by PC width minus 1 (we pack 2 instructions in a word)
    logic [$clog2(ROM_REGISTER_COUNT)-1:0] inst_address  /*verilator public*/ ;

    logic [9:0] pixel_x;
    logic [9:0] pixel_y;
    logic [DATA_WIDTH-1:0] vga_word_value;

    logic [DATA_WIDTH-1:0] vga_word_address;

    logic resetN;
    logic rst1, rst2;

    // make sure that when BUTTON[0] is released, it goes to 1 synchronously
    always @(posedge cpu_clk)
    begin
        rst1 <=  BUTTON[0];
        rst2 <=  rst1;
    end
    assign resetN = (rst2 && BUTTON[0]);

    logic finished;

/* verilator lint_off WIDTH */
    always_comb
    begin
        // Binary
        if (pixel_x < HEX_START_X)
            vga_word_address = DATA_WIDTH'((pixel_y >> BITS_PER_MEMORY_PIXEL_Y) * WORDS_PER_LINE
                                           + (pixel_x / PIXELS_PER_WORD));
        // HEX
        else
            vga_word_address = DATA_WIDTH'((pixel_y / HEX_DIGIT_HEIGHT) * WORDS_PER_HEX_LINE
                                           + ((pixel_x - HEX_START_X) / HEX_PIXELS_PER_WORD));
    end
/* verilator lint_on WIDTH */

    // VGA : 1 rd port, CPU 1 WR port
    ram #(.DATA_WIDTH(DATA_WIDTH),
        .RAM_REGISTER_COUNT(RAM_REGISTER_COUNT)
    ) ram_vga_inst (
        .address_a (write_data_addr[$clog2(RAM_REGISTER_COUNT)-1:0]),
        .address_b ($clog2(RAM_REGISTER_COUNT)'(RAM_SCREEN_OFFSET +  vga_word_address)),
        .clock_a (cpu_clk),
        .clock_b (CLK_50),
        .data_a (out_m),
        .data_b (16'b0),
        .wren_a (write_m),
        .wren_b (~resetN),
        .q_a (),
        .q_b (vga_word_value)
    );

    // CPU: 1 wr + 1 rd port
    ram #(.DATA_WIDTH(DATA_WIDTH),
        .RAM_REGISTER_COUNT(RAM_REGISTER_COUNT)
    ) ram_cpu_inst (
        .address_a (write_data_addr[$clog2(RAM_REGISTER_COUNT)-1:0]),
        .address_b (read_data_addr[$clog2(RAM_REGISTER_COUNT)-1:0]),
        .clock_a (cpu_clk),
        .clock_b (cpu_clk),
        .data_a (out_m),
        .data_b (16'b0),
        .wren_a (write_m),
        .wren_b (1'b0),
        .q_a (),
        .q_b (in_m)
    );

    // CPU2: 1 wr + 1 rd port
    ram #(.DATA_WIDTH(DATA_WIDTH),
        .RAM_REGISTER_COUNT(RAM_REGISTER_COUNT)
    ) ram_cpu2_inst (
        .address_a (write_data_addr[$clog2(RAM_REGISTER_COUNT)-1:0]),
        .address_b (read_data2_addr[$clog2(RAM_REGISTER_COUNT)-1:0]),
        .clock_a (cpu_clk),
        .clock_b (cpu_clk),
        .data_a (out_m),
        .data_b (16'b0),
        .wren_a (write_m),
        .wren_b (1'b0),
        .q_a (),
        .q_b (in_m2)
    );

    vga #(.DATA_WIDTH(16), .BITS_PER_MEMORY_PIXEL_X(BITS_PER_MEMORY_PIXEL_X), .BITS_PER_MEMORY_PIXEL_Y(BITS_PER_MEMORY_PIXEL_Y),
          .HEX_START_X(HEX_START_X), .HEX_DIGIT_WIDTH(HEX_DIGIT_WIDTH))
        vga_inst(.CLK_50(CLK_50),
                 .hex_drawing_request(hex_drawing_request),
                 .hex_rgb(hex_rgb),
                 .perf_drawing_request(perf_drawing_request),
                 .perf_rgb(perf_rgb),
                 .pixel_in(vga_word_value),

                 .RED(RED),
                 .GREEN(GREEN),
                 .BLUE(BLUE),
                 .h_sync(h_sync),
                 .v_sync(v_sync),
                 .pixel_x(pixel_x),
                 .pixel_y(pixel_y)
                );

    // HEX (NOT 7 Segment, but display on screen)
    logic hex_drawing_request;
    logic [7:0] hex_rgb;
    hex_display #(
                    .DATA_WIDTH(DATA_WIDTH),
                    .HEX_DIGITS_PER_LINE(HEX_DIGITS_PER_LINE),
                    .HEX_DIGIT_WIDTH(HEX_DIGIT_WIDTH),
                    .HEX_START_X(HEX_START_X),
                    .HEX_PIXELS_PER_WORD(HEX_PIXELS_PER_WORD)
                )
                hex_display_inst(
                    .pixel_x(pixel_x),
                    .pixel_y(pixel_y),
                    .word_value(vga_word_value),

                    .hex_drawing_request(hex_drawing_request),
                    .hex_rgb(hex_rgb)
                );

    // performance counter
    logic perf_drawing_request;
    logic [7:0] perf_rgb;
    perf_counter #(
                     .NUMBER_OF_DIGITS(NUMBER_OF_DIGITS_PERF),
                     .HEX_DIGIT_WIDTH(HEX_DIGIT_WIDTH),
                     .HEX_DIGIT_HEIGHT(HEX_DIGIT_HEIGHT),
                     .FINAL_PC(FINAL_PC)
                 )
                 perf_counter_inst(
                     .CLK_50(CLK_50),
                     .resetN(resetN),
                     .pixel_x(pixel_x),
                     .pixel_y(pixel_y),
                     .pc({{16-$bits(inst_address){1'b0}}, inst_address}),
                     .SW(SW),

                     .perf_drawing_request(perf_drawing_request),
                     .perf_rgb(perf_rgb),

                     .HEX0(HEX0),
                     .HEX1(HEX1),
                     .HEX2(HEX2),

                     .LED(LED),
                     .finished(finished)
                 );

    rom #(.INSTR_WIDTH(INSTR_WIDTH),
          .ROM_REGISTER_COUNT(ROM_REGISTER_COUNT))
        rom_inst
        (
            .address(inst_address),
            .clock(cpu_clk),
            .q(instruction)
        );

    wire [(13-$bits(inst_address))-1:0]dummy_pad;
    cpu cpu_inst (
            .clk(cpu_clk),
            .resetN(resetN),

            .SW(SW),

            .inst(instruction),
            .inst_addr({dummy_pad, inst_address}),

            .out_m(out_m),
            .write_m(write_m),
            .write_data_addr(write_data_addr),
            .in_m(in_m),
            .in_m2(in_m2),
            .read_data_addr(read_data_addr),
            .read_data2_addr(read_data2_addr)
        );
endmodule
