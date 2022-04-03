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
    parameter DATA_WIDTH = 16, INSTR_WIDTH = 32; // ram and instruction widths in bits (default is 16 for both).
    parameter PC_WIDTH = 10;
    parameter ROM_REGISTER_COUNT = 2**9, RAM_REGISTER_COUNT = 2**10;

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
    logic [14:0]ram_address  /*verilator public*/ ;
    logic ram_write_m  /*verilator public*/ ;
    logic [DATA_WIDTH-1:0] rdata  /*verilator public*/ ;
    logic [DATA_WIDTH-1:0] ram_out_m  /*verilator public*/ ;
    // ROM
    logic [INSTR_WIDTH-1:0] instruction  /*verilator public*/ ;
    logic [14:0] inst_address  /*verilator public*/ ;
    // CPU DATA
    logic [14:0]cpu_data_addr;
    logic cpu_read_m, cpu_write_m, cpu_stall;
    logic [DATA_WIDTH-1:0] cpu_in_m;
    logic [DATA_WIDTH-1:0] cpu_out_m  /*verilator public*/ ;

    logic [9:0] pixel_x;
    logic [9:0] pixel_y;
    logic [DATA_WIDTH-1:0] vga_word_value;

    logic [DATA_WIDTH-1:0] vga_word_address;

    logic resetN;
    assign resetN = BUTTON[0];

    logic finished;

`ifdef VERILATOR
    initial
    begin
        if ($test$plusargs("trace") != 0)
        begin
            $display("[%0t] Tracing to logs/vlt_dump.vcd...\n", $time);
            $dumpfile("logs/vlt_dump.vcd");
            $dumpvars();
        end
        $display("[%0t] Model running...\n", $time);
   end
`endif

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

    ram #(.DATA_WIDTH(DATA_WIDTH),
          .RAM_REGISTER_COUNT(RAM_REGISTER_COUNT))
        ram_inst (
            .address_a (ram_address[$clog2(RAM_REGISTER_COUNT)-1:0]),
            .address_b ($clog2(RAM_REGISTER_COUNT)'(RAM_SCREEN_OFFSET +  vga_word_address)),
            .clock_a (cpu_clk),
            .clock_b (CLK_50),
            .data_a (ram_out_m),
            .data_b (16'b0),
            .wren_a (ram_write_m),
            .wren_b (~resetN),
            .q_a (rdata),
            .q_b (vga_word_value)
        );

    ram_cache #(.DATA_WIDTH(DATA_WIDTH),
          .RAM_REGISTER_COUNT(RAM_REGISTER_COUNT))
        ram_cache_inst (
            .clk(cpu_clk),
            .resetN(resetN),

            .cpu_in_m(cpu_in_m),
            .cpu_out_m(cpu_out_m),
            .cpu_write_m(cpu_write_m),
            .cpu_read_m(cpu_read_m),
            .cpu_data_addr(cpu_data_addr[$clog2(RAM_REGISTER_COUNT)-1:0]),

            .cpu_stall(cpu_stall),

            .ram_in_m(rdata),
            .ram_out_m(ram_out_m),
            .ram_write_m(ram_write_m),
            .ram_data_addr(ram_address[$clog2(RAM_REGISTER_COUNT)-1:0])
);

`ifndef NO_IO
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
`endif
  
`ifndef NO_PERF
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
                     .pc(inst_address[$clog2(ROM_REGISTER_COUNT)-1:0]),
                     .SW(SW),

                     .perf_drawing_request(perf_drawing_request),
                     .perf_rgb(perf_rgb),

                     .HEX0(HEX0),
                     .HEX1(HEX1),
                     .HEX2(HEX2),

                     .LED(LED),
                     .finished(finished)
                 );
`endif

    rom #(.INSTR_WIDTH(INSTR_WIDTH),
          .ROM_REGISTER_COUNT(ROM_REGISTER_COUNT))
        rom_inst
        (
            .address(inst_address[$clog2(ROM_REGISTER_COUNT)-1:0]),
            .clock(cpu_clk),
            .q(instruction)
        );


    cpu #(
        .PC_WIDTH(PC_WIDTH)
    ) cpu_inst (
            .clk(cpu_clk),
            .resetN(resetN),

            .SW(SW),

            .inst(instruction),
            .inst_addr(inst_address[$clog2(ROM_REGISTER_COUNT)-1:0]),

            .in_m(cpu_in_m),
            .out_m(cpu_out_m),
            .write_m(cpu_write_m),
            .read_m(cpu_read_m),
            .stall(cpu_stall),
            .data_addr(cpu_data_addr)
        );
endmodule