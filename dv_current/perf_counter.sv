//
// Dummy perf counter for simulation
//
`include "definitions.sv"

module perf_counter(
        input logic CLK_50,
        input logic resetN,
        input logic [9:0] pixel_x,
        input logic [9:0] pixel_y,
        input logic [15:0] pc,
        input logic [3:0] SW,

        output logic perf_drawing_request,
        output logic [7:0] perf_rgb,

        output logic [6:0]  HEX0,
        output logic [6:0]  HEX1,
        output logic [6:0]  HEX2,

        output logic [9:0] LED,

        output logic finished
    );

    parameter NUMBER_OF_DIGITS;
    parameter HEX_DIGIT_WIDTH, HEX_DIGIT_HEIGHT;
    parameter logic [15:0] FINAL_PC;

    //
    // We took over perf_counter.sv
    //

    integer i, j;
    initial
    begin
        $display("[%0t] Model running...\n", $time);
        if ($test$plusargs("trace") != 0)
        begin
            $display("[%0t] Tracing to logs/vlt_dump.vcd...\n", $time);
            $dumpfile("logs/vlt_dump.fst");
            $dumpvars();
        end
        for (i = 0; i < $size(ram_vga_inst.mem); i = i + 1)
        begin
            ram_vga_inst.mem[i] = 0;
            ram_cpu_inst.mem[i] = 0;
        end
    end

    always @(posedge CLK_50)
    begin
        if (cpu_inst.pc == 160)
            $write(ram_cpu_inst.mem[4], "\r");
        //if (cpu_inst.pc > 162)
        //    $display(cpu_inst.pc);
        if (cpu_inst.pc > 400) // should be 1023 but there is a verilator issue with ROM content not being 0
        begin
            $display("Got to final address\n");
            for (i = 0; i < 24; i = i + 1)
            begin
                for (j = 15; j >= 0; j = j - 1)
                    if (ram_cpu_inst.mem[i][j])
                        $write("*");
                    else
                        $write(" ");
                if (i[0])
                    $write("||%04x|%04x\n", ram_cpu_inst.mem[i-1], ram_cpu_inst.mem[i]);
            end
            $finish();
        end
    end

endmodule
