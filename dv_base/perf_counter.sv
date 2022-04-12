//
// Dummy perf counter for simulation
// Do not confuse with FPGA perf_counter!!!
// We use the same file to save changes in the top level
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
    integer dumpflag = 0;
    integer memlog = 0;
    integer disasm = 0;
    string dst;
    string op;
    string dst_tbl[8] = '{"", "M", "D", "MD", "A", "AM", "AD", "AMD"};
    string alu_tbl[2][int] = '{'{
        32'b101010: "0",
        32'b111111: "1",
        32'b111010: "-1",
        32'b001100: "D",
        32'b110000: "A",
        32'b001101: "!D",
        32'b110001: "!A",
        32'b001111: "-D",
        32'b110011: "-A",
        32'b011111: "D+1",
        32'b110111: "A+1",
        32'b001110: "D-1",
        32'b110010: "A-1",
        32'b000010: "D+A",
        32'b010011: "D-A",
        32'b000111: "A-D",
        32'b000000: "D&A",
        32'b010101: "D|A"
    }, '{
        32'b101010: "0",
        32'b111111: "1",
        32'b111010: "-1",
        32'b110000: "M",
        32'b110001: "!M",
        32'b110011: "-M",
        32'b110111: "M+1",
        32'b110010: "M-1",
        32'b000010: "D+M",
        32'b010011: "D-M",
        32'b000111: "M-D",
        32'b000000: "D&M",
        32'b010101: "D|M"
    } };
    string jmp_tbl[8] = '{"", "; JGT", "; JEQ", "; JGE", "; JLT", "; JNE", "; JLE", "; JMP"};
    initial
    begin
        $display("[%0t] Model running...\n", $time);
        if ($test$plusargs("trace") != 0)
        begin
            $display("[%0t] Tracing to logs/vlt_dump.vcd...\n", $time);
            $dumpfile("logs/vlt_dump.vcd");
            $dumpvars;
            $dumpon;
        end
        if ($test$plusargs("memlog") != 0)
            memlog = 1;
        if ($test$plusargs("disasm") != 0)
            disasm = 1;
        for (i = 0; i < $size(ram_inst.mem); i = i + 1)
        begin
            // ram_vga_inst doesn't have to be cleared
            //ram_vga_inst.mem[i] = 0;
            ram_inst.mem[i] = 0;
        end
    end

    reg [14:0] last_pc;
    reg [15:0]inst;
    always @(posedge CLK_50 or negedge resetN)
    if (!resetN)
    begin
    end
    else
    begin
        if (memlog==1 && cpu_inst.write_m)
        begin
            $display("pc=%04x address=%08x data=%d", cpu_inst.pc, top.write_data_addr, cpu_inst.out_m);
        end
        if (disasm==1)
        begin
            if (last_pc != cpu_inst.pc)
            begin
                last_pc <= cpu_inst.pc;
                for (i = 0; i < $bits(top.instruction); i = i + 16)
                begin
                    inst = top.instruction[16*i +: 16];
                    if (inst[15])
                    begin
                        if (inst[15:13] != 3'b111)
                            $display("%04x: Illegal instruction %04xa", pc, inst);
                        else
                        begin
                            $display("%04x: %s=%s%s", pc, dst_tbl[inst[5:3]], alu_tbl[inst[12]][{26'd0, inst[11:6]}], jmp_tbl[inst[2:0]]);
                        end
                    end
                    else
                    begin
                        $display("%04x: @%0d (0x%04x)", pc, inst[14:0], inst[14:0]);
                    end
                end
            end
        end
        if (cpu_inst.pc == 160)
            $write(ram_inst.mem[4], "\r");
        if (dumpflag == 0 /*&& cpu_inst.pc > 60 /*&& ram_inst.mem[4] < 2 */)
        begin
            dumpflag = 1;
            //$display("[%0t] Tracing to logs/vlt_dump.vcd...\n", $time);
            //$dumpfile("logs/vlt_dump.vcd");
            //$dumpvars();
        end
        if (cpu_inst.pc > 400) // should be 1023 but there is a verilator issue with ROM content not being 0
        begin
            $display("Got to final address\n");
            for (i = 0; i < 24; i = i + 1)
            begin
                for (j = 15; j >= 0; j = j - 1)
                    if (ram_inst.mem[i][j])
                        $write("*");
                    else
                        $write(" ");
                if (i[0])
                    $write("||%04x|%04x\n", ram_inst.mem[i-1], ram_inst.mem[i]);
            end
            $finish();
        end
    end

endmodule
