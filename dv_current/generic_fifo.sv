//
// Generic FIFO
// At themoment, please use a 2^N depth
//
module generic_fifo #(
    parameter DEPTH = 2,
    parameter WIDTH = 16
) (
    input clk,
    input resetN,
    input push,
    input pop,
    input flush,
    input [WIDTH-1:0]wdata,

    output reg full,
    output reg empty,
    output [WIDTH-1:0]rdata

);
    localparam D = $clog2(DEPTH);

    logic [D:0]rptr;
    logic [D:0]wptr;
    logic [D:0]dptr;
    assign dptr = wptr - rptr;

    logic [WIDTH-1:0]mem[0:DEPTH-1];

    logic push_mem;
    assign push_mem = push && (pop || !full);

    assign rdata = mem[rptr[D-1:0]];

    always @(posedge clk)
    begin
        if (push_mem)
        begin
            mem[wptr[D-1:0]] <= wdata;
        end
    end

    always @(posedge clk or negedge resetN)
    begin
        if (!resetN)
        begin
            wptr <= {(D + 1){1'b0}};
            rptr <= {(D + 1){1'b0}};
            empty <= 1'b1;
            full <= 1'b0;
        end
        else
        begin
            wptr <= flush ? {(D + 1){1'b0}} : push_mem        ? (wptr + {{(D){1'b0}}, 1'b1}) : wptr;
            rptr <= flush ? {(D + 1){1'b0}} : (pop && !empty) ? (rptr + {{(D){1'b0}}, 1'b1}) : rptr;
            empty <=
                flush ? 1'b1 :
                (push && (!pop || empty)) ?  1'b0 :
                (pop && !push && (dptr == {{(D){1'b0}}, 1'b1})) ? 1'b1 :
                empty;
            full <=
                flush ? 1'b0 :
                (push && !pop && (dptr == (DEPTH - {{D{1'b0}}, 1'b1}))) ?  1'b1 :
                (pop && !push) ? 1'b0 :
                full;
        end
    end
endmodule