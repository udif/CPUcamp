//
// Future ram cache module. sits between the CPU and RAM.
//
module ram_cache # (
    parameter DATA_WIDTH = 16,
    parameter RAM_REGISTER_COUNT = 1024
) (
        input  logic                                  clk,
        input  logic                                  resetN,

        output logic                 [DATA_WIDTH-1:0] cpu_in_m,
        input  logic                 [DATA_WIDTH-1:0] cpu_out_m,
        input  logic                                  cpu_write_m,
        input  logic                                  cpu_read_m,
        input  logic [$clog2(RAM_REGISTER_COUNT)-1:0] cpu_data_addr,

        output logic                                  cpu_stall,

        input  logic                 [DATA_WIDTH-1:0] ram_in_m,
        output logic                 [DATA_WIDTH-1:0] ram_out_m,
        output logic                                  ram_write_m,
        output logic [$clog2(RAM_REGISTER_COUNT)-1:0] ram_data_addr
);

    logic cpu_stall_q;

    always_ff @(posedge clk or negedge resetN)
    begin
        if (!resetN)
            cpu_stall_q <= 1'b1;
        else
            cpu_stall_q  <= cpu_stall;
    end

    // At the moment we just short out things to make them work.
    assign cpu_in_m      = ram_in_m;
    assign ram_out_m     = cpu_out_m;
    assign ram_data_addr = cpu_data_addr;
    assign ram_write_m   = cpu_write_m;

    assign cpu_stall     = cpu_read_m && !cpu_stall_q;

endmodule
