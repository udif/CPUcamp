//
// Future ram cache module. sits between the CPU and RAM.
//
module ram_cache # (
    parameter DATA_WIDTH = 16,
    parameter RAM_REGISTER_COUNT = 1024,
    CACHE_SIZE = 4
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

    reg [15:0]cache[0:CACHE_SIZE-1];
    reg [$clog2(RAM_REGISTER_COUNT)-$clog2(CACHE_SIZE)-1:0]tag[0:CACHE_SIZE-1];
    reg [CACHE_SIZE-1:0]valid;
    
    wire [$clog2(CACHE_SIZE)-1:0]index = cpu_data_addr[$clog2(CACHE_SIZE)-1:0];
    wire [$clog2(RAM_REGISTER_COUNT)-1:$clog2(CACHE_SIZE)]cpu_addr_tag =  cpu_data_addr[$clog2(RAM_REGISTER_COUNT)-1:$clog2(CACHE_SIZE)];
    wire cache_hit = valid[index] && (cpu_addr_tag == tag[index]);
    
    
    // At the moment we just short out things to make them work.
    assign cpu_in_m      = cache_hit ? cache[index] : ram_in_m;
    assign ram_out_m     = cpu_out_m;
    assign ram_data_addr = cpu_data_addr;
    assign ram_write_m   = cpu_write_m;

    assign cpu_stall     = cpu_read_m && !cache_hit && !cpu_stall_q;

    always @(posedge clk)
        if (!resetN)
            valid <= {CACHE_SIZE{1'b0}};
        else if (cpu_write_m)
        begin
            valid[index] <= 1'b1;
            tag[index] <= cpu_addr_tag;
            cache[index] <= cpu_out_m;
        end
endmodule
