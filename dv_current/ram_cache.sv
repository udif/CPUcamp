//
// ram cache module. sits between the CPU and RAM.
// This is a direct cache (only 1 entry per index)
// However, we only look at a subset of the memory based on 0 upwards
//
// cache support address of the following form:
// 0 ........... 0 T .......... T I ............ I
// <- ZERO_BITS -> <- TAG_BITS -> <- INDEX_BITS ->
//
module ram_cache # (
    parameter DATA_WIDTH = 16,
    parameter RAM_REGISTER_COUNT = 1024,
    INDEX_BITS = 4,
    TAG_BITS = 0
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
    localparam ADDR_BITS = $clog2(RAM_REGISTER_COUNT);
    localparam ZERO_BITS = ADDR_BITS - INDEX_BITS - TAG_BITS;
    localparam CACHE_SIZE = 1 << INDEX_BITS;

    logic cpu_stall_q;

    always_ff @(posedge clk or negedge resetN)
    begin
        if (!resetN)
            cpu_stall_q <= 1'b1;
        else
            cpu_stall_q  <= cpu_stall;
    end

    reg [15:0]cache[0:CACHE_SIZE-1];
    wire cache_hit;
    reg [CACHE_SIZE-1:0]valid;    

    // split cpu address bus into bits we check for 0, bits we compare with tag, and index
    wire [INDEX_BITS-1:0]index = cpu_data_addr[INDEX_BITS-1:0];
    wire [TAG_BITS-1:0]cpu_addr_tag =  cpu_data_addr[INDEX_BITS +: TAG_BITS];
    wire [ZERO_BITS-1:0]cpu_addr_zero = cpu_data_addr[(INDEX_BITS + TAG_BITS) +: ZERO_BITS];

    reg [TAG_BITS-1:0]tag[0:CACHE_SIZE-1];
    wire zero_page = (cpu_addr_zero == '0);

    assign cache_hit = valid[index] && zero_page && (cpu_addr_tag == tag[index]) ;
    always @(posedge clk)
        if (!resetN)
            valid <= {CACHE_SIZE{1'b0}};
        else if (cpu_write_m && zero_page)
        begin
            valid[index] <= 1'b1;
            cache[index] <= cpu_out_m;
            tag[index] <= cpu_addr_tag;
        end

    // At the moment we just short out things to make them work.
    assign cpu_in_m      = cache_hit ? cache[index] : ram_in_m;
    assign ram_out_m     = cpu_out_m;
    assign ram_data_addr = cpu_data_addr;
    assign ram_write_m   = cpu_write_m;

    assign cpu_stall     = cpu_read_m && !cache_hit && !cpu_stall_q;

endmodule
