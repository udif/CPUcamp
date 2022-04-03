`define SEL0_A   15
`define SEL0_AM  12
`define SEL0_C    6 +: 6
`define SEL0_LD_A 5
`define SEL0_LD_D 4
`define SEL0_LD_M 3
`define SEL0_LT0  2
`define SEL0_0    1
`define SEL0_GT0  0

`define SEL1_A    (15 + 16)
`define SEL1_AM   (12 + 16)
`define SEL1_C    (6 + 16) +: 6
`define SEL1_LD_A (5 + 16)
`define SEL1_LD_D (4 + 16)
`define SEL1_LD_M (3 + 16)
`define SEL1_LT0  (2 + 16)
`define SEL1_0    (1 + 16)
`define SEL1_GT0  (0 + 16)

module cpu #(
    parameter PC_WIDTH
) (
        input logic         clk,
        input logic [3:0]   SW,
        input logic [31:0]  inst,
        input logic [15:0]  in_m,
        input logic         resetN,

        input logic         stall,

        output logic [15:0] out_m,
        output logic        write_m,
        output logic        read_m,
        output logic [14:0] data_addr,
        output logic [PC_WIDTH-2:0] inst_addr
    );

    //////////////////////////////////////////
    reg [PC_WIDTH-1:0] pc;
    reg [15:0] a;
    reg [15:0] d;

    wire stall_reset = stall || !resetN;

    wire [15:0]m = in_m;
    wire [15:0]am =
        sel_am     ? m :
        dual_issue ? {1'b0, inst[0 +: 15]} : // in dual issue mode, alu is processing inst1 with A data from inst0
                     a; // decide whether the alu will use the data in memory or in the A register
    wire [15:0]alu_out;
    wire  [5:0]alu_fn = (dual_issue || pc[0]) ? inst[`SEL1_C] :
                                                inst[`SEL0_C]; //function for the ALU

    //ALU module instantiation
    alu alu0(
            .x(d),
            .y(am),
            .out(alu_out),
            .fn(alu_fn),
            .zero(zero)
        );
    //
    // Dual issue logic
    //
    // since 99% of the code uses an A instruction followed by a C instruction,
    // lets optimize this case.
    // HOWEVER, if inst[1] may jump, we don't dual issue to let A gets its value first
    // supporting this for the 1% of jumps we have at inst[1] would have cost us in a long
    // path from in_m, through the ALU, to new_pc and the instruction ROM
    
    wire dual_issue = !inst[`SEL0_A] &&
                       inst[`SEL1_A] &&
                      !inst[`SEL1_LT0] && !inst[`SEL1_0] && !inst[`SEL1_GT0];
    wire [1:0]pc_inc = dual_issue ? 2'd2 : 2'd1;

    // ALU flags
    wire zero; //zero flag from ALU
    wire less_than_zero = alu_out[15];
    wire greater_than_zero = !(less_than_zero || zero);

    // load A/D can come from either instruction. we cover the following cases:
    // pc[0] == 0, dual issue, if inst[0] is A instruction
    // pc[0] == 0, single issue, if inst[0] is C instruction
    // pc[0] == 1, single issue
    wire load_a =               !pc[0]  && (!inst[`SEL0_A] || inst[`SEL0_LD_A]) ||
                  (dual_issue || pc[0]) && (!inst[`SEL1_A] || inst[`SEL1_LD_A]);
    wire load_d =               !pc[0]  && ( inst[`SEL0_A] && inst[`SEL0_LD_D]) ||
                  (dual_issue || pc[0]) && ( inst[`SEL1_A] && inst[`SEL1_LD_D]);
    wire load_m =               !pc[0]  && ( inst[`SEL0_A] && inst[`SEL0_LD_M]) ||
                  (dual_issue || pc[0]) && ( inst[`SEL1_A] && inst[`SEL1_LD_M]);
    wire jump =   pc[0] ?   inst[`SEL1_A] && ((less_than_zero && inst[`SEL1_LT0]) || (zero && inst[`SEL1_0]) || (greater_than_zero && inst[`SEL1_GT0])) :
                            inst[`SEL0_A] && ((less_than_zero && inst[`SEL0_LT0]) || (zero && inst[`SEL0_0]) || (greater_than_zero && inst[`SEL0_GT0]));

    wire sel_a  = pc[0] ? inst[`SEL1_A] :
                          inst[`SEL0_A];
    //select if the ALU's Y input is from ram or from A register
    wire sel_am = (dual_issue || pc[0]) ? inst[`SEL1_AM] :
                                          inst[`SEL0_AM];

    // if we autoinc, we go to the beginning of the next 32-bit word
    wire [PC_WIDTH-1:0] new_pc =
        jump ? a[PC_WIDTH-1:0] :
        pc + {{(PC_WIDTH-2){1'b0}}, pc_inc};

    wire [15:0] next_a =
        stall ? a :
        (!pc[0] && !inst[`SEL0_A]) ? {1'b0, inst[0  +: 15]} :
        ( pc[0] && !inst[`SEL1_A]) ? {1'b0, inst[16 +: 15]} :
        alu_out;
    wire [15:0] next_d =
        stall ? d :
        alu_out;
    assign data_addr =
        dual_issue ? inst[0  +: 15] :
                     a[14:0];
    assign out_m = alu_out;
    assign write_m = load_m && !stall;
    assign read_m  =
        (dual_issue ? inst[`SEL1_AM] :
         pc[0]      ? inst[`SEL1_AM] && inst[`SEL1_A]:
                      inst[`SEL0_AM] && inst[`SEL0_A]);
    assign inst_addr = !stall_reset ? new_pc[PC_WIDTH-1:1] : pc[PC_WIDTH-1:1];

    always @(posedge clk)
        if (!resetN)
            pc <= {PC_WIDTH{1'b0}};
        else if (!stall)
            pc <= new_pc;

    always @(posedge clk)
        if (load_a)
            a <= next_a;

    always @(posedge clk)
        if (load_d)
            d <= next_d;

endmodule
