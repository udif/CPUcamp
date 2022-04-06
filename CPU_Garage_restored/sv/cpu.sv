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

`define PC_WIDTH 15

module cpu (
        input logic         clk,
        input logic [3:0]   SW,
        input logic         resetN,

        // RAM write interface
        output logic [15:0] out_m,
        output logic        write_m,
        output logic [14:0] write_data_addr,

        // RAM read interface
        //output logic        read_m,
        output logic [14:0] read_data_addr,
        input logic [15:0]  in_m,

        // ROM interface, 1 cycle latency
        output logic [`PC_WIDTH-2:0] inst_addr,
        input logic [31:0]inst
    );

    localparam PC_WIDTH = `PC_WIDTH;

    //////////////////////////////////////////
    reg [PC_WIDTH-1:0] pc;
    logic [PC_WIDTH-2:0] fetch_addr;
    logic [PC_WIDTH-2:0] fetch_addr_q;
    logic [15:0]inst_high_q;

    // **********************
    // *** Prefetch logic ***
    // **********************

    // always increment, but watch for jumps and assume taken
    // mark jumps, and if it fails, restart from not jump.
    // we don't allow nested speculative jumps!
    // So we stall when we run into another one if the 1st one is not cleared yet.

    logic inst_speculative, inst_speculative_q; // true once we start fetching after a jump until we confirm the jump was really taken
    logic inst_valid, inst_valid_q; // true if decode should look at this word

    // check if we have a speculative jump on inst0 (15:0)
    wire speculative_jump0 = inst[`SEL0_A] && (inst[`SEL0_LT0] || inst[`SEL0_0] || inst[`SEL0_GT0]) && !inst_high_q[`SEL0_A];
    // check if we have a speculative jump on inst1 (31:16)
    wire speculative_jump1 = !inst[`SEL0_A] && inst[`SEL1_A] && (inst[`SEL1_LT0] || inst[`SEL1_0] || inst[`SEL1_GT0]);
    assign fetch_addr =
        !inst_valid_q ? fetch_addr_q :
        // assume jump on inst1 is taken
        speculative_jump1 && !inst_speculative_q ? inst[PC_WIDTH-1:1] :
        // assume jump on inst0 is taken
        speculative_jump0 && !inst_speculative_q ? inst_high_q[PC_WIDTH-1:1] :
        // restart on missed jump
        missed_jump ? pc[PC_WIDTH-1:1] :
        // if we already speculated and have another jump, just stall
        (speculative_jump0 || speculative_jump1) && inst_speculative_q ? fetch_addr_q :
        // increment by default
        fetch_addr_q + {{(PC_WIDTH-2){1'b0}}, 1'b1};

    // Is the new word a speculative fetch after jump ?
    assign inst_speculative =
        // We just got into a speculative jump and either we also cleared previous speculation or we wen't in one
        (speculative_jump0 || speculative_jump1) && (jump_ok || !inst_speculative_q) ? 1'b1 :
        // we just cleared a speculation
        (missed_jump || jump_ok) ? 1'b0 :
        // keep last state
        inst_speculative_q;

    always @(posedge clk or negedge resetN)
    begin
        if (!resetN)
        begin
            fetch_addr_q <= {(PC_WIDTH-1){1'b0}};
            inst_valid <= 1'b0;
            inst_valid_q <= 1'b0;
        end
        else
        begin
            fetch_addr_q <= fetch_addr;
            inst_valid <= 1'b1;
            inst_valid_q <= inst_valid && !(!dual_issue && !empty_out);
        end
    end
    always @(posedge clk)
    begin
        inst_speculative_q <= inst_speculative; // same timing as inst
        inst_high_q <= inst[31:16];
    end

    assign inst_addr = fetch_addr;

    logic empty_out, speculative_out;
    logic [PC_WIDTH-2:0]fetch_addr_out;
    logic [31:0]inst_out;
    logic pop, pop_d, pop_q2;
    assign pop = dual_issue || !dual_issue_d;

    //
    // instruction FIFO
    // data is instruction word + speculative bit + address
    //
    generic_fifo #(
        .DEPTH(4),
        .WIDTH(32 + 1 + (PC_WIDTH - 1))
    ) inst_fifo (
        .clk(clk),
        .resetN(resetN),
        .push(inst_valid_q),
        .pop(pop),
        .wdata({inst_speculative_q, fetch_addr_q, inst}),

        .flush(missed_jump),
        .full(),
        .empty(empty_out),
        .rdata({speculative_out, fetch_addr_out, inst_out})
    );

    // ********************
    // *** decode stage ***
    // ********************
    // calculate all signals that are based only on the instruction word

    // handle missed jump predictions ASAP
    logic dual_issue, dual_issue_d, issue_sel, issue_sel_d;
    assign dual_issue = !empty_out && !inst_out[`SEL0_A] && inst_out[`SEL1_A];
    // if dual issue, it's a new cycle, so start with inst0
    // if last cycle was dual issue, then this cycle starts with inst0 regardless of single or dual issue
    assign issue_sel = (dual_issue || dual_issue_d) ? 1'b0 : 1'b1 ;
    // decode stage vars
    logic [1:0]pc_inc_d;
    logic load_a, load_d, load_m;
    logic load_a_d, load_d_d, load_m_d;
    logic [5:0]alu_fn_d;
    logic sel_am_nobypass_d, sel_a_d;
    logic [15:0]sel_a_inst_d;
    logic speculative_d;
    logic jump0_lt_d, jump0_0_d, jump0_gt_d;
    logic jump1_lt_d, jump1_0_d, jump1_gt_d;
    logic jump_a_en_d;
    logic [31:0]inst_out_d;
    logic valid_decode, valid_speculative_d, valid_d;
    logic [PC_WIDTH-2:0]fetch_addr_d;

    logic sel_am;
    assign sel_am = (dual_issue || !pop_q2) ? inst_out[`SEL1_AM] :
                                            inst_out[`SEL0_AM];

    // These signals are not qualifiers so we don't need to reset them
    always @(posedge clk)
    begin
        alu_fn_d <= (dual_issue || !pop) ? inst_out[`SEL1_C] :
                                             inst_out[`SEL0_C]; //function for the ALU
        //select if the ALU's Y input is from ram or from A register
        // jump qualifiers for inst0. if inst0 qualifier as a jump there is nothing we can do.
        jump0_lt_d <= !empty_out && inst_out[`SEL0_A] && inst_out[`SEL0_LT0];
        jump0_0_d  <= !empty_out && inst_out[`SEL0_A] && inst_out[`SEL0_0];
        jump0_gt_d <= !empty_out && inst_out[`SEL0_A] && inst_out[`SEL0_GT0];
        // jump qualifiers for ALU outputs on inst1
        // 2 cases:
        // 1. dual_issue, in which case inst0 is only a type A, so just check inst1 flags
        // 2. regular case, make sure PC points to inst[1].
        jump1_lt_d <= !empty_out && (!pop_q2 || dual_issue) && inst_out[`SEL1_LT0];
        jump1_0_d  <= !empty_out && (!pop_q2 || dual_issue) && inst_out[`SEL1_0];
        jump1_gt_d <= !empty_out && (!pop_q2 || dual_issue) && inst_out[`SEL1_GT0];
        speculative_d <= speculative_out && !empty_out;
        inst_out_d <= {32{!empty_out}} & inst_out;
        jump_a_en_d <=  pop_d && !dual_issue;
        valid_d <= !empty_out;
        // in dual issue mode, alu is processing inst1 with A data from inst0,
        // but if m data was written 1 cycle earlier, we need to bypass it.
        sel_am_nobypass_d <= sel_am && !in_m_bypass_next;
        sel_a_inst_d <= {16{dual_issue & ~sel_am}} & {1'b0, inst_out[0 +: 15]};
        sel_a_d <= !sel_am && !dual_issue;
        dual_issue_d <= dual_issue;
        issue_sel_d <= issue_sel;
        fetch_addr_d <= fetch_addr_out;
        pop_d <= pop;
        pop_q2 <= pop_d;
    end

    assign load_a =                      (!inst_out[`SEL0_A] || inst_out[`SEL0_LD_A]) ||
        (dual_issue || !dual_issue_d) && (!inst_out[`SEL1_A] || inst_out[`SEL1_LD_A]);
    assign load_d =                      ( inst_out[`SEL0_A] && inst_out[`SEL0_LD_D]) ||
        (dual_issue || !dual_issue_d) && ( inst_out[`SEL1_A] && inst_out[`SEL1_LD_D]);
    assign load_m =                      ( inst_out[`SEL0_A] && inst_out[`SEL0_LD_M]) ||
        (dual_issue || !dual_issue_d) && ( inst_out[`SEL1_A] && inst_out[`SEL1_LD_M]);

    always @(posedge clk)
    begin
        pc_inc_d <= dual_issue ? 2'd2 : 2'd1;
        // load A/D can come from either instruction. we cover the following cases:
        // pc[0] == 0, dual issue, if inst_out[0] is A instruction
        // pc[0] == 0, single issue, if inst_out[0] is C instruction
        // pc[0] == 1, single issue
        load_a_d <= !empty_out && !missed_jump && load_a;
        load_d_d <= !empty_out && !missed_jump && load_d;
        load_m_d <= !empty_out && !missed_jump && load_m;
    end

    //
    // Read data port:
    // at the moment, address is sent on decode stage and result returns to execute stage
    //
    assign read_data_addr =
        dual_issue ? inst_out[0  +: 15] :
                     a[14:0];
    //assign read_m  =
    //    (dual_issue ? inst_out[`SEL1_AM] :
    //     pc[0]      ? inst_out[`SEL1_AM] && inst_out[`SEL1_A]:
    //                  inst_out[`SEL0_AM] && inst_out[`SEL0_A]);

    // *********************
    // *** execute stage ***
    // *********************

    wire [15:0]am =
        {16{in_m_bypass}} & out_m_q |
        {16{sel_am_nobypass_d}} & in_m |
        sel_a_inst_d |
        {16{sel_a_d}} & a; // decide whether the alu will use the data in memory or in the A register
    
    wire [15:0]alu_out;
    logic in_m_bypass;
    logic [15:0]out_m_q;

    reg [15:0] a;
    reg [15:0] d;

    //ALU module instantiation
    alu alu0(
            .x(d),
            .y(am),
            .out(alu_out),
            .fn(alu_fn_d),
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

    wire in_m_bypass_next = write_m && sel_am && (read_data_addr == write_data_addr);
    always@(posedge clk)
    begin
        // if next cycle read is the same address we write, and we need it for the ALU, bypass value
        in_m_bypass <= in_m_bypass_next;
        out_m_q <= out_m;
    end

    // ALU flags
    wire zero; //zero flag from ALU
    wire less_than_zero = alu_out[15];
    wire greater_than_zero = !(less_than_zero || zero);

    logic jump_ok;
    logic missed_jump;
    assign jump_ok      = speculative_d && (fetch_addr_d == pc[PC_WIDTH-1:1]);
    assign missed_jump  = speculative_d && (fetch_addr_d != pc[PC_WIDTH-1:1]);
    assign valid_decode = valid_d && !missed_jump;

    wire jump0 = (less_than_zero && jump0_lt_d) || (zero && jump0_0_d) || (greater_than_zero && jump0_gt_d);
    wire jump1 = (less_than_zero && jump1_lt_d) || (zero && jump1_0_d) || (greater_than_zero && jump1_gt_d);
    wire jump_a = jump1 && jump_a_en_d || jump0;
    wire jump_inst = jump1 && dual_issue_d;
    // if we autoinc, we go to the beginning of the next 32-bit word
    wire [PC_WIDTH-1:0] new_pc =
        jump_inst    ? inst_out_d[PC_WIDTH-1:0] :
        jump_a       ? a[PC_WIDTH-1:0] :
        valid_decode ? pc + {{(PC_WIDTH-2){1'b0}}, pc_inc_d} :
                     pc;

    wire [15:0] next_a =
        (!pc[0] && !inst_out_d[`SEL0_A]) ? {1'b0, inst_out_d[0  +: 15]} :
        ( pc[0] && !inst_out_d[`SEL1_A]) ? {1'b0, inst_out_d[16 +: 15]} :
        alu_out;
    wire [15:0] next_d =
        alu_out;
    assign write_data_addr =
        dual_issue_d ? inst_out_d[14:0] :
                       a[14:0];
    assign out_m = alu_out;
    assign write_m = load_m_d && valid_decode;

    always @(posedge clk or negedge resetN)
        if (!resetN)
            pc <= {PC_WIDTH{1'b0}};
        else
            pc <= new_pc;

    always @(posedge clk)
    begin
        if (load_a_d && valid_decode)
            a <= next_a;
        if (load_d_d && valid_decode)
            d <= next_d;
    end

endmodule
