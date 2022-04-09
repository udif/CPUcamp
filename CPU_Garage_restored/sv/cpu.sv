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
`define SEL1_C    (6  + 16) +: 6
`define SEL1_LD_A (5  + 16)
`define SEL1_LD_D (4  + 16)
`define SEL1_LD_M (3  + 16)
`define SEL1_LT0  (2  + 16)
`define SEL1_0    (1  + 16)
`define SEL1_GT0  (0  + 16)

`define SEL2_A    (15 + 32)
`define SEL2_AM   (12 + 32)
`define SEL2_C    (6  + 32) +: 6
`define SEL2_LD_A (5  + 32)
`define SEL2_LD_D (4  + 32)
`define SEL2_LD_M (3  + 32)
`define SEL2_LT0  (2  + 32)
`define SEL2_0    (1  + 32)
`define SEL2_GT0  (0  + 32)

`define SEL3_A    (15 + 48)
`define SEL3_AM   (12 + 48)
//`define SEL3_C    (6  + 48) +: 6
`define SEL3_C    59:54
`define SEL3_LD_A (5  + 48)
`define SEL3_LD_D (4  + 48)
`define SEL3_LD_M (3  + 48)
`define SEL3_LT0  (2  + 48)
`define SEL3_0    (1  + 48)
`define SEL3_GT0  (0  + 48)

`define PC_WIDTH 15

`define FETCH_W  (`PC_WIDTH - 2)

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
        output logic [14:0] read_data2_addr,
        input logic [15:0]  in_m,
        input logic [15:0]  in_m2,

        // ROM interface, 1 cycle latency
        output logic [`FETCH_W-1:0] inst_addr,
        input logic [63:0]inst
    );

    localparam PC_WIDTH = `PC_WIDTH;
    localparam FETCH_W = `FETCH_W;
    localparam DW = 32;

    //////////////////////////////////////////
    reg [PC_WIDTH-1:0] pc;
    logic [FETCH_W-1:0] fetch_addr;
    logic [FETCH_W-1:0] fetch_addr_q;
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
    wire speculative_jump2 = !inst[`SEL1_A] && inst[`SEL2_A] && (inst[`SEL2_LT0] || inst[`SEL2_0] || inst[`SEL2_GT0]);
    wire speculative_jump3 = !inst[`SEL2_A] && inst[`SEL3_A] && (inst[`SEL3_LT0] || inst[`SEL3_0] || inst[`SEL3_GT0]);
    assign fetch_addr =
        // restart on missed jump
        missed_flush ? pc[PC_WIDTH-1:2] :
        !inst_valid_q ? fetch_addr_q :
        // assume jump is taken
        // if we stall on empty FIFO due to previous misprediction and wait for inst1, we can't prefetch based on inst0 jump!!!
        (speculative_jump0) && !(empty_out && pc[0]) ? inst_high_q[PC_WIDTH-1:2] :
        (speculative_jump1) ? inst[2 +: FETCH_W] :
        (speculative_jump2) ? inst[18 +: FETCH_W] :
        (speculative_jump3) ? inst[34 +: FETCH_W] :
        // if we already speculated and have another jump, just stall
        (speculative_jump0 || speculative_jump1) && inst_speculative_q ? fetch_addr_q :
        // increment by default
        fetch_addr_q + {{(FETCH_W-1){1'b0}}, 1'b1};

    // Is the new word a speculative fetch after jump ?
    assign inst_speculative =
        // We just got into a speculative jump and either we also cleared previous speculation or we weren't in one
        (speculative_jump0 || speculative_jump1 || speculative_jump2 || speculative_jump3) && (jump_ok1 || jump_ok2 || !inst_speculative_q) ? 1'b1 :
        // we just cleared a speculation
        (jump_ok1 || jump_ok2) ? 1'b0 :
        // keep last state
        inst_speculative_q;

    // inst_valid_q is a qualifier for the ROM q output
    // we always fetch, UNLESS the non-empty FIFO output indicates we have a single issue,
    // which will take 2 cycles to consume, so we don't increment for 1 cycle

    always @(posedge clk or negedge resetN)
    begin
        if (!resetN)
        begin
            fetch_addr_q <= {FETCH_W{1'b0}};
            inst_valid <= 1'b0;
            inst_valid_q <= 1'b0;
            inst_speculative_q <= 1'b0;
        end
        else
        begin
            fetch_addr_q <= fetch_addr;
            inst_valid <= (depth < 4'h4) ? 1'b1 : 1'b0;
            inst_valid_q <= inst_valid;
            //inst_valid_q <= inst_valid && !almost_full_out && !(!dual_issue_out && !empty_out);
            inst_speculative_q <= inst_speculative; // same timing as inst
        end
    end
    always @(posedge clk)
        if (inst_valid_q)
            inst_high_q <= inst[63:48];

    //
    // ROM address bus output
    //
    assign inst_addr = fetch_addr;

    //
    // End of prefetch
    // if/when we have time, we will move it to a stand alone module
    //

    logic empty_out, speculative_out;
    logic [FETCH_W-1:0]fetch_addr_out;
    logic [63:0]inst_wide;
    logic [DW-1:0]inst_out;
    logic pop_d;
        logic high_low;
    always @(posedge clk)
        high_low <=
            empty_out ? 1'b0 :
            wide_pop ? 1'b0 :
            (pop && !empty_out) ? !high_low :
            high_low;
    wire wide_pop = pop && (high_low || quad_issue_out);

    assign inst_out = high_low ? inst_wide[63:32] : inst_wide[31:0];

    // always pop, UNLESS output output valid, we don't have dual issue, and are only on inst0
    // (must keep output for inst1 next cycle)
    wire pop = !(valid_out && !dual_issue_out && !pc0_out);

    //
    // instruction FIFO
    // data is instruction word + speculative bit + address
    //
    wire [3:0]depth;
    generic_fifo #(
        .DEPTH(8),
        .WIDTH(64 + 1 + FETCH_W)
    ) inst_fifo (
        .clk(clk),
        .resetN(resetN),
        .push(inst_valid_q),
        .pop(wide_pop),
        .wdata({inst_speculative_q, fetch_addr_q, inst}),

        .flush(missed_flush),
        .full(),
        .empty(empty_out),
        .rdata({speculative_out, fetch_addr_out, inst_wide}),
        .depth(depth)
    );

    //
    // Jump misprediction recovery
    // we detect missed jumps in Execute stage since only then we know if the jump took place or not
    // once we know this we set a flag here that will clear right on the fifo output.
    //

    // we use PC although it is set in execute stage because it is frozen once in misprediction
    // This is the fast recovery in case we already stalled
    wire jump_ok1 = stall_on_mispredict && (fetch_addr_out == pc[PC_WIDTH-1:2]);
    // save misprediction state
    logic stall_on_mispredict;
    always @(posedge clk or negedge resetN)
        if (!resetN)
            stall_on_mispredict <= 1'b0;
        else
            stall_on_mispredict <=
                missed_jump ? 1'b1 :
                (jump_ok1 || jump_ok2) ? 1'b0 :
                stall_on_mispredict;
    // valid flag - we must have a word and must not be in a misprediction where we have words, but they are ignored
    // jump_ok is to catch the 1st cycle after misprediction, !stall_on_mispredict is for that point onwards
    wire valid_out = !empty_out && (!stall_on_mispredict || jump_ok1 || jump_ok2);

    //
    // track which instructions got already executed
    // pc0_out is the value of pc[0] on the output of the FIFO
    //
    logic pc0_out, pc0_d, pc0_e;
    always @(posedge clk or negedge resetN)
    begin
        if (!resetN)
            pc0_out <= 1'b0;
        else
            if (valid_out && !dual_issue_out )
                pc0_out <= !pc0_out;
            else if (valid_out && dual_issue_out )
                pc0_out <= 1'b0;
            else if (missed_jump)
                pc0_out <= pc0_e; // restore last good value
    end
    // we keep the last 2 stage history and if we detect misprediction we rollback!
    // pc0_e will be the last valid value of pc0_out on the cycle prior to the misprediction
    always @(posedge clk)
    begin
        pc0_d <= pc0_out;
        pc0_e <= pc0_d; // 2 cycle latency
    end

    //
    // ********************
    // *** decode stage ***
    // ********************
    // calculate all signals that are based only on the instruction word

    //
    // Dual issue handling
    // dual issue detection is critical, and must handle mispredictions
    // (must remember which instruction was executed before misprediction, if jump was on inst0)
    logic dual_issue_d, quad_issue_d;
    // True if we can issue 2 instruction on this cycle
    // we do this with type A on inst0, and type C on inst1
    // perhaps we could dual issue on two type A, but this is a useless sequence, as inst0 is simply ignored
    wire dual_issue_out = !inst_out[`SEL0_A] && inst_out[`SEL1_A];
    wire quad_issue_out = 
        !inst_wide[`SEL0_A] && inst_wide[`SEL1_A] && !inst_wide[`SEL2_A] && inst_wide[`SEL3_A] &&
        ((inst_wide[`SEL3_C] == 6'h30) || (inst_wide[`SEL3_C] == 6'h0c)) && // just data transfer A/M/D
        (!inst_wide[`SEL3_LD_M] || !inst_wide[`SEL1_LD_M]) && //only one Memory writes
        !inst_wide[`SEL3_LD_A] &&
        !inst_wide[`SEL1_LD_A] && // no A write by ALU
        !(inst_wide[`SEL3_LT0] || inst_wide[`SEL3_0]); // no jumps, at least for the moment
    // True if inst1 is executed this cycle, either with dual_issue (type C), or as single instruction (type A or C)
    wire inst0_out = dual_issue_out || !pc0_out;
    wire inst1_out = dual_issue_out ||  pc0_out;

    // decode stage vars
    logic [2:0]pc_inc_d;
    logic load_a, load_d, load_m;
    logic load_a_d, load_d_d, load_m_d;
    logic [5:0]alu_fn_d;
    logic sel_am_nobypass_d, sel_a_d;
    logic speculative_d;
    logic jump0_lt_d, jump0_0_d, jump0_gt_d;
    logic jump1_lt_d, jump1_0_d, jump1_gt_d;
    logic jump_a_en_d;
    logic [31:0]inst_out_d;
    logic [63:0]inst_wide_d;
    logic valid_d;
    logic [FETCH_W-1:0]fetch_addr_d;

    // select A or M
    // we have one ALU, that works on the instruction pointed by pc0_out,
    // but in dual_issue cycles, inst0 is A type, and inst1 is C type
    wire sel_am = inst1_out ? inst_out[`SEL1_AM] :
                              inst_out[`SEL0_AM];

    // These signals are not feedback based so we don't need to reset them
    always @(posedge clk)
    begin
        alu_fn_d <= inst1_out ? inst_out[`SEL1_C] :
                                inst_out[`SEL0_C]; //function for the ALU

        //select if the ALU's Y input is from ram or from A register
        // jump qualifiers for inst0. if inst0 qualifier as a jump there is nothing we can do.
        jump0_lt_d <= valid_out && inst_out[`SEL0_A] && inst_out[`SEL0_LT0];
        jump0_0_d  <= valid_out && inst_out[`SEL0_A] && inst_out[`SEL0_0];
        jump0_gt_d <= valid_out && inst_out[`SEL0_A] && inst_out[`SEL0_GT0];

        // jump qualifiers for ALU outputs on inst1
        // 2 cases:
        // 1. dual_issue, in which case inst0 is only a type A, so just check inst1 flags
        // 2. regular case, make sure PC points to inst[1].
        jump1_lt_d <= valid_out && inst1_out && inst_out[`SEL1_LT0];
        jump1_0_d  <= valid_out && inst1_out && inst_out[`SEL1_0];
        jump1_gt_d <= valid_out && inst1_out && inst_out[`SEL1_GT0];
        // If this signal is active, the decode stage is carrying a speculative word
        speculative_d <= speculative_out && !empty_out;
        inst_out_d <= inst_out;
        inst_wide_d <= inst_wide[63:0];
        jump_a_en_d <=  pop_d && !dual_issue_out; // TODO try replacing with saomething more reliable

        valid_d <= valid_out;

        // in dual issue mode, alu is processing inst1 with A data from inst0,
        // but if m data was written 1 cycle earlier, we need to bypass it.
        sel_am_nobypass_d <= sel_am && !in_m_bypass;

        // A is selected from active inst. if not dual issue, then either 
        // ALU is in inst0 and A was set last cycle, or if ALU on inst1
        // and not dual issue, A was set from inst0 a cycle before
        sel_a_d <= !sel_am && !dual_issue_out;
        dual_issue_d <= dual_issue_out;
        quad_issue_d <= quad_issue_out;
        fetch_addr_d <= fetch_addr_out;
        pop_d <= pop;
    end

    assign load_a =
        quad_issue_out && !inst_wide[`SEL2_A] ||
        inst0_out && (!inst_out[`SEL0_A] || inst_out[`SEL0_LD_A]) ||
        inst1_out && (!inst_out[`SEL1_A] || inst_out[`SEL1_LD_A]);
    assign load_d =
        quad_issue_out && inst_wide[`SEL3_LD_D] ||
        inst0_out && ( inst_out[`SEL0_A] && inst_out[`SEL0_LD_D]) ||
        inst1_out && ( inst_out[`SEL1_A] && inst_out[`SEL1_LD_D]);
    assign load_m =
        quad_issue_out && (inst_wide[`SEL1_LD_M] || inst_wide[`SEL3_LD_M]) ||
        inst0_out      && ( inst_out[`SEL0_A] && inst_out[`SEL0_LD_M]) ||
        inst1_out      && ( inst_out[`SEL1_A] && inst_out[`SEL1_LD_M]);

    assign pc_inc_d = quad_issue_d ? 3'd4 : dual_issue_d ? 3'd2 : 3'd1;
    always @(posedge clk)
    begin
        // load A/D can come from either instruction. we cover the following cases:
        // pc[0] == 0, dual issue, if inst_out[0] is A instruction
        // pc[0] == 0, single issue, if inst_out[0] is C instruction
        // pc[0] == 1, single issue
        load_a_d <= valid_out && !missed_jump && load_a;
        load_d_d <= valid_out && !missed_jump && load_d;
        load_m_d <= valid_out && !missed_jump && load_m;
    end

    //
    // Read data port:
    // at the moment, address is sent on decode stage and result returns to execute stage
    //
    assign read_data_addr =
        // latest
        dual_issue_out ? inst_out[0  +: 15] :
        // 1 cycle before
        load_a_d ? next_a[14:0] :
        // older
                   a[14:0];

    assign read_data2_addr = inst_wide[32  +: 15];

    // *********************
    // *** execute stage ***
    // *********************

    //
    // ALU a/m input:
    // Can come from many sources. we use OR on all sources, and make sure they are 0 when not relevant
    //
    // 1. ALU on inst1, A selected from inst0 (type A) due to dual issue => sel_a_inst
    // 2. ALU on inst0 or no dual issue so A already valid               => sel_a_sampled
    // 3. M is selected, and no write to same address in last cycle      => sel_m_from_mem
    // 4. M is selected, but write to same address happenned last cycle,
    //    so bypass from write bus, and don't count on RAM R/w priority  => in_m_bypassed_by_out_m
    wire [15:0]sel_a_inst = {16{dual_issue_out & ~sel_am}} & {1'b0, inst_out[0 +: 15]};
    wire [15:0]sel_a_sampled = {16{sel_a_d}} & a;
    wire [15:0]sel_m_from_mem = {16{sel_am_nobypass_d}} & in_m;
    wire [15:0]in_m_bypassed_by_out_m = {16{in_m_bypass}} & out_m;
    // qualifier for bypass operation
    wire in_m_bypass = write_m && sel_am && (read_data_addr == write_data_addr);

    // Two of these busses are already valid 1 cycle before, so we OR them and sample
    logic [15:0]am_partial;
    always@(posedge clk)
        am_partial <=  in_m_bypassed_by_out_m | sel_a_inst;

    // Final A/M ALU input bus, ORing all 4 sources above
    wire [15:0]am =
        am_partial |
        sel_m_from_mem |
        sel_a_sampled; // decide whether the alu will use the data in memory or in the A register
    
    wire [15:0]alu_out;
    wire [15:0]alu_out2;

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
    //
    // TODO <udif> is this comment still relevant for new version?? probaly not


    // ALU flags
    wire zero; //zero flag from ALU
    wire less_than_zero = alu_out[15];
    wire greater_than_zero = !(less_than_zero || zero);

    // if fetch indicates this is a speculative word and the addresses don't match, it s a misprediction
    wire missed_jump = valid_d && (fetch_addr_d != pc[PC_WIDTH-1:2]);
    // we use this in case we didn't stall yet, the PC moves and we can't use jump_ok1
    wire jump_ok2    = speculative_d && (fetch_addr_d == pc[PC_WIDTH-1:2]);
    // make a 1 cycle pulse
    wire missed_flush = missed_jump && !missed_flush_q;
    
    logic missed_flush_q;
    always @(posedge clk)
        missed_flush_q <= missed_flush;

    wire jump0 = (less_than_zero && jump0_lt_d) || (zero && jump0_0_d) || (greater_than_zero && jump0_gt_d);
    wire jump1 = (less_than_zero && jump1_lt_d) || (zero && jump1_0_d) || (greater_than_zero && jump1_gt_d);
    // jump to address on A
    // this is either on jump0 (because A was set last cycle), or on jump1 if jump_a_en was true last cycle (jump_a_en_d)
    wire jump_a = jump1 && jump_a_en_d || jump0;
    wire jump_inst = jump1 && dual_issue_d;
    wire jump_inst_quad = quad_issue_d && (!next_d[15]) && (|next_d[14:0]) && inst_wide_d[`SEL1_LD_D] && inst_wide_d[`SEL3_GT0];
    // if we autoinc, we go to the beginning of the next 32-bit word
    wire [PC_WIDTH-1:0] new_pc =
        !valid_d     ? pc :
        jump_inst_quad ? inst_wide_d[32 +: 15] :
        jump_inst    ? inst_out_d[PC_WIDTH-1:0] :
        jump_a       ? a[PC_WIDTH-1:0] :
        !missed_jump ? pc + {{(PC_WIDTH-3){1'b0}}, pc_inc_d} :
                     pc;

    logic next_a_inst_sel_d;
    logic [14:0] next_a_inst_d;
    wire [14:0]next_a_inst =
        (quad_issue_out && !inst_wide[`SEL2_A]) ? inst_wide[32 +: 15] :
        (quad_issue_out && !inst_wide[`SEL0_A]) ? inst_wide[0 +: 15] :
        (!pc0_out && !inst_out[`SEL0_A]) ? inst_out[0  +: 15] :
                                           inst_out[16 +: 15];
    always @(posedge clk)
    begin
        next_a_inst_d = next_a_inst;
        next_a_inst_sel_d <=
            quad_issue_out ||
            (!pc0_out && !inst_out[`SEL0_A]) ||
            ( pc0_out && !inst_out[`SEL1_A]);
    end
            
    wire [15:0] next_a =
        next_a_inst_sel_d ? {1'b0, next_a_inst_d} :
        alu_out;
    wire [15:0] next_d =
        (quad_issue_d && inst_wide_d[`SEL3_LD_D] && !inst_wide_d[`SEL3_AM] && (inst_wide_d[`SEL3_C] == 6'h30) ) ? next_a :
        (quad_issue_d && inst_wide_d[`SEL3_LD_D] &&  inst_wide_d[`SEL3_AM] && (inst_wide_d[`SEL3_C] == 6'h30) ) ? in_m2 :
        alu_out;
    assign write_data_addr =
        (quad_issue_d && inst_wide_d[`SEL3_LD_M]) ?  inst_wide_d[32 +: 15] :
        dual_issue_d ? inst_out_d[14:0] :
                       a[14:0];
    assign out_m =
        (quad_issue_d && inst_wide_d[`SEL3_LD_M] && (inst_wide_d[`SEL3_C] == 6'h0c)) ? next_d : 
                                                         alu_out;
    assign write_m = load_m_d && !missed_jump;

    always @(posedge clk or negedge resetN)
        if (!resetN)
            pc <= {PC_WIDTH{1'b0}};
        else
            pc <= new_pc;

    always @(posedge clk)
    begin
        if (load_a_d && !missed_jump)
            a <= next_a;
        if (load_d_d && !missed_jump)
            d <= next_d;
    end

endmodule
