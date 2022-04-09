// DESCRIPTION: Verilator: Verilog example module
//
// This file ONLY is placed under the Creative Commons Public Domain, for
// any use, without warranty, 2017 by Wilson Snyder.
// SPDX-License-Identifier: CC0-1.0
//======================================================================

#include <time.h>
#include <iostream>
#include <iomanip>
#include <bitset>
#include <iomanip>
#include <string>
// For std::unique_ptr
#include <memory>

// Include common routines
#include <verilated.h>

// Include model header, generated from Verilating "top.v"
#include "Vtop.h"
#include "Vtop_top.h"

int main(int argc, char** argv, char** env) {
    // See a similar example walkthrough in the verilator manpage.

    // This is intended to be a minimal example.  Before copying this to start a
    // real project, it is better to start with a more complete example,
    // e.g. examples/c_tracing.

    // Prevent unused variable warnings
    if (false && argc && argv && env) {}

    // Create logs/ directory in case we have traces to put under it
    Verilated::mkdir("logs");

    // Construct a VerilatedContext to hold simulation time, etc.
    // Multiple modules (made later below with Vtop) may share the same
    // context to share time, or modules may have different contexts if
    // they should be independent from each other.

    // Using unique_ptr is similar to
    // "VerilatedContext* contextp = new VerilatedContext" then deleting at end.
    const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};

    // Set debug level, 0 is off, 9 is highest presently used
    // May be overridden by commandArgs argument parsing
    contextp->debug(0);

    // Randomization reset policy
    // May be overridden by commandArgs argument parsing
    contextp->randReset(2);

    // Verilator must compute traced signals
    contextp->traceEverOn(true);

    // Pass arguments so Verilated code can see them, e.g. $value$plusargs
    // This needs to be called before you create any model
    contextp->commandArgs(argc, argv);

    // Construct the Verilated model, from Vtop.h generated from Verilating "top.v".
    // Using unique_ptr is similar to "Vtop* top = new Vtop" then deleting at end.
    // "TOP" will be the hierarchical name of the module.
    const std::unique_ptr<Vtop> top{new Vtop{contextp.get(), "TOP"}};

    top->BUTTON = 0;
    int outer_loop = 0;
    for (int i = 0; i < 5; i++) {
    	top->CLK_50 = 0;
        contextp->timeInc(1);
    	top->eval();
        top->CLK_50 = 1;
        contextp->timeInc(1);
        top->eval();
    }
    int wide = 0;
    int t;
    int count = 0;
    clock_t start_time = clock();
    // Simulate until $finish
    while (!Verilated::gotFinish()) {
        // Evaluate model
    	top->CLK_50 = 0;
        contextp->timeInc(1);
    	top->eval();
        top->CLK_50 = 1;
        contextp->timeInc(1);
        top->eval();
        t = contextp->time();
        if (t > 10)
            top->BUTTON = 1;
        //if (count > 1000)
        //    break;
        if (top->top->ram_write_m && (top->top->ram_address & 0x3ff) > 2) {
            //std::cout << "address=" << std::setfill('0') << std::setw(3) << std::hex << top->top->__PVT__cpu_inst__DOT__pc;
            //std::cout << " inst="   << std::setfill('0') << std::setw(4) << std::hex << top->top->instruction;
            std::cout << " ram_address=" << std::setfill('0') << std::setw(3) << (top->top->ram_address & 0x3ff);
            std::cout << " write_data="  << std::setfill('0') << std::setw(5) << std::dec << top->top->cpu_out_m;
            std::cout << std::endl;
        }
        if (top->top->__PVT__cpu_inst__DOT__pc < 24)
            outer_loop = 0;
        if (top->top->__PVT__cpu_inst__DOT__pc > 159) {
            count++;
            //std::cout << count << std::endl;
            //std::cout << outer_loop << "pc:" << top->top->__PVT__cpu_inst__DOT__pc << std::endl;
            if (++outer_loop == 20) {
                std::cout << "Got to final address\n";
                break;
            }
        }
    }
    clock_t end_time = clock();
    for (int i = 0 ; i < 24; i += 2) {
        std::string s1 = std::bitset<16>{top->top->__PVT__ram_inst__DOT__mem[i]}.to_string(' ', '*');
        std::string s2 = std::bitset<16>{top->top->__PVT__ram_inst__DOT__mem[i + 1]}.to_string(' ', '*');
        std::cout << std::setfill('0') << std::setw(16) << s1 <<
                     std::setfill('0') << std::setw(16) << s2 << "||" <<
                     std::setfill('0') << std::setw(4)  << std::hex << top->top->__PVT__ram_inst__DOT__mem[i] << '|' <<
                     std::setfill('0') << std::setw(4)  << std::hex << top->top->__PVT__ram_inst__DOT__mem[i + 1] << std::endl;
    }
    std::cout << "Total runtime: " << t << std::dec << " cycles\n";
    std::cout << "Total runtime: " << t/50e6 << std::dec << " seconds (Assuming default 50MHz clock)\n";
    std::cout << "Elapsed: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << " seconds\n";

    // Final model cleanup
    top->final();

    // Coverage analysis (calling write only after the test is known to pass)
#if VM_COVERAGE
    Verilated::mkdir("logs");
    contextp->coveragep()->write("logs/coverage.dat");
#endif

    // Return good completion status
    // Don't use exit() or destructor won't get called
    return 0;
}
