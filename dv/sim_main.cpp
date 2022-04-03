// DESCRIPTION: Verilator: Verilog example module
//
// This file ONLY is placed under the Creative Commons Public Domain, for
// any use, without warranty, 2017 by Wilson Snyder.
// SPDX-License-Identifier: CC0-1.0
//======================================================================

// Include common routines
#include <verilated.h>

// Include model header, generated from Verilating "top.v"
#include "Vtop.h"
#include "Vtop_top.h"
#include <iostream>
#include <iomanip>

int main(int argc, char** argv, char** env) {
    // See a similar example walkthrough in the verilator manpage.

    // This is intended to be a minimal example.  Before copying this to start a
    // real project, it is better to start with a more complete example,
    // e.g. examples/c_tracing.

    // Prevent unused variable warnings
    if (false && argc && argv && env) {}

    // Construct the Verilated model, from Vtop.h generated from Verilating "top.v"
    Vtop* top = new Vtop;

    top->BUTTON = 0;
    // Simulate until $finish
    while (!Verilated::gotFinish()) {
        // Evaluate model
    	top->CLK_50 = 0;
    	top->eval();
        top->CLK_50 = 1;
        top->eval();
        top->BUTTON = 1;
        if (top->top->we) {
            std::cout << "address=" << std::setfill('0') << std::setw(3) << std::hex << top->top->__PVT__cpu_inst__DOT__pc;
            std::cout << " inst="   << std::setfill('0') << std::setw(4) << std::hex << top->top->instruction;
            std::cout << " ram_address=" << std::setfill('0') << std::setw(3) << std::hex << top->top->ram_address;
            std::cout << " write_data=" << std::setw(5) << std::dec << top->top->cpu_out_m;
            std::cout << std::endl;
        }
        if (top->top->inst_address > 159)
            exit(0);
    }

    // Final model cleanup
    top->final();

    // Destroy model
    delete top;

    // Return good completion status
    return 0;
}
