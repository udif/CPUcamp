## Generated SDC file "CPU_GARAGE.out.sdc"

## Copyright (C) 2017  Intel Corporation. All rights reserved.
## Your use of Intel Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Intel Program License 
## Subscription Agreement, the Intel Quartus Prime License Agreement,
## the Intel MegaCore Function License Agreement, or other 
## applicable license agreement, including, without limitation, 
## that your use is for the sole purpose of programming logic 
## devices manufactured by Intel and sold by Intel or its 
## authorized distributors.  Please refer to the applicable 
## agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus Prime"
## VERSION "Version 17.0.0 Build 595 04/25/2017 SJ Lite Edition"

## DATE    "Tue Mar 29 02:02:31 2022"

##
## DEVICE  "10M50DAF484C7G"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {altera_reserved_tck} -period 100.000 -waveform { 0.000 50.000 } [get_ports {altera_reserved_tck}]
create_clock -name {CLK_50} -period 20.000 -waveform { 0.000 10.000 } [get_ports {CLK_50}]


#**************************************************************
# Create Generated Clock
#**************************************************************

# **************************************************************************
# *** Synchronize "multiple_by" value with the one from "definitions.sv" ***
# **************************************************************************
create_generated_clock -name {pll:pll_inst|altpll:altpll_component|pll_altpll:auto_generated|wire_pll1_clk[0]} \
                       -source [get_pins {pll_inst|altpll_component|auto_generated|pll1|inclk[0]}] \
                       -duty_cycle 50/1 -multiply_by 290 -divide_by 100 \
                       -master_clock {CLK_50} [get_pins {pll_inst|altpll_component|auto_generated|pll1|clk[0]}] 


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************



#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************

set_clock_groups -exclusive -group [get_clocks {altera_reserved_tck}] 
# Set clkA and clkB to be mutually exclusive clocks.
# set_clock_groups -exclusive -group {CLK_50} -group {pll:pll_inst|altpll:altpll_component|pll_altpll:auto_generated|wire_pll1_clk[0]}


#**************************************************************
# Set False Path
#**************************************************************



#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

