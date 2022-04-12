######################################################################
#
# DESCRIPTION: Verilator Example: Small Makefile
#
# This calls the object directory makefile.  That allows the objects to
# be placed in the "current directory" which simplifies the Makefile.
#
# This file ONLY is placed under the Creative Commons Public Domain, for
# any use, without warranty, 2020 by Wilson Snyder.
# SPDX-License-Identifier: CC0-1.0
#
######################################################################
# Check for sanity to avoid later confusion

ifneq ($(words $(CURDIR)),1)
 $(error Unsupported: GNU Make cannot build in directories containing spaces, build elsewhere: '$(CURDIR)')
endif

######################################################################

# This is intended to be a minimal example.  Before copying this to start a
# real project, it is better to start with a more complete example,
# e.g. examples/make_tracing_c.

# If $VERILATOR_ROOT isn't in the environment, we assume it is part of a
# package install, and verilator is in your path. Otherwise find the
# binary relative to $VERILATOR_ROOT (such as when inside the git sources).
ifeq ($(VERILATOR_ROOT),)
VERILATOR = verilator
else
export VERILATOR_ROOT
VERILATOR = $(VERILATOR_ROOT)/bin/verilator
endif

V_FILES = top.sv cpu.sv  alu.sv  ram_sim.v  rom_sim.v perf_counter.sv

fast:
	$(VERILATOR) -cc --exe --build -j $(V_FILES) sim_main.cpp
	@echo "-- RUN ---------------------"
	obj_dir/Vtop
	@echo "-- DONE --------------------"

memlog:
	$(VERILATOR) -cc --exe --build -j $(V_FILES) sim_main.cpp
	@echo "-- RUN ---------------------"
	obj_dir/Vtop +memlog
	@echo "-- DONE --------------------"

disasm:
	$(VERILATOR) -cc --exe --build -j $(V_FILES) sim_main.cpp
	@echo "-- RUN ---------------------"
	obj_dir/Vtop +memlog +disasm
	@echo "-- DONE --------------------"

rand:
	$(VERILATOR) -cc --exe --build -j $(V_FILES) sim_main.cpp
	@echo "-- RUN ---------------------"
	../rand/rand_hackcpu.py -l 128 -w 1 -r 1024
	obj_dir/Vtop +rom=rom_rand_plain16.hex +memlog
	@echo "-- DONE --------------------"

trace:
	# removing --trace causes issues with test not ending as it seems to start fetching instructions
	# from supposedly-blank ROM areas around PC=400 and above.
	$(VERILATOR) -cc --exe --trace --build -j $(V_FILES) sim_main.cpp
	@echo "-- RUN ---------------------"
	obj_dir/Vtop +trace
	@echo "-- DONE --------------------"

######################################################################

maintainer-copy::
clean mostlyclean distclean maintainer-clean::
	-rm -rf obj_dir *.log *.dmp *.vpd core
