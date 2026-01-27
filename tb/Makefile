# Makefile for cocotb testbench
# DVS Gesture Accelerator with Temporal Voxel Binning

TOPLEVEL_LANG = verilog
VERILOG_SOURCES = $(wildcard ../rtl/*.sv)

# Default: test UART top module (full system)
TOPLEVEL ?= uart_gesture_top
MODULE ?= test_dvs_gesture

SIM = icarus

# Enable SystemVerilog
COMPILE_ARGS += -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim

# Convenience targets
.PHONY: test test_full clean_all

test:
	$(MAKE) TOPLEVEL=uart_gesture_top MODULE=test_dvs_gesture

test_full:
	$(MAKE) clean
	$(MAKE) test

clean_all:
	rm -rf sim_build results.xml __pycache__
