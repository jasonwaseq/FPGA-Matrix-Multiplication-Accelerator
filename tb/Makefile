# Cocotb simulation Makefile for DVS gesture detector

SIM ?= icarus
TOPLEVEL_LANG ?= verilog
TOPLEVEL = dvs_gesture_detector_top
MODULE = dvs_gesture_testbench

# Verilog source files (relative to project root)
VERILOG_SOURCES += $(PWD)/../rtl/ram_1r1w_sync.sv
VERILOG_SOURCES += $(PWD)/../rtl/fifo_1r1w_sync.sv
VERILOG_SOURCES += $(PWD)/../rtl/dvs_event_buffer.sv
VERILOG_SOURCES += $(PWD)/../rtl/activity_map.sv
VERILOG_SOURCES += $(PWD)/../rtl/matmul_core.sv
VERILOG_SOURCES += $(PWD)/../rtl/matmul_weights.sv
VERILOG_SOURCES += $(PWD)/../rtl/matmul_gesture_classifier.sv
VERILOG_SOURCES += $(PWD)/../rtl/dvs_gesture_detector_top.sv

# Icarus Verilog specific flags
ifeq ($(SIM), icarus)
    COMPILE_ARGS += -g2009
endif

# Verilator specific flags
ifeq ($(SIM), verilator)
    COMPILE_ARGS += --trace --timing
endif

# Include cocotb makefiles
include $(shell cocotb-config --makefiles)/Makefile.sim
