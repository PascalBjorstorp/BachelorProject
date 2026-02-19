# ============================================================================
# Vitis HLS TCL Script for Pure Pursuit FPGA Synthesis
# ============================================================================
# Target: Xilinx Ultra96-V2 (Zynq UltraScale+ ZU3EG)
#
# Usage:
#   vitis_hls -f run_hls.tcl
#
# Output:
#   - C simulation results
#   - RTL synthesis report
#   - Exported IP for Vivado integration
# ============================================================================

# Project settings
set PROJECT_NAME "pure_pursuit_fpga"
set TOP_FUNCTION "pure_pursuit_fpga"
# Ultra96-V2
set PART_NUMBER "xczu3eg-sbva484-1-i"
# 100 MHz (10ns period)
set CLOCK_PERIOD 10

# Create project
open_project ${PROJECT_NAME}_hls

# Set top function
set_top ${TOP_FUNCTION}

# Add source files
add_files src/pure_pursuit_fpga.c -cflags "-I./include"
add_files src/fp_math_hls.c -cflags "-I./include"

# Add testbench
add_files -tb test/test_cosim.c -cflags "-I./include"

# Create solution
open_solution "solution1"

# Set target device
set_part ${PART_NUMBER}

# Set clock
create_clock -period ${CLOCK_PERIOD} -name default

# Configure solution
config_interface -m_axi_latency 64
config_compile -pipeline_loops 0
config_schedule -effort medium

# ============================================================================
# C Simulation
# ============================================================================
puts "Running C simulation..."
csim_design

# ============================================================================
# Synthesis
# ============================================================================
puts "Running synthesis..."
csynth_design

# ============================================================================
# RTL Co-Simulation (optional, takes longer)
# ============================================================================
# To verify RTL matches C, use run_cosim.tcl instead
# cosim_design -trace_level all

# ============================================================================
# Export IP
# ============================================================================
puts "Exporting IP..."
export_design -format ip_catalog \
    -description "Pure Pursuit Path Tracking for F1Tenth" \
    -vendor "f1tenth" \
    -library "control" \
    -version "1.0" \
    -display_name "Pure Pursuit FPGA"

# Close project
close_project

puts "============================================"
puts "HLS synthesis complete!"
puts "IP exported to: ${PROJECT_NAME}_hls/solution1/impl/ip"
puts "============================================"
