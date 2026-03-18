# ============================================================================
# Vitis HLS TCL Script - RTL Co-Simulation Only
# ============================================================================
# Uses a minimal testbench that only calls the top-level function.
#
# Usage:
#   source /tools/Xilinx/2025.1/Vitis/settings64.sh
#   vitis-run --mode hls --tcl run_cosim.tcl
# ============================================================================

# Project settings
set PROJECT_NAME "pure_pursuit_fpga"
set TOP_FUNCTION "pure_pursuit_fpga"
# Ultra96-V2
set PART_NUMBER "xczu3eg-sbva484-1-i"
# 100 MHz (10ns period)
set CLOCK_PERIOD 10

# Create project
open_project ${PROJECT_NAME}_cosim

# Set top function
set_top ${TOP_FUNCTION}

# Add source files
add_files src/pure_pursuit_fpga.c -cflags "-I./include"
add_files src/fp_math_hls.c -cflags "-I./include"

# Add co-sim testbench (scalar-only calls for RTL co-sim compatibility)
add_files -tb test/test_cosim.c -cflags "-I./include"

# Create solution
open_solution "solution1"
set_part ${PART_NUMBER}
create_clock -period ${CLOCK_PERIOD} -name default

config_interface -m_axi_latency 64
config_compile -pipeline_loops 0

# C Simulation
puts "Running C simulation..."
csim_design

# Synthesis
puts "Running synthesis..."
csynth_design

# RTL Co-Simulation
puts "Running RTL Co-Simulation..."
cosim_design

close_project

puts "============================================"
puts "RTL Co-Simulation complete!"
puts "============================================"
