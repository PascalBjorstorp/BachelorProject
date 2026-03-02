# ============================================================================
# Vitis HLS TCL Script for MPC FPGA Synthesis
# ============================================================================
# Target: Xilinx Ultra96-V2 (Zynq UltraScale+ ZU3EG)
#
# Usage:
#   cd MPC_FPGA
#   vitis_hls -f run_hls.tcl
#
# Output:
#   - C simulation results
#   - RTL synthesis report (latency, resource utilization)
#   - Exported IP for Vivado integration
#
# Resources (estimated):
#   - DSP48E2: 120-200 of 360 (33-56%)
#   - BRAM:    ~48 KB for trajectory + ~15 KB for QP arrays
#   - LUT:     ~20K-35K of 70K
#   - Clock:   100 MHz (10 ns)
# ============================================================================

# Project settings
set PROJECT_NAME "mpc_fpga"
set TOP_FUNCTION "mpc_fpga"
# Ultra96-V2
set PART_NUMBER "xczu3eg-sbva484-1-i"
# 100 MHz (10 ns period)
set CLOCK_PERIOD 10

# Change to this script's directory so relative paths work
set SCRIPT_DIR [file dirname [file normalize [info script]]]
cd $SCRIPT_DIR

# Create project
open_project ${PROJECT_NAME}_hls

# Set top function
set_top ${TOP_FUNCTION}

# Add source files
# The top-level file #includes all MPC core sources for single-TU synthesis
add_files hls/src/mpc_fpga_top.c -cflags "-I./include -I./hls/include -DMPC_HLS_TARGET"

# Add testbench
add_files -tb hls/test/test_mpc_cosim.c -cflags "-I./include -I./hls/include -DMPC_HLS_TARGET"

# Create solution
open_solution "solution1"

# Set target device
set_part ${PART_NUMBER}

# Set clock
create_clock -period ${CLOCK_PERIOD} -name default

# Configure solution
config_interface -m_axi_latency 64
config_compile -pipeline_loops 0

# ============================================================================
# Aggressive HLS Optimization Directives (supplement source pragmas)
# ============================================================================

# LUT→DSP trade: Source-level BIND_OP pragmas in fp_mul(), fp_mat_vec_mul(),
# and fp_symmetric_mat_vec_mul() force fixed-point multiplications to DSP48E2.

# Pipeline the top-level function for throughput
set_directive_pipeline -II 1 "build_reference_from_bram/BUILD_REF_LOOP"

# QP solver arrays: sequential access pattern (one element per II=1 cycle)
# → default LUTRAM is sufficient. Complete partition wastes LUT on 42:1 MUX.
# Removed: gradient, next_variables, hessian_times_variables, inv_row_sum

# Accumulator in symmetric mat-vec: scattered 2×2 block writes need
# bank-level parallelism. Cyclic factor 2 gives even/odd banks for
# the paired (ri,ri+1)/(rj,rj+1) access pattern, saving ~half the MUX
# vs complete while keeping reasonable pipeline II.
set_directive_array_partition -type cyclic -factor 2 "fp_symmetric_mat_vec_mul" accum

# Partition Phi and PhiQ arrays for pipelined Hessian k-loop.
# The k-loop reads PhiQ[mi][0..1][0..4] and Phi[mj][0..4][0..1]
# simultaneously (20 reads per cycle). Complete partition on the inner
# dimensions creates separate banks so all reads can proceed in parallel.
# Storage: 20 entries per bank × 32-bit → uses LUTRAM (~50 LUTs total).
set_directive_array_partition -type complete -dim 2 "build_qp_from_prediction" Phi
set_directive_array_partition -type complete -dim 3 "build_qp_from_prediction" Phi
set_directive_array_partition -type complete -dim 2 "build_qp_from_prediction" PhiQ
set_directive_array_partition -type complete -dim 3 "build_qp_from_prediction" PhiQ

# Set latency constraints for the QP solver main loop
set_directive_latency -min 1 "mpc_fpga"

# Aggressive optimization: allow HLS to optimize across function boundaries
config_compile -unsafe_math_optimizations

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
# Export IP
# ============================================================================
puts "Exporting IP..."
export_design -format ip_catalog \
    -description "MPC Path Tracking Controller for F1Tenth" \
    -vendor "f1tenth" \
    -library "control" \
    -version "1.0" \
    -display_name "MPC FPGA Controller"

# Close project
close_project

puts "============================================"
puts "HLS synthesis complete!"
puts "IP exported to: ${PROJECT_NAME}_hls/solution1/impl/ip"
puts "============================================"
