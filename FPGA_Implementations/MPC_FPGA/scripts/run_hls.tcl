#===========================================================================
# run_hls.tcl — Vitis HLS Synthesis Script for MPC FPGA IP
#
# Target: Xilinx Zynq UltraScale+ ZU3EG (Ultra96-V2)
# Part:   xczu3eg-sbva484-1-e
# Clock:  100 MHz (10 ns period)
#
# Usage:
#   source /tools/Xilinx/2025.1/Vitis/settings64.sh
#   vitis-run --mode hls --tcl scripts/run_hls.tcl
#
# Optional run mode with Vitis 2025.1:
#   HLS_RUN_MODE=csim   vitis-run --mode hls --tcl scripts/run_hls.tcl
#   HLS_RUN_MODE=synth  vitis-run --mode hls --tcl scripts/run_hls.tcl
#   HLS_RUN_MODE=cosim  vitis-run --mode hls --tcl scripts/run_hls.tcl
#   HLS_RUN_MODE=export vitis-run --mode hls --tcl scripts/run_hls.tcl
#   HLS_RUN_MODE=all    vitis-run --mode hls --tcl scripts/run_hls.tcl
#===========================================================================

# Parse optional argument for selective execution
# Valid modes: csim, synth, cosim, export, all
set run_mode "synth"

# Prefer environment override for vitis-run workflows.
if {[info exists ::env(HLS_RUN_MODE)]} {
    set env_mode $::env(HLS_RUN_MODE)
    if {$env_mode eq "csim" || $env_mode eq "synth" || $env_mode eq "cosim" || $env_mode eq "export" || $env_mode eq "all"} {
        set run_mode $env_mode
    }
}

# Also allow argv override when arguments are provided.
if {$argc > 0} {
    set arg0 [lindex $argv 0]
    if {$arg0 eq "csim" || $arg0 eq "synth" || $arg0 eq "cosim" || $arg0 eq "export" || $arg0 eq "all"} {
        set run_mode $arg0
    }
}

# Project configuration
set PROJECT_NAME    "mpc_fpga_hls"
set SOLUTION_NAME   "ultra96v2"
set TOP_FUNCTION    "mpc_fpga_top"
set PART            "xczu3eg-sbva484-1-e"
set CLOCK_PERIOD    "10"
set CLOCK_UNCERT    "12.5%"

# Paths (relative to script location, run from project root)
set SRC_DIR     "src"
set INC_DIR     "include"
set TB_DIR      "testbench"

# Extra C flags (can be set externally via: set ::env(HLS_EXTRA_CFLAGS) "-DMPC_HLS_MUL_LIMIT=10")
set EXTRA_CFLAGS ""
if {[info exists ::env(HLS_EXTRA_CFLAGS)]} {
    set EXTRA_CFLAGS $::env(HLS_EXTRA_CFLAGS)
    puts "INFO: Extra CFLAGS: $EXTRA_CFLAGS"
}

#===========================================================================
# Create or open project
#===========================================================================
if {[file exists $PROJECT_NAME]} {
    open_project $PROJECT_NAME
} else {
    open_project -reset $PROJECT_NAME
}

#===========================================================================
# Add source files
#===========================================================================
set COMMON_CFLAGS "-I${INC_DIR} -DMPC_HLS_TARGET ${EXTRA_CFLAGS}"
add_files ${SRC_DIR}/fp_math_hls.c       -cflags "$COMMON_CFLAGS"
add_files ${SRC_DIR}/vehicle_model_hls.c  -cflags "$COMMON_CFLAGS"
add_files ${SRC_DIR}/riccati_solver_hls.c -cflags "$COMMON_CFLAGS"
add_files ${SRC_DIR}/mpc_riccati_hls.c    -cflags "$COMMON_CFLAGS"
add_files ${SRC_DIR}/mpc_fpga_top.c       -cflags "$COMMON_CFLAGS"

# Set top function
set_top $TOP_FUNCTION

# Add testbench
add_files -tb ${TB_DIR}/tb_mpc_fpga.c -cflags "-I${INC_DIR} -lm"

#===========================================================================
# Create or open solution
#===========================================================================
if {[file exists ${PROJECT_NAME}/${SOLUTION_NAME}]} {
    open_solution $SOLUTION_NAME
} else {
    open_solution -reset $SOLUTION_NAME
}

# Target device and clock
set_part $PART
create_clock -period $CLOCK_PERIOD -name default
set_clock_uncertainty $CLOCK_UNCERT

# Optimization directives
# pipeline_loops: 0 = disable auto-pipelining — required to fit ZU3EG (340/360 DSP)
# pipeline_loops: 6 = auto-pipeline innermost loops (original, DOES NOT FIT: 585 DSP)
if {[info exists ::env(HLS_PIPELINE_LOOPS)]} {
    config_compile -pipeline_loops $::env(HLS_PIPELINE_LOOPS)
    puts "INFO: pipeline_loops = $::env(HLS_PIPELINE_LOOPS)"
} else {
    config_compile -pipeline_loops 0
    puts "INFO: pipeline_loops = 0 (default, fits ZU3EG)"
}
# Note: config_schedule -effort and config_bind -effort deprecated in Vitis 2025.1+

#===========================================================================
# Execution based on run_mode
#===========================================================================

if {$run_mode eq "all" || $run_mode eq "csim"} {
    puts "========================================"
    puts "  Running C Simulation"
    puts "========================================"
    csim_design -clean
}

if {$run_mode eq "all" || $run_mode eq "synth"} {
    puts "========================================"
    puts "  Running C Synthesis"
    puts "========================================"
    csynth_design
}

if {$run_mode eq "all" || $run_mode eq "cosim"} {
    puts "========================================"
    puts "  Running C/RTL Co-Simulation"
    puts "========================================"
    cosim_design -trace_level all
}

if {$run_mode eq "all" || $run_mode eq "export"} {
    puts "========================================"
    puts "  Exporting RTL as Vivado IP"
    puts "========================================"
    export_design -format ip_catalog \
        -description "MPC Riccati-ADMM Solver for F1/10th" \
        -vendor "f1tenth" \
        -library "mpc" \
        -version "1.0" \
        -display_name "MPC_FPGA_Solver"
}

puts "========================================"
puts "  Done: $run_mode"
puts "========================================"

exit
