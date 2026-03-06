#===========================================================================
# run_hls.tcl — Vitis HLS Synthesis Script for MPC FPGA IP
#
# Target: Xilinx Zynq UltraScale+ ZU3EG (Ultra96-V2)
# Part:   xczu3eg-sbva484-1-e
# Clock:  100 MHz (10 ns period)
#
# Usage:
#   vitis_hls -f scripts/run_hls.tcl
#
# Or step-by-step:
#   vitis_hls -f scripts/run_hls.tcl -tclargs csim
#   vitis_hls -f scripts/run_hls.tcl -tclargs synth
#   vitis_hls -f scripts/run_hls.tcl -tclargs cosim
#   vitis_hls -f scripts/run_hls.tcl -tclargs export
#===========================================================================

# Parse optional argument for selective execution
set run_mode "all"
if {$argc > 0} {
    set run_mode [lindex $argv 0]
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
add_files ${SRC_DIR}/fp_math_hls.c       -cflags "-I${INC_DIR} -DMPC_HLS_TARGET"
add_files ${SRC_DIR}/vehicle_model_hls.c  -cflags "-I${INC_DIR} -DMPC_HLS_TARGET"
add_files ${SRC_DIR}/riccati_solver_hls.c -cflags "-I${INC_DIR} -DMPC_HLS_TARGET"
add_files ${SRC_DIR}/mpc_riccati_hls.c    -cflags "-I${INC_DIR} -DMPC_HLS_TARGET"
add_files ${SRC_DIR}/mpc_fpga_top.c       -cflags "-I${INC_DIR} -DMPC_HLS_TARGET"

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
config_compile -pipeline_loops 6
config_schedule -effort high
config_bind -effort high

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
