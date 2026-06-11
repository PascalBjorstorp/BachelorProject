# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_26s_23ns_49_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statbkb BINDTYPE {storage} TYPE {ram} IMPL {auto} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statcud BINDTYPE {storage} TYPE {ram} IMPL {auto} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statrcU BINDTYPE {storage} TYPE {ram} IMPL {auto} LATENCY 2 ALLOW_PRAGMA 1
}


# clear list
if {${::AESL::PGuard_autoexp_gen}} {
    cg_default_interface_gen_dc_begin
    cg_default_interface_gen_bundle_begin
    AESL_LIB_XILADAPTER::native_axis_begin
}

# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2140 \
    name ref_reference_heading_error \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_heading_error \
    op interface \
    ports { ref_reference_heading_error_address0 { O 5 vector } ref_reference_heading_error_ce0 { O 1 bit } ref_reference_heading_error_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_heading_error'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2141 \
    name ref_reference_lateral_error \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_lateral_error \
    op interface \
    ports { ref_reference_lateral_error_address0 { O 5 vector } ref_reference_lateral_error_ce0 { O 1 bit } ref_reference_lateral_error_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_lateral_error'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2142 \
    name ref_reference_velocity \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_velocity \
    op interface \
    ports { ref_reference_velocity_address0 { O 5 vector } ref_reference_velocity_ce0 { O 1 bit } ref_reference_velocity_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_velocity'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2143 \
    name ref_reference_lateral_velocity \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_lateral_velocity \
    op interface \
    ports { ref_reference_lateral_velocity_address0 { O 5 vector } ref_reference_lateral_velocity_ce0 { O 1 bit } ref_reference_lateral_velocity_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_lateral_velocity'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2144 \
    name ref_reference_yaw_rate \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_yaw_rate \
    op interface \
    ports { ref_reference_yaw_rate_address0 { O 5 vector } ref_reference_yaw_rate_ce0 { O 1 bit } ref_reference_yaw_rate_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_yaw_rate'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2145 \
    name ref_path_curvature \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_path_curvature \
    op interface \
    ports { ref_path_curvature_address0 { O 5 vector } ref_path_curvature_ce0 { O 1 bit } ref_path_curvature_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_path_curvature'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2146 \
    name ref_left_wall_bound \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_left_wall_bound \
    op interface \
    ports { ref_left_wall_bound_address0 { O 5 vector } ref_left_wall_bound_ce0 { O 1 bit } ref_left_wall_bound_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_left_wall_bound'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2147 \
    name ref_right_wall_bound \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_right_wall_bound \
    op interface \
    ports { ref_right_wall_bound_address0 { O 5 vector } ref_right_wall_bound_ce0 { O 1 bit } ref_right_wall_bound_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_right_wall_bound'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2132 \
    name ey \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_ey \
    op interface \
    ports { ey { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2133 \
    name epsi \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_epsi \
    op interface \
    ports { epsi { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2134 \
    name vx \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_vx \
    op interface \
    ports { vx { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2135 \
    name vy \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_vy \
    op interface \
    ports { vy { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2136 \
    name omega \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_omega \
    op interface \
    ports { omega { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2137 \
    name steering \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_steering \
    op interface \
    ports { steering { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2138 \
    name prev_accel \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_prev_accel \
    op interface \
    ports { prev_accel { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2139 \
    name control_flags \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_control_flags \
    op interface \
    ports { control_flags { I 3 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2148 \
    name out_steering \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_out_steering \
    op interface \
    ports { out_steering { O 26 vector } out_steering_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2149 \
    name out_accel \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_out_accel \
    op interface \
    ports { out_accel { O 26 vector } out_accel_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2150 \
    name out_status \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_out_status \
    op interface \
    ports { out_status { O 1 vector } out_status_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2151 \
    name out_iters \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_out_iters \
    op interface \
    ports { out_iters { O 32 vector } out_iters_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2152 \
    name out_steering_arg_index \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_out_steering_arg_index \
    op interface \
    ports { out_steering_arg_index { I 11 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2153 \
    name out_accel_arg_index \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_out_accel_arg_index \
    op interface \
    ports { out_accel_arg_index { I 11 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2154 \
    name out_status_arg_index \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_out_status_arg_index \
    op interface \
    ports { out_status_arg_index { I 11 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2155 \
    name out_iters_arg_index \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_out_iters_arg_index \
    op interface \
    ports { out_iters_arg_index { I 11 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id -1 \
    name ap_ctrl \
    type ap_ctrl \
    reset_level 1 \
    sync_rst true \
    corename ap_ctrl \
    op interface \
    ports { ap_start { I 1 bit } ap_ready { O 1 bit } ap_done { O 1 bit } ap_idle { O 1 bit } } \
} "
}


# Adapter definition:
set PortName ap_clk
set DataWd 1 
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc cg_default_interface_gen_clock] == "cg_default_interface_gen_clock"} {
eval "cg_default_interface_gen_clock { \
    id -2 \
    name ${PortName} \
    reset_level 1 \
    sync_rst true \
    corename apif_ap_clk \
    data_wd ${DataWd} \
    op interface \
}"
} else {
puts "@W \[IMPL-113\] Cannot find bus interface model in the library. Ignored generation of bus interface for '${PortName}'"
}
}


# Adapter definition:
set PortName ap_rst
set DataWd 1 
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc cg_default_interface_gen_reset] == "cg_default_interface_gen_reset"} {
eval "cg_default_interface_gen_reset { \
    id -3 \
    name ${PortName} \
    reset_level 1 \
    sync_rst true \
    corename apif_ap_rst \
    data_wd ${DataWd} \
    op interface \
}"
} else {
puts "@W \[IMPL-114\] Cannot find bus interface model in the library. Ignored generation of bus interface for '${PortName}'"
}
}



# merge
if {${::AESL::PGuard_autoexp_gen}} {
    cg_default_interface_gen_dc_end
    cg_default_interface_gen_bundle_end
    AESL_LIB_XILADAPTER::native_axis_end
}


