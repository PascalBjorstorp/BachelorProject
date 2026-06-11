# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_26s_20ns_46_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_26s_22ns_48_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_26s_25s_51_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_25ns_11ns_35_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_7_2_26_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {onehotencoding_realdef}
}


set name mpc_fpga_top_opencl_mul_26s_26s_40_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_26s_26s_40_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_mpc_compute_hls_z_x_RAM_2P_LUTRAM_1R1W BINDTYPE {storage} TYPE {ram_2p} IMPL {lutram} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_mpc_compute_hls_z_u_RAM_AUTO_1R1W BINDTYPE {storage} TYPE {ram} IMPL {auto} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_mpc_compute_hls_sol_u_RAM_AUTO_1R1W BINDTYPE {storage} TYPE {ram} IMPL {auto} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_mpc_compute_hls_K_RAM_AUTO_1R1W BINDTYPE {storage} TYPE {ram} IMPL {auto} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_mpc_compute_hls_kk_RAM_AUTO_1R1W BINDTYPE {storage} TYPE {ram} IMPL {auto} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_mpc_compute_hls_step_data_0_0_RAM_2P_BRAM_1R1W BINDTYPE {storage} TYPE {ram_2p} IMPL {bram} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_mpc_compute_hls_step_data_3_RAM_2P_BRAM_1R1W BINDTYPE {storage} TYPE {ram_2p} IMPL {bram} LATENCY 2 ALLOW_PRAGMA 1
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
    id 2091 \
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
    id 2092 \
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
    id 2093 \
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
    id 2094 \
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
    id 2095 \
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
    id 2096 \
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
    id 2097 \
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
    id 2098 \
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


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2105 \
    name p_anonymous_namespace_g_core_state_admm_z_x_0 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_0 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_0_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_0_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_0_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_0_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_0_q0 { I 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_0_address1 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_0_ce1 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_0_q1 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2106 \
    name p_anonymous_namespace_g_core_state_admm_z_x_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_1 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_1_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_1_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_1_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_1_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2107 \
    name p_anonymous_namespace_g_core_state_admm_z_x_2 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_2 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_2_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_2_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_2_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_2_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_2_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2108 \
    name p_anonymous_namespace_g_core_state_admm_z_x_3 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_3 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_3_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_3_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_3_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_3_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_3_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2109 \
    name p_anonymous_namespace_g_core_state_admm_z_x_4 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_4 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_4_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_4_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_4_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_4_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_4_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2110 \
    name p_anonymous_namespace_g_core_state_admm_z_x_5 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_5 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_5_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_5_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_5_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_5_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_5_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2111 \
    name p_anonymous_namespace_g_core_state_admm_z_x_6 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_6 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_6_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_6_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_6_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_6_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_6_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2112 \
    name p_anonymous_namespace_g_core_state_admm_z_x_7 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_x_7 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_7_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_7_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_7_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_7_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_x_7_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2113 \
    name p_anonymous_namespace_g_core_state_admm_y_x_0 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_0 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_0_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_0_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_0_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_0_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_0_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2114 \
    name p_anonymous_namespace_g_core_state_admm_y_x_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_1 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_1_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_1_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_1_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_1_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2115 \
    name p_anonymous_namespace_g_core_state_admm_y_x_2 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_2 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_2_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_2_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_2_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_2_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_2_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2116 \
    name p_anonymous_namespace_g_core_state_admm_y_x_3 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_3 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_3_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_3_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_3_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_3_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_3_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2117 \
    name p_anonymous_namespace_g_core_state_admm_y_x_4 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_4 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_4_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_4_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_4_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_4_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_4_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2118 \
    name p_anonymous_namespace_g_core_state_admm_y_x_5 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_5 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_5_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_5_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_5_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_5_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_5_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2119 \
    name p_anonymous_namespace_g_core_state_admm_y_x_6 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_6 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_6_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_6_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_6_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_6_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_6_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2120 \
    name p_anonymous_namespace_g_core_state_admm_y_x_7 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_x_7 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_7_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_7_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_7_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_7_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_x_7_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2121 \
    name p_anonymous_namespace_g_core_state_admm_z_u_0 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_u_0 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_u_0_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_u_0_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_u_0_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_u_0_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_u_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2122 \
    name p_anonymous_namespace_g_core_state_admm_z_u_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_z_u_1 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_u_1_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_u_1_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_u_1_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_z_u_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_u_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2123 \
    name p_anonymous_namespace_g_core_state_admm_y_u_0 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_u_0 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_u_0_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_u_0_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_u_0_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_u_0_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_u_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2124 \
    name p_anonymous_namespace_g_core_state_admm_y_u_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename p_anonymous_namespace_g_core_state_admm_y_u_1 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_u_1_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_u_1_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_u_1_d0 { O 26 vector } p_anonymous_namespace_g_core_state_admm_y_u_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_u_1'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2086 \
    name state_ey \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_ey \
    op interface \
    ports { state_ey { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2087 \
    name state_epsi \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_epsi \
    op interface \
    ports { state_epsi { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2088 \
    name state_vx \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_vx \
    op interface \
    ports { state_vx { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2089 \
    name state_vy \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_vy \
    op interface \
    ports { state_vy { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2090 \
    name state_omega \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_omega \
    op interface \
    ports { state_omega { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2099 \
    name p_anonymous_namespace_g_core_state_persist_actual_steering \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_actual_steering \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_actual_steering { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2100 \
    name p_anonymous_namespace_g_core_state_persist_prev_steer_rate \
    type other \
    dir IO \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_steer_rate \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_steer_rate_i { I 26 vector } p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o { O 26 vector } p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2101 \
    name p_anonymous_namespace_g_core_state_persist_prev_accel \
    type other \
    dir IO \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_accel \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_accel_i { I 26 vector } p_anonymous_namespace_g_core_state_persist_prev_accel_o { O 26 vector } p_anonymous_namespace_g_core_state_persist_prev_accel_o_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2102 \
    name p_anonymous_namespace_g_core_state_persist_prev_curvature \
    type other \
    dir IO \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_curvature \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_curvature_i { I 26 vector } p_anonymous_namespace_g_core_state_persist_prev_curvature_o { O 26 vector } p_anonymous_namespace_g_core_state_persist_prev_curvature_o_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2103 \
    name p_anonymous_namespace_g_core_state_persist_prev_model_signature \
    type other \
    dir IO \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_model_signature \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_model_signature_i { I 1 vector } p_anonymous_namespace_g_core_state_persist_prev_model_signature_o { O 1 vector } p_anonymous_namespace_g_core_state_persist_prev_model_signature_o_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2104 \
    name p_anonymous_namespace_g_core_state_admm_initialized \
    type other \
    dir IO \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_admm_initialized \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_initialized_i { I 1 vector } p_anonymous_namespace_g_core_state_admm_initialized_o { O 1 vector } p_anonymous_namespace_g_core_state_admm_initialized_o_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2125 \
    name p_anonymous_namespace_g_core_state_admm_rho \
    type other \
    dir IO \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_admm_rho \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_rho_i { I 26 vector } p_anonymous_namespace_g_core_state_admm_rho_o { O 26 vector } p_anonymous_namespace_g_core_state_admm_rho_o_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2126 \
    name p_anonymous_namespace_g_core_state_admm_rho_u \
    type other \
    dir IO \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_admm_rho_u \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_rho_u_i { I 26 vector } p_anonymous_namespace_g_core_state_admm_rho_u_o { O 26 vector } p_anonymous_namespace_g_core_state_admm_rho_u_o_ap_vld { O 1 bit } } \
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

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id -2 \
    name ap_return \
    type ap_return \
    reset_level 1 \
    sync_rst true \
    corename ap_return \
    op interface \
    ports { ap_return { O 1 vector } } \
} "
}


# Adapter definition:
set PortName ap_clk
set DataWd 1 
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc cg_default_interface_gen_clock] == "cg_default_interface_gen_clock"} {
eval "cg_default_interface_gen_clock { \
    id -3 \
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
    id -4 \
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


