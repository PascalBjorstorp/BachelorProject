# This script segment is generated automatically by AutoPilot

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
    id 37 \
    name lane_words_0 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_0 \
    op interface \
    ports { lane_words_0_address0 { O 4 vector } lane_words_0_ce0 { O 1 bit } lane_words_0_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 38 \
    name lane_words_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_1 \
    op interface \
    ports { lane_words_1_address0 { O 4 vector } lane_words_1_ce0 { O 1 bit } lane_words_1_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 39 \
    name lane_words_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_2 \
    op interface \
    ports { lane_words_2_address0 { O 4 vector } lane_words_2_ce0 { O 1 bit } lane_words_2_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 40 \
    name lane_words_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_3 \
    op interface \
    ports { lane_words_3_address0 { O 4 vector } lane_words_3_ce0 { O 1 bit } lane_words_3_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 41 \
    name lane_words_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_4 \
    op interface \
    ports { lane_words_4_address0 { O 4 vector } lane_words_4_ce0 { O 1 bit } lane_words_4_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 42 \
    name lane_words_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_5 \
    op interface \
    ports { lane_words_5_address0 { O 4 vector } lane_words_5_ce0 { O 1 bit } lane_words_5_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 43 \
    name lane_words_6 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_6 \
    op interface \
    ports { lane_words_6_address0 { O 4 vector } lane_words_6_ce0 { O 1 bit } lane_words_6_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 44 \
    name lane_words_7 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_7 \
    op interface \
    ports { lane_words_7_address0 { O 4 vector } lane_words_7_ce0 { O 1 bit } lane_words_7_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 45 \
    name lane_words_8 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_8 \
    op interface \
    ports { lane_words_8_address0 { O 4 vector } lane_words_8_ce0 { O 1 bit } lane_words_8_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_8'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 46 \
    name lane_words_9 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_9 \
    op interface \
    ports { lane_words_9_address0 { O 4 vector } lane_words_9_ce0 { O 1 bit } lane_words_9_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_9'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 47 \
    name lane_words_10 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_10 \
    op interface \
    ports { lane_words_10_address0 { O 4 vector } lane_words_10_ce0 { O 1 bit } lane_words_10_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_10'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 48 \
    name lane_words_11 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_11 \
    op interface \
    ports { lane_words_11_address0 { O 4 vector } lane_words_11_ce0 { O 1 bit } lane_words_11_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_11'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 49 \
    name lane_words_12 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_12 \
    op interface \
    ports { lane_words_12_address0 { O 4 vector } lane_words_12_ce0 { O 1 bit } lane_words_12_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_12'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 50 \
    name lane_words_13 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_13 \
    op interface \
    ports { lane_words_13_address0 { O 4 vector } lane_words_13_ce0 { O 1 bit } lane_words_13_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_13'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 51 \
    name lane_words_14 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_14 \
    op interface \
    ports { lane_words_14_address0 { O 4 vector } lane_words_14_ce0 { O 1 bit } lane_words_14_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_14'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 52 \
    name lane_words_15 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename lane_words_15 \
    op interface \
    ports { lane_words_15_address0 { O 4 vector } lane_words_15_ce0 { O 1 bit } lane_words_15_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'lane_words_15'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 53 \
    name out_ref_reference_heading_error \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_reference_heading_error \
    op interface \
    ports { out_ref_reference_heading_error_address0 { O 5 vector } out_ref_reference_heading_error_ce0 { O 1 bit } out_ref_reference_heading_error_we0 { O 1 bit } out_ref_reference_heading_error_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_reference_heading_error'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 54 \
    name out_ref_reference_lateral_error \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_reference_lateral_error \
    op interface \
    ports { out_ref_reference_lateral_error_address0 { O 5 vector } out_ref_reference_lateral_error_ce0 { O 1 bit } out_ref_reference_lateral_error_we0 { O 1 bit } out_ref_reference_lateral_error_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_reference_lateral_error'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 55 \
    name out_ref_reference_velocity \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_reference_velocity \
    op interface \
    ports { out_ref_reference_velocity_address0 { O 5 vector } out_ref_reference_velocity_ce0 { O 1 bit } out_ref_reference_velocity_we0 { O 1 bit } out_ref_reference_velocity_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_reference_velocity'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 56 \
    name out_ref_reference_lateral_velocity \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_reference_lateral_velocity \
    op interface \
    ports { out_ref_reference_lateral_velocity_address0 { O 5 vector } out_ref_reference_lateral_velocity_ce0 { O 1 bit } out_ref_reference_lateral_velocity_we0 { O 1 bit } out_ref_reference_lateral_velocity_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_reference_lateral_velocity'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 57 \
    name out_ref_reference_yaw_rate \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_reference_yaw_rate \
    op interface \
    ports { out_ref_reference_yaw_rate_address0 { O 5 vector } out_ref_reference_yaw_rate_ce0 { O 1 bit } out_ref_reference_yaw_rate_we0 { O 1 bit } out_ref_reference_yaw_rate_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_reference_yaw_rate'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 58 \
    name out_ref_path_curvature \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_path_curvature \
    op interface \
    ports { out_ref_path_curvature_address0 { O 5 vector } out_ref_path_curvature_ce0 { O 1 bit } out_ref_path_curvature_we0 { O 1 bit } out_ref_path_curvature_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_path_curvature'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 59 \
    name out_ref_left_wall_bound \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_left_wall_bound \
    op interface \
    ports { out_ref_left_wall_bound_address0 { O 5 vector } out_ref_left_wall_bound_ce0 { O 1 bit } out_ref_left_wall_bound_we0 { O 1 bit } out_ref_left_wall_bound_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_left_wall_bound'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 60 \
    name out_ref_right_wall_bound \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename out_ref_right_wall_bound \
    op interface \
    ports { out_ref_right_wall_bound_address0 { O 5 vector } out_ref_right_wall_bound_ce0 { O 1 bit } out_ref_right_wall_bound_we0 { O 1 bit } out_ref_right_wall_bound_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'out_ref_right_wall_bound'"
}
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


# flow_control definition:
set InstName mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init_U
set CompName mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init
set name flow_control_loop_pipe_sequential_init
if {${::AESL::PGuard_autocg_gen} && ${::AESL::PGuard_autocg_ipmgen}} {
if {[info proc ::AESL_LIB_VIRTEX::xil_gen_UPC_flow_control] == "::AESL_LIB_VIRTEX::xil_gen_UPC_flow_control"} {
eval "::AESL_LIB_VIRTEX::xil_gen_UPC_flow_control { \
    name ${name} \
    prefix mpc_fpga_top_opencl_ \
}"
} else {
puts "@W \[IMPL-107\] Cannot find ::AESL_LIB_VIRTEX::xil_gen_UPC_flow_control, check your platform lib"
}
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $CompName BINDTYPE interface TYPE internal_upc_flow_control INSTNAME $InstName
}


