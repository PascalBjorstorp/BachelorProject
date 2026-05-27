# This script segment is generated automatically by AutoPilot

if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_17_3_40_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {compactencoding_dontcare}
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
    id 1031 \
    name P \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P \
    op interface \
    ports { P_address0 { O 3 vector } P_ce0 { O 1 bit } P_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1033 \
    name P_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_1 \
    op interface \
    ports { P_1_address0 { O 3 vector } P_1_ce0 { O 1 bit } P_1_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1035 \
    name P_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_2 \
    op interface \
    ports { P_2_address0 { O 3 vector } P_2_ce0 { O 1 bit } P_2_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1037 \
    name P_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_3 \
    op interface \
    ports { P_3_address0 { O 3 vector } P_3_ce0 { O 1 bit } P_3_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1039 \
    name P_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_4 \
    op interface \
    ports { P_4_address0 { O 3 vector } P_4_ce0 { O 1 bit } P_4_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1041 \
    name P_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_5 \
    op interface \
    ports { P_5_address0 { O 3 vector } P_5_ce0 { O 1 bit } P_5_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_5'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1032 \
    name sext_ln259 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln259 \
    op interface \
    ports { sext_ln259 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1034 \
    name sext_ln259_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln259_1 \
    op interface \
    ports { sext_ln259_1 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1036 \
    name sext_ln259_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln259_2 \
    op interface \
    ports { sext_ln259_2 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1038 \
    name sext_ln259_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln259_3 \
    op interface \
    ports { sext_ln259_3 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1040 \
    name sext_ln766 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln766 \
    op interface \
    ports { sext_ln766 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1042 \
    name sext_ln766_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln766_1 \
    op interface \
    ports { sext_ln766_1 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1043 \
    name p_2_r \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_2_r \
    op interface \
    ports { p_2_r { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1044 \
    name p_3_r \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_3_r \
    op interface \
    ports { p_3_r { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1045 \
    name p_4_r \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_4_r \
    op interface \
    ports { p_4_r { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1046 \
    name p_5_r \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_5_r \
    op interface \
    ports { p_5_r { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1047 \
    name p_6 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_6 \
    op interface \
    ports { p_6 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1048 \
    name p_7 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_7 \
    op interface \
    ports { p_7 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1049 \
    name p_8 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_8 \
    op interface \
    ports { p_8 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1050 \
    name p_9 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_9 \
    op interface \
    ports { p_9 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1051 \
    name p_shift_7_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_7_out \
    op interface \
    ports { p_shift_7_out { O 40 vector } p_shift_7_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1052 \
    name p_shift_6_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_6_out \
    op interface \
    ports { p_shift_6_out { O 40 vector } p_shift_6_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1053 \
    name p_shift_5_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_5_out \
    op interface \
    ports { p_shift_5_out { O 40 vector } p_shift_5_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1054 \
    name p_shift_4_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_4_out \
    op interface \
    ports { p_shift_4_out { O 40 vector } p_shift_4_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1055 \
    name p_shift_3_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_3_out \
    op interface \
    ports { p_shift_3_out { O 40 vector } p_shift_3_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1056 \
    name p_shift_2_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_2_out \
    op interface \
    ports { p_shift_2_out { O 40 vector } p_shift_2_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1057 \
    name p_shift_1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_1_out \
    op interface \
    ports { p_shift_1_out { O 34 vector } p_shift_1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1058 \
    name p_shift_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_shift_out \
    op interface \
    ports { p_shift_out { O 33 vector } p_shift_out_ap_vld { O 1 bit } } \
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


