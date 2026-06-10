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
    id 2054 \
    name z_u_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_u_1 \
    op interface \
    ports { z_u_1_address0 { O 5 vector } z_u_1_ce0 { O 1 bit } z_u_1_we0 { O 1 bit } z_u_1_d0 { O 32 vector } z_u_1_address1 { O 5 vector } z_u_1_ce1 { O 1 bit } z_u_1_q1 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_u_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2055 \
    name z_u \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_u \
    op interface \
    ports { z_u_address0 { O 5 vector } z_u_ce0 { O 1 bit } z_u_we0 { O 1 bit } z_u_d0 { O 32 vector } z_u_address1 { O 5 vector } z_u_ce1 { O 1 bit } z_u_q1 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_u'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2056 \
    name y_u_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_u_1 \
    op interface \
    ports { y_u_1_address0 { O 5 vector } y_u_1_ce0 { O 1 bit } y_u_1_we0 { O 1 bit } y_u_1_d0 { O 32 vector } y_u_1_address1 { O 5 vector } y_u_1_ce1 { O 1 bit } y_u_1_q1 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_u_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2057 \
    name y_u \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_u \
    op interface \
    ports { y_u_address0 { O 5 vector } y_u_ce0 { O 1 bit } y_u_we0 { O 1 bit } y_u_d0 { O 32 vector } y_u_address1 { O 5 vector } y_u_ce1 { O 1 bit } y_u_q1 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_u'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2058 \
    name sol_u \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_u \
    op interface \
    ports { sol_u_address0 { O 5 vector } sol_u_ce0 { O 1 bit } sol_u_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_u'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2059 \
    name sol_u_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_u_1 \
    op interface \
    ports { sol_u_1_address0 { O 5 vector } sol_u_1_ce0 { O 1 bit } sol_u_1_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_u_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2061 \
    name step_data_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_5 \
    op interface \
    ports { step_data_5_address0 { O 5 vector } step_data_5_ce0 { O 1 bit } step_data_5_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_5'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2060 \
    name sext_ln1395 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln1395 \
    op interface \
    ports { sext_ln1395 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2062 \
    name lnorm_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_u1_out \
    op interface \
    ports { lnorm_u1_out { O 32 vector } lnorm_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2063 \
    name znorm_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_u1_out \
    op interface \
    ports { znorm_u1_out { O 32 vector } znorm_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2064 \
    name dual_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_u1_out \
    op interface \
    ports { dual_u1_out { O 32 vector } dual_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2065 \
    name primal_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_u1_out \
    op interface \
    ports { primal_u1_out { O 32 vector } primal_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2066 \
    name lnorm_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_u0_out \
    op interface \
    ports { lnorm_u0_out { O 32 vector } lnorm_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2067 \
    name znorm_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_u0_out \
    op interface \
    ports { znorm_u0_out { O 31 vector } znorm_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2068 \
    name dual_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_u0_out \
    op interface \
    ports { dual_u0_out { O 32 vector } dual_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2069 \
    name primal_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_u0_out \
    op interface \
    ports { primal_u0_out { O 32 vector } primal_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2070 \
    name u_norm_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_u_norm_out \
    op interface \
    ports { u_norm_out { O 32 vector } u_norm_out_ap_vld { O 1 bit } } \
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


