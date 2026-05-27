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
    id 2018 \
    name y_x_5 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_x_5 \
    op interface \
    ports { y_x_5_address0 { O 5 vector } y_x_5_ce0 { O 1 bit } y_x_5_q0 { I 32 vector } y_x_5_address1 { O 5 vector } y_x_5_ce1 { O 1 bit } y_x_5_we1 { O 1 bit } y_x_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2019 \
    name y_x \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_x \
    op interface \
    ports { y_x_address0 { O 5 vector } y_x_ce0 { O 1 bit } y_x_q0 { I 32 vector } y_x_address1 { O 5 vector } y_x_ce1 { O 1 bit } y_x_we1 { O 1 bit } y_x_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2020 \
    name z_x_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_7 \
    op interface \
    ports { z_x_7_address1 { O 5 vector } z_x_7_ce1 { O 1 bit } z_x_7_we1 { O 1 bit } z_x_7_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2021 \
    name z_x_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_6 \
    op interface \
    ports { z_x_6_address1 { O 5 vector } z_x_6_ce1 { O 1 bit } z_x_6_we1 { O 1 bit } z_x_6_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2022 \
    name z_x_5 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_x_5 \
    op interface \
    ports { z_x_5_address0 { O 5 vector } z_x_5_ce0 { O 1 bit } z_x_5_q0 { I 32 vector } z_x_5_address1 { O 5 vector } z_x_5_ce1 { O 1 bit } z_x_5_we1 { O 1 bit } z_x_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2023 \
    name z_x_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_4 \
    op interface \
    ports { z_x_4_address1 { O 5 vector } z_x_4_ce1 { O 1 bit } z_x_4_we1 { O 1 bit } z_x_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2024 \
    name z_x_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_3 \
    op interface \
    ports { z_x_3_address1 { O 5 vector } z_x_3_ce1 { O 1 bit } z_x_3_we1 { O 1 bit } z_x_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2025 \
    name z_x_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_2 \
    op interface \
    ports { z_x_2_address1 { O 5 vector } z_x_2_ce1 { O 1 bit } z_x_2_we1 { O 1 bit } z_x_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2026 \
    name z_x_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_1 \
    op interface \
    ports { z_x_1_address1 { O 5 vector } z_x_1_ce1 { O 1 bit } z_x_1_we1 { O 1 bit } z_x_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2027 \
    name z_x \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_x \
    op interface \
    ports { z_x_address0 { O 5 vector } z_x_ce0 { O 1 bit } z_x_q0 { I 32 vector } z_x_address1 { O 5 vector } z_x_ce1 { O 1 bit } z_x_we1 { O 1 bit } z_x_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2028 \
    name sol_x_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_1 \
    op interface \
    ports { sol_x_1_address0 { O 5 vector } sol_x_1_ce0 { O 1 bit } sol_x_1_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2029 \
    name sol_x_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_2 \
    op interface \
    ports { sol_x_2_address0 { O 5 vector } sol_x_2_ce0 { O 1 bit } sol_x_2_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2030 \
    name sol_x_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_3 \
    op interface \
    ports { sol_x_3_address0 { O 5 vector } sol_x_3_ce0 { O 1 bit } sol_x_3_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2031 \
    name sol_x_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_4 \
    op interface \
    ports { sol_x_4_address0 { O 5 vector } sol_x_4_ce0 { O 1 bit } sol_x_4_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2032 \
    name sol_x_6 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_6 \
    op interface \
    ports { sol_x_6_address0 { O 5 vector } sol_x_6_ce0 { O 1 bit } sol_x_6_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2033 \
    name sol_x_7 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_7 \
    op interface \
    ports { sol_x_7_address0 { O 5 vector } sol_x_7_ce0 { O 1 bit } sol_x_7_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2034 \
    name sol_x \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x \
    op interface \
    ports { sol_x_address0 { O 5 vector } sol_x_ce0 { O 1 bit } sol_x_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2035 \
    name sol_x_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_5 \
    op interface \
    ports { sol_x_5_address0 { O 5 vector } sol_x_5_ce0 { O 1 bit } sol_x_5_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2036 \
    name step_data_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_3 \
    op interface \
    ports { step_data_3_address0 { O 5 vector } step_data_3_ce0 { O 1 bit } step_data_3_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2037 \
    name step_data_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_4 \
    op interface \
    ports { step_data_4_address0 { O 5 vector } step_data_4_ce0 { O 1 bit } step_data_4_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_4'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2038 \
    name terminal_wall_x_lb_con_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_terminal_wall_x_lb_con_reload \
    op interface \
    ports { terminal_wall_x_lb_con_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2039 \
    name terminal_wall_x_ub_con_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_terminal_wall_x_ub_con_reload \
    op interface \
    ports { terminal_wall_x_ub_con_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2040 \
    name sext_ln1337 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln1337 \
    op interface \
    ports { sext_ln1337 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2041 \
    name lnorm_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_da_out \
    op interface \
    ports { lnorm_da_out { O 32 vector } lnorm_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2042 \
    name znorm_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_da_out \
    op interface \
    ports { znorm_da_out { O 31 vector } znorm_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2043 \
    name dual_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_da_out \
    op interface \
    ports { dual_da_out { O 32 vector } dual_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2044 \
    name primal_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_da_out \
    op interface \
    ports { primal_da_out { O 32 vector } primal_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2045 \
    name lnorm_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_ey_out \
    op interface \
    ports { lnorm_ey_out { O 32 vector } lnorm_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2046 \
    name znorm_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_ey_out \
    op interface \
    ports { znorm_ey_out { O 32 vector } znorm_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2047 \
    name dual_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_ey_out \
    op interface \
    ports { dual_ey_out { O 32 vector } dual_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2048 \
    name primal_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_ey_out \
    op interface \
    ports { primal_ey_out { O 32 vector } primal_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2049 \
    name x_norm_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_x_norm_out \
    op interface \
    ports { x_norm_out { O 32 vector } x_norm_out_ap_vld { O 1 bit } } \
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


