# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_26s_26s_39_3_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 2 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_13_5_25_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {onehotencoding_realdef}
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_11_4_39_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {onehotencoding_realdef}
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
    id 1637 \
    name x_out_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_0 \
    op interface \
    ports { x_out_0_address1 { O 5 vector } x_out_0_ce1 { O 1 bit } x_out_0_we1 { O 1 bit } x_out_0_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1639 \
    name x_out_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_1 \
    op interface \
    ports { x_out_1_address1 { O 5 vector } x_out_1_ce1 { O 1 bit } x_out_1_we1 { O 1 bit } x_out_1_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1640 \
    name x_out_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_2 \
    op interface \
    ports { x_out_2_address1 { O 5 vector } x_out_2_ce1 { O 1 bit } x_out_2_we1 { O 1 bit } x_out_2_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1641 \
    name x_out_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_3 \
    op interface \
    ports { x_out_3_address1 { O 5 vector } x_out_3_ce1 { O 1 bit } x_out_3_we1 { O 1 bit } x_out_3_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1642 \
    name x_out_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_4 \
    op interface \
    ports { x_out_4_address1 { O 5 vector } x_out_4_ce1 { O 1 bit } x_out_4_we1 { O 1 bit } x_out_4_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1643 \
    name x_out_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_5 \
    op interface \
    ports { x_out_5_address1 { O 5 vector } x_out_5_ce1 { O 1 bit } x_out_5_we1 { O 1 bit } x_out_5_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1645 \
    name step_data_0_0 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_0_0 \
    op interface \
    ports { step_data_0_0_address0 { O 7 vector } step_data_0_0_ce0 { O 1 bit } step_data_0_0_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1646 \
    name step_data_0_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_0_1 \
    op interface \
    ports { step_data_0_1_address0 { O 7 vector } step_data_0_1_ce0 { O 1 bit } step_data_0_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1647 \
    name step_data_0_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_0_2 \
    op interface \
    ports { step_data_0_2_address0 { O 7 vector } step_data_0_2_ce0 { O 1 bit } step_data_0_2_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1648 \
    name step_data_0_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_0_3 \
    op interface \
    ports { step_data_0_3_address0 { O 7 vector } step_data_0_3_ce0 { O 1 bit } step_data_0_3_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1649 \
    name step_data_0_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_0_4 \
    op interface \
    ports { step_data_0_4_address0 { O 7 vector } step_data_0_4_ce0 { O 1 bit } step_data_0_4_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1650 \
    name step_data_0_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_0_5 \
    op interface \
    ports { step_data_0_5_address0 { O 7 vector } step_data_0_5_ce0 { O 1 bit } step_data_0_5_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_5'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1638 \
    name zext_ln428 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_zext_ln428 \
    op interface \
    ports { zext_ln428 { I 5 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1644 \
    name mul_ln409_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_mul_ln409_1 \
    op interface \
    ports { mul_ln409_1 { I 7 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1651 \
    name empty_564 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_564 \
    op interface \
    ports { empty_564 { I 25 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1652 \
    name empty_565 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_565 \
    op interface \
    ports { empty_565 { I 25 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1653 \
    name empty_566 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_566 \
    op interface \
    ports { empty_566 { I 25 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1654 \
    name empty_567 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_567 \
    op interface \
    ports { empty_567 { I 25 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1655 \
    name empty_568 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_568 \
    op interface \
    ports { empty_568 { I 25 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1656 \
    name empty \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty \
    op interface \
    ports { empty { I 25 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1657 \
    name sext_ln156 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156 \
    op interface \
    ports { sext_ln156 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1658 \
    name sext_ln156_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_1 \
    op interface \
    ports { sext_ln156_1 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1659 \
    name sext_ln156_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_2 \
    op interface \
    ports { sext_ln156_2 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1660 \
    name sext_ln156_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_3 \
    op interface \
    ports { sext_ln156_3 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1661 \
    name sext_ln156_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_4 \
    op interface \
    ports { sext_ln156_4 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1662 \
    name sext_ln422 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln422 \
    op interface \
    ports { sext_ln422 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1663 \
    name sext_ln156_11 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_11 \
    op interface \
    ports { sext_ln156_11 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1664 \
    name sext_ln425 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln425 \
    op interface \
    ports { sext_ln425 { I 25 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1665 \
    name sext_ln156_12 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_12 \
    op interface \
    ports { sext_ln156_12 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1666 \
    name sext_ln156_13 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_13 \
    op interface \
    ports { sext_ln156_13 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1667 \
    name sext_ln156_14 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_14 \
    op interface \
    ports { sext_ln156_14 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1668 \
    name sext_ln436 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln436 \
    op interface \
    ports { sext_ln436 { I 25 vector } } \
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


