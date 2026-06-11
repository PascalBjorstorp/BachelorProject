# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_21s_7ns_28_1_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {auto} LATENCY 0 ALLOW_PRAGMA 1
}


# clear list
if {${::AESL::PGuard_autoexp_gen}} {
    cg_default_interface_gen_dc_begin
    cg_default_interface_gen_bundle_begin
    AESL_LIB_XILADAPTER::native_axis_begin
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 474 \
    name tr_dFyf_dvx_val \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_tr_dFyf_dvx_val \
    op interface \
    ports { tr_dFyf_dvx_val { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 475 \
    name tr_dFyf_dvy_val \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_tr_dFyf_dvy_val \
    op interface \
    ports { tr_dFyf_dvy_val { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 476 \
    name tr_dFyf_dom_val \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_tr_dFyf_dom_val \
    op interface \
    ports { tr_dFyf_dom_val { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 477 \
    name tr_dFyr_dvx_val \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_tr_dFyr_dvx_val \
    op interface \
    ports { tr_dFyr_dvx_val { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 478 \
    name tr_dFyr_dvy_val \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_tr_dFyr_dvy_val \
    op interface \
    ports { tr_dFyr_dvy_val { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 479 \
    name tr_dFyr_dom_val \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_tr_dFyr_dom_val \
    op interface \
    ports { tr_dFyr_dom_val { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 480 \
    name p_0_0_05039 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_0_0_05039 \
    op interface \
    ports { p_0_0_05039 { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 481 \
    name p_0_0_05038 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_p_0_0_05038 \
    op interface \
    ports { p_0_0_05038 { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 482 \
    name conv5_i_i_i_i465 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_conv5_i_i_i_i465 \
    op interface \
    ports { conv5_i_i_i_i465 { I 21 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 483 \
    name conv5_i_i_i_i629 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_conv5_i_i_i_i629 \
    op interface \
    ports { conv5_i_i_i_i629 { I 21 vector } } \
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


