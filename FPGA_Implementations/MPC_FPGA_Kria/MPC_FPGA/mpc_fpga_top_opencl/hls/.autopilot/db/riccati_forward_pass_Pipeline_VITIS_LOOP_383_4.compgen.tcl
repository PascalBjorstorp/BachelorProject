# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_32s_27s_45_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_32s_32s_45_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_13_5_27_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {onehotencoding_realdef}
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_11_4_45_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {onehotencoding_realdef}
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
    id 1650 \
    name x_out_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_0 \
    op interface \
    ports { x_out_0_address1 { O 5 vector } x_out_0_ce1 { O 1 bit } x_out_0_we1 { O 1 bit } x_out_0_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1652 \
    name x_out_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_1 \
    op interface \
    ports { x_out_1_address1 { O 5 vector } x_out_1_ce1 { O 1 bit } x_out_1_we1 { O 1 bit } x_out_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1653 \
    name x_out_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_2 \
    op interface \
    ports { x_out_2_address1 { O 5 vector } x_out_2_ce1 { O 1 bit } x_out_2_we1 { O 1 bit } x_out_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1654 \
    name x_out_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_3 \
    op interface \
    ports { x_out_3_address1 { O 5 vector } x_out_3_ce1 { O 1 bit } x_out_3_we1 { O 1 bit } x_out_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1655 \
    name x_out_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_4 \
    op interface \
    ports { x_out_4_address1 { O 5 vector } x_out_4_ce1 { O 1 bit } x_out_4_we1 { O 1 bit } x_out_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1656 \
    name x_out_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename x_out_5 \
    op interface \
    ports { x_out_5_address1 { O 5 vector } x_out_5_ce1 { O 1 bit } x_out_5_we1 { O 1 bit } x_out_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'x_out_5'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1651 \
    name zext_ln409 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_zext_ln409 \
    op interface \
    ports { zext_ln409 { I 5 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1657 \
    name empty_519 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_519 \
    op interface \
    ports { empty_519 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1658 \
    name empty_520 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_520 \
    op interface \
    ports { empty_520 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1659 \
    name empty_521 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_521 \
    op interface \
    ports { empty_521 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1660 \
    name empty_522 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_522 \
    op interface \
    ports { empty_522 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1661 \
    name empty_523 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty_523 \
    op interface \
    ports { empty_523 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1662 \
    name empty \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty \
    op interface \
    ports { empty { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1663 \
    name step_data_0_0_0_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_0_0_load \
    op interface \
    ports { step_data_0_0_0_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1664 \
    name step_data_0_1_0_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_1_0_load \
    op interface \
    ports { step_data_0_1_0_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1665 \
    name step_data_0_2_0_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_2_0_load \
    op interface \
    ports { step_data_0_2_0_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1666 \
    name step_data_0_3_0_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_3_0_load \
    op interface \
    ports { step_data_0_3_0_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1667 \
    name step_data_0_4_0_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_4_0_load \
    op interface \
    ports { step_data_0_4_0_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1668 \
    name step_data_0_5_0_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_5_0_load \
    op interface \
    ports { step_data_0_5_0_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1669 \
    name sext_ln159 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159 \
    op interface \
    ports { sext_ln159 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1670 \
    name step_data_0_0_1_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_0_1_load \
    op interface \
    ports { step_data_0_0_1_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1671 \
    name step_data_0_1_1_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_1_1_load \
    op interface \
    ports { step_data_0_1_1_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1672 \
    name step_data_0_2_1_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_2_1_load \
    op interface \
    ports { step_data_0_2_1_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1673 \
    name step_data_0_3_1_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_3_1_load \
    op interface \
    ports { step_data_0_3_1_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1674 \
    name step_data_0_4_1_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_4_1_load \
    op interface \
    ports { step_data_0_4_1_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1675 \
    name step_data_0_5_1_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_5_1_load \
    op interface \
    ports { step_data_0_5_1_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1676 \
    name sext_ln159_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_1 \
    op interface \
    ports { sext_ln159_1 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1677 \
    name step_data_0_0_2_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_0_2_load \
    op interface \
    ports { step_data_0_0_2_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1678 \
    name step_data_0_1_2_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_1_2_load \
    op interface \
    ports { step_data_0_1_2_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1679 \
    name step_data_0_2_2_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_2_2_load \
    op interface \
    ports { step_data_0_2_2_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1680 \
    name step_data_0_3_2_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_3_2_load \
    op interface \
    ports { step_data_0_3_2_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1681 \
    name step_data_0_4_2_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_4_2_load \
    op interface \
    ports { step_data_0_4_2_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1682 \
    name step_data_0_5_2_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_5_2_load \
    op interface \
    ports { step_data_0_5_2_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1683 \
    name sext_ln159_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_2 \
    op interface \
    ports { sext_ln159_2 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1684 \
    name step_data_0_0_3_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_0_3_load \
    op interface \
    ports { step_data_0_0_3_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1685 \
    name step_data_0_1_3_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_1_3_load \
    op interface \
    ports { step_data_0_1_3_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1686 \
    name step_data_0_2_3_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_2_3_load \
    op interface \
    ports { step_data_0_2_3_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1687 \
    name step_data_0_3_3_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_3_3_load \
    op interface \
    ports { step_data_0_3_3_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1688 \
    name step_data_0_4_3_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_4_3_load \
    op interface \
    ports { step_data_0_4_3_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1689 \
    name step_data_0_5_3_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_5_3_load \
    op interface \
    ports { step_data_0_5_3_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1690 \
    name sext_ln159_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_3 \
    op interface \
    ports { sext_ln159_3 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1691 \
    name step_data_0_0_4_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_0_4_load \
    op interface \
    ports { step_data_0_0_4_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1692 \
    name step_data_0_1_4_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_1_4_load \
    op interface \
    ports { step_data_0_1_4_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1693 \
    name step_data_0_2_4_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_2_4_load \
    op interface \
    ports { step_data_0_2_4_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1694 \
    name step_data_0_3_4_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_3_4_load \
    op interface \
    ports { step_data_0_3_4_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1695 \
    name step_data_0_4_4_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_4_4_load \
    op interface \
    ports { step_data_0_4_4_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1696 \
    name step_data_0_5_4_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_5_4_load \
    op interface \
    ports { step_data_0_5_4_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1697 \
    name sext_ln159_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_4 \
    op interface \
    ports { sext_ln159_4 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1698 \
    name step_data_0_0_5_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_0_5_load \
    op interface \
    ports { step_data_0_0_5_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1699 \
    name step_data_0_1_5_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_1_5_load \
    op interface \
    ports { step_data_0_1_5_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1700 \
    name step_data_0_2_5_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_2_5_load \
    op interface \
    ports { step_data_0_2_5_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1701 \
    name step_data_0_3_5_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_3_5_load \
    op interface \
    ports { step_data_0_3_5_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1702 \
    name step_data_0_4_5_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_4_5_load \
    op interface \
    ports { step_data_0_4_5_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1703 \
    name step_data_0_5_5_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_step_data_0_5_5_load \
    op interface \
    ports { step_data_0_5_5_load { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1704 \
    name sext_ln406 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln406 \
    op interface \
    ports { sext_ln406 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1705 \
    name sext_ln159_11 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_11 \
    op interface \
    ports { sext_ln159_11 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1706 \
    name sext_ln420 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln420 \
    op interface \
    ports { sext_ln420 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1707 \
    name sext_ln159_12 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_12 \
    op interface \
    ports { sext_ln159_12 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1708 \
    name sext_ln159_13 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_13 \
    op interface \
    ports { sext_ln159_13 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1709 \
    name sext_ln159_14 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln159_14 \
    op interface \
    ports { sext_ln159_14 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1710 \
    name sext_ln420_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln420_1 \
    op interface \
    ports { sext_ln420_1 { I 27 vector } } \
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


