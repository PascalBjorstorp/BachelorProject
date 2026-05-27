# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_27s_22ns_49_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_31ns_19ns_49_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_31s_22ns_50_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
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
    id 629 \
    name step_data_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_2 \
    op interface \
    ports { step_data_2_address1 { O 7 vector } step_data_2_ce1 { O 1 bit } step_data_2_we1 { O 1 bit } step_data_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 630 \
    name step_data_0_5_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_5_5 \
    op interface \
    ports { step_data_0_5_5_address1 { O 5 vector } step_data_0_5_5_ce1 { O 1 bit } step_data_0_5_5_we1 { O 1 bit } step_data_0_5_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_5_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 631 \
    name step_data_0_5_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_5_4 \
    op interface \
    ports { step_data_0_5_4_address1 { O 5 vector } step_data_0_5_4_ce1 { O 1 bit } step_data_0_5_4_we1 { O 1 bit } step_data_0_5_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_5_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 632 \
    name step_data_0_5_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_5_3 \
    op interface \
    ports { step_data_0_5_3_address1 { O 5 vector } step_data_0_5_3_ce1 { O 1 bit } step_data_0_5_3_we1 { O 1 bit } step_data_0_5_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_5_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 633 \
    name step_data_0_5_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_5_2 \
    op interface \
    ports { step_data_0_5_2_address1 { O 5 vector } step_data_0_5_2_ce1 { O 1 bit } step_data_0_5_2_we1 { O 1 bit } step_data_0_5_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_5_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 634 \
    name step_data_0_5_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_5_1 \
    op interface \
    ports { step_data_0_5_1_address1 { O 5 vector } step_data_0_5_1_ce1 { O 1 bit } step_data_0_5_1_we1 { O 1 bit } step_data_0_5_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_5_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 635 \
    name step_data_0_5_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_5_0 \
    op interface \
    ports { step_data_0_5_0_address1 { O 5 vector } step_data_0_5_0_ce1 { O 1 bit } step_data_0_5_0_we1 { O 1 bit } step_data_0_5_0_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_5_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 636 \
    name step_data_0_4_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_4_5 \
    op interface \
    ports { step_data_0_4_5_address1 { O 5 vector } step_data_0_4_5_ce1 { O 1 bit } step_data_0_4_5_we1 { O 1 bit } step_data_0_4_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_4_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 637 \
    name step_data_0_4_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_4_4 \
    op interface \
    ports { step_data_0_4_4_address1 { O 5 vector } step_data_0_4_4_ce1 { O 1 bit } step_data_0_4_4_we1 { O 1 bit } step_data_0_4_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_4_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 638 \
    name step_data_0_4_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_4_3 \
    op interface \
    ports { step_data_0_4_3_address1 { O 5 vector } step_data_0_4_3_ce1 { O 1 bit } step_data_0_4_3_we1 { O 1 bit } step_data_0_4_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_4_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 639 \
    name step_data_0_4_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_4_2 \
    op interface \
    ports { step_data_0_4_2_address1 { O 5 vector } step_data_0_4_2_ce1 { O 1 bit } step_data_0_4_2_we1 { O 1 bit } step_data_0_4_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_4_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 640 \
    name step_data_0_4_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_4_1 \
    op interface \
    ports { step_data_0_4_1_address1 { O 5 vector } step_data_0_4_1_ce1 { O 1 bit } step_data_0_4_1_we1 { O 1 bit } step_data_0_4_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_4_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 641 \
    name step_data_0_4_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_4_0 \
    op interface \
    ports { step_data_0_4_0_address1 { O 5 vector } step_data_0_4_0_ce1 { O 1 bit } step_data_0_4_0_we1 { O 1 bit } step_data_0_4_0_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_4_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 642 \
    name step_data_0_3_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_3_5 \
    op interface \
    ports { step_data_0_3_5_address1 { O 5 vector } step_data_0_3_5_ce1 { O 1 bit } step_data_0_3_5_we1 { O 1 bit } step_data_0_3_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_3_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 643 \
    name step_data_0_3_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_3_4 \
    op interface \
    ports { step_data_0_3_4_address1 { O 5 vector } step_data_0_3_4_ce1 { O 1 bit } step_data_0_3_4_we1 { O 1 bit } step_data_0_3_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_3_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 644 \
    name step_data_0_3_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_3_3 \
    op interface \
    ports { step_data_0_3_3_address1 { O 5 vector } step_data_0_3_3_ce1 { O 1 bit } step_data_0_3_3_we1 { O 1 bit } step_data_0_3_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_3_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 645 \
    name step_data_0_3_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_3_2 \
    op interface \
    ports { step_data_0_3_2_address1 { O 5 vector } step_data_0_3_2_ce1 { O 1 bit } step_data_0_3_2_we1 { O 1 bit } step_data_0_3_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_3_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 646 \
    name step_data_0_3_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_3_1 \
    op interface \
    ports { step_data_0_3_1_address1 { O 5 vector } step_data_0_3_1_ce1 { O 1 bit } step_data_0_3_1_we1 { O 1 bit } step_data_0_3_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_3_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 647 \
    name step_data_0_3_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_3_0 \
    op interface \
    ports { step_data_0_3_0_address1 { O 5 vector } step_data_0_3_0_ce1 { O 1 bit } step_data_0_3_0_we1 { O 1 bit } step_data_0_3_0_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_3_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 648 \
    name step_data_0_2_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_2_5 \
    op interface \
    ports { step_data_0_2_5_address1 { O 5 vector } step_data_0_2_5_ce1 { O 1 bit } step_data_0_2_5_we1 { O 1 bit } step_data_0_2_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_2_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 649 \
    name step_data_0_2_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_2_4 \
    op interface \
    ports { step_data_0_2_4_address1 { O 5 vector } step_data_0_2_4_ce1 { O 1 bit } step_data_0_2_4_we1 { O 1 bit } step_data_0_2_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_2_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 650 \
    name step_data_0_2_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_2_3 \
    op interface \
    ports { step_data_0_2_3_address1 { O 5 vector } step_data_0_2_3_ce1 { O 1 bit } step_data_0_2_3_we1 { O 1 bit } step_data_0_2_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_2_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 651 \
    name step_data_0_2_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_2_2 \
    op interface \
    ports { step_data_0_2_2_address1 { O 5 vector } step_data_0_2_2_ce1 { O 1 bit } step_data_0_2_2_we1 { O 1 bit } step_data_0_2_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_2_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 652 \
    name step_data_0_2_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_2_1 \
    op interface \
    ports { step_data_0_2_1_address1 { O 5 vector } step_data_0_2_1_ce1 { O 1 bit } step_data_0_2_1_we1 { O 1 bit } step_data_0_2_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_2_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 653 \
    name step_data_0_2_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_2_0 \
    op interface \
    ports { step_data_0_2_0_address1 { O 5 vector } step_data_0_2_0_ce1 { O 1 bit } step_data_0_2_0_we1 { O 1 bit } step_data_0_2_0_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_2_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 654 \
    name step_data_0_1_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_1_5 \
    op interface \
    ports { step_data_0_1_5_address1 { O 5 vector } step_data_0_1_5_ce1 { O 1 bit } step_data_0_1_5_we1 { O 1 bit } step_data_0_1_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_1_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 655 \
    name step_data_0_1_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_1_4 \
    op interface \
    ports { step_data_0_1_4_address1 { O 5 vector } step_data_0_1_4_ce1 { O 1 bit } step_data_0_1_4_we1 { O 1 bit } step_data_0_1_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_1_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 656 \
    name step_data_0_1_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_1_3 \
    op interface \
    ports { step_data_0_1_3_address1 { O 5 vector } step_data_0_1_3_ce1 { O 1 bit } step_data_0_1_3_we1 { O 1 bit } step_data_0_1_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_1_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 657 \
    name step_data_0_1_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_1_2 \
    op interface \
    ports { step_data_0_1_2_address1 { O 5 vector } step_data_0_1_2_ce1 { O 1 bit } step_data_0_1_2_we1 { O 1 bit } step_data_0_1_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_1_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 658 \
    name step_data_0_1_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_1_1 \
    op interface \
    ports { step_data_0_1_1_address1 { O 5 vector } step_data_0_1_1_ce1 { O 1 bit } step_data_0_1_1_we1 { O 1 bit } step_data_0_1_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_1_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 659 \
    name step_data_0_1_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_1_0 \
    op interface \
    ports { step_data_0_1_0_address1 { O 5 vector } step_data_0_1_0_ce1 { O 1 bit } step_data_0_1_0_we1 { O 1 bit } step_data_0_1_0_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_1_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 660 \
    name step_data_0_0_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_0_5 \
    op interface \
    ports { step_data_0_0_5_address1 { O 5 vector } step_data_0_0_5_ce1 { O 1 bit } step_data_0_0_5_we1 { O 1 bit } step_data_0_0_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_0_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 661 \
    name step_data_0_0_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_0_4 \
    op interface \
    ports { step_data_0_0_4_address1 { O 5 vector } step_data_0_0_4_ce1 { O 1 bit } step_data_0_0_4_we1 { O 1 bit } step_data_0_0_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_0_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 662 \
    name step_data_0_0_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_0_3 \
    op interface \
    ports { step_data_0_0_3_address1 { O 5 vector } step_data_0_0_3_ce1 { O 1 bit } step_data_0_0_3_we1 { O 1 bit } step_data_0_0_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_0_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 663 \
    name step_data_0_0_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_0_2 \
    op interface \
    ports { step_data_0_0_2_address1 { O 5 vector } step_data_0_0_2_ce1 { O 1 bit } step_data_0_0_2_we1 { O 1 bit } step_data_0_0_2_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_0_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 664 \
    name step_data_0_0_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_0_1 \
    op interface \
    ports { step_data_0_0_1_address1 { O 5 vector } step_data_0_0_1_ce1 { O 1 bit } step_data_0_0_1_we1 { O 1 bit } step_data_0_0_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_0_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 665 \
    name step_data_0_0_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_0_0_0 \
    op interface \
    ports { step_data_0_0_0_address1 { O 5 vector } step_data_0_0_0_ce1 { O 1 bit } step_data_0_0_0_we1 { O 1 bit } step_data_0_0_0_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_0_0_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 666 \
    name B_sparse_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename B_sparse_3 \
    op interface \
    ports { B_sparse_3_address0 { O 5 vector } B_sparse_3_ce0 { O 1 bit } B_sparse_3_we0 { O 1 bit } B_sparse_3_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'B_sparse_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 667 \
    name B_sparse_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename B_sparse_2 \
    op interface \
    ports { B_sparse_2_address0 { O 5 vector } B_sparse_2_ce0 { O 1 bit } B_sparse_2_we0 { O 1 bit } B_sparse_2_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'B_sparse_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 668 \
    name B_sparse_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename B_sparse_1 \
    op interface \
    ports { B_sparse_1_address0 { O 5 vector } B_sparse_1_ce0 { O 1 bit } B_sparse_1_we0 { O 1 bit } B_sparse_1_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'B_sparse_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 669 \
    name B_sparse \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename B_sparse \
    op interface \
    ports { B_sparse_address0 { O 5 vector } B_sparse_ce0 { O 1 bit } B_sparse_we0 { O 1 bit } B_sparse_d0 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'B_sparse'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 670 \
    name step_data_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_3 \
    op interface \
    ports { step_data_3_address1 { O 5 vector } step_data_3_ce1 { O 1 bit } step_data_3_we1 { O 1 bit } step_data_3_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 671 \
    name step_data_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_4 \
    op interface \
    ports { step_data_4_address1 { O 5 vector } step_data_4_ce1 { O 1 bit } step_data_4_we1 { O 1 bit } step_data_4_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 672 \
    name step_data_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_5 \
    op interface \
    ports { step_data_5_address1 { O 5 vector } step_data_5_ce1 { O 1 bit } step_data_5_we1 { O 1 bit } step_data_5_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 673 \
    name ref_path_curvature \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_path_curvature \
    op interface \
    ports { ref_path_curvature_address0 { O 5 vector } ref_path_curvature_ce0 { O 1 bit } ref_path_curvature_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_path_curvature'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 714 \
    name ref_reference_lateral_error \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_lateral_error \
    op interface \
    ports { ref_reference_lateral_error_address0 { O 5 vector } ref_reference_lateral_error_ce0 { O 1 bit } ref_reference_lateral_error_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_lateral_error'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 715 \
    name ref_reference_velocity \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_velocity \
    op interface \
    ports { ref_reference_velocity_address0 { O 5 vector } ref_reference_velocity_ce0 { O 1 bit } ref_reference_velocity_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_velocity'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 718 \
    name step_data_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename step_data_1 \
    op interface \
    ports { step_data_1_address1 { O 7 vector } step_data_1_ce1 { O 1 bit } step_data_1_we1 { O 1 bit } step_data_1_d1 { O 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 719 \
    name ref_reference_heading_error \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_heading_error \
    op interface \
    ports { ref_reference_heading_error_address0 { O 5 vector } ref_reference_heading_error_ce0 { O 1 bit } ref_reference_heading_error_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_heading_error'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 720 \
    name ref_reference_lateral_velocity \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_lateral_velocity \
    op interface \
    ports { ref_reference_lateral_velocity_address0 { O 5 vector } ref_reference_lateral_velocity_ce0 { O 1 bit } ref_reference_lateral_velocity_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_lateral_velocity'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 721 \
    name ref_reference_yaw_rate \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename ref_reference_yaw_rate \
    op interface \
    ports { ref_reference_yaw_rate_address0 { O 5 vector } ref_reference_yaw_rate_ce0 { O 1 bit } ref_reference_yaw_rate_q0 { I 32 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'ref_reference_yaw_rate'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 622 \
    name x0_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_x0_1 \
    op interface \
    ports { x0_1 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 623 \
    name state_omega \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_omega \
    op interface \
    ports { state_omega { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 624 \
    name state_vy \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_vy \
    op interface \
    ports { state_vy { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 625 \
    name zext_ln128 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_zext_ln128 \
    op interface \
    ports { zext_ln128 { I 31 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 626 \
    name state_ey \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_ey \
    op interface \
    ports { state_ey { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 627 \
    name state_epsi \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_state_epsi \
    op interface \
    ports { state_epsi { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 628 \
    name sext_ln483 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln483 \
    op interface \
    ports { sext_ln483 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 674 \
    name wall_left_min_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_reload \
    op interface \
    ports { wall_left_min_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 675 \
    name wall_left_min_1_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_1_reload \
    op interface \
    ports { wall_left_min_1_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 676 \
    name wall_left_min_2_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_2_reload \
    op interface \
    ports { wall_left_min_2_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 677 \
    name wall_left_min_3_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_3_reload \
    op interface \
    ports { wall_left_min_3_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 678 \
    name wall_left_min_4_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_4_reload \
    op interface \
    ports { wall_left_min_4_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 679 \
    name wall_left_min_5_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_5_reload \
    op interface \
    ports { wall_left_min_5_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 680 \
    name wall_left_min_6_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_6_reload \
    op interface \
    ports { wall_left_min_6_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 681 \
    name wall_left_min_7_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_7_reload \
    op interface \
    ports { wall_left_min_7_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 682 \
    name wall_left_min_8_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_8_reload \
    op interface \
    ports { wall_left_min_8_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 683 \
    name wall_left_min_9_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_9_reload \
    op interface \
    ports { wall_left_min_9_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 684 \
    name wall_left_min_10_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_10_reload \
    op interface \
    ports { wall_left_min_10_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 685 \
    name wall_left_min_11_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_11_reload \
    op interface \
    ports { wall_left_min_11_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 686 \
    name wall_left_min_12_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_12_reload \
    op interface \
    ports { wall_left_min_12_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 687 \
    name wall_left_min_13_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_13_reload \
    op interface \
    ports { wall_left_min_13_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 688 \
    name wall_left_min_14_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_14_reload \
    op interface \
    ports { wall_left_min_14_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 689 \
    name wall_left_min_15_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_15_reload \
    op interface \
    ports { wall_left_min_15_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 690 \
    name wall_left_min_16_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_16_reload \
    op interface \
    ports { wall_left_min_16_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 691 \
    name wall_left_min_17_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_17_reload \
    op interface \
    ports { wall_left_min_17_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 692 \
    name wall_left_min_18_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_18_reload \
    op interface \
    ports { wall_left_min_18_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 693 \
    name wall_left_min_19_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_left_min_19_reload \
    op interface \
    ports { wall_left_min_19_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 694 \
    name wall_right_min_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_reload \
    op interface \
    ports { wall_right_min_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 695 \
    name wall_right_min_1_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_1_reload \
    op interface \
    ports { wall_right_min_1_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 696 \
    name wall_right_min_2_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_2_reload \
    op interface \
    ports { wall_right_min_2_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 697 \
    name wall_right_min_3_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_3_reload \
    op interface \
    ports { wall_right_min_3_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 698 \
    name wall_right_min_4_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_4_reload \
    op interface \
    ports { wall_right_min_4_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 699 \
    name wall_right_min_5_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_5_reload \
    op interface \
    ports { wall_right_min_5_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 700 \
    name wall_right_min_6_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_6_reload \
    op interface \
    ports { wall_right_min_6_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 701 \
    name wall_right_min_7_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_7_reload \
    op interface \
    ports { wall_right_min_7_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 702 \
    name wall_right_min_8_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_8_reload \
    op interface \
    ports { wall_right_min_8_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 703 \
    name wall_right_min_9_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_9_reload \
    op interface \
    ports { wall_right_min_9_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 704 \
    name wall_right_min_10_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_10_reload \
    op interface \
    ports { wall_right_min_10_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 705 \
    name wall_right_min_11_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_11_reload \
    op interface \
    ports { wall_right_min_11_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 706 \
    name wall_right_min_12_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_12_reload \
    op interface \
    ports { wall_right_min_12_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 707 \
    name wall_right_min_13_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_13_reload \
    op interface \
    ports { wall_right_min_13_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 708 \
    name wall_right_min_14_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_14_reload \
    op interface \
    ports { wall_right_min_14_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 709 \
    name wall_right_min_15_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_15_reload \
    op interface \
    ports { wall_right_min_15_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 710 \
    name wall_right_min_16_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_16_reload \
    op interface \
    ports { wall_right_min_16_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 711 \
    name wall_right_min_17_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_17_reload \
    op interface \
    ports { wall_right_min_17_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 712 \
    name wall_right_min_18_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_18_reload \
    op interface \
    ports { wall_right_min_18_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 713 \
    name wall_right_min_19_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_wall_right_min_19_reload \
    op interface \
    ports { wall_right_min_19_reload { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 716 \
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
    id 717 \
    name x0_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_x0_2 \
    op interface \
    ports { x0_2 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 722 \
    name out_144_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_out_144_out \
    op interface \
    ports { out_144_out { O 32 vector } out_144_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 723 \
    name terminal_dff_raw_0153_0_0_0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_terminal_dff_raw_0153_0_0_0_out \
    op interface \
    ports { terminal_dff_raw_0153_0_0_0_out { O 32 vector } terminal_dff_raw_0153_0_0_0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 724 \
    name terminal_wall_x_ub_con_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_terminal_wall_x_ub_con_out \
    op interface \
    ports { terminal_wall_x_ub_con_out { O 32 vector } terminal_wall_x_ub_con_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 725 \
    name terminal_wall_x_lb_con_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_terminal_wall_x_lb_con_out \
    op interface \
    ports { terminal_wall_x_lb_con_out { O 32 vector } terminal_wall_x_lb_con_out_ap_vld { O 1 bit } } \
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


