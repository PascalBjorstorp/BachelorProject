# This script segment is generated automatically by AutoPilot

if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_17_3_18_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {compactencoding_dontcare}
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_17_3_18_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {compactencoding_dontcare}
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
    id 931 \
    name K_0_0_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_0 \
    op interface \
    ports { K_0_0_0_address0 { O 3 vector } K_0_0_0_ce0 { O 1 bit } K_0_0_0_we0 { O 1 bit } K_0_0_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 933 \
    name K_0_1_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_0 \
    op interface \
    ports { K_0_1_0_address0 { O 3 vector } K_0_1_0_ce0 { O 1 bit } K_0_1_0_we0 { O 1 bit } K_0_1_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 934 \
    name K_0_0_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_1 \
    op interface \
    ports { K_0_0_1_address0 { O 3 vector } K_0_0_1_ce0 { O 1 bit } K_0_0_1_we0 { O 1 bit } K_0_0_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 935 \
    name K_0_1_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_1 \
    op interface \
    ports { K_0_1_1_address0 { O 3 vector } K_0_1_1_ce0 { O 1 bit } K_0_1_1_we0 { O 1 bit } K_0_1_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 936 \
    name K_0_0_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_2 \
    op interface \
    ports { K_0_0_2_address0 { O 3 vector } K_0_0_2_ce0 { O 1 bit } K_0_0_2_we0 { O 1 bit } K_0_0_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 937 \
    name K_0_1_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_2 \
    op interface \
    ports { K_0_1_2_address0 { O 3 vector } K_0_1_2_ce0 { O 1 bit } K_0_1_2_we0 { O 1 bit } K_0_1_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 938 \
    name K_0_0_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_3 \
    op interface \
    ports { K_0_0_3_address0 { O 3 vector } K_0_0_3_ce0 { O 1 bit } K_0_0_3_we0 { O 1 bit } K_0_0_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 939 \
    name K_0_1_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_3 \
    op interface \
    ports { K_0_1_3_address0 { O 3 vector } K_0_1_3_ce0 { O 1 bit } K_0_1_3_we0 { O 1 bit } K_0_1_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 940 \
    name K_0_0_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_4 \
    op interface \
    ports { K_0_0_4_address0 { O 3 vector } K_0_0_4_ce0 { O 1 bit } K_0_0_4_we0 { O 1 bit } K_0_0_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 941 \
    name K_0_1_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_4 \
    op interface \
    ports { K_0_1_4_address0 { O 3 vector } K_0_1_4_ce0 { O 1 bit } K_0_1_4_we0 { O 1 bit } K_0_1_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 942 \
    name K_0_0_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_5 \
    op interface \
    ports { K_0_0_5_address0 { O 3 vector } K_0_0_5_ce0 { O 1 bit } K_0_0_5_we0 { O 1 bit } K_0_0_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 943 \
    name K_0_1_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_5 \
    op interface \
    ports { K_0_1_5_address0 { O 3 vector } K_0_1_5_ce0 { O 1 bit } K_0_1_5_we0 { O 1 bit } K_0_1_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 944 \
    name K_0_0_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_6 \
    op interface \
    ports { K_0_0_6_address0 { O 3 vector } K_0_0_6_ce0 { O 1 bit } K_0_0_6_we0 { O 1 bit } K_0_0_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 945 \
    name K_0_1_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_6 \
    op interface \
    ports { K_0_1_6_address0 { O 3 vector } K_0_1_6_ce0 { O 1 bit } K_0_1_6_we0 { O 1 bit } K_0_1_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 946 \
    name K_0_0_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_0_7 \
    op interface \
    ports { K_0_0_7_address0 { O 3 vector } K_0_0_7_ce0 { O 1 bit } K_0_0_7_we0 { O 1 bit } K_0_0_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_0_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 947 \
    name K_0_1_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_0_1_7 \
    op interface \
    ports { K_0_1_7_address0 { O 3 vector } K_0_1_7_ce0 { O 1 bit } K_0_1_7_we0 { O 1 bit } K_0_1_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_0_1_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 948 \
    name K_1_0_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_0 \
    op interface \
    ports { K_1_0_0_address0 { O 3 vector } K_1_0_0_ce0 { O 1 bit } K_1_0_0_we0 { O 1 bit } K_1_0_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 949 \
    name K_1_1_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_0 \
    op interface \
    ports { K_1_1_0_address0 { O 3 vector } K_1_1_0_ce0 { O 1 bit } K_1_1_0_we0 { O 1 bit } K_1_1_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 950 \
    name K_1_0_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_1 \
    op interface \
    ports { K_1_0_1_address0 { O 3 vector } K_1_0_1_ce0 { O 1 bit } K_1_0_1_we0 { O 1 bit } K_1_0_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 951 \
    name K_1_1_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_1 \
    op interface \
    ports { K_1_1_1_address0 { O 3 vector } K_1_1_1_ce0 { O 1 bit } K_1_1_1_we0 { O 1 bit } K_1_1_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 952 \
    name K_1_0_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_2 \
    op interface \
    ports { K_1_0_2_address0 { O 3 vector } K_1_0_2_ce0 { O 1 bit } K_1_0_2_we0 { O 1 bit } K_1_0_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 953 \
    name K_1_1_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_2 \
    op interface \
    ports { K_1_1_2_address0 { O 3 vector } K_1_1_2_ce0 { O 1 bit } K_1_1_2_we0 { O 1 bit } K_1_1_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 954 \
    name K_1_0_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_3 \
    op interface \
    ports { K_1_0_3_address0 { O 3 vector } K_1_0_3_ce0 { O 1 bit } K_1_0_3_we0 { O 1 bit } K_1_0_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 955 \
    name K_1_1_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_3 \
    op interface \
    ports { K_1_1_3_address0 { O 3 vector } K_1_1_3_ce0 { O 1 bit } K_1_1_3_we0 { O 1 bit } K_1_1_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 956 \
    name K_1_0_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_4 \
    op interface \
    ports { K_1_0_4_address0 { O 3 vector } K_1_0_4_ce0 { O 1 bit } K_1_0_4_we0 { O 1 bit } K_1_0_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 957 \
    name K_1_1_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_4 \
    op interface \
    ports { K_1_1_4_address0 { O 3 vector } K_1_1_4_ce0 { O 1 bit } K_1_1_4_we0 { O 1 bit } K_1_1_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 958 \
    name K_1_0_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_5 \
    op interface \
    ports { K_1_0_5_address0 { O 3 vector } K_1_0_5_ce0 { O 1 bit } K_1_0_5_we0 { O 1 bit } K_1_0_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 959 \
    name K_1_1_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_5 \
    op interface \
    ports { K_1_1_5_address0 { O 3 vector } K_1_1_5_ce0 { O 1 bit } K_1_1_5_we0 { O 1 bit } K_1_1_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 960 \
    name K_1_0_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_6 \
    op interface \
    ports { K_1_0_6_address0 { O 3 vector } K_1_0_6_ce0 { O 1 bit } K_1_0_6_we0 { O 1 bit } K_1_0_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 961 \
    name K_1_1_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_6 \
    op interface \
    ports { K_1_1_6_address0 { O 3 vector } K_1_1_6_ce0 { O 1 bit } K_1_1_6_we0 { O 1 bit } K_1_1_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 962 \
    name K_1_0_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_0_7 \
    op interface \
    ports { K_1_0_7_address0 { O 3 vector } K_1_0_7_ce0 { O 1 bit } K_1_0_7_we0 { O 1 bit } K_1_0_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_0_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 963 \
    name K_1_1_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_1_1_7 \
    op interface \
    ports { K_1_1_7_address0 { O 3 vector } K_1_1_7_ce0 { O 1 bit } K_1_1_7_we0 { O 1 bit } K_1_1_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_1_1_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 964 \
    name K_2_0_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_0 \
    op interface \
    ports { K_2_0_0_address0 { O 3 vector } K_2_0_0_ce0 { O 1 bit } K_2_0_0_we0 { O 1 bit } K_2_0_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 965 \
    name K_2_1_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_0 \
    op interface \
    ports { K_2_1_0_address0 { O 3 vector } K_2_1_0_ce0 { O 1 bit } K_2_1_0_we0 { O 1 bit } K_2_1_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 966 \
    name K_2_0_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_1 \
    op interface \
    ports { K_2_0_1_address0 { O 3 vector } K_2_0_1_ce0 { O 1 bit } K_2_0_1_we0 { O 1 bit } K_2_0_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 967 \
    name K_2_1_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_1 \
    op interface \
    ports { K_2_1_1_address0 { O 3 vector } K_2_1_1_ce0 { O 1 bit } K_2_1_1_we0 { O 1 bit } K_2_1_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 968 \
    name K_2_0_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_2 \
    op interface \
    ports { K_2_0_2_address0 { O 3 vector } K_2_0_2_ce0 { O 1 bit } K_2_0_2_we0 { O 1 bit } K_2_0_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 969 \
    name K_2_1_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_2 \
    op interface \
    ports { K_2_1_2_address0 { O 3 vector } K_2_1_2_ce0 { O 1 bit } K_2_1_2_we0 { O 1 bit } K_2_1_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 970 \
    name K_2_0_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_3 \
    op interface \
    ports { K_2_0_3_address0 { O 3 vector } K_2_0_3_ce0 { O 1 bit } K_2_0_3_we0 { O 1 bit } K_2_0_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 971 \
    name K_2_1_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_3 \
    op interface \
    ports { K_2_1_3_address0 { O 3 vector } K_2_1_3_ce0 { O 1 bit } K_2_1_3_we0 { O 1 bit } K_2_1_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 972 \
    name K_2_0_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_4 \
    op interface \
    ports { K_2_0_4_address0 { O 3 vector } K_2_0_4_ce0 { O 1 bit } K_2_0_4_we0 { O 1 bit } K_2_0_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 973 \
    name K_2_1_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_4 \
    op interface \
    ports { K_2_1_4_address0 { O 3 vector } K_2_1_4_ce0 { O 1 bit } K_2_1_4_we0 { O 1 bit } K_2_1_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 974 \
    name K_2_0_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_5 \
    op interface \
    ports { K_2_0_5_address0 { O 3 vector } K_2_0_5_ce0 { O 1 bit } K_2_0_5_we0 { O 1 bit } K_2_0_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 975 \
    name K_2_1_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_5 \
    op interface \
    ports { K_2_1_5_address0 { O 3 vector } K_2_1_5_ce0 { O 1 bit } K_2_1_5_we0 { O 1 bit } K_2_1_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 976 \
    name K_2_0_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_6 \
    op interface \
    ports { K_2_0_6_address0 { O 3 vector } K_2_0_6_ce0 { O 1 bit } K_2_0_6_we0 { O 1 bit } K_2_0_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 977 \
    name K_2_1_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_6 \
    op interface \
    ports { K_2_1_6_address0 { O 3 vector } K_2_1_6_ce0 { O 1 bit } K_2_1_6_we0 { O 1 bit } K_2_1_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 978 \
    name K_2_0_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_0_7 \
    op interface \
    ports { K_2_0_7_address0 { O 3 vector } K_2_0_7_ce0 { O 1 bit } K_2_0_7_we0 { O 1 bit } K_2_0_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_0_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 979 \
    name K_2_1_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_2_1_7 \
    op interface \
    ports { K_2_1_7_address0 { O 3 vector } K_2_1_7_ce0 { O 1 bit } K_2_1_7_we0 { O 1 bit } K_2_1_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_2_1_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 980 \
    name K_3_0_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_0 \
    op interface \
    ports { K_3_0_0_address0 { O 3 vector } K_3_0_0_ce0 { O 1 bit } K_3_0_0_we0 { O 1 bit } K_3_0_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 981 \
    name K_3_1_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_0 \
    op interface \
    ports { K_3_1_0_address0 { O 3 vector } K_3_1_0_ce0 { O 1 bit } K_3_1_0_we0 { O 1 bit } K_3_1_0_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 982 \
    name K_3_0_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_1 \
    op interface \
    ports { K_3_0_1_address0 { O 3 vector } K_3_0_1_ce0 { O 1 bit } K_3_0_1_we0 { O 1 bit } K_3_0_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 983 \
    name K_3_1_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_1 \
    op interface \
    ports { K_3_1_1_address0 { O 3 vector } K_3_1_1_ce0 { O 1 bit } K_3_1_1_we0 { O 1 bit } K_3_1_1_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 984 \
    name K_3_0_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_2 \
    op interface \
    ports { K_3_0_2_address0 { O 3 vector } K_3_0_2_ce0 { O 1 bit } K_3_0_2_we0 { O 1 bit } K_3_0_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 985 \
    name K_3_1_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_2 \
    op interface \
    ports { K_3_1_2_address0 { O 3 vector } K_3_1_2_ce0 { O 1 bit } K_3_1_2_we0 { O 1 bit } K_3_1_2_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 986 \
    name K_3_0_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_3 \
    op interface \
    ports { K_3_0_3_address0 { O 3 vector } K_3_0_3_ce0 { O 1 bit } K_3_0_3_we0 { O 1 bit } K_3_0_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 987 \
    name K_3_1_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_3 \
    op interface \
    ports { K_3_1_3_address0 { O 3 vector } K_3_1_3_ce0 { O 1 bit } K_3_1_3_we0 { O 1 bit } K_3_1_3_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 988 \
    name K_3_0_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_4 \
    op interface \
    ports { K_3_0_4_address0 { O 3 vector } K_3_0_4_ce0 { O 1 bit } K_3_0_4_we0 { O 1 bit } K_3_0_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 989 \
    name K_3_1_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_4 \
    op interface \
    ports { K_3_1_4_address0 { O 3 vector } K_3_1_4_ce0 { O 1 bit } K_3_1_4_we0 { O 1 bit } K_3_1_4_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 990 \
    name K_3_0_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_5 \
    op interface \
    ports { K_3_0_5_address0 { O 3 vector } K_3_0_5_ce0 { O 1 bit } K_3_0_5_we0 { O 1 bit } K_3_0_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 991 \
    name K_3_1_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_5 \
    op interface \
    ports { K_3_1_5_address0 { O 3 vector } K_3_1_5_ce0 { O 1 bit } K_3_1_5_we0 { O 1 bit } K_3_1_5_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 992 \
    name K_3_0_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_6 \
    op interface \
    ports { K_3_0_6_address0 { O 3 vector } K_3_0_6_ce0 { O 1 bit } K_3_0_6_we0 { O 1 bit } K_3_0_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 993 \
    name K_3_1_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_6 \
    op interface \
    ports { K_3_1_6_address0 { O 3 vector } K_3_1_6_ce0 { O 1 bit } K_3_1_6_we0 { O 1 bit } K_3_1_6_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 994 \
    name K_3_0_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_0_7 \
    op interface \
    ports { K_3_0_7_address0 { O 3 vector } K_3_0_7_ce0 { O 1 bit } K_3_0_7_we0 { O 1 bit } K_3_0_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_0_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 995 \
    name K_3_1_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename K_3_1_7 \
    op interface \
    ports { K_3_1_7_address0 { O 3 vector } K_3_1_7_ce0 { O 1 bit } K_3_1_7_we0 { O 1 bit } K_3_1_7_d0 { O 17 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'K_3_1_7'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 932 \
    name zext_ln491 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_zext_ln491 \
    op interface \
    ports { zext_ln491 { I 3 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 996 \
    name G_12_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_12_reload \
    op interface \
    ports { G_12_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 997 \
    name G_11_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_11_reload \
    op interface \
    ports { G_11_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 998 \
    name G_10_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_10_reload \
    op interface \
    ports { G_10_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 999 \
    name G_9_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_9_reload \
    op interface \
    ports { G_9_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1000 \
    name G_8_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_8_reload \
    op interface \
    ports { G_8_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1001 \
    name G_7_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_7_reload \
    op interface \
    ports { G_7_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1002 \
    name select_ln739 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_select_ln739 \
    op interface \
    ports { select_ln739 { I 6 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1003 \
    name sext_ln280 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln280 \
    op interface \
    ports { sext_ln280 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1004 \
    name G_6_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_6_reload \
    op interface \
    ports { G_6_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1005 \
    name G_5_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_5_reload \
    op interface \
    ports { G_5_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1006 \
    name G_4_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_4_reload \
    op interface \
    ports { G_4_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1007 \
    name G_3_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_3_reload \
    op interface \
    ports { G_3_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1008 \
    name G_2_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_2_reload \
    op interface \
    ports { G_2_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1009 \
    name G_1_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_1_reload \
    op interface \
    ports { G_1_reload { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1010 \
    name sext_ln280_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln280_1 \
    op interface \
    ports { sext_ln280_1 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1011 \
    name sext_ln280_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln280_2 \
    op interface \
    ports { sext_ln280_2 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1012 \
    name sext_ln777 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln777 \
    op interface \
    ports { sext_ln777 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1013 \
    name empty \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_empty \
    op interface \
    ports { empty { I 2 vector } } \
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


