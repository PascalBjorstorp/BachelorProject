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
    id 2013 \
    name z_x \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x \
    op interface \
    ports { z_x_address0 { O 5 vector } z_x_ce0 { O 1 bit } z_x_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2014 \
    name y_x \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x \
    op interface \
    ports { y_x_address0 { O 5 vector } y_x_ce0 { O 1 bit } y_x_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2015 \
    name z_x_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x_1 \
    op interface \
    ports { z_x_1_address0 { O 5 vector } z_x_1_ce0 { O 1 bit } z_x_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2016 \
    name y_x_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x_1 \
    op interface \
    ports { y_x_1_address0 { O 5 vector } y_x_1_ce0 { O 1 bit } y_x_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2017 \
    name z_x_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x_2 \
    op interface \
    ports { z_x_2_address0 { O 5 vector } z_x_2_ce0 { O 1 bit } z_x_2_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2018 \
    name y_x_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x_2 \
    op interface \
    ports { y_x_2_address0 { O 5 vector } y_x_2_ce0 { O 1 bit } y_x_2_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2019 \
    name z_x_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x_3 \
    op interface \
    ports { z_x_3_address0 { O 5 vector } z_x_3_ce0 { O 1 bit } z_x_3_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2020 \
    name y_x_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x_3 \
    op interface \
    ports { y_x_3_address0 { O 5 vector } y_x_3_ce0 { O 1 bit } y_x_3_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2021 \
    name z_x_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x_4 \
    op interface \
    ports { z_x_4_address0 { O 5 vector } z_x_4_ce0 { O 1 bit } z_x_4_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2022 \
    name y_x_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x_4 \
    op interface \
    ports { y_x_4_address0 { O 5 vector } y_x_4_ce0 { O 1 bit } y_x_4_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2023 \
    name z_x_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x_5 \
    op interface \
    ports { z_x_5_address0 { O 5 vector } z_x_5_ce0 { O 1 bit } z_x_5_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2024 \
    name y_x_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x_5 \
    op interface \
    ports { y_x_5_address0 { O 5 vector } y_x_5_ce0 { O 1 bit } y_x_5_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2025 \
    name z_x_6 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x_6 \
    op interface \
    ports { z_x_6_address0 { O 5 vector } z_x_6_ce0 { O 1 bit } z_x_6_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2026 \
    name y_x_6 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x_6 \
    op interface \
    ports { y_x_6_address0 { O 5 vector } y_x_6_ce0 { O 1 bit } y_x_6_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2027 \
    name z_x_7 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename z_x_7 \
    op interface \
    ports { z_x_7_address0 { O 5 vector } z_x_7_ce0 { O 1 bit } z_x_7_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2028 \
    name y_x_7 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename y_x_7 \
    op interface \
    ports { y_x_7_address0 { O 5 vector } y_x_7_ce0 { O 1 bit } y_x_7_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2029 \
    name p_anonymous_namespace_g_core_state_admm_z_x_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_0 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_0_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_0_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_0_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_0_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2030 \
    name p_anonymous_namespace_g_core_state_admm_z_x_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_1 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_1_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_1_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_1_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_1_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2031 \
    name p_anonymous_namespace_g_core_state_admm_z_x_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_2 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_2_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_2_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_2_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_2_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2032 \
    name p_anonymous_namespace_g_core_state_admm_z_x_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_3 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_3_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_3_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_3_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_3_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2033 \
    name p_anonymous_namespace_g_core_state_admm_z_x_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_4 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_4_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_4_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_4_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_4_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2034 \
    name p_anonymous_namespace_g_core_state_admm_z_x_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_5 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_5_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_5_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_5_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_5_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2035 \
    name p_anonymous_namespace_g_core_state_admm_z_x_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_6 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_6_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_6_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_6_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_6_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2036 \
    name p_anonymous_namespace_g_core_state_admm_z_x_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_z_x_7 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_z_x_7_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_z_x_7_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_7_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_z_x_7_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_z_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2037 \
    name p_anonymous_namespace_g_core_state_admm_y_x_0 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_0 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_0_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_0_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_0_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_0_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_0'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2038 \
    name p_anonymous_namespace_g_core_state_admm_y_x_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_1 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_1_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_1_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_1_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_1_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2039 \
    name p_anonymous_namespace_g_core_state_admm_y_x_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_2 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_2_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_2_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_2_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_2_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2040 \
    name p_anonymous_namespace_g_core_state_admm_y_x_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_3 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_3_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_3_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_3_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_3_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2041 \
    name p_anonymous_namespace_g_core_state_admm_y_x_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_4 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_4_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_4_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_4_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_4_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2042 \
    name p_anonymous_namespace_g_core_state_admm_y_x_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_5 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_5_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_5_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_5_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_5_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2043 \
    name p_anonymous_namespace_g_core_state_admm_y_x_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_6 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_6_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_6_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_6_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_6_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 2044 \
    name p_anonymous_namespace_g_core_state_admm_y_x_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename p_anonymous_namespace_g_core_state_admm_y_x_7 \
    op interface \
    ports { p_anonymous_namespace_g_core_state_admm_y_x_7_address0 { O 5 vector } p_anonymous_namespace_g_core_state_admm_y_x_7_ce0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_7_we0 { O 1 bit } p_anonymous_namespace_g_core_state_admm_y_x_7_d0 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'p_anonymous_namespace_g_core_state_admm_y_x_7'"
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


