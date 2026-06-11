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
    id 1921 \
    name z_u_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_u_1 \
    op interface \
    ports { z_u_1_address0 { O 5 vector } z_u_1_ce0 { O 1 bit } z_u_1_we0 { O 1 bit } z_u_1_d0 { O 26 vector } z_u_1_address1 { O 5 vector } z_u_1_ce1 { O 1 bit } z_u_1_q1 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_u_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1922 \
    name z_u \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_u \
    op interface \
    ports { z_u_address0 { O 5 vector } z_u_ce0 { O 1 bit } z_u_we0 { O 1 bit } z_u_d0 { O 26 vector } z_u_address1 { O 5 vector } z_u_ce1 { O 1 bit } z_u_q1 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_u'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1923 \
    name y_u_1 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_u_1 \
    op interface \
    ports { y_u_1_address0 { O 5 vector } y_u_1_ce0 { O 1 bit } y_u_1_we0 { O 1 bit } y_u_1_d0 { O 26 vector } y_u_1_address1 { O 5 vector } y_u_1_ce1 { O 1 bit } y_u_1_q1 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_u_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1924 \
    name y_u \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_u \
    op interface \
    ports { y_u_address0 { O 5 vector } y_u_ce0 { O 1 bit } y_u_we0 { O 1 bit } y_u_d0 { O 26 vector } y_u_address1 { O 5 vector } y_u_ce1 { O 1 bit } y_u_q1 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_u'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1925 \
    name y_x_5 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_x_5 \
    op interface \
    ports { y_x_5_address0 { O 5 vector } y_x_5_ce0 { O 1 bit } y_x_5_q0 { I 26 vector } y_x_5_address1 { O 5 vector } y_x_5_ce1 { O 1 bit } y_x_5_we1 { O 1 bit } y_x_5_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1926 \
    name y_x \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename y_x \
    op interface \
    ports { y_x_address0 { O 5 vector } y_x_ce0 { O 1 bit } y_x_q0 { I 26 vector } y_x_address1 { O 5 vector } y_x_ce1 { O 1 bit } y_x_we1 { O 1 bit } y_x_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'y_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1927 \
    name z_x_7 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_7 \
    op interface \
    ports { z_x_7_address1 { O 5 vector } z_x_7_ce1 { O 1 bit } z_x_7_we1 { O 1 bit } z_x_7_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1928 \
    name z_x_6 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_6 \
    op interface \
    ports { z_x_6_address1 { O 5 vector } z_x_6_ce1 { O 1 bit } z_x_6_we1 { O 1 bit } z_x_6_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1929 \
    name z_x_5 \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_x_5 \
    op interface \
    ports { z_x_5_address0 { O 5 vector } z_x_5_ce0 { O 1 bit } z_x_5_q0 { I 26 vector } z_x_5_address1 { O 5 vector } z_x_5_ce1 { O 1 bit } z_x_5_we1 { O 1 bit } z_x_5_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1930 \
    name z_x_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_4 \
    op interface \
    ports { z_x_4_address1 { O 5 vector } z_x_4_ce1 { O 1 bit } z_x_4_we1 { O 1 bit } z_x_4_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1931 \
    name z_x_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_3 \
    op interface \
    ports { z_x_3_address1 { O 5 vector } z_x_3_ce1 { O 1 bit } z_x_3_we1 { O 1 bit } z_x_3_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1932 \
    name z_x_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_2 \
    op interface \
    ports { z_x_2_address1 { O 5 vector } z_x_2_ce1 { O 1 bit } z_x_2_we1 { O 1 bit } z_x_2_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1933 \
    name z_x_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename z_x_1 \
    op interface \
    ports { z_x_1_address1 { O 5 vector } z_x_1_ce1 { O 1 bit } z_x_1_we1 { O 1 bit } z_x_1_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1934 \
    name z_x \
    reset_level 1 \
    sync_rst true \
    dir IO \
    corename z_x \
    op interface \
    ports { z_x_address0 { O 5 vector } z_x_ce0 { O 1 bit } z_x_q0 { I 26 vector } z_x_address1 { O 5 vector } z_x_ce1 { O 1 bit } z_x_we1 { O 1 bit } z_x_d1 { O 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'z_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1935 \
    name sol_x_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_1 \
    op interface \
    ports { sol_x_1_address0 { O 5 vector } sol_x_1_ce0 { O 1 bit } sol_x_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1936 \
    name sol_x_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_2 \
    op interface \
    ports { sol_x_2_address0 { O 5 vector } sol_x_2_ce0 { O 1 bit } sol_x_2_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1937 \
    name sol_x_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_3 \
    op interface \
    ports { sol_x_3_address0 { O 5 vector } sol_x_3_ce0 { O 1 bit } sol_x_3_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1938 \
    name sol_x_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_4 \
    op interface \
    ports { sol_x_4_address0 { O 5 vector } sol_x_4_ce0 { O 1 bit } sol_x_4_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1939 \
    name sol_x_6 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_6 \
    op interface \
    ports { sol_x_6_address0 { O 5 vector } sol_x_6_ce0 { O 1 bit } sol_x_6_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_6'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1940 \
    name sol_x_7 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_7 \
    op interface \
    ports { sol_x_7_address0 { O 5 vector } sol_x_7_ce0 { O 1 bit } sol_x_7_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_7'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1941 \
    name sol_x \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x \
    op interface \
    ports { sol_x_address0 { O 5 vector } sol_x_ce0 { O 1 bit } sol_x_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1942 \
    name sol_x_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_x_5 \
    op interface \
    ports { sol_x_5_address0 { O 5 vector } sol_x_5_ce0 { O 1 bit } sol_x_5_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_x_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1943 \
    name step_data_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_3 \
    op interface \
    ports { step_data_3_address0 { O 5 vector } step_data_3_ce0 { O 1 bit } step_data_3_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1944 \
    name step_data_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_4 \
    op interface \
    ports { step_data_4_address0 { O 5 vector } step_data_4_ce0 { O 1 bit } step_data_4_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1948 \
    name sol_u \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_u \
    op interface \
    ports { sol_u_address0 { O 5 vector } sol_u_ce0 { O 1 bit } sol_u_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_u'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1949 \
    name sol_u_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename sol_u_1 \
    op interface \
    ports { sol_u_1_address0 { O 5 vector } sol_u_1_ce0 { O 1 bit } sol_u_1_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'sol_u_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1951 \
    name step_data_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename step_data_5 \
    op interface \
    ports { step_data_5_address0 { O 5 vector } step_data_5_ce0 { O 1 bit } step_data_5_q0 { I 26 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'step_data_5'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1945 \
    name terminal_wall_x_lb_con_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_terminal_wall_x_lb_con_reload \
    op interface \
    ports { terminal_wall_x_lb_con_reload { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1946 \
    name terminal_wall_x_ub_con_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_terminal_wall_x_ub_con_reload \
    op interface \
    ports { terminal_wall_x_ub_con_reload { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1947 \
    name sext_ln156_35 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln156_35 \
    op interface \
    ports { sext_ln156_35 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1950 \
    name sext_ln1362 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln1362 \
    op interface \
    ports { sext_ln1362 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1952 \
    name lnorm_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_u1_out \
    op interface \
    ports { lnorm_u1_out { O 26 vector } lnorm_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1953 \
    name znorm_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_u1_out \
    op interface \
    ports { znorm_u1_out { O 26 vector } znorm_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1954 \
    name dual_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_u1_out \
    op interface \
    ports { dual_u1_out { O 26 vector } dual_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1955 \
    name primal_u1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_u1_out \
    op interface \
    ports { primal_u1_out { O 26 vector } primal_u1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1956 \
    name lnorm_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_u0_out \
    op interface \
    ports { lnorm_u0_out { O 26 vector } lnorm_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1957 \
    name znorm_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_u0_out \
    op interface \
    ports { znorm_u0_out { O 26 vector } znorm_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1958 \
    name dual_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_u0_out \
    op interface \
    ports { dual_u0_out { O 26 vector } dual_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1959 \
    name primal_u0_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_u0_out \
    op interface \
    ports { primal_u0_out { O 26 vector } primal_u0_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1960 \
    name lnorm_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_da_out \
    op interface \
    ports { lnorm_da_out { O 26 vector } lnorm_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1961 \
    name znorm_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_da_out \
    op interface \
    ports { znorm_da_out { O 25 vector } znorm_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1962 \
    name dual_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_da_out \
    op interface \
    ports { dual_da_out { O 26 vector } dual_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1963 \
    name primal_da_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_da_out \
    op interface \
    ports { primal_da_out { O 26 vector } primal_da_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1964 \
    name lnorm_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_lnorm_ey_out \
    op interface \
    ports { lnorm_ey_out { O 26 vector } lnorm_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1965 \
    name znorm_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_znorm_ey_out \
    op interface \
    ports { znorm_ey_out { O 26 vector } znorm_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1966 \
    name dual_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_dual_ey_out \
    op interface \
    ports { dual_ey_out { O 26 vector } dual_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1967 \
    name primal_ey_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_primal_ey_out \
    op interface \
    ports { primal_ey_out { O 26 vector } primal_ey_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1968 \
    name u_norm_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_u_norm_out \
    op interface \
    ports { u_norm_out { O 26 vector } u_norm_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1969 \
    name x_norm_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_x_norm_out \
    op interface \
    ports { x_norm_out { O 26 vector } x_norm_out_ap_vld { O 1 bit } } \
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


