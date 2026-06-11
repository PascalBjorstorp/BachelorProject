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
    id 1116 \
    name P \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P \
    op interface \
    ports { P_address0 { O 3 vector } P_ce0 { O 1 bit } P_q0 { I 27 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1117 \
    name P_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_1 \
    op interface \
    ports { P_1_address0 { O 3 vector } P_1_ce0 { O 1 bit } P_1_q0 { I 27 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1118 \
    name P_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_2 \
    op interface \
    ports { P_2_address0 { O 3 vector } P_2_ce0 { O 1 bit } P_2_q0 { I 27 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1119 \
    name P_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_3 \
    op interface \
    ports { P_3_address0 { O 3 vector } P_3_ce0 { O 1 bit } P_3_q0 { I 27 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1120 \
    name P_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_4 \
    op interface \
    ports { P_4_address0 { O 3 vector } P_4_ce0 { O 1 bit } P_4_q0 { I 27 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1121 \
    name P_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_5 \
    op interface \
    ports { P_5_address0 { O 3 vector } P_5_ce0 { O 1 bit } P_5_q0 { I 27 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_5'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1086 \
    name PA_5_load_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_5_load_4 \
    op interface \
    ports { PA_5_load_4 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1087 \
    name PA_5_load_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_5_load_3 \
    op interface \
    ports { PA_5_load_3 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1088 \
    name PA_5_load_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_5_load_2 \
    op interface \
    ports { PA_5_load_2 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1089 \
    name PA_5_load_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_5_load_1 \
    op interface \
    ports { PA_5_load_1 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1090 \
    name PA_5_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_5_load \
    op interface \
    ports { PA_5_load { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1091 \
    name PA_load_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_load_4 \
    op interface \
    ports { PA_load_4 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1092 \
    name PA_load_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_load_3 \
    op interface \
    ports { PA_load_3 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1093 \
    name PA_load_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_load_2 \
    op interface \
    ports { PA_load_2 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1094 \
    name PA_load_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_load_1 \
    op interface \
    ports { PA_load_1 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1095 \
    name PA_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_load \
    op interface \
    ports { PA_load { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1096 \
    name PA_1_load_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_1_load_4 \
    op interface \
    ports { PA_1_load_4 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1097 \
    name PA_1_load_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_1_load_3 \
    op interface \
    ports { PA_1_load_3 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1098 \
    name PA_1_load_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_1_load_2 \
    op interface \
    ports { PA_1_load_2 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1099 \
    name PA_1_load_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_1_load_1 \
    op interface \
    ports { PA_1_load_1 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1100 \
    name PA_1_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_1_load \
    op interface \
    ports { PA_1_load { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1101 \
    name PA_2_load_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_2_load_4 \
    op interface \
    ports { PA_2_load_4 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1102 \
    name PA_2_load_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_2_load_3 \
    op interface \
    ports { PA_2_load_3 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1103 \
    name PA_2_load_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_2_load_2 \
    op interface \
    ports { PA_2_load_2 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1104 \
    name PA_2_load_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_2_load_1 \
    op interface \
    ports { PA_2_load_1 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1105 \
    name PA_2_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_2_load \
    op interface \
    ports { PA_2_load { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1106 \
    name PA_3_load_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_3_load_4 \
    op interface \
    ports { PA_3_load_4 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1107 \
    name PA_3_load_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_3_load_3 \
    op interface \
    ports { PA_3_load_3 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1108 \
    name PA_3_load_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_3_load_2 \
    op interface \
    ports { PA_3_load_2 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1109 \
    name PA_3_load_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_3_load_1 \
    op interface \
    ports { PA_3_load_1 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1110 \
    name PA_3_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_3_load \
    op interface \
    ports { PA_3_load { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1111 \
    name PA_4_load_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_4_load_4 \
    op interface \
    ports { PA_4_load_4 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1112 \
    name PA_4_load_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_4_load_3 \
    op interface \
    ports { PA_4_load_3 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1113 \
    name PA_4_load_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_4_load_2 \
    op interface \
    ports { PA_4_load_2 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1114 \
    name PA_4_load_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_4_load_1 \
    op interface \
    ports { PA_4_load_1 { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1115 \
    name PA_4_load \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_PA_4_load \
    op interface \
    ports { PA_4_load { I 27 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1122 \
    name sext_ln256_5 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_5 \
    op interface \
    ports { sext_ln256_5 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1123 \
    name sext_ln256_7 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_7 \
    op interface \
    ports { sext_ln256_7 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1124 \
    name sext_ln256_9 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_9 \
    op interface \
    ports { sext_ln256_9 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1125 \
    name sext_ln256_11 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_11 \
    op interface \
    ports { sext_ln256_11 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1126 \
    name sext_ln256_13 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_13 \
    op interface \
    ports { sext_ln256_13 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1127 \
    name sext_ln256_15 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_15 \
    op interface \
    ports { sext_ln256_15 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1128 \
    name sext_ln256_17 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_17 \
    op interface \
    ports { sext_ln256_17 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1129 \
    name sext_ln256_19 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_19 \
    op interface \
    ports { sext_ln256_19 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1130 \
    name sext_ln256_21 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_21 \
    op interface \
    ports { sext_ln256_21 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1131 \
    name sext_ln256_23 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_23 \
    op interface \
    ports { sext_ln256_23 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1132 \
    name sext_ln256_25 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_25 \
    op interface \
    ports { sext_ln256_25 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1133 \
    name sext_ln256_27 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_27 \
    op interface \
    ports { sext_ln256_27 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1134 \
    name sext_ln256_29 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_29 \
    op interface \
    ports { sext_ln256_29 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1135 \
    name sext_ln256_31 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_31 \
    op interface \
    ports { sext_ln256_31 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1136 \
    name sext_ln256_33 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_33 \
    op interface \
    ports { sext_ln256_33 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1137 \
    name sext_ln256_35 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_35 \
    op interface \
    ports { sext_ln256_35 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1138 \
    name sext_ln256_37 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_37 \
    op interface \
    ports { sext_ln256_37 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1139 \
    name sext_ln256_39 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_39 \
    op interface \
    ports { sext_ln256_39 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1140 \
    name sext_ln256_41 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_41 \
    op interface \
    ports { sext_ln256_41 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1141 \
    name sext_ln256_43 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_43 \
    op interface \
    ports { sext_ln256_43 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1142 \
    name sext_ln256_45 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_45 \
    op interface \
    ports { sext_ln256_45 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1143 \
    name sext_ln256_47 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_47 \
    op interface \
    ports { sext_ln256_47 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1144 \
    name sext_ln256_49 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_49 \
    op interface \
    ports { sext_ln256_49 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1145 \
    name sext_ln256_51 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_51 \
    op interface \
    ports { sext_ln256_51 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1146 \
    name sext_ln256_53 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_53 \
    op interface \
    ports { sext_ln256_53 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1147 \
    name sext_ln256_55 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_55 \
    op interface \
    ports { sext_ln256_55 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1148 \
    name sext_ln256_57 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_57 \
    op interface \
    ports { sext_ln256_57 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1149 \
    name sext_ln256_59 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_59 \
    op interface \
    ports { sext_ln256_59 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1150 \
    name sext_ln256_61 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln256_61 \
    op interface \
    ports { sext_ln256_61 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1151 \
    name sext_ln848 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln848 \
    op interface \
    ports { sext_ln848 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1152 \
    name conv_i_i13988_459_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_459_out \
    op interface \
    ports { conv_i_i13988_459_out { O 27 vector } conv_i_i13988_459_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1153 \
    name conv_i_i13988_357_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_357_out \
    op interface \
    ports { conv_i_i13988_357_out { O 27 vector } conv_i_i13988_357_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1154 \
    name conv_i_i13988_255_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_255_out \
    op interface \
    ports { conv_i_i13988_255_out { O 27 vector } conv_i_i13988_255_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1155 \
    name conv_i_i13988_153_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_153_out \
    op interface \
    ports { conv_i_i13988_153_out { O 27 vector } conv_i_i13988_153_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1156 \
    name conv_i_i1398851_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i1398851_out \
    op interface \
    ports { conv_i_i1398851_out { O 27 vector } conv_i_i1398851_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1157 \
    name conv_i_i13988_449_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_449_out \
    op interface \
    ports { conv_i_i13988_449_out { O 27 vector } conv_i_i13988_449_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1158 \
    name conv_i_i13988_347_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_347_out \
    op interface \
    ports { conv_i_i13988_347_out { O 27 vector } conv_i_i13988_347_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1159 \
    name conv_i_i13988_245_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_245_out \
    op interface \
    ports { conv_i_i13988_245_out { O 27 vector } conv_i_i13988_245_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1160 \
    name conv_i_i13988_143_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_143_out \
    op interface \
    ports { conv_i_i13988_143_out { O 27 vector } conv_i_i13988_143_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1161 \
    name conv_i_i1398841_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i1398841_out \
    op interface \
    ports { conv_i_i1398841_out { O 27 vector } conv_i_i1398841_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1162 \
    name conv_i_i13988_439_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_439_out \
    op interface \
    ports { conv_i_i13988_439_out { O 27 vector } conv_i_i13988_439_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1163 \
    name conv_i_i13988_337_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_337_out \
    op interface \
    ports { conv_i_i13988_337_out { O 27 vector } conv_i_i13988_337_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1164 \
    name conv_i_i13988_235_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_235_out \
    op interface \
    ports { conv_i_i13988_235_out { O 27 vector } conv_i_i13988_235_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1165 \
    name conv_i_i13988_133_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_133_out \
    op interface \
    ports { conv_i_i13988_133_out { O 27 vector } conv_i_i13988_133_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1166 \
    name conv_i_i1398831_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i1398831_out \
    op interface \
    ports { conv_i_i1398831_out { O 27 vector } conv_i_i1398831_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1167 \
    name conv_i_i13988_429_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_429_out \
    op interface \
    ports { conv_i_i13988_429_out { O 27 vector } conv_i_i13988_429_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1168 \
    name conv_i_i13988_327_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_327_out \
    op interface \
    ports { conv_i_i13988_327_out { O 27 vector } conv_i_i13988_327_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1169 \
    name conv_i_i13988_225_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_225_out \
    op interface \
    ports { conv_i_i13988_225_out { O 27 vector } conv_i_i13988_225_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1170 \
    name conv_i_i13988_123_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_123_out \
    op interface \
    ports { conv_i_i13988_123_out { O 27 vector } conv_i_i13988_123_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1171 \
    name conv_i_i1398821_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i1398821_out \
    op interface \
    ports { conv_i_i1398821_out { O 27 vector } conv_i_i1398821_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1172 \
    name conv_i_i13988_419_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_419_out \
    op interface \
    ports { conv_i_i13988_419_out { O 27 vector } conv_i_i13988_419_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1173 \
    name conv_i_i13988_317_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_317_out \
    op interface \
    ports { conv_i_i13988_317_out { O 27 vector } conv_i_i13988_317_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1174 \
    name conv_i_i13988_215_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_215_out \
    op interface \
    ports { conv_i_i13988_215_out { O 27 vector } conv_i_i13988_215_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1175 \
    name conv_i_i13988_113_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_113_out \
    op interface \
    ports { conv_i_i13988_113_out { O 27 vector } conv_i_i13988_113_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1176 \
    name conv_i_i1398811_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i1398811_out \
    op interface \
    ports { conv_i_i1398811_out { O 27 vector } conv_i_i1398811_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1177 \
    name conv_i_i13988_49_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_49_out \
    op interface \
    ports { conv_i_i13988_49_out { O 27 vector } conv_i_i13988_49_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1178 \
    name conv_i_i13988_37_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_37_out \
    op interface \
    ports { conv_i_i13988_37_out { O 27 vector } conv_i_i13988_37_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1179 \
    name conv_i_i13988_25_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_25_out \
    op interface \
    ports { conv_i_i13988_25_out { O 27 vector } conv_i_i13988_25_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1180 \
    name conv_i_i13988_13_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i13988_13_out \
    op interface \
    ports { conv_i_i13988_13_out { O 27 vector } conv_i_i13988_13_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1181 \
    name conv_i_i139881_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_conv_i_i139881_out \
    op interface \
    ports { conv_i_i139881_out { O 27 vector } conv_i_i139881_out_ap_vld { O 1 bit } } \
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


