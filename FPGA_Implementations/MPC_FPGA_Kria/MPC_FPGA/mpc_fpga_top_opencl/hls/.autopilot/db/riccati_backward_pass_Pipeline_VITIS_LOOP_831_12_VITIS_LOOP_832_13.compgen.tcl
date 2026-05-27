# This script segment is generated automatically by AutoPilot

if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_11_3_32_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {compactencoding_dontcare}
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
    id 1072 \
    name P \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P \
    op interface \
    ports { P_address0 { O 3 vector } P_ce0 { O 1 bit } P_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1073 \
    name P_1 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_1 \
    op interface \
    ports { P_1_address0 { O 3 vector } P_1_ce0 { O 1 bit } P_1_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1074 \
    name P_2 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_2 \
    op interface \
    ports { P_2_address0 { O 3 vector } P_2_ce0 { O 1 bit } P_2_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1075 \
    name P_3 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_3 \
    op interface \
    ports { P_3_address0 { O 3 vector } P_3_ce0 { O 1 bit } P_3_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1076 \
    name P_4 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_4 \
    op interface \
    ports { P_4_address0 { O 3 vector } P_4_ce0 { O 1 bit } P_4_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1077 \
    name P_5 \
    reset_level 1 \
    sync_rst true \
    dir I \
    corename P_5 \
    op interface \
    ports { P_5_address0 { O 3 vector } P_5_ce0 { O 1 bit } P_5_q0 { I 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'P_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1078 \
    name PA_5 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename PA_5 \
    op interface \
    ports { PA_5_address1 { O 3 vector } PA_5_ce1 { O 1 bit } PA_5_we1 { O 1 bit } PA_5_d1 { O 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'PA_5'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1079 \
    name PA_4 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename PA_4 \
    op interface \
    ports { PA_4_address1 { O 3 vector } PA_4_ce1 { O 1 bit } PA_4_we1 { O 1 bit } PA_4_d1 { O 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'PA_4'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1080 \
    name PA_3 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename PA_3 \
    op interface \
    ports { PA_3_address1 { O 3 vector } PA_3_ce1 { O 1 bit } PA_3_we1 { O 1 bit } PA_3_d1 { O 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'PA_3'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1081 \
    name PA_2 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename PA_2 \
    op interface \
    ports { PA_2_address1 { O 3 vector } PA_2_ce1 { O 1 bit } PA_2_we1 { O 1 bit } PA_2_d1 { O 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'PA_2'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1082 \
    name PA_1 \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename PA_1 \
    op interface \
    ports { PA_1_address1 { O 3 vector } PA_1_ce1 { O 1 bit } PA_1_we1 { O 1 bit } PA_1_d1 { O 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'PA_1'"
}
}


# XIL_BRAM:
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc ::AESL_LIB_XILADAPTER::xil_bram_gen] == "::AESL_LIB_XILADAPTER::xil_bram_gen"} {
eval "::AESL_LIB_XILADAPTER::xil_bram_gen { \
    id 1083 \
    name PA \
    reset_level 1 \
    sync_rst true \
    dir O \
    corename PA \
    op interface \
    ports { PA_address1 { O 3 vector } PA_ce1 { O 1 bit } PA_we1 { O 1 bit } PA_d1 { O 40 vector } } \
} "
} else {
puts "@W \[IMPL-110\] Cannot find bus interface model in the library. Ignored generation of bus interface for 'PA'"
}
}


# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1084 \
    name a_60 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_60 \
    op interface \
    ports { a_60 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1085 \
    name a_68 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_68 \
    op interface \
    ports { a_68 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1086 \
    name a_76 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_76 \
    op interface \
    ports { a_76 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1087 \
    name a_84 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_84 \
    op interface \
    ports { a_84 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1088 \
    name a_92 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_92 \
    op interface \
    ports { a_92 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1089 \
    name a_61 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_61 \
    op interface \
    ports { a_61 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1090 \
    name a_69 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_69 \
    op interface \
    ports { a_69 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1091 \
    name a_77 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_77 \
    op interface \
    ports { a_77 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1092 \
    name a_85 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_85 \
    op interface \
    ports { a_85 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1093 \
    name a_93 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_93 \
    op interface \
    ports { a_93 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1094 \
    name a_62 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_62 \
    op interface \
    ports { a_62 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1095 \
    name a_70 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_70 \
    op interface \
    ports { a_70 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1096 \
    name a_78 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_78 \
    op interface \
    ports { a_78 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1097 \
    name a_86 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_86 \
    op interface \
    ports { a_86 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1098 \
    name a_94 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_94 \
    op interface \
    ports { a_94 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1099 \
    name a_63 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_63 \
    op interface \
    ports { a_63 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1100 \
    name a_71 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_71 \
    op interface \
    ports { a_71 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1101 \
    name a_79 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_79 \
    op interface \
    ports { a_79 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1102 \
    name a_87 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_87 \
    op interface \
    ports { a_87 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1103 \
    name a_95 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_95 \
    op interface \
    ports { a_95 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1104 \
    name a_64 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_64 \
    op interface \
    ports { a_64 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1105 \
    name a_72 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_72 \
    op interface \
    ports { a_72 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1106 \
    name a_80 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_80 \
    op interface \
    ports { a_80 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1107 \
    name a_88 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_88 \
    op interface \
    ports { a_88 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1108 \
    name a_96 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_96 \
    op interface \
    ports { a_96 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1109 \
    name a_65 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_65 \
    op interface \
    ports { a_65 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1110 \
    name a_73 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_73 \
    op interface \
    ports { a_73 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1111 \
    name a_81 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_81 \
    op interface \
    ports { a_81 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1112 \
    name a_89 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_89 \
    op interface \
    ports { a_89 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1113 \
    name a_97 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_97 \
    op interface \
    ports { a_97 { I 32 vector } } \
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


