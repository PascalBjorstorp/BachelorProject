# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_26s_18s_44_2_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 1 ALLOW_PRAGMA 1
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
    id 863 \
    name s1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_s1 \
    op interface \
    ports { s1 { I 35 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 864 \
    name s0 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_s0 \
    op interface \
    ports { s0 { I 35 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 865 \
    name s1_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_s1_1 \
    op interface \
    ports { s1_1 { I 35 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 866 \
    name s0_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_s0_1 \
    op interface \
    ports { s0_1 { I 35 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 867 \
    name M_8 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_8 \
    op interface \
    ports { M_8 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 868 \
    name M \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M \
    op interface \
    ports { M { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 869 \
    name M_10 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_10 \
    op interface \
    ports { M_10 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 870 \
    name M_9 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_9 \
    op interface \
    ports { M_9 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 871 \
    name M_12 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_12 \
    op interface \
    ports { M_12 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 872 \
    name M_11 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_11 \
    op interface \
    ports { M_11 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 873 \
    name M_14 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_14 \
    op interface \
    ports { M_14 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 874 \
    name M_13 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_13 \
    op interface \
    ports { M_13 { I 18 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 875 \
    name b_215_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_215_cast \
    op interface \
    ports { b_215_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 876 \
    name b_216_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_216_cast \
    op interface \
    ports { b_216_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 877 \
    name b_217_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_217_cast \
    op interface \
    ports { b_217_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 878 \
    name b_218_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_218_cast \
    op interface \
    ports { b_218_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 879 \
    name b_219_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_219_cast \
    op interface \
    ports { b_219_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 880 \
    name b_220_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_220_cast \
    op interface \
    ports { b_220_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 881 \
    name b_158_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_158_cast \
    op interface \
    ports { b_158_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 882 \
    name b_159_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_159_cast \
    op interface \
    ports { b_159_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 883 \
    name b_160_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_160_cast \
    op interface \
    ports { b_160_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 884 \
    name b_161_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_161_cast \
    op interface \
    ports { b_161_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 885 \
    name b_162_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_162_cast \
    op interface \
    ports { b_162_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 886 \
    name b_163_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_163_cast \
    op interface \
    ports { b_163_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 887 \
    name b_164_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_164_cast \
    op interface \
    ports { b_164_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 888 \
    name b_165_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_165_cast \
    op interface \
    ports { b_165_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 889 \
    name b_166_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_166_cast \
    op interface \
    ports { b_166_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 890 \
    name b_167_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_167_cast \
    op interface \
    ports { b_167_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 891 \
    name b_168_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_168_cast \
    op interface \
    ports { b_168_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 892 \
    name b_169_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_169_cast \
    op interface \
    ports { b_169_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 893 \
    name b_170_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_170_cast \
    op interface \
    ports { b_170_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 894 \
    name b_171_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_171_cast \
    op interface \
    ports { b_171_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 895 \
    name b_172_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_172_cast \
    op interface \
    ports { b_172_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 896 \
    name b_173_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_173_cast \
    op interface \
    ports { b_173_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 897 \
    name b_174_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_174_cast \
    op interface \
    ports { b_174_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 898 \
    name b_175_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_175_cast \
    op interface \
    ports { b_175_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 899 \
    name b_176_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_176_cast \
    op interface \
    ports { b_176_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 900 \
    name b_177_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_177_cast \
    op interface \
    ports { b_177_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 901 \
    name b_178_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_178_cast \
    op interface \
    ports { b_178_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 902 \
    name b_179_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_179_cast \
    op interface \
    ports { b_179_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 903 \
    name b_180_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_180_cast \
    op interface \
    ports { b_180_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 904 \
    name b_181_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_181_cast \
    op interface \
    ports { b_181_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 905 \
    name b_182_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_182_cast \
    op interface \
    ports { b_182_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 906 \
    name b_183_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_183_cast \
    op interface \
    ports { b_183_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 907 \
    name b_184_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_184_cast \
    op interface \
    ports { b_184_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 908 \
    name b_185_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_185_cast \
    op interface \
    ports { b_185_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 909 \
    name b_186_cast \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_b_186_cast \
    op interface \
    ports { b_186_cast { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 910 \
    name sext_ln742 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln742 \
    op interface \
    ports { sext_ln742 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 911 \
    name G_12_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_12_out \
    op interface \
    ports { G_12_out { O 18 vector } G_12_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 912 \
    name G_11_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_11_out \
    op interface \
    ports { G_11_out { O 18 vector } G_11_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 913 \
    name G_10_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_10_out \
    op interface \
    ports { G_10_out { O 18 vector } G_10_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 914 \
    name G_9_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_9_out \
    op interface \
    ports { G_9_out { O 18 vector } G_9_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 915 \
    name G_8_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_8_out \
    op interface \
    ports { G_8_out { O 18 vector } G_8_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 916 \
    name G_7_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_7_out \
    op interface \
    ports { G_7_out { O 18 vector } G_7_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 917 \
    name G_6_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_6_out \
    op interface \
    ports { G_6_out { O 18 vector } G_6_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 918 \
    name G_5_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_5_out \
    op interface \
    ports { G_5_out { O 18 vector } G_5_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 919 \
    name G_4_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_4_out \
    op interface \
    ports { G_4_out { O 18 vector } G_4_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 920 \
    name G_3_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_3_out \
    op interface \
    ports { G_3_out { O 18 vector } G_3_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 921 \
    name G_2_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_2_out \
    op interface \
    ports { G_2_out { O 18 vector } G_2_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 922 \
    name G_1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_1_out \
    op interface \
    ports { G_1_out { O 18 vector } G_1_out_ap_vld { O 1 bit } } \
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


