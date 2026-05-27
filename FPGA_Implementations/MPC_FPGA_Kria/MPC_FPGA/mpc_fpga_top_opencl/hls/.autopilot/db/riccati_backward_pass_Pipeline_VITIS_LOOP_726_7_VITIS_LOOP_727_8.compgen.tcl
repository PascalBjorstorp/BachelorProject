# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_34s_32s_52_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_13_3_32_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {compactencoding_dontcare}
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
    id 866 \
    name sext_ln167_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln167_1 \
    op interface \
    ports { sext_ln167_1 { I 33 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 867 \
    name M \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M \
    op interface \
    ports { M { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 868 \
    name sext_ln167_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln167_2 \
    op interface \
    ports { sext_ln167_2 { I 33 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 869 \
    name M_2 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_2 \
    op interface \
    ports { M_2 { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 870 \
    name sext_ln167_3 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln167_3 \
    op interface \
    ports { sext_ln167_3 { I 33 vector } } \
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
    ports { M_12 { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 872 \
    name sext_ln167_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln167_4 \
    op interface \
    ports { sext_ln167_4 { I 33 vector } } \
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
    ports { M_14 { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 874 \
    name sext_ln167_5 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln167_5 \
    op interface \
    ports { sext_ln167_5 { I 33 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 875 \
    name M_16 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_16 \
    op interface \
    ports { M_16 { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 876 \
    name sext_ln167_6 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln167_6 \
    op interface \
    ports { sext_ln167_6 { I 33 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 877 \
    name M_18 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_M_18 \
    op interface \
    ports { M_18 { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 878 \
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
    id 879 \
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
    id 880 \
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
    id 881 \
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
    id 882 \
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
    id 883 \
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
    id 884 \
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
    id 885 \
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
    id 886 \
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
    id 887 \
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
    id 888 \
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
    id 889 \
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
    id 890 \
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
    id 891 \
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
    id 892 \
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
    id 893 \
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
    id 894 \
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
    id 895 \
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
    id 896 \
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
    id 897 \
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
    id 898 \
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
    id 899 \
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
    id 900 \
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
    id 901 \
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
    id 902 \
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
    id 903 \
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
    id 904 \
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
    id 905 \
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
    id 906 \
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
    id 907 \
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
    id 908 \
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
    id 909 \
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
    id 910 \
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
    id 911 \
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
    id 912 \
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
    id 913 \
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
    id 914 \
    name G_13_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_13_out \
    op interface \
    ports { G_13_out { O 34 vector } G_13_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 915 \
    name G_12_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_12_out \
    op interface \
    ports { G_12_out { O 34 vector } G_12_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 916 \
    name G_11_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_11_out \
    op interface \
    ports { G_11_out { O 34 vector } G_11_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 917 \
    name G_10_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_10_out \
    op interface \
    ports { G_10_out { O 34 vector } G_10_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 918 \
    name G_9_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_9_out \
    op interface \
    ports { G_9_out { O 34 vector } G_9_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 919 \
    name G_8_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_8_out \
    op interface \
    ports { G_8_out { O 34 vector } G_8_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 920 \
    name G_7_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_7_out \
    op interface \
    ports { G_7_out { O 34 vector } G_7_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 921 \
    name G_6_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_6_out \
    op interface \
    ports { G_6_out { O 34 vector } G_6_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 922 \
    name G_5_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_5_out \
    op interface \
    ports { G_5_out { O 34 vector } G_5_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 923 \
    name G_4_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_4_out \
    op interface \
    ports { G_4_out { O 34 vector } G_4_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 924 \
    name G_3_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_3_out \
    op interface \
    ports { G_3_out { O 34 vector } G_3_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 925 \
    name G_2_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_G_2_out \
    op interface \
    ports { G_2_out { O 34 vector } G_2_out_ap_vld { O 1 bit } } \
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


