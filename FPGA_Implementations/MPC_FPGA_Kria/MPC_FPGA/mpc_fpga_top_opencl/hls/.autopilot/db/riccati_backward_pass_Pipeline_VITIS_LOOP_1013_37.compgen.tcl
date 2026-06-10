# This script segment is generated automatically by AutoPilot

set name mpc_fpga_top_opencl_mul_34s_26s_57_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
}


set name mpc_fpga_top_opencl_mul_40s_32s_57_4_1
if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler $name BINDTYPE {op} TYPE {mul} IMPL {dsp} LATENCY 3 ALLOW_PRAGMA 1
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_13_3_34_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {compactencoding_dontcare}
}


if {${::AESL::PGuard_rtl_comp_handler}} {
	::AP::rtl_comp_handler mpc_fpga_top_opencl_sparsemux_13_3_35_1_1 BINDTYPE {op} TYPE {sparsemux} IMPL {compactencoding_dontcare}
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
    id 1151 \
    name a_52 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_52 \
    op interface \
    ports { a_52 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1152 \
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
    id 1153 \
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
    id 1154 \
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
    id 1155 \
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
    id 1156 \
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
    id 1157 \
    name sext_ln267_121 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln267_121 \
    op interface \
    ports { sext_ln267_121 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1158 \
    name a_53 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_53 \
    op interface \
    ports { a_53 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1159 \
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
    id 1160 \
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
    id 1161 \
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
    id 1162 \
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
    id 1163 \
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
    id 1164 \
    name sext_ln291_41 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln291_41 \
    op interface \
    ports { sext_ln291_41 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1165 \
    name a_54 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_54 \
    op interface \
    ports { a_54 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1166 \
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
    id 1167 \
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
    id 1168 \
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
    id 1169 \
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
    id 1170 \
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
    id 1171 \
    name sext_ln267_47 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln267_47 \
    op interface \
    ports { sext_ln267_47 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1172 \
    name a_55 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_55 \
    op interface \
    ports { a_55 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1173 \
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
    id 1174 \
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
    id 1175 \
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
    id 1176 \
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
    id 1177 \
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
    id 1178 \
    name sext_ln267_49 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln267_49 \
    op interface \
    ports { sext_ln267_49 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1179 \
    name a_56 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_56 \
    op interface \
    ports { a_56 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1180 \
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
    id 1181 \
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
    id 1182 \
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
    id 1183 \
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
    id 1184 \
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
    id 1185 \
    name sext_ln267_51 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln267_51 \
    op interface \
    ports { sext_ln267_51 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1186 \
    name a_57 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_a_57 \
    op interface \
    ports { a_57 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1187 \
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
    id 1188 \
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
    id 1189 \
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
    id 1190 \
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
    id 1191 \
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
    id 1192 \
    name sext_ln267_45 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln267_45 \
    op interface \
    ports { sext_ln267_45 { I 40 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1193 \
    name G_11_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_11_reload \
    op interface \
    ports { G_11_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1194 \
    name G_12_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_12_reload \
    op interface \
    ports { G_12_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1195 \
    name G_13_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_13_reload \
    op interface \
    ports { G_13_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1196 \
    name G_10_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_10_reload \
    op interface \
    ports { G_10_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1197 \
    name G_9_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_9_reload \
    op interface \
    ports { G_9_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1198 \
    name G_8_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_8_reload \
    op interface \
    ports { G_8_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1199 \
    name sext_ln291_43 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln291_43 \
    op interface \
    ports { sext_ln291_43 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1200 \
    name G_7_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_7_reload \
    op interface \
    ports { G_7_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1201 \
    name G_6_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_6_reload \
    op interface \
    ports { G_6_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1202 \
    name G_5_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_5_reload \
    op interface \
    ports { G_5_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1203 \
    name G_4_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_4_reload \
    op interface \
    ports { G_4_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1204 \
    name G_3_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_3_reload \
    op interface \
    ports { G_3_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1205 \
    name G_2_reload \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_G_2_reload \
    op interface \
    ports { G_2_reload { I 34 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1206 \
    name sext_ln1013_1 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln1013_1 \
    op interface \
    ports { sext_ln1013_1 { I 26 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1207 \
    name q_aug_linear_4 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_q_aug_linear_4 \
    op interface \
    ports { q_aug_linear_4 { I 35 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1208 \
    name sext_ln572 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln572 \
    op interface \
    ports { sext_ln572 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1209 \
    name sext_ln573 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln573 \
    op interface \
    ports { sext_ln573 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1210 \
    name sext_ln574 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln574 \
    op interface \
    ports { sext_ln574 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1211 \
    name sext_ln575 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_sext_ln575 \
    op interface \
    ports { sext_ln575 { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1212 \
    name q_aug_linear_5 \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_q_aug_linear_5 \
    op interface \
    ports { q_aug_linear_5 { I 35 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1213 \
    name p_new_5_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_new_5_out \
    op interface \
    ports { p_new_5_out { O 40 vector } p_new_5_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1214 \
    name p_new_4_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_new_4_out \
    op interface \
    ports { p_new_4_out { O 40 vector } p_new_4_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1215 \
    name p_new_3_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_new_3_out \
    op interface \
    ports { p_new_3_out { O 40 vector } p_new_3_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1216 \
    name p_new_2_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_new_2_out \
    op interface \
    ports { p_new_2_out { O 40 vector } p_new_2_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1217 \
    name p_new_1_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_new_1_out \
    op interface \
    ports { p_new_1_out { O 40 vector } p_new_1_out_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 1218 \
    name p_new_out \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_new_out \
    op interface \
    ports { p_new_out { O 40 vector } p_new_out_ap_vld { O 1 bit } } \
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


