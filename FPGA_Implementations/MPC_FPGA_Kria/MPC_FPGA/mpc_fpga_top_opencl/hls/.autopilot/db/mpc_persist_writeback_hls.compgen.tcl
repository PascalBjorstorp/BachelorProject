# This script segment is generated automatically by AutoPilot

# clear list
if {${::AESL::PGuard_autoexp_gen}} {
    cg_default_interface_gen_dc_begin
    cg_default_interface_gen_bundle_begin
    AESL_LIB_XILADAPTER::native_axis_begin
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2154 \
    name prev_steer_rate \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_prev_steer_rate \
    op interface \
    ports { prev_steer_rate { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2155 \
    name prev_accel \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_prev_accel \
    op interface \
    ports { prev_accel { I 22 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2156 \
    name prev_curvature \
    type other \
    dir I \
    reset_level 1 \
    sync_rst true \
    corename dc_prev_curvature \
    op interface \
    ports { prev_curvature { I 32 vector } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2157 \
    name p_anonymous_namespace_g_core_state_persist_prev_steer_rate \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_steer_rate \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_steer_rate { O 32 vector } p_anonymous_namespace_g_core_state_persist_prev_steer_rate_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2158 \
    name p_anonymous_namespace_g_core_state_persist_prev_accel \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_accel \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_accel { O 32 vector } p_anonymous_namespace_g_core_state_persist_prev_accel_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2159 \
    name p_anonymous_namespace_g_core_state_persist_prev_curvature \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_curvature \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_curvature { O 32 vector } p_anonymous_namespace_g_core_state_persist_prev_curvature_ap_vld { O 1 bit } } \
} "
}

# Direct connection:
if {${::AESL::PGuard_autoexp_gen}} {
eval "cg_default_interface_gen_dc { \
    id 2160 \
    name p_anonymous_namespace_g_core_state_persist_prev_model_signature \
    type other \
    dir O \
    reset_level 1 \
    sync_rst true \
    corename dc_p_anonymous_namespace_g_core_state_persist_prev_model_signature \
    op interface \
    ports { p_anonymous_namespace_g_core_state_persist_prev_model_signature { O 1 vector } p_anonymous_namespace_g_core_state_persist_prev_model_signature_ap_vld { O 1 bit } } \
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
    ports { ap_ready { O 1 bit } } \
} "
}


# Adapter definition:
set PortName ap_rst
set DataWd 1 
if {${::AESL::PGuard_autoexp_gen}} {
if {[info proc cg_default_interface_gen_reset] == "cg_default_interface_gen_reset"} {
eval "cg_default_interface_gen_reset { \
    id -2 \
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


