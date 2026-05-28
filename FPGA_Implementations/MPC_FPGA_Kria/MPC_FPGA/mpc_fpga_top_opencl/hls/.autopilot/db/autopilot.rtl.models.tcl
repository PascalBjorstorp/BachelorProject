set SynModuleInfo {
  {SRCNAME {(anonymous namespace)unpack_input_lane_words_Pipeline_VITIS_LOOP_216_1} MODELNAME p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_216_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_216_1
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init RTLNAME mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init BINDTYPE interface TYPE internal_upc_flow_control INSTNAME mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init_U}
    }
  }
  {SRCNAME {(anonymous namespace)unpack_input_lane_words} MODELNAME p_anonymous_namespace_unpack_input_lane_words RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_unpack_input_lane_words}
  {SRCNAME {(anonymous namespace)fill_mpc_reference_trajectory_from_lane_words} MODELNAME p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words}
  {SRCNAME {(anonymous namespace)reset_core_state_hls_Pipeline_VITIS_LOOP_364_1} MODELNAME p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_364_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_364_1}
  {SRCNAME {(anonymous namespace)reset_core_state_hls_Pipeline_VITIS_LOOP_372_3} MODELNAME p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_372_3 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_372_3}
  {SRCNAME {(anonymous namespace)reset_core_state_hls} MODELNAME p_anonymous_namespace_reset_core_state_hls RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_reset_core_state_hls}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_390_1} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_390_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_390_1}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_397_3} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_397_3 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_397_3}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_364_1} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_364_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_364_1}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_372_3} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_372_3 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_372_3}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_356_1 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_356_1 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_356_1}
  {SRCNAME compute_window_min_hls MODELNAME compute_window_min_hls RTLNAME mpc_fpga_top_opencl_compute_window_min_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_35_5_32_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_35_5_32_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_37_5_32_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_37_5_32_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_39_5_32_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_39_5_32_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_41_5_32_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_41_5_32_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_366_2 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_366_2 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_366_2}
  {SRCNAME fp_atan_lut MODELNAME fp_atan_lut RTLNAME mpc_fpga_top_opencl_fp_atan_lut
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_32s_20s_51_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_20s_51_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_atan_lut_atan_lut_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_atan_lut_atan_lut_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_recip_fn MODELNAME fp_recip_fn RTLNAME mpc_fpga_top_opencl_fp_recip_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_ctlz_26_26_1_0 RTLNAME mpc_fpga_top_opencl_ctlz_26_26_1_0 BINDTYPE op TYPE ctlz IMPL auto}
      {MODELNAME mpc_fpga_top_opencl_mul_19s_8ns_27_1_0 RTLNAME mpc_fpga_top_opencl_mul_19s_8ns_27_1_0 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_7_2_23_1_0 RTLNAME mpc_fpga_top_opencl_sparsemux_7_2_23_1_0 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
      {MODELNAME mpc_fpga_top_opencl_fp_recip_fn_recip_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_recip_fn_recip_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_trig_pair_fused_fn MODELNAME fp_trig_pair_fused_fn RTLNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20s_26s_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_20s_26s_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_25ns_27ns_51_4_1 RTLNAME mpc_fpga_top_opencl_mul_25ns_27ns_51_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_19s_45_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_19s_45_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_sin_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_sin_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_cos_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_cos_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_slip_terms MODELNAME fp_slip_terms RTLNAME mpc_fpga_top_opencl_fp_slip_terms
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_23s_26s_49_4_1 RTLNAME mpc_fpga_top_opencl_mul_23s_26s_49_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_23s_49_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_23s_49_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_16ns_42_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_16ns_42_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_atan_lut_fn MODELNAME fp_atan_lut_fn RTLNAME mpc_fpga_top_opencl_fp_atan_lut_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_fp_atan_lut_fn_atan_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_atan_lut_fn_atan_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_pacejka_ceff MODELNAME fp_pacejka_ceff RTLNAME mpc_fpga_top_opencl_fp_pacejka_ceff
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_23s_26s_49_4_1_x RTLNAME mpc_fpga_top_opencl_mul_23s_26s_49_4_1_x BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_43_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_26s_43_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_front_force_jacobians_fn MODELNAME fp_front_force_jacobians_fn RTLNAME mpc_fpga_top_opencl_fp_front_force_jacobians_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_23ns_23s_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_23ns_23s_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_23s_25ns_48_4_1 RTLNAME mpc_fpga_top_opencl_mul_23s_25ns_48_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_43_4_1_x RTLNAME mpc_fpga_top_opencl_mul_26s_26s_43_4_1_x BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_25ns_16ns_40_4_1 RTLNAME mpc_fpga_top_opencl_mul_25ns_16ns_40_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_front_tire_path_fn MODELNAME compute_front_tire_path_fn RTLNAME mpc_fpga_top_opencl_compute_front_tire_path_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_20ns_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_20ns_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_21ns_47_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_21ns_47_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_19ns_45_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_19ns_45_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_rear_force_jacobians_fn MODELNAME fp_rear_force_jacobians_fn RTLNAME mpc_fpga_top_opencl_fp_rear_force_jacobians_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_25s_26s_43_4_1 RTLNAME mpc_fpga_top_opencl_mul_25s_26s_43_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_rear_tire_path_fn MODELNAME compute_rear_tire_path_fn RTLNAME mpc_fpga_top_opencl_compute_rear_tire_path_fn}
  {SRCNAME compute_frenet_tire_hls MODELNAME compute_frenet_tire_hls RTLNAME mpc_fpga_top_opencl_compute_frenet_tire_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_25ns_25ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_25ns_25ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_42_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_26s_42_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_18ns_44_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_18ns_44_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_frenet_recip MODELNAME fp_frenet_recip RTLNAME mpc_fpga_top_opencl_fp_frenet_recip}
  {SRCNAME fp_frenet_rows01_fn MODELNAME fp_frenet_rows01_fn RTLNAME mpc_fpga_top_opencl_fp_frenet_rows01_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_22s_25ns_47_4_1 RTLNAME mpc_fpga_top_opencl_mul_22s_25ns_47_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_22s_26s_43_4_1 RTLNAME mpc_fpga_top_opencl_mul_22s_26s_43_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_23s_23s_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_23s_23s_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_25ns_26s_51_4_1 RTLNAME mpc_fpga_top_opencl_mul_25ns_26s_51_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_22s_43_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_22s_43_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_25ns_51_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_25ns_51_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_13ns_39_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_13ns_39_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_rollout_from_forces_fn MODELNAME fp_rollout_from_forces_fn RTLNAME mpc_fpga_top_opencl_fp_rollout_from_forces_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_23ns_49_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_23ns_49_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_17ns_43_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_17ns_43_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_am_addmul_26s_26s_16ns_43_4_1 RTLNAME mpc_fpga_top_opencl_am_addmul_26s_26s_16ns_43_4_1 BINDTYPE op TYPE all IMPL dsp_slice LATENCY 3}
    }
  }
  {SRCNAME compute_stage_shared_products MODELNAME compute_stage_shared_products RTLNAME mpc_fpga_top_opencl_compute_stage_shared_products
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_12ns_38_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_12ns_38_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_B_steering MODELNAME compute_B_steering RTLNAME mpc_fpga_top_opencl_compute_B_steering
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_12s_38_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_12s_38_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_am_submul_26s_26s_15ns_42_4_1 RTLNAME mpc_fpga_top_opencl_am_submul_26s_26s_15ns_42_4_1 BINDTYPE op TYPE all IMPL dsp_slice LATENCY 3}
    }
  }
  {SRCNAME compute_B_accel_load_transfer MODELNAME compute_B_accel_load_transfer RTLNAME mpc_fpga_top_opencl_compute_B_accel_load_transfer
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_18s_44_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_18s_44_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_16s_42_4_1 RTLNAME mpc_fpga_top_opencl_mul_26s_16s_42_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_am_addmul_26s_26s_11ns_38_4_1 RTLNAME mpc_fpga_top_opencl_am_addmul_26s_26s_11ns_38_4_1 BINDTYPE op TYPE all IMPL dsp_slice LATENCY 3}
    }
  }
  {SRCNAME compute_frenet_AB_hls MODELNAME compute_frenet_AB_hls RTLNAME mpc_fpga_top_opencl_compute_frenet_AB_hls}
  {SRCNAME compute_frenet_AB_and_next_hls MODELNAME compute_frenet_AB_and_next_hls RTLNAME mpc_fpga_top_opencl_compute_frenet_AB_and_next_hls}
  {SRCNAME affine_b0_term_hls MODELNAME affine_b0_term_hls RTLNAME mpc_fpga_top_opencl_affine_b0_term_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_18s_32s_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_18s_32s_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_affine_bias_dense_hls MODELNAME compute_affine_bias_dense_hls RTLNAME mpc_fpga_top_opencl_compute_affine_bias_dense_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_32s_32s_43_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_32s_43_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_14ns_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_14ns_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_recip MODELNAME fp_recip RTLNAME mpc_fpga_top_opencl_fp_recip
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_ctlz_32_32_1_1 RTLNAME mpc_fpga_top_opencl_ctlz_32_32_1_1 BINDTYPE op TYPE ctlz IMPL auto}
      {MODELNAME mpc_fpga_top_opencl_mul_20s_9ns_29_1_1 RTLNAME mpc_fpga_top_opencl_mul_20s_9ns_29_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_recip_recip_lut_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_recip_recip_lut_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_371_3 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_371_3 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_371_3
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_27s_22ns_49_4_1 RTLNAME mpc_fpga_top_opencl_mul_27s_22ns_49_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_31ns_19ns_49_4_1 RTLNAME mpc_fpga_top_opencl_mul_31ns_19ns_49_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_31s_22ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_31s_22ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_364_1 MODELNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_364_1 RTLNAME mpc_fpga_top_opencl_reset_admm_state_hls_Pipeline_VITIS_LOOP_364_1}
  {SRCNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_372_3 MODELNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_372_3 RTLNAME mpc_fpga_top_opencl_reset_admm_state_hls_Pipeline_VITIS_LOOP_372_3}
  {SRCNAME reset_admm_state_hls MODELNAME reset_admm_state_hls RTLNAME mpc_fpga_top_opencl_reset_admm_state_hls}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1183_1 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1183_1 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1183_1}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1195_4 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1195_4 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1195_4}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1203_6 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1203_6 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1203_6}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9}
  {SRCNAME invert_2x2_qp_hls MODELNAME invert_2x2_qp_hls RTLNAME mpc_fpga_top_opencl_invert_2x2_qp_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_27s_29s_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_27s_29s_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_27s_32s_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_27s_32s_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_28s_28s_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_28s_28s_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_13_5_3_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_13_5_3_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
    }
  }
  {SRCNAME sum6_MG_QP_raw MODELNAME sum6_MG_QP_raw RTLNAME mpc_fpga_top_opencl_sum6_MG_QP_raw}
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_34s_32s_52_4_1 RTLNAME mpc_fpga_top_opencl_mul_34s_32s_52_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_13_3_32_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_13_3_32_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_750_9 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_750_9 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_750_9
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_17_3_34_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_17_3_34_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME sum6_P_QP_raw MODELNAME sum6_P_QP_raw RTLNAME mpc_fpga_top_opencl_sum6_P_QP_raw}
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_766_10 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_766_10 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_766_10
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_17_3_40_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_17_3_40_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_11_3_32_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_11_3_32_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME sum8_P_MIX_raw_pupdate MODELNAME sum8_P_MIX_raw_pupdate RTLNAME mpc_fpga_top_opencl_sum8_P_MIX_raw_pupdate}
  {SRCNAME sum8_P_MIX_raw MODELNAME sum8_P_MIX_raw RTLNAME mpc_fpga_top_opencl_sum8_P_MIX_raw}
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_34s_26s_57_4_1 RTLNAME mpc_fpga_top_opencl_mul_34s_26s_57_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_40s_32s_57_4_1 RTLNAME mpc_fpga_top_opencl_mul_40s_32s_57_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_13_3_34_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_13_3_34_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_13_3_35_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_13_3_35_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_1029_38 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_1029_38 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_1029_38}
  {SRCNAME riccati_backward_pass MODELNAME riccati_backward_pass RTLNAME mpc_fpga_top_opencl_riccati_backward_pass
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_31s_32s_45_4_1 RTLNAME mpc_fpga_top_opencl_mul_31s_32s_45_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_31s_32s_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_31s_32s_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_32s_52_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_32s_52_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_33s_32s_45_4_1 RTLNAME mpc_fpga_top_opencl_mul_33s_32s_45_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_33s_32s_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_33s_32s_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_34s_26s_49_4_1 RTLNAME mpc_fpga_top_opencl_mul_34s_26s_49_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_34s_26s_58_4_1 RTLNAME mpc_fpga_top_opencl_mul_34s_26s_58_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_34s_32s_44_4_1 RTLNAME mpc_fpga_top_opencl_mul_34s_32s_44_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_34s_32s_45_4_1 RTLNAME mpc_fpga_top_opencl_mul_34s_32s_45_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_34s_32s_46_4_1 RTLNAME mpc_fpga_top_opencl_mul_34s_32s_46_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_40s_32s_51_4_1 RTLNAME mpc_fpga_top_opencl_mul_40s_32s_51_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_40s_32s_52_4_1 RTLNAME mpc_fpga_top_opencl_mul_40s_32s_52_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_40s_32s_58_4_1 RTLNAME mpc_fpga_top_opencl_mul_40s_32s_58_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_9_2_26_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_9_2_26_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_riccati_backward_pass_P_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_P_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_riccati_backward_pass_P_6_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_P_6_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_riccati_backward_pass_PA_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_PA_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME sum8_K_QP_raw MODELNAME sum8_K_QP_raw RTLNAME mpc_fpga_top_opencl_sum8_K_QP_raw}
  {SRCNAME sum6_QP_raw MODELNAME sum6_QP_raw RTLNAME mpc_fpga_top_opencl_sum6_QP_raw}
  {SRCNAME riccati_forward_pass_Pipeline_VITIS_LOOP_383_4 MODELNAME riccati_forward_pass_Pipeline_VITIS_LOOP_383_4 RTLNAME mpc_fpga_top_opencl_riccati_forward_pass_Pipeline_VITIS_LOOP_383_4
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_32s_27s_45_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_27s_45_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_32s_45_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_32s_45_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_13_5_27_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_13_5_27_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_11_4_45_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_11_4_45_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
    }
  }
  {SRCNAME riccati_forward_pass MODELNAME riccati_forward_pass RTLNAME mpc_fpga_top_opencl_riccati_forward_pass
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_32s_26s_44_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_26s_44_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME riccati_pass_hls MODELNAME riccati_pass_hls RTLNAME mpc_fpga_top_opencl_riccati_pass_hls}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1511_21 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1511_21 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1511_21}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1525_22 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1525_22 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1525_22}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1246_12 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1246_12 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1246_12}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1282_14 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1282_14 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1282_14}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1305_17 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1305_17 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1305_17}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1555_24 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1555_24 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1555_24}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1562_26 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1562_26 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1562_26}
  {SRCNAME mpc_persist_writeback_hls MODELNAME mpc_persist_writeback_hls RTLNAME mpc_fpga_top_opencl_mpc_persist_writeback_hls}
  {SRCNAME mpc_compute_hls MODELNAME mpc_compute_hls RTLNAME mpc_fpga_top_opencl_mpc_compute_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_31ns_14ns_44_4_1 RTLNAME mpc_fpga_top_opencl_mul_31ns_14ns_44_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_14ns_46_4_1_x RTLNAME mpc_fpga_top_opencl_mul_32s_14ns_46_4_1_x BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_31ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_31ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_23ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_23ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_28ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_28ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_24ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_24ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_21ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_21ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_7_2_32_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_7_2_32_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
      {MODELNAME mpc_fpga_top_opencl_mul_32s_32s_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_32s_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_0_0_0_RAM_2P_BRAM_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_0_0_0_RAM_2P_BRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_z_x_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_z_x_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_z_u_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_z_u_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_sol_u_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_sol_u_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_K_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_K_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_kk_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_kk_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_1_RAM_2P_BRAM_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_1_RAM_2P_BRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_32s_27ns_50_4_1 RTLNAME mpc_fpga_top_opencl_mul_32s_27ns_50_4_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 3 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statbkb RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statbkb BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statcud RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statcud BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statrcU RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statrcU BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME mpc_fpga_top_opencl MODELNAME mpc_fpga_top_opencl RTLNAME mpc_fpga_top_opencl IS_TOP 1
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_lane_words_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_lane_words_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_ref_reference_heading_error_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_ref_reference_heading_error_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_gmem0_m_axi RTLNAME mpc_fpga_top_opencl_gmem0_m_axi BINDTYPE interface TYPE adapter IMPL m_axi}
      {MODELNAME mpc_fpga_top_opencl_gmem1_m_axi RTLNAME mpc_fpga_top_opencl_gmem1_m_axi BINDTYPE interface TYPE adapter IMPL m_axi}
      {MODELNAME mpc_fpga_top_opencl_control_s_axi RTLNAME mpc_fpga_top_opencl_control_s_axi BINDTYPE interface TYPE interface_s_axilite}
    }
  }
}
