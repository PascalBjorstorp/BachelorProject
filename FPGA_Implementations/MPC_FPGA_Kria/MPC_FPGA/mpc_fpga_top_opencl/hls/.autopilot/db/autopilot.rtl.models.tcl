set SynModuleInfo {
  {SRCNAME {(anonymous namespace)unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1} MODELNAME p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init RTLNAME mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init BINDTYPE interface TYPE internal_upc_flow_control INSTNAME mpc_fpga_top_opencl_flow_control_loop_pipe_sequential_init_U}
    }
  }
  {SRCNAME {(anonymous namespace)unpack_input_lane_words} MODELNAME p_anonymous_namespace_unpack_input_lane_words RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_unpack_input_lane_words}
  {SRCNAME {(anonymous namespace)fill_mpc_reference_trajectory_from_lane_words} MODELNAME p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words}
  {SRCNAME {(anonymous namespace)reset_core_state_hls_Pipeline_VITIS_LOOP_322_1} MODELNAME p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_322_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_322_1}
  {SRCNAME {(anonymous namespace)reset_core_state_hls_Pipeline_VITIS_LOOP_330_3} MODELNAME p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_330_3 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_330_3}
  {SRCNAME {(anonymous namespace)reset_core_state_hls} MODELNAME p_anonymous_namespace_reset_core_state_hls RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_reset_core_state_hls}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_348_1} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_348_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_348_1}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_355_3} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_355_3 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_355_3}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_322_1} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_322_1 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_322_1}
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3 RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_354_1 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_354_1 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_354_1}
  {SRCNAME compute_window_min_hls MODELNAME compute_window_min_hls RTLNAME mpc_fpga_top_opencl_compute_window_min_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_35_5_26_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_35_5_26_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_37_5_26_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_37_5_26_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_39_5_26_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_39_5_26_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_41_5_26_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_41_5_26_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_364_2 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_364_2 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_364_2}
  {SRCNAME fp_atan_lut MODELNAME fp_atan_lut RTLNAME mpc_fpga_top_opencl_fp_atan_lut
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_16s_42_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_16s_42_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_atan_lut_atan_lut_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_atan_lut_atan_lut_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_recip_fn MODELNAME fp_recip_fn RTLNAME mpc_fpga_top_opencl_fp_recip_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_ctlz_21_21_1_0 RTLNAME mpc_fpga_top_opencl_ctlz_21_21_1_0 BINDTYPE op TYPE ctlz IMPL auto}
      {MODELNAME mpc_fpga_top_opencl_mul_14s_3ns_17_1_0 RTLNAME mpc_fpga_top_opencl_mul_14s_3ns_17_1_0 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_7_2_20_1_0 RTLNAME mpc_fpga_top_opencl_sparsemux_7_2_20_1_0 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
      {MODELNAME mpc_fpga_top_opencl_fp_recip_fn_recip_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_recip_fn_recip_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_trig_pair_fused_fn MODELNAME fp_trig_pair_fused_fn RTLNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20ns_22ns_41_3_1 RTLNAME mpc_fpga_top_opencl_mul_20ns_22ns_41_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_14s_35_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_14s_35_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_15s_21s_36_3_1 RTLNAME mpc_fpga_top_opencl_mul_15s_21s_36_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_sin_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_sin_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_cos_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_trig_pair_fused_fn_cos_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_slip_terms MODELNAME fp_slip_terms RTLNAME mpc_fpga_top_opencl_fp_slip_terms
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20s_21s_41_3_1 RTLNAME mpc_fpga_top_opencl_mul_20s_21s_41_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_20s_41_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_20s_41_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_11ns_32_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_11ns_32_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_atan_lut_fn MODELNAME fp_atan_lut_fn RTLNAME mpc_fpga_top_opencl_fp_atan_lut_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_fp_atan_lut_fn_atan_lut_fn_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_atan_lut_fn_atan_lut_fn_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_pacejka_ceff MODELNAME fp_pacejka_ceff RTLNAME mpc_fpga_top_opencl_fp_pacejka_ceff
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20s_21s_41_3_1_x RTLNAME mpc_fpga_top_opencl_mul_20s_21s_41_3_1_x BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_21s_42_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_21s_42_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_front_force_jacobians_fn MODELNAME fp_front_force_jacobians_fn RTLNAME mpc_fpga_top_opencl_fp_front_force_jacobians_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20s_20ns_40_3_1 RTLNAME mpc_fpga_top_opencl_mul_20s_20ns_40_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_21s_42_3_1_x RTLNAME mpc_fpga_top_opencl_mul_21s_21s_42_3_1_x BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_20ns_11ns_30_1_1 RTLNAME mpc_fpga_top_opencl_mul_20ns_11ns_30_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_18ns_20s_38_3_1 RTLNAME mpc_fpga_top_opencl_mul_18ns_20s_38_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_front_tire_path_fn MODELNAME compute_front_tire_path_fn RTLNAME mpc_fpga_top_opencl_compute_front_tire_path_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_21s_16ns_33_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_16ns_33_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_15ns_33_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_15ns_33_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_14ns_33_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_14ns_33_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_rear_force_jacobians_fn MODELNAME fp_rear_force_jacobians_fn RTLNAME mpc_fpga_top_opencl_fp_rear_force_jacobians_fn}
  {SRCNAME compute_rear_tire_path_fn MODELNAME compute_rear_tire_path_fn RTLNAME mpc_fpga_top_opencl_compute_rear_tire_path_fn}
  {SRCNAME compute_frenet_tire_hls MODELNAME compute_frenet_tire_hls RTLNAME mpc_fpga_top_opencl_compute_frenet_tire_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20ns_20ns_40_3_1 RTLNAME mpc_fpga_top_opencl_mul_20ns_20ns_40_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_13ns_33_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_13ns_33_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_frenet_recip MODELNAME fp_frenet_recip RTLNAME mpc_fpga_top_opencl_fp_frenet_recip}
  {SRCNAME fp_frenet_rows01_fn MODELNAME fp_frenet_rows01_fn RTLNAME mpc_fpga_top_opencl_fp_frenet_rows01_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20ns_21s_41_3_1 RTLNAME mpc_fpga_top_opencl_mul_20ns_21s_41_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_20s_20s_40_3_1 RTLNAME mpc_fpga_top_opencl_mul_20s_20s_40_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_20ns_41_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_20ns_41_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_8ns_29_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_8ns_29_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_17s_20ns_37_3_1 RTLNAME mpc_fpga_top_opencl_mul_17s_20ns_37_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_17s_21s_38_3_1 RTLNAME mpc_fpga_top_opencl_mul_17s_21s_38_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_17s_38_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_17s_38_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_rollout_from_forces_fn MODELNAME fp_rollout_from_forces_fn RTLNAME mpc_fpga_top_opencl_fp_rollout_from_forces_fn
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_21s_12ns_33_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_12ns_33_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_8ns_29_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_8ns_29_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_am_addmul_21s_21s_11ns_33_4_1 RTLNAME mpc_fpga_top_opencl_am_addmul_21s_21s_11ns_33_4_1 BINDTYPE op TYPE all IMPL dsp_slice LATENCY 3}
      {MODELNAME mpc_fpga_top_opencl_am_submul_20s_20s_17ns_33_4_1 RTLNAME mpc_fpga_top_opencl_am_submul_20s_20s_17ns_33_4_1 BINDTYPE op TYPE all IMPL dsp_slice LATENCY 3}
    }
  }
  {SRCNAME compute_stage_shared_products MODELNAME compute_stage_shared_products RTLNAME mpc_fpga_top_opencl_compute_stage_shared_products
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_21s_7ns_28_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_7ns_28_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_B_steering MODELNAME compute_B_steering RTLNAME mpc_fpga_top_opencl_compute_B_steering
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_21s_11ns_32_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_11ns_32_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_7s_28_1_1 RTLNAME mpc_fpga_top_opencl_mul_21s_7s_28_1_1 BINDTYPE op TYPE mul IMPL auto LATENCY 0 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_B_accel_load_transfer MODELNAME compute_B_accel_load_transfer RTLNAME mpc_fpga_top_opencl_compute_B_accel_load_transfer
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_21s_13s_34_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_13s_34_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_13ns_34_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_13ns_34_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_11s_32_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_11s_32_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_frenet_AB_hls MODELNAME compute_frenet_AB_hls RTLNAME mpc_fpga_top_opencl_compute_frenet_AB_hls}
  {SRCNAME compute_frenet_AB_and_next_hls MODELNAME compute_frenet_AB_and_next_hls RTLNAME mpc_fpga_top_opencl_compute_frenet_AB_and_next_hls}
  {SRCNAME affine_b0_term_hls MODELNAME affine_b0_term_hls RTLNAME mpc_fpga_top_opencl_affine_b0_term_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_14s_26s_40_3_1 RTLNAME mpc_fpga_top_opencl_mul_14s_26s_40_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME compute_affine_bias_dense_hls MODELNAME compute_affine_bias_dense_hls RTLNAME mpc_fpga_top_opencl_compute_affine_bias_dense_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_37_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_26s_37_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_10ns_36_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_10ns_36_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME fp_recip MODELNAME fp_recip RTLNAME mpc_fpga_top_opencl_fp_recip
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_ctlz_26_26_1_1 RTLNAME mpc_fpga_top_opencl_ctlz_26_26_1_1 BINDTYPE op TYPE ctlz IMPL auto}
      {MODELNAME mpc_fpga_top_opencl_mul_8s_5ns_13_2_1 RTLNAME mpc_fpga_top_opencl_mul_8s_5ns_13_2_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 1 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_recip_recip_lut_ROM_2P_BRAM_1R RTLNAME mpc_fpga_top_opencl_fp_recip_recip_lut_ROM_2P_BRAM_1R BINDTYPE storage TYPE rom_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_fp_recip_slope_lut_ROM_AUTO_1R RTLNAME mpc_fpga_top_opencl_fp_recip_slope_lut_ROM_AUTO_1R BINDTYPE storage TYPE rom IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_369_3 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_369_3 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_369_3
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_14ns_40_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_14ns_40_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_17ns_43_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_17ns_43_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_15ns_41_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_15ns_41_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_25ns_15ns_39_3_1 RTLNAME mpc_fpga_top_opencl_mul_25ns_15ns_39_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_17s_18ns_35_3_1 RTLNAME mpc_fpga_top_opencl_mul_17s_18ns_35_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_21s_19ns_40_3_1 RTLNAME mpc_fpga_top_opencl_mul_21s_19ns_40_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_322_1 MODELNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_322_1 RTLNAME mpc_fpga_top_opencl_reset_admm_state_hls_Pipeline_VITIS_LOOP_322_1}
  {SRCNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_330_3 MODELNAME reset_admm_state_hls_Pipeline_VITIS_LOOP_330_3 RTLNAME mpc_fpga_top_opencl_reset_admm_state_hls_Pipeline_VITIS_LOOP_330_3}
  {SRCNAME reset_admm_state_hls MODELNAME reset_admm_state_hls RTLNAME mpc_fpga_top_opencl_reset_admm_state_hls}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1214_4 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1214_4 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1214_4}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1222_6 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1222_6 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1222_6}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1234_9 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1234_9 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1234_9}
  {SRCNAME invert_2x2_qp_hls MODELNAME invert_2x2_qp_hls RTLNAME mpc_fpga_top_opencl_invert_2x2_qp_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_9_3_3_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_9_3_3_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
      {MODELNAME mpc_fpga_top_opencl_mul_17s_26s_43_3_1 RTLNAME mpc_fpga_top_opencl_mul_17s_26s_43_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME sum6_MG_QP_raw MODELNAME sum6_MG_QP_raw RTLNAME mpc_fpga_top_opencl_sum6_MG_QP_raw}
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_742_9 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_742_9 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_742_9
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_18s_44_2_1 RTLNAME mpc_fpga_top_opencl_mul_26s_18s_44_2_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 1 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_767_11 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_767_11 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_767_11
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_17_3_18_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_17_3_18_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME sum6_P_QP_raw MODELNAME sum6_P_QP_raw RTLNAME mpc_fpga_top_opencl_sum6_P_QP_raw}
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_783_12 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_783_12 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_783_12
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_sparsemux_17_3_27_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_17_3_27_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
    }
  }
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_848_14 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_848_14 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_848_14}
  {SRCNAME sum8_P_MIX_raw_pupdate MODELNAME sum8_P_MIX_raw_pupdate RTLNAME mpc_fpga_top_opencl_sum8_P_MIX_raw_pupdate}
  {SRCNAME sum8_P_MIX_raw MODELNAME sum8_P_MIX_raw RTLNAME mpc_fpga_top_opencl_sum8_P_MIX_raw}
  {SRCNAME riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40 MODELNAME riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40 RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_6s_17s_23_2_1 RTLNAME mpc_fpga_top_opencl_mul_6s_17s_23_2_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 1 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME riccati_backward_pass MODELNAME riccati_backward_pass RTLNAME mpc_fpga_top_opencl_riccati_backward_pass
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_20s_26s_46_3_1 RTLNAME mpc_fpga_top_opencl_mul_20s_26s_46_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_43_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_26s_43_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_44_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_26s_44_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_27s_26s_35_3_1 RTLNAME mpc_fpga_top_opencl_mul_27s_26s_35_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_27s_26s_42_3_1 RTLNAME mpc_fpga_top_opencl_mul_27s_26s_42_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_9_2_17_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_9_2_17_1_1 BINDTYPE op TYPE sparsemux IMPL compactencoding_dontcare}
      {MODELNAME mpc_fpga_top_opencl_mul_18s_17s_35_2_1 RTLNAME mpc_fpga_top_opencl_mul_18s_17s_35_2_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 1 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_27s_26s_43_3_1 RTLNAME mpc_fpga_top_opencl_mul_27s_26s_43_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_riccati_backward_pass_P_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_P_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_riccati_backward_pass_P_6_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_P_6_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_riccati_backward_pass_PA_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_riccati_backward_pass_PA_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME sum8_K_QP_raw MODELNAME sum8_K_QP_raw RTLNAME mpc_fpga_top_opencl_sum8_K_QP_raw}
  {SRCNAME sum6_QP_raw MODELNAME sum6_QP_raw RTLNAME mpc_fpga_top_opencl_sum6_QP_raw}
  {SRCNAME riccati_forward_pass_Pipeline_VITIS_LOOP_399_4 MODELNAME riccati_forward_pass_Pipeline_VITIS_LOOP_399_4 RTLNAME mpc_fpga_top_opencl_riccati_forward_pass_Pipeline_VITIS_LOOP_399_4
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_39_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_26s_39_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_13_5_25_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_13_5_25_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_11_4_39_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_11_4_39_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
    }
  }
  {SRCNAME riccati_forward_pass MODELNAME riccati_forward_pass RTLNAME mpc_fpga_top_opencl_riccati_forward_pass
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_17s_43_2_1 RTLNAME mpc_fpga_top_opencl_mul_26s_17s_43_2_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 1 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME riccati_pass_hls MODELNAME riccati_pass_hls RTLNAME mpc_fpga_top_opencl_riccati_pass_hls}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1531_20 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1531_20 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1531_20}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1545_21 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1545_21 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1545_21}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1265_12 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1265_12 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1265_12}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1301_14 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1301_14 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1301_14}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1320_16 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1320_16 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1320_16}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1324_17 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1324_17 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1324_17}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1575_23 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1575_23 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1575_23}
  {SRCNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1582_25 MODELNAME mpc_compute_hls_Pipeline_VITIS_LOOP_1582_25 RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_Pipeline_VITIS_LOOP_1582_25}
  {SRCNAME mpc_persist_writeback_hls MODELNAME mpc_persist_writeback_hls RTLNAME mpc_fpga_top_opencl_mpc_persist_writeback_hls}
  {SRCNAME mpc_compute_hls MODELNAME mpc_compute_hls RTLNAME mpc_fpga_top_opencl_mpc_compute_hls
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_20ns_46_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_20ns_46_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_22ns_48_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_22ns_48_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_25s_51_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_25s_51_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mul_25ns_11ns_35_3_1 RTLNAME mpc_fpga_top_opencl_mul_25ns_11ns_35_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_sparsemux_7_2_26_1_1 RTLNAME mpc_fpga_top_opencl_sparsemux_7_2_26_1_1 BINDTYPE op TYPE sparsemux IMPL onehotencoding_realdef}
      {MODELNAME mpc_fpga_top_opencl_mul_26s_26s_40_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_26s_40_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_z_x_RAM_2P_LUTRAM_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_z_x_RAM_2P_LUTRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL lutram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_z_u_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_z_u_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_sol_u_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_sol_u_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_K_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_K_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_kk_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_kk_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_0_0_RAM_2P_BRAM_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_0_0_RAM_2P_BRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_3_RAM_2P_BRAM_1R1W RTLNAME mpc_fpga_top_opencl_mpc_compute_hls_step_data_3_RAM_2P_BRAM_1R1W BINDTYPE storage TYPE ram_2p IMPL bram LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME {(anonymous namespace)mpc_fpga_compute_core} MODELNAME p_anonymous_namespace_mpc_fpga_compute_core RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_mul_26s_23ns_49_3_1 RTLNAME mpc_fpga_top_opencl_mul_26s_23ns_49_3_1 BINDTYPE op TYPE mul IMPL dsp LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statbkb RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statbkb BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statcud RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statcud BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statrcU RTLNAME mpc_fpga_top_opencl_p_anonymous_namespace_mpc_fpga_compute_core_p_anonymous_namespace_g_core_statrcU BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
    }
  }
  {SRCNAME mpc_fpga_top_opencl MODELNAME mpc_fpga_top_opencl RTLNAME mpc_fpga_top_opencl IS_TOP 1
    SUBMODULES {
      {MODELNAME mpc_fpga_top_opencl_lane_words_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_lane_words_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_lane_words_3_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_lane_words_3_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_ref_reference_heading_error_RAM_AUTO_1R1W RTLNAME mpc_fpga_top_opencl_ref_reference_heading_error_RAM_AUTO_1R1W BINDTYPE storage TYPE ram IMPL auto LATENCY 2 ALLOW_PRAGMA 1}
      {MODELNAME mpc_fpga_top_opencl_gmem0_m_axi RTLNAME mpc_fpga_top_opencl_gmem0_m_axi BINDTYPE interface TYPE adapter IMPL m_axi}
      {MODELNAME mpc_fpga_top_opencl_gmem1_m_axi RTLNAME mpc_fpga_top_opencl_gmem1_m_axi BINDTYPE interface TYPE adapter IMPL m_axi}
      {MODELNAME mpc_fpga_top_opencl_control_s_axi RTLNAME mpc_fpga_top_opencl_control_s_axi BINDTYPE interface TYPE interface_s_axilite}
    }
  }
}
