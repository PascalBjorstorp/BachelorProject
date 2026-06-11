set ModuleHierarchy {[{
"Name" : "mpc_fpga_top_opencl", "RefName" : "mpc_fpga_top_opencl","ID" : "0","Type" : "sequential",
"SubInsts" : [
	{"Name" : "grp_p_anonymous_namespace_unpack_input_lane_words_fu_434", "RefName" : "p_anonymous_namespace_unpack_input_lane_words","ID" : "1","Type" : "sequential",
		"SubInsts" : [
		{"Name" : "grp_p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1_fu_83", "RefName" : "p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1","ID" : "2","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_213_1","RefName" : "VITIS_LOOP_213_1","ID" : "3","Type" : "pipeline"},]},]},
	{"Name" : "grp_p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words_fu_457", "RefName" : "p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words","ID" : "4","Type" : "sequential",
		"SubLoops" : [
		{"Name" : "VITIS_LOOP_184_1","RefName" : "VITIS_LOOP_184_1","ID" : "5","Type" : "pipeline"},]},
	{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_fu_485", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core","ID" : "6","Type" : "sequential",
		"SubInsts" : [
		{"Name" : "grp_p_anonymous_namespace_reset_core_state_hls_fu_272", "RefName" : "p_anonymous_namespace_reset_core_state_hls","ID" : "7","Type" : "sequential",
			"SubInsts" : [
			{"Name" : "grp_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_322_1_fu_70", "RefName" : "p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_322_1","ID" : "8","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_322_1","RefName" : "VITIS_LOOP_322_1","ID" : "9","Type" : "pipeline"},]},
			{"Name" : "grp_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_330_3_fu_106", "RefName" : "p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_330_3","ID" : "10","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_330_3","RefName" : "VITIS_LOOP_330_3","ID" : "11","Type" : "pipeline"},]},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_348_1_fu_336", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_348_1","ID" : "12","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_348_1","RefName" : "VITIS_LOOP_348_1","ID" : "13","Type" : "pipeline"},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_355_3_fu_356", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_355_3","ID" : "14","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_355_3","RefName" : "VITIS_LOOP_355_3","ID" : "15","Type" : "pipeline"},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_322_1_fu_364", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_322_1","ID" : "16","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_322_1","RefName" : "VITIS_LOOP_322_1","ID" : "17","Type" : "pipeline"},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3_fu_400", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3","ID" : "18","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_330_3","RefName" : "VITIS_LOOP_330_3","ID" : "19","Type" : "pipeline"},]},
		{"Name" : "grp_mpc_compute_hls_fu_412", "RefName" : "mpc_compute_hls","ID" : "20","Type" : "sequential",
			"SubInsts" : [
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_354_1_fu_2190", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_354_1","ID" : "21","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_354_1","RefName" : "VITIS_LOOP_354_1","ID" : "22","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_364_2_fu_2238", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_364_2","ID" : "23","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_364_2","RefName" : "VITIS_LOOP_364_2","ID" : "24","Type" : "pipeline",
				"SubInsts" : [
				{"Name" : "grp_compute_window_min_hls_fu_908", "RefName" : "compute_window_min_hls","ID" : "25","Type" : "pipeline"},
				{"Name" : "grp_compute_window_min_hls_fu_933", "RefName" : "compute_window_min_hls","ID" : "26","Type" : "pipeline"},]},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_369_3_fu_2322", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_369_3","ID" : "27","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_369_3","RefName" : "VITIS_LOOP_369_3","ID" : "28","Type" : "pipeline",
				"SubInsts" : [
				{"Name" : "grp_fp_atan_lut_fu_1397", "RefName" : "fp_atan_lut","ID" : "29","Type" : "pipeline"},
				{"Name" : "grp_compute_frenet_AB_and_next_hls_fu_1404", "RefName" : "compute_frenet_AB_and_next_hls","ID" : "30","Type" : "pipeline",
						"SubInsts" : [
						{"Name" : "grp_compute_frenet_AB_hls_fu_78", "RefName" : "compute_frenet_AB_hls","ID" : "31","Type" : "pipeline",
							"SubInsts" : [
							{"Name" : "grp_compute_frenet_tire_hls_fu_140", "RefName" : "compute_frenet_tire_hls","ID" : "32","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_recip_fn_fu_144", "RefName" : "fp_recip_fn","ID" : "33","Type" : "pipeline"},
								{"Name" : "grp_fp_slip_terms_fu_151", "RefName" : "fp_slip_terms","ID" : "34","Type" : "pipeline"},
								{"Name" : "grp_compute_front_tire_path_fn_fu_158", "RefName" : "compute_front_tire_path_fn","ID" : "35","Type" : "pipeline",
									"SubInsts" : [
									{"Name" : "grp_fp_atan_lut_fn_fu_138", "RefName" : "fp_atan_lut_fn","ID" : "36","Type" : "pipeline"},
									{"Name" : "grp_fp_recip_fn_fu_146", "RefName" : "fp_recip_fn","ID" : "37","Type" : "pipeline"},
									{"Name" : "grp_fp_trig_pair_fused_fn_fu_154", "RefName" : "fp_trig_pair_fused_fn","ID" : "38","Type" : "pipeline"},
									{"Name" : "grp_fp_pacejka_ceff_fu_163", "RefName" : "fp_pacejka_ceff","ID" : "39","Type" : "pipeline"},
									{"Name" : "grp_fp_front_force_jacobians_fn_fu_170", "RefName" : "fp_front_force_jacobians_fn","ID" : "40","Type" : "pipeline"},]},
								{"Name" : "grp_compute_rear_tire_path_fn_fu_179", "RefName" : "compute_rear_tire_path_fn","ID" : "41","Type" : "pipeline",
									"SubInsts" : [
									{"Name" : "grp_fp_atan_lut_fn_fu_128", "RefName" : "fp_atan_lut_fn","ID" : "42","Type" : "pipeline"},
									{"Name" : "grp_fp_recip_fn_fu_136", "RefName" : "fp_recip_fn","ID" : "43","Type" : "pipeline"},
									{"Name" : "grp_fp_trig_pair_fused_fn_fu_144", "RefName" : "fp_trig_pair_fused_fn","ID" : "44","Type" : "pipeline"},
									{"Name" : "grp_fp_pacejka_ceff_fu_153", "RefName" : "fp_pacejka_ceff","ID" : "45","Type" : "pipeline"},
									{"Name" : "grp_fp_rear_force_jacobians_fn_fu_160", "RefName" : "fp_rear_force_jacobians_fn","ID" : "46","Type" : "pipeline"},]},
								{"Name" : "grp_fp_trig_pair_fused_fn_fu_199", "RefName" : "fp_trig_pair_fused_fn","ID" : "47","Type" : "pipeline"},]},
							{"Name" : "grp_fp_trig_pair_fused_fn_fu_162", "RefName" : "fp_trig_pair_fused_fn","ID" : "48","Type" : "pipeline"},
							{"Name" : "grp_fp_frenet_recip_fu_171", "RefName" : "fp_frenet_recip","ID" : "49","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_recip_fn_fu_62", "RefName" : "fp_recip_fn","ID" : "50","Type" : "pipeline"},]},
							{"Name" : "grp_compute_stage_shared_products_fu_179", "RefName" : "compute_stage_shared_products","ID" : "51","Type" : "pipeline"},
							{"Name" : "grp_compute_B_accel_load_transfer_fu_193", "RefName" : "compute_B_accel_load_transfer","ID" : "52","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_recip_fn_fu_138", "RefName" : "fp_recip_fn","ID" : "53","Type" : "pipeline"},]},
							{"Name" : "grp_fp_rollout_from_forces_fn_fu_206", "RefName" : "fp_rollout_from_forces_fn","ID" : "54","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_recip_fn_fu_214", "RefName" : "fp_recip_fn","ID" : "55","Type" : "pipeline"},]},
							{"Name" : "grp_compute_B_steering_fu_225", "RefName" : "compute_B_steering","ID" : "56","Type" : "pipeline"},
							{"Name" : "grp_fp_frenet_rows01_fn_fu_234", "RefName" : "fp_frenet_rows01_fn","ID" : "57","Type" : "pipeline"},]},]},
				{"Name" : "fp_recip", "RefName" : "fp_recip","ID" : "58","Type" : "pipeline"},
				{"Name" : "grp_compute_affine_bias_dense_hls_fu_1433", "RefName" : "compute_affine_bias_dense_hls","ID" : "59","Type" : "pipeline",
						"SubInsts" : [
						{"Name" : "grp_affine_b0_term_hls_fu_595", "RefName" : "affine_b0_term_hls","ID" : "60","Type" : "pipeline"},]},]},]},
			{"Name" : "grp_reset_admm_state_hls_fu_2424", "RefName" : "reset_admm_state_hls","ID" : "61","Type" : "sequential",
				"SubInsts" : [
				{"Name" : "grp_reset_admm_state_hls_Pipeline_VITIS_LOOP_322_1_fu_54", "RefName" : "reset_admm_state_hls_Pipeline_VITIS_LOOP_322_1","ID" : "62","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_322_1","RefName" : "VITIS_LOOP_322_1","ID" : "63","Type" : "pipeline"},]},
				{"Name" : "grp_reset_admm_state_hls_Pipeline_VITIS_LOOP_330_3_fu_90", "RefName" : "reset_admm_state_hls_Pipeline_VITIS_LOOP_330_3","ID" : "64","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_330_3","RefName" : "VITIS_LOOP_330_3","ID" : "65","Type" : "pipeline"},]},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1_fu_2474", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1","ID" : "66","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1202_1","RefName" : "VITIS_LOOP_1202_1","ID" : "67","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1214_4_fu_2494", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1214_4","ID" : "68","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1214_4","RefName" : "VITIS_LOOP_1214_4","ID" : "69","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1222_6_fu_2502", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1222_6","ID" : "70","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1222_6","RefName" : "VITIS_LOOP_1222_6","ID" : "71","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1234_9_fu_2554", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1234_9","ID" : "72","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1234_9","RefName" : "VITIS_LOOP_1234_9","ID" : "73","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1575_23_fu_2630", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1575_23","ID" : "74","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1575_23","RefName" : "VITIS_LOOP_1575_23","ID" : "75","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1582_25_fu_2682", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1582_25","ID" : "76","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1582_25","RefName" : "VITIS_LOOP_1582_25","ID" : "77","Type" : "pipeline"},]},
			{"Name" : "call_ln669_mpc_persist_writeback_hls_fu_2758", "RefName" : "mpc_persist_writeback_hls","ID" : "78","Type" : "sequential"},],
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_1250_11","RefName" : "VITIS_LOOP_1250_11","ID" : "79","Type" : "no",
			"SubInsts" : [
			{"Name" : "grp_riccati_pass_hls_fu_1947", "RefName" : "riccati_pass_hls","ID" : "80","Type" : "sequential",
					"SubInsts" : [
					{"Name" : "grp_riccati_backward_pass_fu_342", "RefName" : "riccati_backward_pass","ID" : "81","Type" : "sequential",
						"SubLoops" : [
						{"Name" : "VITIS_LOOP_551_4","RefName" : "VITIS_LOOP_551_4","ID" : "82","Type" : "no",
						"SubInsts" : [
						{"Name" : "grp_invert_2x2_qp_hls_fu_3692", "RefName" : "invert_2x2_qp_hls","ID" : "83","Type" : "sequential",
								"SubInsts" : [
								{"Name" : "fp_recip", "RefName" : "fp_recip","ID" : "84","Type" : "pipeline"},]},
						{"Name" : "fp_recip", "RefName" : "fp_recip","ID" : "85","Type" : "pipeline"},
						{"Name" : "grp_fp_recip_fu_3716", "RefName" : "fp_recip","ID" : "86","Type" : "pipeline"},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_742_9_fu_3725", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_742_9","ID" : "87","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_742_9","RefName" : "VITIS_LOOP_742_9","ID" : "88","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "sum_sum6_MG_QP_raw_fu_676", "RefName" : "sum6_MG_QP_raw","ID" : "89","Type" : "pipeline"},
								{"Name" : "sum_11_sum6_MG_QP_raw_fu_686", "RefName" : "sum6_MG_QP_raw","ID" : "90","Type" : "pipeline"},
								{"Name" : "sum_12_sum6_MG_QP_raw_fu_696", "RefName" : "sum6_MG_QP_raw","ID" : "91","Type" : "pipeline"},
								{"Name" : "sum_13_sum6_MG_QP_raw_fu_706", "RefName" : "sum6_MG_QP_raw","ID" : "92","Type" : "pipeline"},
								{"Name" : "sum_14_sum6_MG_QP_raw_fu_716", "RefName" : "sum6_MG_QP_raw","ID" : "93","Type" : "pipeline"},
								{"Name" : "sum_15_sum6_MG_QP_raw_fu_726", "RefName" : "sum6_MG_QP_raw","ID" : "94","Type" : "pipeline"},]},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_783_12_fu_3801", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_783_12","ID" : "95","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_783_12","RefName" : "VITIS_LOOP_783_12","ID" : "96","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "sum6_P_QP_raw", "RefName" : "sum6_P_QP_raw","ID" : "97","Type" : "pipeline"},]},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_767_11_fu_3833", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_767_11","ID" : "98","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_767_11","RefName" : "VITIS_LOOP_767_11","ID" : "99","Type" : "pipeline"},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_848_14_fu_3984", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_848_14","ID" : "100","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_848_14","RefName" : "VITIS_LOOP_848_14","ID" : "101","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "sum_sum6_P_QP_raw_fu_1092", "RefName" : "sum6_P_QP_raw","ID" : "102","Type" : "pipeline"},
								{"Name" : "sum_7_sum6_P_QP_raw_fu_1102", "RefName" : "sum6_P_QP_raw","ID" : "103","Type" : "pipeline"},
								{"Name" : "sum_8_sum6_P_QP_raw_fu_1112", "RefName" : "sum6_P_QP_raw","ID" : "104","Type" : "pipeline"},
								{"Name" : "sum_9_sum6_P_QP_raw_fu_1122", "RefName" : "sum6_P_QP_raw","ID" : "105","Type" : "pipeline"},
								{"Name" : "sum6_P_QP_raw", "RefName" : "sum6_P_QP_raw","ID" : "106","Type" : "pipeline"},]},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40_fu_4096", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40","ID" : "107","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_1048_40","RefName" : "VITIS_LOOP_1048_40","ID" : "108","Type" : "pipeline"},]},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4105", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "109","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4117", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "110","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4129", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "111","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4141", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "112","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4153", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "113","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4165", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "114","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4177", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "115","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4189", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "116","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4201", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "117","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4213", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "118","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4225", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "119","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4237", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "120","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4249", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "121","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4261", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "122","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4273", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "123","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4285", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "124","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4297", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "125","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4309", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "126","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4321", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "127","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4333", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "128","Type" : "pipeline"},
						{"Name" : "total_sum8_P_MIX_raw_fu_4345", "RefName" : "sum8_P_MIX_raw","ID" : "129","Type" : "pipeline"},
						{"Name" : "total_1_sum8_P_MIX_raw_fu_4357", "RefName" : "sum8_P_MIX_raw","ID" : "130","Type" : "pipeline"},
						{"Name" : "total_2_sum8_P_MIX_raw_fu_4369", "RefName" : "sum8_P_MIX_raw","ID" : "131","Type" : "pipeline"},
						{"Name" : "total_3_sum8_P_MIX_raw_fu_4381", "RefName" : "sum8_P_MIX_raw","ID" : "132","Type" : "pipeline"},
						{"Name" : "total_4_sum8_P_MIX_raw_fu_4393", "RefName" : "sum8_P_MIX_raw","ID" : "133","Type" : "pipeline"},
						{"Name" : "total_5_sum8_P_MIX_raw_fu_4405", "RefName" : "sum8_P_MIX_raw","ID" : "134","Type" : "pipeline"},]},]},
					{"Name" : "grp_riccati_forward_pass_fu_537", "RefName" : "riccati_forward_pass","ID" : "135","Type" : "sequential",
						"SubLoops" : [
						{"Name" : "VITIS_LOOP_349_2","RefName" : "VITIS_LOOP_349_2","ID" : "136","Type" : "no",
						"SubInsts" : [
						{"Name" : "prod_sum0_sum8_K_QP_raw_fu_1595", "RefName" : "sum8_K_QP_raw","ID" : "137","Type" : "pipeline"},
						{"Name" : "prod_sum1_sum8_K_QP_raw_fu_1607", "RefName" : "sum8_K_QP_raw","ID" : "138","Type" : "pipeline"},
						{"Name" : "grp_riccati_forward_pass_Pipeline_VITIS_LOOP_399_4_fu_1619", "RefName" : "riccati_forward_pass_Pipeline_VITIS_LOOP_399_4","ID" : "139","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_399_4","RefName" : "VITIS_LOOP_399_4","ID" : "140","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "ax_sum_sum6_QP_raw_fu_502", "RefName" : "sum6_QP_raw","ID" : "141","Type" : "pipeline"},]},]},]},]},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19_fu_2570", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19","ID" : "142","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1362_19","RefName" : "VITIS_LOOP_1362_19","ID" : "143","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1531_20_fu_2623", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1531_20","ID" : "144","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1531_20","RefName" : "VITIS_LOOP_1531_20","ID" : "145","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1545_21_fu_2698", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1545_21","ID" : "146","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1545_21","RefName" : "VITIS_LOOP_1545_21","ID" : "147","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1265_12_fu_2705", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1265_12","ID" : "148","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1265_12","RefName" : "VITIS_LOOP_1265_12","ID" : "149","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1301_14_fu_2729", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1301_14","ID" : "150","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1301_14","RefName" : "VITIS_LOOP_1301_14","ID" : "151","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1320_16_fu_2738", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1320_16","ID" : "152","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1320_16","RefName" : "VITIS_LOOP_1320_16","ID" : "153","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1324_17_fu_2748", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1324_17","ID" : "154","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1324_17","RefName" : "VITIS_LOOP_1324_17","ID" : "155","Type" : "pipeline"},]},]},]},]},]
}]}