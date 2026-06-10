set ModuleHierarchy {[{
"Name" : "mpc_fpga_top_opencl", "RefName" : "mpc_fpga_top_opencl","ID" : "0","Type" : "sequential",
"SubInsts" : [
	{"Name" : "grp_p_anonymous_namespace_unpack_input_lane_words_fu_440", "RefName" : "p_anonymous_namespace_unpack_input_lane_words","ID" : "1","Type" : "sequential",
		"SubInsts" : [
		{"Name" : "grp_p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_216_1_fu_83", "RefName" : "p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_216_1","ID" : "2","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_216_1","RefName" : "VITIS_LOOP_216_1","ID" : "3","Type" : "pipeline"},]},]},
	{"Name" : "grp_p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words_fu_463", "RefName" : "p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words","ID" : "4","Type" : "sequential",
		"SubLoops" : [
		{"Name" : "VITIS_LOOP_187_1","RefName" : "VITIS_LOOP_187_1","ID" : "5","Type" : "pipeline"},]},
	{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_fu_491", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core","ID" : "6","Type" : "sequential",
		"SubInsts" : [
		{"Name" : "grp_p_anonymous_namespace_reset_core_state_hls_fu_264", "RefName" : "p_anonymous_namespace_reset_core_state_hls","ID" : "7","Type" : "sequential",
			"SubInsts" : [
			{"Name" : "grp_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_364_1_fu_68", "RefName" : "p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_364_1","ID" : "8","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_364_1","RefName" : "VITIS_LOOP_364_1","ID" : "9","Type" : "pipeline"},]},
			{"Name" : "grp_p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_372_3_fu_104", "RefName" : "p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_372_3","ID" : "10","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_372_3","RefName" : "VITIS_LOOP_372_3","ID" : "11","Type" : "pipeline"},]},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_390_1_fu_328", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_390_1","ID" : "12","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_390_1","RefName" : "VITIS_LOOP_390_1","ID" : "13","Type" : "pipeline"},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_397_3_fu_348", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_397_3","ID" : "14","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_397_3","RefName" : "VITIS_LOOP_397_3","ID" : "15","Type" : "pipeline"},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_364_1_fu_356", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_364_1","ID" : "16","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_364_1","RefName" : "VITIS_LOOP_364_1","ID" : "17","Type" : "pipeline"},]},
		{"Name" : "grp_p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_372_3_fu_392", "RefName" : "p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_372_3","ID" : "18","Type" : "sequential",
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_372_3","RefName" : "VITIS_LOOP_372_3","ID" : "19","Type" : "pipeline"},]},
		{"Name" : "grp_mpc_compute_hls_fu_404", "RefName" : "mpc_compute_hls","ID" : "20","Type" : "sequential",
			"SubInsts" : [
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_356_1_fu_2352", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_356_1","ID" : "21","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_356_1","RefName" : "VITIS_LOOP_356_1","ID" : "22","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_366_2_fu_2400", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_366_2","ID" : "23","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_366_2","RefName" : "VITIS_LOOP_366_2","ID" : "24","Type" : "pipeline",
				"SubInsts" : [
				{"Name" : "grp_compute_window_min_hls_fu_908", "RefName" : "compute_window_min_hls","ID" : "25","Type" : "pipeline"},
				{"Name" : "grp_compute_window_min_hls_fu_933", "RefName" : "compute_window_min_hls","ID" : "26","Type" : "pipeline"},]},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_371_3_fu_2484", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_371_3","ID" : "27","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_371_3","RefName" : "VITIS_LOOP_371_3","ID" : "28","Type" : "pipeline",
				"SubInsts" : [
				{"Name" : "grp_fp_atan_lut_fu_1707", "RefName" : "fp_atan_lut","ID" : "29","Type" : "pipeline"},
				{"Name" : "grp_compute_frenet_AB_and_next_hls_fu_1714", "RefName" : "compute_frenet_AB_and_next_hls","ID" : "30","Type" : "pipeline",
						"SubInsts" : [
						{"Name" : "grp_compute_frenet_AB_hls_fu_78", "RefName" : "compute_frenet_AB_hls","ID" : "31","Type" : "pipeline",
							"SubInsts" : [
							{"Name" : "grp_compute_frenet_tire_hls_fu_178", "RefName" : "compute_frenet_tire_hls","ID" : "32","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_trig_pair_fused_fn_fu_154", "RefName" : "fp_trig_pair_fused_fn","ID" : "33","Type" : "pipeline"},
								{"Name" : "grp_fp_recip_fn_fu_184", "RefName" : "fp_recip_fn","ID" : "34","Type" : "pipeline"},
								{"Name" : "grp_fp_slip_terms_fu_191", "RefName" : "fp_slip_terms","ID" : "35","Type" : "pipeline"},
								{"Name" : "grp_compute_front_tire_path_fn_fu_198", "RefName" : "compute_front_tire_path_fn","ID" : "36","Type" : "pipeline",
									"SubInsts" : [
									{"Name" : "grp_fp_atan_lut_fn_fu_150", "RefName" : "fp_atan_lut_fn","ID" : "37","Type" : "pipeline"},
									{"Name" : "grp_fp_recip_fn_fu_158", "RefName" : "fp_recip_fn","ID" : "38","Type" : "pipeline"},
									{"Name" : "grp_fp_trig_pair_fused_fn_fu_166", "RefName" : "fp_trig_pair_fused_fn","ID" : "39","Type" : "pipeline"},
									{"Name" : "grp_fp_pacejka_ceff_fu_175", "RefName" : "fp_pacejka_ceff","ID" : "40","Type" : "pipeline"},
									{"Name" : "grp_fp_front_force_jacobians_fn_fu_182", "RefName" : "fp_front_force_jacobians_fn","ID" : "41","Type" : "pipeline"},]},
								{"Name" : "grp_compute_rear_tire_path_fn_fu_219", "RefName" : "compute_rear_tire_path_fn","ID" : "42","Type" : "pipeline",
									"SubInsts" : [
									{"Name" : "grp_fp_atan_lut_fn_fu_138", "RefName" : "fp_atan_lut_fn","ID" : "43","Type" : "pipeline"},
									{"Name" : "grp_fp_recip_fn_fu_146", "RefName" : "fp_recip_fn","ID" : "44","Type" : "pipeline"},
									{"Name" : "grp_fp_trig_pair_fused_fn_fu_154", "RefName" : "fp_trig_pair_fused_fn","ID" : "45","Type" : "pipeline"},
									{"Name" : "grp_fp_pacejka_ceff_fu_163", "RefName" : "fp_pacejka_ceff","ID" : "46","Type" : "pipeline"},
									{"Name" : "grp_fp_rear_force_jacobians_fn_fu_170", "RefName" : "fp_rear_force_jacobians_fn","ID" : "47","Type" : "pipeline"},]},]},
							{"Name" : "grp_fp_trig_pair_fused_fn_fu_200", "RefName" : "fp_trig_pair_fused_fn","ID" : "48","Type" : "pipeline"},
							{"Name" : "grp_fp_frenet_recip_fu_209", "RefName" : "fp_frenet_recip","ID" : "49","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_recip_fn_fu_54", "RefName" : "fp_recip_fn","ID" : "50","Type" : "pipeline"},]},
							{"Name" : "grp_compute_stage_shared_products_fu_217", "RefName" : "compute_stage_shared_products","ID" : "51","Type" : "pipeline"},
							{"Name" : "grp_compute_B_accel_load_transfer_fu_231", "RefName" : "compute_B_accel_load_transfer","ID" : "52","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_recip_fn_fu_140", "RefName" : "fp_recip_fn","ID" : "53","Type" : "pipeline"},]},
							{"Name" : "grp_fp_rollout_from_forces_fn_fu_244", "RefName" : "fp_rollout_from_forces_fn","ID" : "54","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "grp_fp_recip_fn_fu_214", "RefName" : "fp_recip_fn","ID" : "55","Type" : "pipeline"},]},
							{"Name" : "grp_compute_B_steering_fu_263", "RefName" : "compute_B_steering","ID" : "56","Type" : "pipeline"},
							{"Name" : "grp_fp_frenet_rows01_fn_fu_272", "RefName" : "fp_frenet_rows01_fn","ID" : "57","Type" : "pipeline"},]},]},
				{"Name" : "fp_recip", "RefName" : "fp_recip","ID" : "58","Type" : "pipeline"},
				{"Name" : "grp_compute_affine_bias_dense_hls_fu_1741", "RefName" : "compute_affine_bias_dense_hls","ID" : "59","Type" : "pipeline",
						"SubInsts" : [
						{"Name" : "grp_affine_b0_term_hls_fu_591", "RefName" : "affine_b0_term_hls","ID" : "60","Type" : "pipeline"},]},]},]},
			{"Name" : "grp_reset_admm_state_hls_fu_2614", "RefName" : "reset_admm_state_hls","ID" : "61","Type" : "sequential",
				"SubInsts" : [
				{"Name" : "grp_reset_admm_state_hls_Pipeline_VITIS_LOOP_364_1_fu_54", "RefName" : "reset_admm_state_hls_Pipeline_VITIS_LOOP_364_1","ID" : "62","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_364_1","RefName" : "VITIS_LOOP_364_1","ID" : "63","Type" : "pipeline"},]},
				{"Name" : "grp_reset_admm_state_hls_Pipeline_VITIS_LOOP_372_3_fu_90", "RefName" : "reset_admm_state_hls_Pipeline_VITIS_LOOP_372_3","ID" : "64","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_372_3","RefName" : "VITIS_LOOP_372_3","ID" : "65","Type" : "pipeline"},]},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1183_1_fu_2664", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1183_1","ID" : "66","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1183_1","RefName" : "VITIS_LOOP_1183_1","ID" : "67","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1195_4_fu_2684", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1195_4","ID" : "68","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1195_4","RefName" : "VITIS_LOOP_1195_4","ID" : "69","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1203_6_fu_2692", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1203_6","ID" : "70","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1203_6","RefName" : "VITIS_LOOP_1203_6","ID" : "71","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9_fu_2744", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9","ID" : "72","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1215_9","RefName" : "VITIS_LOOP_1215_9","ID" : "73","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1555_24_fu_2824", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1555_24","ID" : "74","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1555_24","RefName" : "VITIS_LOOP_1555_24","ID" : "75","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1562_26_fu_2876", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1562_26","ID" : "76","Type" : "sequential",
				"SubLoops" : [
				{"Name" : "VITIS_LOOP_1562_26","RefName" : "VITIS_LOOP_1562_26","ID" : "77","Type" : "pipeline"},]},
			{"Name" : "call_ln671_mpc_persist_writeback_hls_fu_2952", "RefName" : "mpc_persist_writeback_hls","ID" : "78","Type" : "sequential"},],
			"SubLoops" : [
			{"Name" : "VITIS_LOOP_1231_11","RefName" : "VITIS_LOOP_1231_11","ID" : "79","Type" : "no",
			"SubInsts" : [
			{"Name" : "grp_riccati_pass_hls_fu_2053", "RefName" : "riccati_pass_hls","ID" : "80","Type" : "sequential",
					"SubInsts" : [
					{"Name" : "grp_riccati_backward_pass_fu_400", "RefName" : "riccati_backward_pass","ID" : "81","Type" : "sequential",
						"SubLoops" : [
						{"Name" : "VITIS_LOOP_535_4","RefName" : "VITIS_LOOP_535_4","ID" : "82","Type" : "no",
						"SubInsts" : [
						{"Name" : "grp_invert_2x2_qp_hls_fu_3970", "RefName" : "invert_2x2_qp_hls","ID" : "83","Type" : "sequential",
								"SubInsts" : [
								{"Name" : "fp_recip", "RefName" : "fp_recip","ID" : "84","Type" : "pipeline"},]},
						{"Name" : "fp_recip", "RefName" : "fp_recip","ID" : "85","Type" : "pipeline"},
						{"Name" : "grp_fp_recip_fu_3990", "RefName" : "fp_recip","ID" : "86","Type" : "pipeline"},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8_fu_3997", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8","ID" : "87","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_726_7_VITIS_LOOP_727_8","RefName" : "VITIS_LOOP_726_7_VITIS_LOOP_727_8","ID" : "88","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "sum_sum6_MG_QP_raw_fu_636", "RefName" : "sum6_MG_QP_raw","ID" : "89","Type" : "pipeline"},]},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_766_10_fu_4097", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_766_10","ID" : "90","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_766_10","RefName" : "VITIS_LOOP_766_10","ID" : "91","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "sum6_P_QP_raw", "RefName" : "sum6_P_QP_raw","ID" : "92","Type" : "pipeline"},]},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_750_9_fu_4129", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_750_9","ID" : "93","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_750_9","RefName" : "VITIS_LOOP_750_9","ID" : "94","Type" : "pipeline"},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13_fu_4280", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13","ID" : "95","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_831_12_VITIS_LOOP_832_13","RefName" : "VITIS_LOOP_831_12_VITIS_LOOP_832_13","ID" : "96","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "sum6_P_QP_raw", "RefName" : "sum6_P_QP_raw","ID" : "97","Type" : "pipeline"},]},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37_fu_4356", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37","ID" : "98","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_1013_37","RefName" : "VITIS_LOOP_1013_37","ID" : "99","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "total_sum8_P_MIX_raw_fu_604", "RefName" : "sum8_P_MIX_raw","ID" : "100","Type" : "pipeline"},]},]},
						{"Name" : "grp_riccati_backward_pass_Pipeline_VITIS_LOOP_1029_38_fu_4428", "RefName" : "riccati_backward_pass_Pipeline_VITIS_LOOP_1029_38","ID" : "101","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_1029_38","RefName" : "VITIS_LOOP_1029_38","ID" : "102","Type" : "pipeline"},]},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4437", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "103","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4449", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "104","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4461", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "105","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4473", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "106","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4485", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "107","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4497", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "108","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4509", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "109","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4521", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "110","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4533", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "111","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4545", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "112","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4557", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "113","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4569", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "114","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4581", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "115","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4593", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "116","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4605", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "117","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4617", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "118","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4629", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "119","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4641", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "120","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4653", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "121","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4665", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "122","Type" : "pipeline"},
						{"Name" : "grp_sum8_P_MIX_raw_pupdate_fu_4677", "RefName" : "sum8_P_MIX_raw_pupdate","ID" : "123","Type" : "pipeline"},]},]},
					{"Name" : "grp_riccati_forward_pass_fu_653", "RefName" : "riccati_forward_pass","ID" : "124","Type" : "sequential",
						"SubLoops" : [
						{"Name" : "VITIS_LOOP_332_2","RefName" : "VITIS_LOOP_332_2","ID" : "125","Type" : "no",
						"SubInsts" : [
						{"Name" : "prod_sum0_sum8_K_QP_raw_fu_2119", "RefName" : "sum8_K_QP_raw","ID" : "126","Type" : "pipeline"},
						{"Name" : "prod_sum1_sum8_K_QP_raw_fu_2131", "RefName" : "sum8_K_QP_raw","ID" : "127","Type" : "pipeline"},
						{"Name" : "grp_riccati_forward_pass_Pipeline_VITIS_LOOP_383_4_fu_2143", "RefName" : "riccati_forward_pass_Pipeline_VITIS_LOOP_383_4","ID" : "128","Type" : "sequential",
								"SubLoops" : [
								{"Name" : "VITIS_LOOP_383_4","RefName" : "VITIS_LOOP_383_4","ID" : "129","Type" : "pipeline",
								"SubInsts" : [
								{"Name" : "ax_sum_sum6_QP_raw_fu_690", "RefName" : "sum6_QP_raw","ID" : "130","Type" : "pipeline"},]},]},]},]},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19_fu_2760", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19","ID" : "131","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1337_19","RefName" : "VITIS_LOOP_1337_19","ID" : "132","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20_fu_2796", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20","ID" : "133","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1395_20","RefName" : "VITIS_LOOP_1395_20","ID" : "134","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1511_21_fu_2817", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1511_21","ID" : "135","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1511_21","RefName" : "VITIS_LOOP_1511_21","ID" : "136","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1525_22_fu_2892", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1525_22","ID" : "137","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1525_22","RefName" : "VITIS_LOOP_1525_22","ID" : "138","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1246_12_fu_2899", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1246_12","ID" : "139","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1246_12","RefName" : "VITIS_LOOP_1246_12","ID" : "140","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1282_14_fu_2923", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1282_14","ID" : "141","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1282_14","RefName" : "VITIS_LOOP_1282_14","ID" : "142","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16_fu_2932", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16","ID" : "143","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1301_16","RefName" : "VITIS_LOOP_1301_16","ID" : "144","Type" : "pipeline"},]},
			{"Name" : "grp_mpc_compute_hls_Pipeline_VITIS_LOOP_1305_17_fu_2942", "RefName" : "mpc_compute_hls_Pipeline_VITIS_LOOP_1305_17","ID" : "145","Type" : "sequential",
					"SubLoops" : [
					{"Name" : "VITIS_LOOP_1305_17","RefName" : "VITIS_LOOP_1305_17","ID" : "146","Type" : "pipeline"},]},]},]},]},]
}]}