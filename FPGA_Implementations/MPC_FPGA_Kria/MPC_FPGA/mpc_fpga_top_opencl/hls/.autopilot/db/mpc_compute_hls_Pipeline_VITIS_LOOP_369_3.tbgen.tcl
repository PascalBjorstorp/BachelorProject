set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_369_3
set isTopModule 0
set isCombinational 0
set isDatapathOnly 0
set isPipelined 1
set isPipelined_legacy 1
set pipeline_type loop_auto_rewind
set FunctionProtocol ap_ctrl_hs
set restart_counter_num 0
set isOneStateSeq 0
set ProfileFlag 0
set StallSigGenFlag 0
set isEnableWaveformDebug 1
set hasInterrupt 0
set DLRegFirstOffset 0
set DLRegItemOffset 0
set svuvm_can_support 1
set cdfgNum 74
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_369_3}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict step_data_0_0 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_0_1 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_0_2 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_0_3 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_0_4 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_0_5 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_2 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict B_sparse_3 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict B_sparse_2 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict B_sparse_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict B_sparse { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_3 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_4 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_5 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict ref_path_curvature { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_lateral_error { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_velocity { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_1 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_heading_error { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_lateral_velocity { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_yaw_rate { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ x0_1 int 26 regular  }
	{ state_omega int 26 regular  }
	{ state_vy int 26 regular  }
	{ zext_ln143 int 25 regular  }
	{ state_ey int 26 regular  }
	{ state_epsi int 26 regular  }
	{ sext_ln481 int 14 regular  }
	{ step_data_0_0 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ step_data_0_1 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ step_data_0_2 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ step_data_0_3 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ step_data_0_4 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ step_data_0_5 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ step_data_2 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ B_sparse_3 int 26 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ B_sparse_2 int 26 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ B_sparse_1 int 26 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ B_sparse int 26 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ step_data_3 int 26 regular {array 20 { 3 0 } 0 1 bus  }  }
	{ step_data_4 int 26 regular {array 20 { 3 0 } 0 1 bus  }  }
	{ step_data_5 int 26 regular {array 20 { 3 0 } 0 1 bus  }  }
	{ ref_path_curvature int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ wall_left_min_reload int 26 regular  }
	{ wall_left_min_1_reload int 26 regular  }
	{ wall_left_min_2_reload int 26 regular  }
	{ wall_left_min_3_reload int 26 regular  }
	{ wall_left_min_4_reload int 26 regular  }
	{ wall_left_min_5_reload int 26 regular  }
	{ wall_left_min_6_reload int 26 regular  }
	{ wall_left_min_7_reload int 26 regular  }
	{ wall_left_min_8_reload int 26 regular  }
	{ wall_left_min_9_reload int 26 regular  }
	{ wall_left_min_10_reload int 26 regular  }
	{ wall_left_min_11_reload int 26 regular  }
	{ wall_left_min_12_reload int 26 regular  }
	{ wall_left_min_13_reload int 26 regular  }
	{ wall_left_min_14_reload int 26 regular  }
	{ wall_left_min_15_reload int 26 regular  }
	{ wall_left_min_16_reload int 26 regular  }
	{ wall_left_min_17_reload int 26 regular  }
	{ wall_left_min_18_reload int 26 regular  }
	{ wall_left_min_19_reload int 26 regular  }
	{ wall_right_min_reload int 26 regular  }
	{ wall_right_min_1_reload int 26 regular  }
	{ wall_right_min_2_reload int 26 regular  }
	{ wall_right_min_3_reload int 26 regular  }
	{ wall_right_min_4_reload int 26 regular  }
	{ wall_right_min_5_reload int 26 regular  }
	{ wall_right_min_6_reload int 26 regular  }
	{ wall_right_min_7_reload int 26 regular  }
	{ wall_right_min_8_reload int 26 regular  }
	{ wall_right_min_9_reload int 26 regular  }
	{ wall_right_min_10_reload int 26 regular  }
	{ wall_right_min_11_reload int 26 regular  }
	{ wall_right_min_12_reload int 26 regular  }
	{ wall_right_min_13_reload int 26 regular  }
	{ wall_right_min_14_reload int 26 regular  }
	{ wall_right_min_15_reload int 26 regular  }
	{ wall_right_min_16_reload int 26 regular  }
	{ wall_right_min_17_reload int 26 regular  }
	{ wall_right_min_18_reload int 26 regular  }
	{ wall_right_min_19_reload int 26 regular  }
	{ ref_reference_lateral_error int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_velocity int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ empty int 23 regular  }
	{ x0_2 int 26 regular  }
	{ step_data_1 int 26 regular {array 120 { 3 0 } 0 1 bus  }  }
	{ ref_reference_heading_error int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_lateral_velocity int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_yaw_rate int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ out_136_out int 26 regular {pointer 1}  }
	{ terminal_dff_raw_0169_0_0_0_out int 26 regular {pointer 1}  }
	{ terminal_wall_x_ub_con_out int 26 regular {pointer 1}  }
	{ terminal_wall_x_lb_con_out int 26 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "x0_1", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "state_omega", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "state_vy", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "zext_ln143", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "state_ey", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "state_epsi", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln481", "interface" : "wire", "bitwidth" : 14, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_0_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_0_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_0_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_0_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_0_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "B_sparse_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "B_sparse_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "B_sparse_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "B_sparse", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "step_data_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "ref_path_curvature", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_1_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_2_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_3_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_4_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_5_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_6_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_7_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_8_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_9_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_10_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_11_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_12_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_13_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_14_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_15_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_16_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_17_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_18_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_left_min_19_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_1_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_2_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_3_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_4_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_5_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_6_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_7_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_8_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_9_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_10_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_11_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_12_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_13_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_14_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_15_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_16_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_17_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_18_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "wall_right_min_19_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_lateral_error", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_velocity", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "empty", "interface" : "wire", "bitwidth" : 23, "direction" : "READONLY"} , 
 	{ "Name" : "x0_2", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "ref_reference_heading_error", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_lateral_velocity", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_yaw_rate", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "out_136_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "terminal_dff_raw_0169_0_0_0_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "terminal_wall_x_ub_con_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "terminal_wall_x_lb_con_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 155
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ x0_1 sc_in sc_lv 26 signal 0 } 
	{ state_omega sc_in sc_lv 26 signal 1 } 
	{ state_vy sc_in sc_lv 26 signal 2 } 
	{ zext_ln143 sc_in sc_lv 25 signal 3 } 
	{ state_ey sc_in sc_lv 26 signal 4 } 
	{ state_epsi sc_in sc_lv 26 signal 5 } 
	{ sext_ln481 sc_in sc_lv 14 signal 6 } 
	{ step_data_0_0_address1 sc_out sc_lv 7 signal 7 } 
	{ step_data_0_0_ce1 sc_out sc_logic 1 signal 7 } 
	{ step_data_0_0_we1 sc_out sc_logic 1 signal 7 } 
	{ step_data_0_0_d1 sc_out sc_lv 26 signal 7 } 
	{ step_data_0_1_address1 sc_out sc_lv 7 signal 8 } 
	{ step_data_0_1_ce1 sc_out sc_logic 1 signal 8 } 
	{ step_data_0_1_we1 sc_out sc_logic 1 signal 8 } 
	{ step_data_0_1_d1 sc_out sc_lv 26 signal 8 } 
	{ step_data_0_2_address1 sc_out sc_lv 7 signal 9 } 
	{ step_data_0_2_ce1 sc_out sc_logic 1 signal 9 } 
	{ step_data_0_2_we1 sc_out sc_logic 1 signal 9 } 
	{ step_data_0_2_d1 sc_out sc_lv 26 signal 9 } 
	{ step_data_0_3_address1 sc_out sc_lv 7 signal 10 } 
	{ step_data_0_3_ce1 sc_out sc_logic 1 signal 10 } 
	{ step_data_0_3_we1 sc_out sc_logic 1 signal 10 } 
	{ step_data_0_3_d1 sc_out sc_lv 26 signal 10 } 
	{ step_data_0_4_address1 sc_out sc_lv 7 signal 11 } 
	{ step_data_0_4_ce1 sc_out sc_logic 1 signal 11 } 
	{ step_data_0_4_we1 sc_out sc_logic 1 signal 11 } 
	{ step_data_0_4_d1 sc_out sc_lv 26 signal 11 } 
	{ step_data_0_5_address1 sc_out sc_lv 7 signal 12 } 
	{ step_data_0_5_ce1 sc_out sc_logic 1 signal 12 } 
	{ step_data_0_5_we1 sc_out sc_logic 1 signal 12 } 
	{ step_data_0_5_d1 sc_out sc_lv 26 signal 12 } 
	{ step_data_2_address1 sc_out sc_lv 7 signal 13 } 
	{ step_data_2_ce1 sc_out sc_logic 1 signal 13 } 
	{ step_data_2_we1 sc_out sc_logic 1 signal 13 } 
	{ step_data_2_d1 sc_out sc_lv 26 signal 13 } 
	{ B_sparse_3_address0 sc_out sc_lv 5 signal 14 } 
	{ B_sparse_3_ce0 sc_out sc_logic 1 signal 14 } 
	{ B_sparse_3_we0 sc_out sc_logic 1 signal 14 } 
	{ B_sparse_3_d0 sc_out sc_lv 26 signal 14 } 
	{ B_sparse_2_address0 sc_out sc_lv 5 signal 15 } 
	{ B_sparse_2_ce0 sc_out sc_logic 1 signal 15 } 
	{ B_sparse_2_we0 sc_out sc_logic 1 signal 15 } 
	{ B_sparse_2_d0 sc_out sc_lv 26 signal 15 } 
	{ B_sparse_1_address0 sc_out sc_lv 5 signal 16 } 
	{ B_sparse_1_ce0 sc_out sc_logic 1 signal 16 } 
	{ B_sparse_1_we0 sc_out sc_logic 1 signal 16 } 
	{ B_sparse_1_d0 sc_out sc_lv 26 signal 16 } 
	{ B_sparse_address0 sc_out sc_lv 5 signal 17 } 
	{ B_sparse_ce0 sc_out sc_logic 1 signal 17 } 
	{ B_sparse_we0 sc_out sc_logic 1 signal 17 } 
	{ B_sparse_d0 sc_out sc_lv 26 signal 17 } 
	{ step_data_3_address1 sc_out sc_lv 5 signal 18 } 
	{ step_data_3_ce1 sc_out sc_logic 1 signal 18 } 
	{ step_data_3_we1 sc_out sc_logic 1 signal 18 } 
	{ step_data_3_d1 sc_out sc_lv 26 signal 18 } 
	{ step_data_4_address1 sc_out sc_lv 5 signal 19 } 
	{ step_data_4_ce1 sc_out sc_logic 1 signal 19 } 
	{ step_data_4_we1 sc_out sc_logic 1 signal 19 } 
	{ step_data_4_d1 sc_out sc_lv 26 signal 19 } 
	{ step_data_5_address1 sc_out sc_lv 5 signal 20 } 
	{ step_data_5_ce1 sc_out sc_logic 1 signal 20 } 
	{ step_data_5_we1 sc_out sc_logic 1 signal 20 } 
	{ step_data_5_d1 sc_out sc_lv 26 signal 20 } 
	{ ref_path_curvature_address0 sc_out sc_lv 5 signal 21 } 
	{ ref_path_curvature_ce0 sc_out sc_logic 1 signal 21 } 
	{ ref_path_curvature_q0 sc_in sc_lv 26 signal 21 } 
	{ wall_left_min_reload sc_in sc_lv 26 signal 22 } 
	{ wall_left_min_1_reload sc_in sc_lv 26 signal 23 } 
	{ wall_left_min_2_reload sc_in sc_lv 26 signal 24 } 
	{ wall_left_min_3_reload sc_in sc_lv 26 signal 25 } 
	{ wall_left_min_4_reload sc_in sc_lv 26 signal 26 } 
	{ wall_left_min_5_reload sc_in sc_lv 26 signal 27 } 
	{ wall_left_min_6_reload sc_in sc_lv 26 signal 28 } 
	{ wall_left_min_7_reload sc_in sc_lv 26 signal 29 } 
	{ wall_left_min_8_reload sc_in sc_lv 26 signal 30 } 
	{ wall_left_min_9_reload sc_in sc_lv 26 signal 31 } 
	{ wall_left_min_10_reload sc_in sc_lv 26 signal 32 } 
	{ wall_left_min_11_reload sc_in sc_lv 26 signal 33 } 
	{ wall_left_min_12_reload sc_in sc_lv 26 signal 34 } 
	{ wall_left_min_13_reload sc_in sc_lv 26 signal 35 } 
	{ wall_left_min_14_reload sc_in sc_lv 26 signal 36 } 
	{ wall_left_min_15_reload sc_in sc_lv 26 signal 37 } 
	{ wall_left_min_16_reload sc_in sc_lv 26 signal 38 } 
	{ wall_left_min_17_reload sc_in sc_lv 26 signal 39 } 
	{ wall_left_min_18_reload sc_in sc_lv 26 signal 40 } 
	{ wall_left_min_19_reload sc_in sc_lv 26 signal 41 } 
	{ wall_right_min_reload sc_in sc_lv 26 signal 42 } 
	{ wall_right_min_1_reload sc_in sc_lv 26 signal 43 } 
	{ wall_right_min_2_reload sc_in sc_lv 26 signal 44 } 
	{ wall_right_min_3_reload sc_in sc_lv 26 signal 45 } 
	{ wall_right_min_4_reload sc_in sc_lv 26 signal 46 } 
	{ wall_right_min_5_reload sc_in sc_lv 26 signal 47 } 
	{ wall_right_min_6_reload sc_in sc_lv 26 signal 48 } 
	{ wall_right_min_7_reload sc_in sc_lv 26 signal 49 } 
	{ wall_right_min_8_reload sc_in sc_lv 26 signal 50 } 
	{ wall_right_min_9_reload sc_in sc_lv 26 signal 51 } 
	{ wall_right_min_10_reload sc_in sc_lv 26 signal 52 } 
	{ wall_right_min_11_reload sc_in sc_lv 26 signal 53 } 
	{ wall_right_min_12_reload sc_in sc_lv 26 signal 54 } 
	{ wall_right_min_13_reload sc_in sc_lv 26 signal 55 } 
	{ wall_right_min_14_reload sc_in sc_lv 26 signal 56 } 
	{ wall_right_min_15_reload sc_in sc_lv 26 signal 57 } 
	{ wall_right_min_16_reload sc_in sc_lv 26 signal 58 } 
	{ wall_right_min_17_reload sc_in sc_lv 26 signal 59 } 
	{ wall_right_min_18_reload sc_in sc_lv 26 signal 60 } 
	{ wall_right_min_19_reload sc_in sc_lv 26 signal 61 } 
	{ ref_reference_lateral_error_address0 sc_out sc_lv 5 signal 62 } 
	{ ref_reference_lateral_error_ce0 sc_out sc_logic 1 signal 62 } 
	{ ref_reference_lateral_error_q0 sc_in sc_lv 26 signal 62 } 
	{ ref_reference_velocity_address0 sc_out sc_lv 5 signal 63 } 
	{ ref_reference_velocity_ce0 sc_out sc_logic 1 signal 63 } 
	{ ref_reference_velocity_q0 sc_in sc_lv 26 signal 63 } 
	{ empty sc_in sc_lv 23 signal 64 } 
	{ x0_2 sc_in sc_lv 26 signal 65 } 
	{ step_data_1_address1 sc_out sc_lv 7 signal 66 } 
	{ step_data_1_ce1 sc_out sc_logic 1 signal 66 } 
	{ step_data_1_we1 sc_out sc_logic 1 signal 66 } 
	{ step_data_1_d1 sc_out sc_lv 26 signal 66 } 
	{ ref_reference_heading_error_address0 sc_out sc_lv 5 signal 67 } 
	{ ref_reference_heading_error_ce0 sc_out sc_logic 1 signal 67 } 
	{ ref_reference_heading_error_q0 sc_in sc_lv 26 signal 67 } 
	{ ref_reference_lateral_velocity_address0 sc_out sc_lv 5 signal 68 } 
	{ ref_reference_lateral_velocity_ce0 sc_out sc_logic 1 signal 68 } 
	{ ref_reference_lateral_velocity_q0 sc_in sc_lv 26 signal 68 } 
	{ ref_reference_yaw_rate_address0 sc_out sc_lv 5 signal 69 } 
	{ ref_reference_yaw_rate_ce0 sc_out sc_logic 1 signal 69 } 
	{ ref_reference_yaw_rate_q0 sc_in sc_lv 26 signal 69 } 
	{ out_136_out sc_out sc_lv 26 signal 70 } 
	{ out_136_out_ap_vld sc_out sc_logic 1 outvld 70 } 
	{ terminal_dff_raw_0169_0_0_0_out sc_out sc_lv 26 signal 71 } 
	{ terminal_dff_raw_0169_0_0_0_out_ap_vld sc_out sc_logic 1 outvld 71 } 
	{ terminal_wall_x_ub_con_out sc_out sc_lv 26 signal 72 } 
	{ terminal_wall_x_ub_con_out_ap_vld sc_out sc_logic 1 outvld 72 } 
	{ terminal_wall_x_lb_con_out sc_out sc_lv 26 signal 73 } 
	{ terminal_wall_x_lb_con_out_ap_vld sc_out sc_logic 1 outvld 73 } 
	{ grp_fp_recip_fu_5937_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fp_recip_fu_5937_p_dout0 sc_in sc_lv 17 signal -1 } 
	{ grp_fu_2774_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_2774_p_din1 sc_out sc_lv 20 signal -1 } 
	{ grp_fu_2774_p_dout0 sc_in sc_lv 46 signal -1 } 
	{ grp_fu_2774_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_2779_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_2779_p_din1 sc_out sc_lv 22 signal -1 } 
	{ grp_fu_2779_p_dout0 sc_in sc_lv 48 signal -1 } 
	{ grp_fu_2779_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_2784_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_2784_p_din1 sc_out sc_lv 25 signal -1 } 
	{ grp_fu_2784_p_dout0 sc_in sc_lv 51 signal -1 } 
	{ grp_fu_2784_p_ce sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "x0_1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x0_1", "role": "default" }} , 
 	{ "name": "state_omega", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_omega", "role": "default" }} , 
 	{ "name": "state_vy", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_vy", "role": "default" }} , 
 	{ "name": "zext_ln143", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "zext_ln143", "role": "default" }} , 
 	{ "name": "state_ey", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_ey", "role": "default" }} , 
 	{ "name": "state_epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_epsi", "role": "default" }} , 
 	{ "name": "sext_ln481", "direction": "in", "datatype": "sc_lv", "bitwidth":14, "type": "signal", "bundle":{"name": "sext_ln481", "role": "default" }} , 
 	{ "name": "step_data_0_0_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_0", "role": "address1" }} , 
 	{ "name": "step_data_0_0_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0", "role": "ce1" }} , 
 	{ "name": "step_data_0_0_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0", "role": "we1" }} , 
 	{ "name": "step_data_0_0_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_0", "role": "d1" }} , 
 	{ "name": "step_data_0_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_1", "role": "address1" }} , 
 	{ "name": "step_data_0_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1", "role": "ce1" }} , 
 	{ "name": "step_data_0_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1", "role": "we1" }} , 
 	{ "name": "step_data_0_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_1", "role": "d1" }} , 
 	{ "name": "step_data_0_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_2", "role": "address1" }} , 
 	{ "name": "step_data_0_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2", "role": "ce1" }} , 
 	{ "name": "step_data_0_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2", "role": "we1" }} , 
 	{ "name": "step_data_0_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_2", "role": "d1" }} , 
 	{ "name": "step_data_0_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_3", "role": "address1" }} , 
 	{ "name": "step_data_0_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3", "role": "ce1" }} , 
 	{ "name": "step_data_0_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3", "role": "we1" }} , 
 	{ "name": "step_data_0_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_3", "role": "d1" }} , 
 	{ "name": "step_data_0_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_4", "role": "address1" }} , 
 	{ "name": "step_data_0_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4", "role": "ce1" }} , 
 	{ "name": "step_data_0_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4", "role": "we1" }} , 
 	{ "name": "step_data_0_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_4", "role": "d1" }} , 
 	{ "name": "step_data_0_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_5", "role": "address1" }} , 
 	{ "name": "step_data_0_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5", "role": "ce1" }} , 
 	{ "name": "step_data_0_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5", "role": "we1" }} , 
 	{ "name": "step_data_0_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_5", "role": "d1" }} , 
 	{ "name": "step_data_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_2", "role": "address1" }} , 
 	{ "name": "step_data_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_2", "role": "ce1" }} , 
 	{ "name": "step_data_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_2", "role": "we1" }} , 
 	{ "name": "step_data_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_2", "role": "d1" }} , 
 	{ "name": "B_sparse_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "address0" }} , 
 	{ "name": "B_sparse_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "ce0" }} , 
 	{ "name": "B_sparse_3_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "we0" }} , 
 	{ "name": "B_sparse_3_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "d0" }} , 
 	{ "name": "B_sparse_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "address0" }} , 
 	{ "name": "B_sparse_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "ce0" }} , 
 	{ "name": "B_sparse_2_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "we0" }} , 
 	{ "name": "B_sparse_2_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "d0" }} , 
 	{ "name": "B_sparse_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "address0" }} , 
 	{ "name": "B_sparse_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "ce0" }} , 
 	{ "name": "B_sparse_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "we0" }} , 
 	{ "name": "B_sparse_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "d0" }} , 
 	{ "name": "B_sparse_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse", "role": "address0" }} , 
 	{ "name": "B_sparse_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse", "role": "ce0" }} , 
 	{ "name": "B_sparse_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse", "role": "we0" }} , 
 	{ "name": "B_sparse_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse", "role": "d0" }} , 
 	{ "name": "step_data_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_3", "role": "address1" }} , 
 	{ "name": "step_data_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_3", "role": "ce1" }} , 
 	{ "name": "step_data_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_3", "role": "we1" }} , 
 	{ "name": "step_data_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_3", "role": "d1" }} , 
 	{ "name": "step_data_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_4", "role": "address1" }} , 
 	{ "name": "step_data_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_4", "role": "ce1" }} , 
 	{ "name": "step_data_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_4", "role": "we1" }} , 
 	{ "name": "step_data_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_4", "role": "d1" }} , 
 	{ "name": "step_data_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_5", "role": "address1" }} , 
 	{ "name": "step_data_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_5", "role": "ce1" }} , 
 	{ "name": "step_data_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_5", "role": "we1" }} , 
 	{ "name": "step_data_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_5", "role": "d1" }} , 
 	{ "name": "ref_path_curvature_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "address0" }} , 
 	{ "name": "ref_path_curvature_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "ce0" }} , 
 	{ "name": "ref_path_curvature_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "q0" }} , 
 	{ "name": "wall_left_min_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_1_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_1_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_2_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_2_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_3_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_3_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_4_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_4_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_5_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_5_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_6_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_6_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_7_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_7_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_8_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_8_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_9_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_9_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_10_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_10_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_11_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_11_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_12_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_12_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_13_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_13_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_14_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_14_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_15_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_15_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_16_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_16_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_17_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_17_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_18_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_18_reload", "role": "default" }} , 
 	{ "name": "wall_left_min_19_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_left_min_19_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_1_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_1_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_2_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_2_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_3_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_3_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_4_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_4_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_5_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_5_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_6_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_6_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_7_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_7_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_8_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_8_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_9_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_9_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_10_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_10_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_11_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_11_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_12_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_12_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_13_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_13_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_14_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_14_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_15_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_15_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_16_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_16_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_17_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_17_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_18_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_18_reload", "role": "default" }} , 
 	{ "name": "wall_right_min_19_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "wall_right_min_19_reload", "role": "default" }} , 
 	{ "name": "ref_reference_lateral_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "address0" }} , 
 	{ "name": "ref_reference_lateral_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "ce0" }} , 
 	{ "name": "ref_reference_lateral_error_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "q0" }} , 
 	{ "name": "ref_reference_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "address0" }} , 
 	{ "name": "ref_reference_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "ce0" }} , 
 	{ "name": "ref_reference_velocity_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "q0" }} , 
 	{ "name": "empty", "direction": "in", "datatype": "sc_lv", "bitwidth":23, "type": "signal", "bundle":{"name": "empty", "role": "default" }} , 
 	{ "name": "x0_2", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x0_2", "role": "default" }} , 
 	{ "name": "step_data_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_1", "role": "address1" }} , 
 	{ "name": "step_data_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_1", "role": "ce1" }} , 
 	{ "name": "step_data_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_1", "role": "we1" }} , 
 	{ "name": "step_data_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_1", "role": "d1" }} , 
 	{ "name": "ref_reference_heading_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "address0" }} , 
 	{ "name": "ref_reference_heading_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "ce0" }} , 
 	{ "name": "ref_reference_heading_error_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "q0" }} , 
 	{ "name": "ref_reference_lateral_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "address0" }} , 
 	{ "name": "ref_reference_lateral_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "ce0" }} , 
 	{ "name": "ref_reference_lateral_velocity_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "q0" }} , 
 	{ "name": "ref_reference_yaw_rate_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "address0" }} , 
 	{ "name": "ref_reference_yaw_rate_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "ce0" }} , 
 	{ "name": "ref_reference_yaw_rate_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "q0" }} , 
 	{ "name": "out_136_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "out_136_out", "role": "default" }} , 
 	{ "name": "out_136_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "out_136_out", "role": "ap_vld" }} , 
 	{ "name": "terminal_dff_raw_0169_0_0_0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "terminal_dff_raw_0169_0_0_0_out", "role": "default" }} , 
 	{ "name": "terminal_dff_raw_0169_0_0_0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "terminal_dff_raw_0169_0_0_0_out", "role": "ap_vld" }} , 
 	{ "name": "terminal_wall_x_ub_con_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "terminal_wall_x_ub_con_out", "role": "default" }} , 
 	{ "name": "terminal_wall_x_ub_con_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "terminal_wall_x_ub_con_out", "role": "ap_vld" }} , 
 	{ "name": "terminal_wall_x_lb_con_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "terminal_wall_x_lb_con_out", "role": "default" }} , 
 	{ "name": "terminal_wall_x_lb_con_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "terminal_wall_x_lb_con_out", "role": "ap_vld" }} , 
 	{ "name": "grp_fp_recip_fu_5937_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fp_recip_fu_5937_p_din1", "role": "default" }} , 
 	{ "name": "grp_fp_recip_fu_5937_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "grp_fp_recip_fu_5937_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_2774_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_2774_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_2774_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":20, "type": "signal", "bundle":{"name": "grp_fu_2774_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_2774_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":46, "type": "signal", "bundle":{"name": "grp_fu_2774_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_2774_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_2774_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_2779_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_2779_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_2779_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":22, "type": "signal", "bundle":{"name": "grp_fu_2779_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_2779_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":48, "type": "signal", "bundle":{"name": "grp_fu_2779_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_2779_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_2779_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_2784_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_2784_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_2784_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "grp_fu_2784_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_2784_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":51, "type": "signal", "bundle":{"name": "grp_fu_2784_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_2784_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_2784_p_ce", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_compute_hls_Pipeline_VITIS_LOOP_369_3 {
		x0_1 {Type I LastRead 0 FirstWrite -1}
		state_omega {Type I LastRead 0 FirstWrite -1}
		state_vy {Type I LastRead 0 FirstWrite -1}
		zext_ln143 {Type I LastRead 0 FirstWrite -1}
		state_ey {Type I LastRead 0 FirstWrite -1}
		state_epsi {Type I LastRead 0 FirstWrite -1}
		sext_ln481 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0 {Type O LastRead -1 FirstWrite 0}
		step_data_0_1 {Type O LastRead -1 FirstWrite 0}
		step_data_0_2 {Type O LastRead -1 FirstWrite 0}
		step_data_0_3 {Type O LastRead -1 FirstWrite 0}
		step_data_0_4 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5 {Type O LastRead -1 FirstWrite 0}
		step_data_2 {Type O LastRead -1 FirstWrite 5}
		B_sparse_3 {Type O LastRead -1 FirstWrite 82}
		B_sparse_2 {Type O LastRead -1 FirstWrite 82}
		B_sparse_1 {Type O LastRead -1 FirstWrite 82}
		B_sparse {Type O LastRead -1 FirstWrite 0}
		step_data_3 {Type O LastRead -1 FirstWrite 1}
		step_data_4 {Type O LastRead -1 FirstWrite 1}
		step_data_5 {Type O LastRead -1 FirstWrite 23}
		ref_path_curvature {Type I LastRead 0 FirstWrite -1}
		wall_left_min_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_1_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_2_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_3_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_4_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_5_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_6_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_7_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_8_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_9_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_10_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_11_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_12_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_13_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_14_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_15_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_16_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_17_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_18_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_19_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_1_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_2_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_3_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_4_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_5_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_6_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_7_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_8_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_9_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_10_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_11_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_12_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_13_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_14_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_15_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_16_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_17_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_18_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_19_reload {Type I LastRead 0 FirstWrite -1}
		ref_reference_lateral_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_velocity {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		x0_2 {Type I LastRead 0 FirstWrite -1}
		step_data_1 {Type O LastRead -1 FirstWrite 27}
		ref_reference_heading_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_lateral_velocity {Type I LastRead 0 FirstWrite -1}
		ref_reference_yaw_rate {Type I LastRead 0 FirstWrite -1}
		out_136_out {Type O LastRead -1 FirstWrite 83}
		terminal_dff_raw_0169_0_0_0_out {Type O LastRead -1 FirstWrite 83}
		terminal_wall_x_ub_con_out {Type O LastRead -1 FirstWrite 83}
		terminal_wall_x_lb_con_out {Type O LastRead -1 FirstWrite 83}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}}
	fp_atan_lut {
		x {Type I LastRead 0 FirstWrite -1}
		atan_lut {Type I LastRead -1 FirstWrite -1}}
	compute_frenet_AB_and_next_hls {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 0 FirstWrite -1}
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 0 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		delta {Type I LastRead 0 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_frenet_AB_hls {
		ey {Type I LastRead 49 FirstWrite -1}
		epsi {Type I LastRead 45 FirstWrite -1}
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 0 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		delta {Type I LastRead 0 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		kappa {Type I LastRead 49 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_frenet_tire_hls {
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 1 FirstWrite -1}
		omega {Type I LastRead 8 FirstWrite -1}
		delta {Type I LastRead 17 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_slip_terms {
		vy {Type I LastRead 1 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		inv_vx {Type I LastRead 1 FirstWrite -1}}
	compute_front_tire_path_fn {
		conv5_i_i_i_i1781_i {Type I LastRead 5 FirstWrite -1}
		p_0_0_03467_i {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		conv_i_i_i1795_i {Type I LastRead 20 FirstWrite -1}
		conv_i_i_i2437_i {Type I LastRead 25 FirstWrite -1}
		p_0_0_03463_i {Type I LastRead 25 FirstWrite -1}
		spec_select_i {Type I LastRead 25 FirstWrite -1}
		conv_i_i_i65_i {Type I LastRead 0 FirstWrite -1}
		cmp_i_i {Type I LastRead 32 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_atan_lut_fn {
		x {Type I LastRead 0 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_pacejka_ceff {
		cos_inner {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 0 FirstWrite -1}
		D_cb {Type I LastRead 2 FirstWrite -1}}
	fp_front_force_jacobians_fn {
		C_eff_f {Type I LastRead 3 FirstWrite -1}
		front_num {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_f {Type I LastRead 0 FirstWrite -1}}
	compute_rear_tire_path_fn {
		rear_ratio {Type I LastRead 0 FirstWrite -1}
		D_transfer {Type I LastRead 0 FirstWrite -1}
		D_pac_r {Type I LastRead 20 FirstWrite -1}
		C_min_r {Type I LastRead 25 FirstWrite -1}
		rear_num {Type I LastRead 25 FirstWrite -1}
		vx_safe {Type I LastRead 25 FirstWrite -1}
		D_r_fn {Type I LastRead 0 FirstWrite -1}
		low_speed {Type I LastRead 31 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_atan_lut_fn {
		x {Type I LastRead 0 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_pacejka_ceff {
		cos_inner {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 0 FirstWrite -1}
		D_cb {Type I LastRead 2 FirstWrite -1}}
	fp_rear_force_jacobians_fn {
		C_eff_r {Type I LastRead 2 FirstWrite -1}
		rear_num {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_r {Type I LastRead 0 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_frenet_recip {
		kappa {Type I LastRead 0 FirstWrite -1}
		ey {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_stage_shared_products {
		tr_dFyf_dvx_val {Type I LastRead 0 FirstWrite -1}
		tr_dFyf_dvy_val {Type I LastRead 1 FirstWrite -1}
		tr_dFyf_dom_val {Type I LastRead 2 FirstWrite -1}
		tr_dFyr_dvx_val {Type I LastRead 2 FirstWrite -1}
		tr_dFyr_dvy_val {Type I LastRead 3 FirstWrite -1}
		tr_dFyr_dom_val {Type I LastRead 4 FirstWrite -1}
		p_0_0_05039 {Type I LastRead 0 FirstWrite -1}
		p_0_0_05038 {Type I LastRead 3 FirstWrite -1}
		conv5_i_i_i_i465 {Type I LastRead 1 FirstWrite -1}
		conv5_i_i_i_i629 {Type I LastRead 0 FirstWrite -1}}
	compute_B_accel_load_transfer {
		tr_C_eff_f_raw_val {Type I LastRead 7 FirstWrite -1}
		tr_C_eff_r_val {Type I LastRead 9 FirstWrite -1}
		tr_C_min_f_val {Type I LastRead 8 FirstWrite -1}
		tr_cos_delta_val {Type I LastRead 16 FirstWrite -1}
		tr_alpha_f_op_val {Type I LastRead 10 FirstWrite -1}
		tr_alpha_r_op_val {Type I LastRead 12 FirstWrite -1}
		tr_Fz_transfer_val {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_rollout_from_forces_fn {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 18 FirstWrite -1}
		sin_epsi {Type I LastRead 5 FirstWrite -1}
		cos_epsi {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 1 FirstWrite -1}
		vy {Type I LastRead 6 FirstWrite -1}
		omega {Type I LastRead 7 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		Fx {Type I LastRead 6 FirstWrite -1}
		F_yf {Type I LastRead 2 FirstWrite -1}
		F_yr {Type I LastRead 0 FirstWrite -1}
		sin_delta {Type I LastRead 3 FirstWrite -1}
		cos_delta {Type I LastRead 2 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_B_steering {
		tr_C_eff_f_raw_val {Type I LastRead 0 FirstWrite -1}
		tr_C_min_f_val {Type I LastRead 1 FirstWrite -1}
		tr_F_yf_val {Type I LastRead 2 FirstWrite -1}
		tr_sin_delta_val {Type I LastRead 2 FirstWrite -1}
		tr_cos_delta_val {Type I LastRead 0 FirstWrite -1}}
	fp_frenet_rows01_fn {
		sin_epsi {Type I LastRead 2 FirstWrite -1}
		cos_epsi {Type I LastRead 1 FirstWrite -1}
		vx {Type I LastRead 1 FirstWrite -1}
		vy {Type I LastRead 8 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 3 FirstWrite -1}}
	compute_affine_bias_dense_hls {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 5 FirstWrite -1}
		p_read2 {Type I LastRead 10 FirstWrite -1}
		p_read3 {Type I LastRead 15 FirstWrite -1}
		p_read4 {Type I LastRead 20 FirstWrite -1}
		p_read5 {Type I LastRead 1 FirstWrite -1}
		p_read6 {Type I LastRead 6 FirstWrite -1}
		p_read7 {Type I LastRead 11 FirstWrite -1}
		p_read8 {Type I LastRead 16 FirstWrite -1}
		p_read9 {Type I LastRead 21 FirstWrite -1}
		p_read10 {Type I LastRead 2 FirstWrite -1}
		p_read11 {Type I LastRead 7 FirstWrite -1}
		p_read12 {Type I LastRead 12 FirstWrite -1}
		p_read13 {Type I LastRead 17 FirstWrite -1}
		p_read14 {Type I LastRead 22 FirstWrite -1}
		p_read15 {Type I LastRead 3 FirstWrite -1}
		p_read16 {Type I LastRead 8 FirstWrite -1}
		p_read17 {Type I LastRead 13 FirstWrite -1}
		p_read18 {Type I LastRead 18 FirstWrite -1}
		p_read19 {Type I LastRead 23 FirstWrite -1}
		p_read20 {Type I LastRead 4 FirstWrite -1}
		p_read21 {Type I LastRead 9 FirstWrite -1}
		p_read22 {Type I LastRead 14 FirstWrite -1}
		p_read23 {Type I LastRead 19 FirstWrite -1}
		p_read24 {Type I LastRead 24 FirstWrite -1}
		p_read25 {Type I LastRead 0 FirstWrite -1}
		p_read26 {Type I LastRead 2 FirstWrite -1}
		p_read27 {Type I LastRead 4 FirstWrite -1}
		p_read28 {Type I LastRead 6 FirstWrite -1}
		p_read29 {Type I LastRead 8 FirstWrite -1}
		p_read30 {Type I LastRead 25 FirstWrite -1}
		p_read31 {Type I LastRead 26 FirstWrite -1}
		p_read32 {Type I LastRead 27 FirstWrite -1}
		p_read33 {Type I LastRead 28 FirstWrite -1}
		p_read34 {Type I LastRead 29 FirstWrite -1}
		lin_ey {Type I LastRead 0 FirstWrite -1}
		lin_epsi {Type I LastRead 1 FirstWrite -1}
		lin_vx {Type I LastRead 2 FirstWrite -1}
		lin_vy {Type I LastRead 3 FirstWrite -1}
		lin_omega {Type I LastRead 4 FirstWrite -1}
		lin_delta {Type I LastRead 32 FirstWrite -1}
		uk0 {Type I LastRead 30 FirstWrite -1}
		uk1 {Type I LastRead 25 FirstWrite -1}
		lin_delta_k {Type I LastRead 0 FirstWrite -1}
		next_ey {Type I LastRead 4 FirstWrite -1}
		next_epsi {Type I LastRead 9 FirstWrite -1}
		next_vx {Type I LastRead 14 FirstWrite -1}
		next_vy {Type I LastRead 19 FirstWrite -1}
		next_omega {Type I LastRead 24 FirstWrite -1}
		next_delta {Type I LastRead 32 FirstWrite -1}
		sd_1 {Type O LastRead -1 FirstWrite 27}
		sd_1_offset {Type I LastRead 27 FirstWrite -1}}
	affine_b0_term_hls {
		b0_coeff_raw {Type I LastRead 0 FirstWrite -1}
		lin_delta_k_raw {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "1565", "Max" : "1565"}
	, {"Name" : "Interval", "Min" : "1554", "Max" : "1554"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	x0_1 { ap_none {  { x0_1 in_data 0 26 } } }
	state_omega { ap_none {  { state_omega in_data 0 26 } } }
	state_vy { ap_none {  { state_vy in_data 0 26 } } }
	zext_ln143 { ap_none {  { zext_ln143 in_data 0 25 } } }
	state_ey { ap_none {  { state_ey in_data 0 26 } } }
	state_epsi { ap_none {  { state_epsi in_data 0 26 } } }
	sext_ln481 { ap_none {  { sext_ln481 in_data 0 14 } } }
	step_data_0_0 { ap_memory {  { step_data_0_0_address1 MemPortADDR2 1 7 }  { step_data_0_0_ce1 MemPortCE2 1 1 }  { step_data_0_0_we1 MemPortWE2 1 1 }  { step_data_0_0_d1 MemPortDIN2 1 26 } } }
	step_data_0_1 { ap_memory {  { step_data_0_1_address1 MemPortADDR2 1 7 }  { step_data_0_1_ce1 MemPortCE2 1 1 }  { step_data_0_1_we1 MemPortWE2 1 1 }  { step_data_0_1_d1 MemPortDIN2 1 26 } } }
	step_data_0_2 { ap_memory {  { step_data_0_2_address1 MemPortADDR2 1 7 }  { step_data_0_2_ce1 MemPortCE2 1 1 }  { step_data_0_2_we1 MemPortWE2 1 1 }  { step_data_0_2_d1 MemPortDIN2 1 26 } } }
	step_data_0_3 { ap_memory {  { step_data_0_3_address1 MemPortADDR2 1 7 }  { step_data_0_3_ce1 MemPortCE2 1 1 }  { step_data_0_3_we1 MemPortWE2 1 1 }  { step_data_0_3_d1 MemPortDIN2 1 26 } } }
	step_data_0_4 { ap_memory {  { step_data_0_4_address1 MemPortADDR2 1 7 }  { step_data_0_4_ce1 MemPortCE2 1 1 }  { step_data_0_4_we1 MemPortWE2 1 1 }  { step_data_0_4_d1 MemPortDIN2 1 26 } } }
	step_data_0_5 { ap_memory {  { step_data_0_5_address1 MemPortADDR2 1 7 }  { step_data_0_5_ce1 MemPortCE2 1 1 }  { step_data_0_5_we1 MemPortWE2 1 1 }  { step_data_0_5_d1 MemPortDIN2 1 26 } } }
	step_data_2 { ap_memory {  { step_data_2_address1 MemPortADDR2 1 7 }  { step_data_2_ce1 MemPortCE2 1 1 }  { step_data_2_we1 MemPortWE2 1 1 }  { step_data_2_d1 MemPortDIN2 1 26 } } }
	B_sparse_3 { ap_memory {  { B_sparse_3_address0 mem_address 1 5 }  { B_sparse_3_ce0 mem_ce 1 1 }  { B_sparse_3_we0 mem_we 1 1 }  { B_sparse_3_d0 mem_din 1 26 } } }
	B_sparse_2 { ap_memory {  { B_sparse_2_address0 mem_address 1 5 }  { B_sparse_2_ce0 mem_ce 1 1 }  { B_sparse_2_we0 mem_we 1 1 }  { B_sparse_2_d0 mem_din 1 26 } } }
	B_sparse_1 { ap_memory {  { B_sparse_1_address0 mem_address 1 5 }  { B_sparse_1_ce0 mem_ce 1 1 }  { B_sparse_1_we0 mem_we 1 1 }  { B_sparse_1_d0 mem_din 1 26 } } }
	B_sparse { ap_memory {  { B_sparse_address0 mem_address 1 5 }  { B_sparse_ce0 mem_ce 1 1 }  { B_sparse_we0 mem_we 1 1 }  { B_sparse_d0 mem_din 1 26 } } }
	step_data_3 { ap_memory {  { step_data_3_address1 MemPortADDR2 1 5 }  { step_data_3_ce1 MemPortCE2 1 1 }  { step_data_3_we1 MemPortWE2 1 1 }  { step_data_3_d1 MemPortDIN2 1 26 } } }
	step_data_4 { ap_memory {  { step_data_4_address1 MemPortADDR2 1 5 }  { step_data_4_ce1 MemPortCE2 1 1 }  { step_data_4_we1 MemPortWE2 1 1 }  { step_data_4_d1 MemPortDIN2 1 26 } } }
	step_data_5 { ap_memory {  { step_data_5_address1 MemPortADDR2 1 5 }  { step_data_5_ce1 MemPortCE2 1 1 }  { step_data_5_we1 MemPortWE2 1 1 }  { step_data_5_d1 MemPortDIN2 1 26 } } }
	ref_path_curvature { ap_memory {  { ref_path_curvature_address0 mem_address 1 5 }  { ref_path_curvature_ce0 mem_ce 1 1 }  { ref_path_curvature_q0 mem_dout 0 26 } } }
	wall_left_min_reload { ap_none {  { wall_left_min_reload in_data 0 26 } } }
	wall_left_min_1_reload { ap_none {  { wall_left_min_1_reload in_data 0 26 } } }
	wall_left_min_2_reload { ap_none {  { wall_left_min_2_reload in_data 0 26 } } }
	wall_left_min_3_reload { ap_none {  { wall_left_min_3_reload in_data 0 26 } } }
	wall_left_min_4_reload { ap_none {  { wall_left_min_4_reload in_data 0 26 } } }
	wall_left_min_5_reload { ap_none {  { wall_left_min_5_reload in_data 0 26 } } }
	wall_left_min_6_reload { ap_none {  { wall_left_min_6_reload in_data 0 26 } } }
	wall_left_min_7_reload { ap_none {  { wall_left_min_7_reload in_data 0 26 } } }
	wall_left_min_8_reload { ap_none {  { wall_left_min_8_reload in_data 0 26 } } }
	wall_left_min_9_reload { ap_none {  { wall_left_min_9_reload in_data 0 26 } } }
	wall_left_min_10_reload { ap_none {  { wall_left_min_10_reload in_data 0 26 } } }
	wall_left_min_11_reload { ap_none {  { wall_left_min_11_reload in_data 0 26 } } }
	wall_left_min_12_reload { ap_none {  { wall_left_min_12_reload in_data 0 26 } } }
	wall_left_min_13_reload { ap_none {  { wall_left_min_13_reload in_data 0 26 } } }
	wall_left_min_14_reload { ap_none {  { wall_left_min_14_reload in_data 0 26 } } }
	wall_left_min_15_reload { ap_none {  { wall_left_min_15_reload in_data 0 26 } } }
	wall_left_min_16_reload { ap_none {  { wall_left_min_16_reload in_data 0 26 } } }
	wall_left_min_17_reload { ap_none {  { wall_left_min_17_reload in_data 0 26 } } }
	wall_left_min_18_reload { ap_none {  { wall_left_min_18_reload in_data 0 26 } } }
	wall_left_min_19_reload { ap_none {  { wall_left_min_19_reload in_data 0 26 } } }
	wall_right_min_reload { ap_none {  { wall_right_min_reload in_data 0 26 } } }
	wall_right_min_1_reload { ap_none {  { wall_right_min_1_reload in_data 0 26 } } }
	wall_right_min_2_reload { ap_none {  { wall_right_min_2_reload in_data 0 26 } } }
	wall_right_min_3_reload { ap_none {  { wall_right_min_3_reload in_data 0 26 } } }
	wall_right_min_4_reload { ap_none {  { wall_right_min_4_reload in_data 0 26 } } }
	wall_right_min_5_reload { ap_none {  { wall_right_min_5_reload in_data 0 26 } } }
	wall_right_min_6_reload { ap_none {  { wall_right_min_6_reload in_data 0 26 } } }
	wall_right_min_7_reload { ap_none {  { wall_right_min_7_reload in_data 0 26 } } }
	wall_right_min_8_reload { ap_none {  { wall_right_min_8_reload in_data 0 26 } } }
	wall_right_min_9_reload { ap_none {  { wall_right_min_9_reload in_data 0 26 } } }
	wall_right_min_10_reload { ap_none {  { wall_right_min_10_reload in_data 0 26 } } }
	wall_right_min_11_reload { ap_none {  { wall_right_min_11_reload in_data 0 26 } } }
	wall_right_min_12_reload { ap_none {  { wall_right_min_12_reload in_data 0 26 } } }
	wall_right_min_13_reload { ap_none {  { wall_right_min_13_reload in_data 0 26 } } }
	wall_right_min_14_reload { ap_none {  { wall_right_min_14_reload in_data 0 26 } } }
	wall_right_min_15_reload { ap_none {  { wall_right_min_15_reload in_data 0 26 } } }
	wall_right_min_16_reload { ap_none {  { wall_right_min_16_reload in_data 0 26 } } }
	wall_right_min_17_reload { ap_none {  { wall_right_min_17_reload in_data 0 26 } } }
	wall_right_min_18_reload { ap_none {  { wall_right_min_18_reload in_data 0 26 } } }
	wall_right_min_19_reload { ap_none {  { wall_right_min_19_reload in_data 0 26 } } }
	ref_reference_lateral_error { ap_memory {  { ref_reference_lateral_error_address0 mem_address 1 5 }  { ref_reference_lateral_error_ce0 mem_ce 1 1 }  { ref_reference_lateral_error_q0 mem_dout 0 26 } } }
	ref_reference_velocity { ap_memory {  { ref_reference_velocity_address0 mem_address 1 5 }  { ref_reference_velocity_ce0 mem_ce 1 1 }  { ref_reference_velocity_q0 mem_dout 0 26 } } }
	empty { ap_none {  { empty in_data 0 23 } } }
	x0_2 { ap_none {  { x0_2 in_data 0 26 } } }
	step_data_1 { ap_memory {  { step_data_1_address1 MemPortADDR2 1 7 }  { step_data_1_ce1 MemPortCE2 1 1 }  { step_data_1_we1 MemPortWE2 1 1 }  { step_data_1_d1 MemPortDIN2 1 26 } } }
	ref_reference_heading_error { ap_memory {  { ref_reference_heading_error_address0 mem_address 1 5 }  { ref_reference_heading_error_ce0 mem_ce 1 1 }  { ref_reference_heading_error_q0 mem_dout 0 26 } } }
	ref_reference_lateral_velocity { ap_memory {  { ref_reference_lateral_velocity_address0 mem_address 1 5 }  { ref_reference_lateral_velocity_ce0 mem_ce 1 1 }  { ref_reference_lateral_velocity_q0 mem_dout 0 26 } } }
	ref_reference_yaw_rate { ap_memory {  { ref_reference_yaw_rate_address0 mem_address 1 5 }  { ref_reference_yaw_rate_ce0 mem_ce 1 1 }  { ref_reference_yaw_rate_q0 mem_dout 0 26 } } }
	out_136_out { ap_vld {  { out_136_out out_data 1 26 }  { out_136_out_ap_vld out_vld 1 1 } } }
	terminal_dff_raw_0169_0_0_0_out { ap_vld {  { terminal_dff_raw_0169_0_0_0_out out_data 1 26 }  { terminal_dff_raw_0169_0_0_0_out_ap_vld out_vld 1 1 } } }
	terminal_wall_x_ub_con_out { ap_vld {  { terminal_wall_x_ub_con_out out_data 1 26 }  { terminal_wall_x_ub_con_out_ap_vld out_vld 1 1 } } }
	terminal_wall_x_lb_con_out { ap_vld {  { terminal_wall_x_lb_con_out out_data 1 26 }  { terminal_wall_x_lb_con_out_ap_vld out_vld 1 1 } } }
}
