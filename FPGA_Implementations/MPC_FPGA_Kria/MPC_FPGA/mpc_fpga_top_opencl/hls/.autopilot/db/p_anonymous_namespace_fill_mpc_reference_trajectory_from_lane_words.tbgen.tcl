set moduleName p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words
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
set cdfgNum 76
set C_modelName {(anonymous namespace)fill_mpc_reference_trajectory_from_lane_words}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict lane_words_0 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_1 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_2 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_3 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_4 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_5 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_6 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_7 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_8 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_9 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_10 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_11 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_12 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_13 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_14 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict lane_words_15 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict out_ref_reference_heading_error { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict out_ref_reference_lateral_error { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict out_ref_reference_velocity { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict out_ref_reference_lateral_velocity { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict out_ref_reference_yaw_rate { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict out_ref_path_curvature { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict out_ref_left_wall_bound { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict out_ref_right_wall_bound { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ lane_words_0 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_1 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_2 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_3 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_4 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_5 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_6 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_7 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_8 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_9 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_10 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_11 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_12 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_13 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_14 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ lane_words_15 int 32 regular {array 11 { 1 3 } 1 1 bus  }  }
	{ out_ref_reference_heading_error int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ out_ref_reference_lateral_error int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ out_ref_reference_velocity int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ out_ref_reference_lateral_velocity int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ out_ref_reference_yaw_rate int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ out_ref_path_curvature int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ out_ref_left_wall_bound int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ out_ref_right_wall_bound int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "lane_words_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_6", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_7", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_8", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_9", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_10", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_11", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_12", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_13", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_14", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_15", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "out_ref_reference_heading_error", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_ref_reference_lateral_error", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_ref_reference_velocity", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_ref_reference_lateral_velocity", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_ref_reference_yaw_rate", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_ref_path_curvature", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_ref_left_wall_bound", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_ref_right_wall_bound", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 86
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ lane_words_0_address0 sc_out sc_lv 4 signal 0 } 
	{ lane_words_0_ce0 sc_out sc_logic 1 signal 0 } 
	{ lane_words_0_q0 sc_in sc_lv 32 signal 0 } 
	{ lane_words_1_address0 sc_out sc_lv 4 signal 1 } 
	{ lane_words_1_ce0 sc_out sc_logic 1 signal 1 } 
	{ lane_words_1_q0 sc_in sc_lv 32 signal 1 } 
	{ lane_words_2_address0 sc_out sc_lv 4 signal 2 } 
	{ lane_words_2_ce0 sc_out sc_logic 1 signal 2 } 
	{ lane_words_2_q0 sc_in sc_lv 32 signal 2 } 
	{ lane_words_3_address0 sc_out sc_lv 4 signal 3 } 
	{ lane_words_3_ce0 sc_out sc_logic 1 signal 3 } 
	{ lane_words_3_q0 sc_in sc_lv 32 signal 3 } 
	{ lane_words_4_address0 sc_out sc_lv 4 signal 4 } 
	{ lane_words_4_ce0 sc_out sc_logic 1 signal 4 } 
	{ lane_words_4_q0 sc_in sc_lv 32 signal 4 } 
	{ lane_words_5_address0 sc_out sc_lv 4 signal 5 } 
	{ lane_words_5_ce0 sc_out sc_logic 1 signal 5 } 
	{ lane_words_5_q0 sc_in sc_lv 32 signal 5 } 
	{ lane_words_6_address0 sc_out sc_lv 4 signal 6 } 
	{ lane_words_6_ce0 sc_out sc_logic 1 signal 6 } 
	{ lane_words_6_q0 sc_in sc_lv 32 signal 6 } 
	{ lane_words_7_address0 sc_out sc_lv 4 signal 7 } 
	{ lane_words_7_ce0 sc_out sc_logic 1 signal 7 } 
	{ lane_words_7_q0 sc_in sc_lv 32 signal 7 } 
	{ lane_words_8_address0 sc_out sc_lv 4 signal 8 } 
	{ lane_words_8_ce0 sc_out sc_logic 1 signal 8 } 
	{ lane_words_8_q0 sc_in sc_lv 32 signal 8 } 
	{ lane_words_9_address0 sc_out sc_lv 4 signal 9 } 
	{ lane_words_9_ce0 sc_out sc_logic 1 signal 9 } 
	{ lane_words_9_q0 sc_in sc_lv 32 signal 9 } 
	{ lane_words_10_address0 sc_out sc_lv 4 signal 10 } 
	{ lane_words_10_ce0 sc_out sc_logic 1 signal 10 } 
	{ lane_words_10_q0 sc_in sc_lv 32 signal 10 } 
	{ lane_words_11_address0 sc_out sc_lv 4 signal 11 } 
	{ lane_words_11_ce0 sc_out sc_logic 1 signal 11 } 
	{ lane_words_11_q0 sc_in sc_lv 32 signal 11 } 
	{ lane_words_12_address0 sc_out sc_lv 4 signal 12 } 
	{ lane_words_12_ce0 sc_out sc_logic 1 signal 12 } 
	{ lane_words_12_q0 sc_in sc_lv 32 signal 12 } 
	{ lane_words_13_address0 sc_out sc_lv 4 signal 13 } 
	{ lane_words_13_ce0 sc_out sc_logic 1 signal 13 } 
	{ lane_words_13_q0 sc_in sc_lv 32 signal 13 } 
	{ lane_words_14_address0 sc_out sc_lv 4 signal 14 } 
	{ lane_words_14_ce0 sc_out sc_logic 1 signal 14 } 
	{ lane_words_14_q0 sc_in sc_lv 32 signal 14 } 
	{ lane_words_15_address0 sc_out sc_lv 4 signal 15 } 
	{ lane_words_15_ce0 sc_out sc_logic 1 signal 15 } 
	{ lane_words_15_q0 sc_in sc_lv 32 signal 15 } 
	{ out_ref_reference_heading_error_address0 sc_out sc_lv 5 signal 16 } 
	{ out_ref_reference_heading_error_ce0 sc_out sc_logic 1 signal 16 } 
	{ out_ref_reference_heading_error_we0 sc_out sc_logic 1 signal 16 } 
	{ out_ref_reference_heading_error_d0 sc_out sc_lv 32 signal 16 } 
	{ out_ref_reference_lateral_error_address0 sc_out sc_lv 5 signal 17 } 
	{ out_ref_reference_lateral_error_ce0 sc_out sc_logic 1 signal 17 } 
	{ out_ref_reference_lateral_error_we0 sc_out sc_logic 1 signal 17 } 
	{ out_ref_reference_lateral_error_d0 sc_out sc_lv 32 signal 17 } 
	{ out_ref_reference_velocity_address0 sc_out sc_lv 5 signal 18 } 
	{ out_ref_reference_velocity_ce0 sc_out sc_logic 1 signal 18 } 
	{ out_ref_reference_velocity_we0 sc_out sc_logic 1 signal 18 } 
	{ out_ref_reference_velocity_d0 sc_out sc_lv 32 signal 18 } 
	{ out_ref_reference_lateral_velocity_address0 sc_out sc_lv 5 signal 19 } 
	{ out_ref_reference_lateral_velocity_ce0 sc_out sc_logic 1 signal 19 } 
	{ out_ref_reference_lateral_velocity_we0 sc_out sc_logic 1 signal 19 } 
	{ out_ref_reference_lateral_velocity_d0 sc_out sc_lv 32 signal 19 } 
	{ out_ref_reference_yaw_rate_address0 sc_out sc_lv 5 signal 20 } 
	{ out_ref_reference_yaw_rate_ce0 sc_out sc_logic 1 signal 20 } 
	{ out_ref_reference_yaw_rate_we0 sc_out sc_logic 1 signal 20 } 
	{ out_ref_reference_yaw_rate_d0 sc_out sc_lv 32 signal 20 } 
	{ out_ref_path_curvature_address0 sc_out sc_lv 5 signal 21 } 
	{ out_ref_path_curvature_ce0 sc_out sc_logic 1 signal 21 } 
	{ out_ref_path_curvature_we0 sc_out sc_logic 1 signal 21 } 
	{ out_ref_path_curvature_d0 sc_out sc_lv 32 signal 21 } 
	{ out_ref_left_wall_bound_address0 sc_out sc_lv 5 signal 22 } 
	{ out_ref_left_wall_bound_ce0 sc_out sc_logic 1 signal 22 } 
	{ out_ref_left_wall_bound_we0 sc_out sc_logic 1 signal 22 } 
	{ out_ref_left_wall_bound_d0 sc_out sc_lv 32 signal 22 } 
	{ out_ref_right_wall_bound_address0 sc_out sc_lv 5 signal 23 } 
	{ out_ref_right_wall_bound_ce0 sc_out sc_logic 1 signal 23 } 
	{ out_ref_right_wall_bound_we0 sc_out sc_logic 1 signal 23 } 
	{ out_ref_right_wall_bound_d0 sc_out sc_lv 32 signal 23 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "lane_words_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_0", "role": "address0" }} , 
 	{ "name": "lane_words_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_0", "role": "ce0" }} , 
 	{ "name": "lane_words_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_0", "role": "q0" }} , 
 	{ "name": "lane_words_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_1", "role": "address0" }} , 
 	{ "name": "lane_words_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_1", "role": "ce0" }} , 
 	{ "name": "lane_words_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_1", "role": "q0" }} , 
 	{ "name": "lane_words_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_2", "role": "address0" }} , 
 	{ "name": "lane_words_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_2", "role": "ce0" }} , 
 	{ "name": "lane_words_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_2", "role": "q0" }} , 
 	{ "name": "lane_words_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_3", "role": "address0" }} , 
 	{ "name": "lane_words_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_3", "role": "ce0" }} , 
 	{ "name": "lane_words_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_3", "role": "q0" }} , 
 	{ "name": "lane_words_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_4", "role": "address0" }} , 
 	{ "name": "lane_words_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_4", "role": "ce0" }} , 
 	{ "name": "lane_words_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_4", "role": "q0" }} , 
 	{ "name": "lane_words_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_5", "role": "address0" }} , 
 	{ "name": "lane_words_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_5", "role": "ce0" }} , 
 	{ "name": "lane_words_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_5", "role": "q0" }} , 
 	{ "name": "lane_words_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_6", "role": "address0" }} , 
 	{ "name": "lane_words_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_6", "role": "ce0" }} , 
 	{ "name": "lane_words_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_6", "role": "q0" }} , 
 	{ "name": "lane_words_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_7", "role": "address0" }} , 
 	{ "name": "lane_words_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_7", "role": "ce0" }} , 
 	{ "name": "lane_words_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_7", "role": "q0" }} , 
 	{ "name": "lane_words_8_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_8", "role": "address0" }} , 
 	{ "name": "lane_words_8_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_8", "role": "ce0" }} , 
 	{ "name": "lane_words_8_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_8", "role": "q0" }} , 
 	{ "name": "lane_words_9_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_9", "role": "address0" }} , 
 	{ "name": "lane_words_9_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_9", "role": "ce0" }} , 
 	{ "name": "lane_words_9_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_9", "role": "q0" }} , 
 	{ "name": "lane_words_10_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_10", "role": "address0" }} , 
 	{ "name": "lane_words_10_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_10", "role": "ce0" }} , 
 	{ "name": "lane_words_10_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_10", "role": "q0" }} , 
 	{ "name": "lane_words_11_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_11", "role": "address0" }} , 
 	{ "name": "lane_words_11_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_11", "role": "ce0" }} , 
 	{ "name": "lane_words_11_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_11", "role": "q0" }} , 
 	{ "name": "lane_words_12_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_12", "role": "address0" }} , 
 	{ "name": "lane_words_12_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_12", "role": "ce0" }} , 
 	{ "name": "lane_words_12_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_12", "role": "q0" }} , 
 	{ "name": "lane_words_13_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_13", "role": "address0" }} , 
 	{ "name": "lane_words_13_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_13", "role": "ce0" }} , 
 	{ "name": "lane_words_13_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_13", "role": "q0" }} , 
 	{ "name": "lane_words_14_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_14", "role": "address0" }} , 
 	{ "name": "lane_words_14_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_14", "role": "ce0" }} , 
 	{ "name": "lane_words_14_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_14", "role": "q0" }} , 
 	{ "name": "lane_words_15_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_15", "role": "address0" }} , 
 	{ "name": "lane_words_15_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_15", "role": "ce0" }} , 
 	{ "name": "lane_words_15_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_15", "role": "q0" }} , 
 	{ "name": "out_ref_reference_heading_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_reference_heading_error", "role": "address0" }} , 
 	{ "name": "out_ref_reference_heading_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_heading_error", "role": "ce0" }} , 
 	{ "name": "out_ref_reference_heading_error_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_heading_error", "role": "we0" }} , 
 	{ "name": "out_ref_reference_heading_error_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_reference_heading_error", "role": "d0" }} , 
 	{ "name": "out_ref_reference_lateral_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_error", "role": "address0" }} , 
 	{ "name": "out_ref_reference_lateral_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_error", "role": "ce0" }} , 
 	{ "name": "out_ref_reference_lateral_error_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_error", "role": "we0" }} , 
 	{ "name": "out_ref_reference_lateral_error_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_error", "role": "d0" }} , 
 	{ "name": "out_ref_reference_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_reference_velocity", "role": "address0" }} , 
 	{ "name": "out_ref_reference_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_velocity", "role": "ce0" }} , 
 	{ "name": "out_ref_reference_velocity_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_velocity", "role": "we0" }} , 
 	{ "name": "out_ref_reference_velocity_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_reference_velocity", "role": "d0" }} , 
 	{ "name": "out_ref_reference_lateral_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_velocity", "role": "address0" }} , 
 	{ "name": "out_ref_reference_lateral_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_velocity", "role": "ce0" }} , 
 	{ "name": "out_ref_reference_lateral_velocity_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_velocity", "role": "we0" }} , 
 	{ "name": "out_ref_reference_lateral_velocity_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_reference_lateral_velocity", "role": "d0" }} , 
 	{ "name": "out_ref_reference_yaw_rate_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_reference_yaw_rate", "role": "address0" }} , 
 	{ "name": "out_ref_reference_yaw_rate_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_yaw_rate", "role": "ce0" }} , 
 	{ "name": "out_ref_reference_yaw_rate_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_reference_yaw_rate", "role": "we0" }} , 
 	{ "name": "out_ref_reference_yaw_rate_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_reference_yaw_rate", "role": "d0" }} , 
 	{ "name": "out_ref_path_curvature_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_path_curvature", "role": "address0" }} , 
 	{ "name": "out_ref_path_curvature_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_path_curvature", "role": "ce0" }} , 
 	{ "name": "out_ref_path_curvature_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_path_curvature", "role": "we0" }} , 
 	{ "name": "out_ref_path_curvature_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_path_curvature", "role": "d0" }} , 
 	{ "name": "out_ref_left_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_left_wall_bound", "role": "address0" }} , 
 	{ "name": "out_ref_left_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_left_wall_bound", "role": "ce0" }} , 
 	{ "name": "out_ref_left_wall_bound_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_left_wall_bound", "role": "we0" }} , 
 	{ "name": "out_ref_left_wall_bound_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_left_wall_bound", "role": "d0" }} , 
 	{ "name": "out_ref_right_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "out_ref_right_wall_bound", "role": "address0" }} , 
 	{ "name": "out_ref_right_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_right_wall_bound", "role": "ce0" }} , 
 	{ "name": "out_ref_right_wall_bound_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "out_ref_right_wall_bound", "role": "we0" }} , 
 	{ "name": "out_ref_right_wall_bound_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_ref_right_wall_bound", "role": "d0" }}  ]}

set ArgLastReadFirstWriteLatency {
	p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words {
		lane_words_0 {Type I LastRead 0 FirstWrite -1}
		lane_words_1 {Type I LastRead 0 FirstWrite -1}
		lane_words_2 {Type I LastRead 0 FirstWrite -1}
		lane_words_3 {Type I LastRead 0 FirstWrite -1}
		lane_words_4 {Type I LastRead 0 FirstWrite -1}
		lane_words_5 {Type I LastRead 0 FirstWrite -1}
		lane_words_6 {Type I LastRead 0 FirstWrite -1}
		lane_words_7 {Type I LastRead 0 FirstWrite -1}
		lane_words_8 {Type I LastRead 0 FirstWrite -1}
		lane_words_9 {Type I LastRead 0 FirstWrite -1}
		lane_words_10 {Type I LastRead 0 FirstWrite -1}
		lane_words_11 {Type I LastRead 0 FirstWrite -1}
		lane_words_12 {Type I LastRead 0 FirstWrite -1}
		lane_words_13 {Type I LastRead 0 FirstWrite -1}
		lane_words_14 {Type I LastRead 0 FirstWrite -1}
		lane_words_15 {Type I LastRead 0 FirstWrite -1}
		out_ref_reference_heading_error {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_lateral_error {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_velocity {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_lateral_velocity {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_yaw_rate {Type O LastRead -1 FirstWrite 1}
		out_ref_path_curvature {Type O LastRead -1 FirstWrite 1}
		out_ref_left_wall_bound {Type O LastRead -1 FirstWrite 1}
		out_ref_right_wall_bound {Type O LastRead -1 FirstWrite 1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "22", "Max" : "22"}
	, {"Name" : "Interval", "Min" : "21", "Max" : "21"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	lane_words_0 { ap_memory {  { lane_words_0_address0 mem_address 1 4 }  { lane_words_0_ce0 mem_ce 1 1 }  { lane_words_0_q0 mem_dout 0 32 } } }
	lane_words_1 { ap_memory {  { lane_words_1_address0 mem_address 1 4 }  { lane_words_1_ce0 mem_ce 1 1 }  { lane_words_1_q0 mem_dout 0 32 } } }
	lane_words_2 { ap_memory {  { lane_words_2_address0 mem_address 1 4 }  { lane_words_2_ce0 mem_ce 1 1 }  { lane_words_2_q0 mem_dout 0 32 } } }
	lane_words_3 { ap_memory {  { lane_words_3_address0 mem_address 1 4 }  { lane_words_3_ce0 mem_ce 1 1 }  { lane_words_3_q0 mem_dout 0 32 } } }
	lane_words_4 { ap_memory {  { lane_words_4_address0 mem_address 1 4 }  { lane_words_4_ce0 mem_ce 1 1 }  { lane_words_4_q0 mem_dout 0 32 } } }
	lane_words_5 { ap_memory {  { lane_words_5_address0 mem_address 1 4 }  { lane_words_5_ce0 mem_ce 1 1 }  { lane_words_5_q0 mem_dout 0 32 } } }
	lane_words_6 { ap_memory {  { lane_words_6_address0 mem_address 1 4 }  { lane_words_6_ce0 mem_ce 1 1 }  { lane_words_6_q0 mem_dout 0 32 } } }
	lane_words_7 { ap_memory {  { lane_words_7_address0 mem_address 1 4 }  { lane_words_7_ce0 mem_ce 1 1 }  { lane_words_7_q0 mem_dout 0 32 } } }
	lane_words_8 { ap_memory {  { lane_words_8_address0 mem_address 1 4 }  { lane_words_8_ce0 mem_ce 1 1 }  { lane_words_8_q0 mem_dout 0 32 } } }
	lane_words_9 { ap_memory {  { lane_words_9_address0 mem_address 1 4 }  { lane_words_9_ce0 mem_ce 1 1 }  { lane_words_9_q0 mem_dout 0 32 } } }
	lane_words_10 { ap_memory {  { lane_words_10_address0 mem_address 1 4 }  { lane_words_10_ce0 mem_ce 1 1 }  { lane_words_10_q0 mem_dout 0 32 } } }
	lane_words_11 { ap_memory {  { lane_words_11_address0 mem_address 1 4 }  { lane_words_11_ce0 mem_ce 1 1 }  { lane_words_11_q0 mem_dout 0 32 } } }
	lane_words_12 { ap_memory {  { lane_words_12_address0 mem_address 1 4 }  { lane_words_12_ce0 mem_ce 1 1 }  { lane_words_12_q0 mem_dout 0 32 } } }
	lane_words_13 { ap_memory {  { lane_words_13_address0 mem_address 1 4 }  { lane_words_13_ce0 mem_ce 1 1 }  { lane_words_13_q0 mem_dout 0 32 } } }
	lane_words_14 { ap_memory {  { lane_words_14_address0 mem_address 1 4 }  { lane_words_14_ce0 mem_ce 1 1 }  { lane_words_14_q0 mem_dout 0 32 } } }
	lane_words_15 { ap_memory {  { lane_words_15_address0 mem_address 1 4 }  { lane_words_15_ce0 mem_ce 1 1 }  { lane_words_15_q0 mem_dout 0 32 } } }
	out_ref_reference_heading_error { ap_memory {  { out_ref_reference_heading_error_address0 mem_address 1 5 }  { out_ref_reference_heading_error_ce0 mem_ce 1 1 }  { out_ref_reference_heading_error_we0 mem_we 1 1 }  { out_ref_reference_heading_error_d0 mem_din 1 32 } } }
	out_ref_reference_lateral_error { ap_memory {  { out_ref_reference_lateral_error_address0 mem_address 1 5 }  { out_ref_reference_lateral_error_ce0 mem_ce 1 1 }  { out_ref_reference_lateral_error_we0 mem_we 1 1 }  { out_ref_reference_lateral_error_d0 mem_din 1 32 } } }
	out_ref_reference_velocity { ap_memory {  { out_ref_reference_velocity_address0 mem_address 1 5 }  { out_ref_reference_velocity_ce0 mem_ce 1 1 }  { out_ref_reference_velocity_we0 mem_we 1 1 }  { out_ref_reference_velocity_d0 mem_din 1 32 } } }
	out_ref_reference_lateral_velocity { ap_memory {  { out_ref_reference_lateral_velocity_address0 mem_address 1 5 }  { out_ref_reference_lateral_velocity_ce0 mem_ce 1 1 }  { out_ref_reference_lateral_velocity_we0 mem_we 1 1 }  { out_ref_reference_lateral_velocity_d0 mem_din 1 32 } } }
	out_ref_reference_yaw_rate { ap_memory {  { out_ref_reference_yaw_rate_address0 mem_address 1 5 }  { out_ref_reference_yaw_rate_ce0 mem_ce 1 1 }  { out_ref_reference_yaw_rate_we0 mem_we 1 1 }  { out_ref_reference_yaw_rate_d0 mem_din 1 32 } } }
	out_ref_path_curvature { ap_memory {  { out_ref_path_curvature_address0 mem_address 1 5 }  { out_ref_path_curvature_ce0 mem_ce 1 1 }  { out_ref_path_curvature_we0 mem_we 1 1 }  { out_ref_path_curvature_d0 mem_din 1 32 } } }
	out_ref_left_wall_bound { ap_memory {  { out_ref_left_wall_bound_address0 mem_address 1 5 }  { out_ref_left_wall_bound_ce0 mem_ce 1 1 }  { out_ref_left_wall_bound_we0 mem_we 1 1 }  { out_ref_left_wall_bound_d0 mem_din 1 32 } } }
	out_ref_right_wall_bound { ap_memory {  { out_ref_right_wall_bound_address0 mem_address 1 5 }  { out_ref_right_wall_bound_ce0 mem_ce 1 1 }  { out_ref_right_wall_bound_we0 mem_we 1 1 }  { out_ref_right_wall_bound_d0 mem_din 1 32 } } }
}
