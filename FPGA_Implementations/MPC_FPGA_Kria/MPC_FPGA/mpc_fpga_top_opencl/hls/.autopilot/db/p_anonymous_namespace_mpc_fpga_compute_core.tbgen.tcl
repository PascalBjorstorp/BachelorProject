set moduleName p_anonymous_namespace_mpc_fpga_compute_core
set isTopModule 0
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set isPipelined_legacy 0
set pipeline_type none
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
set C_modelName {(anonymous namespace)mpc_fpga_compute_core}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict ref_reference_heading_error { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_lateral_error { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_velocity { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_lateral_velocity { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_yaw_rate { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_path_curvature { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_left_wall_bound { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_right_wall_bound { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ ey int 32 regular  }
	{ epsi int 32 regular  }
	{ vx int 32 regular  }
	{ vy int 32 regular  }
	{ omega int 32 regular  }
	{ steering int 32 regular  }
	{ prev_accel int 32 regular  }
	{ control_flags int 3 regular  }
	{ ref_reference_heading_error int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_lateral_error int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_velocity int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_lateral_velocity int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_yaw_rate int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_path_curvature int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_left_wall_bound int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_right_wall_bound int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ out_steering int 32 regular {pointer 1}  }
	{ out_accel int 32 regular {pointer 1}  }
	{ out_status int 1 regular {pointer 1}  }
	{ out_iters int 32 regular {pointer 1}  }
	{ out_steering_arg_index int 11 regular {pointer 0} {global 0}  }
	{ out_accel_arg_index int 11 regular {pointer 0} {global 0}  }
	{ out_status_arg_index int 11 regular {pointer 0} {global 0}  }
	{ out_iters_arg_index int 11 regular {pointer 0} {global 0}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "ey", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "epsi", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "vx", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "vy", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "omega", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "steering", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "prev_accel", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "control_flags", "interface" : "wire", "bitwidth" : 3, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_heading_error", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_lateral_error", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_velocity", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_lateral_velocity", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_yaw_rate", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_path_curvature", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_left_wall_bound", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_right_wall_bound", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "out_steering", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_accel", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_status", "interface" : "wire", "bitwidth" : 1, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_iters", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "out_steering_arg_index", "interface" : "wire", "bitwidth" : 11, "direction" : "READONLY", "extern" : 0} , 
 	{ "Name" : "out_accel_arg_index", "interface" : "wire", "bitwidth" : 11, "direction" : "READONLY", "extern" : 0} , 
 	{ "Name" : "out_status_arg_index", "interface" : "wire", "bitwidth" : 11, "direction" : "READONLY", "extern" : 0} , 
 	{ "Name" : "out_iters_arg_index", "interface" : "wire", "bitwidth" : 11, "direction" : "READONLY", "extern" : 0} ]}
# RTL Port declarations: 
set portNum 50
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ ey sc_in sc_lv 32 signal 0 } 
	{ epsi sc_in sc_lv 32 signal 1 } 
	{ vx sc_in sc_lv 32 signal 2 } 
	{ vy sc_in sc_lv 32 signal 3 } 
	{ omega sc_in sc_lv 32 signal 4 } 
	{ steering sc_in sc_lv 32 signal 5 } 
	{ prev_accel sc_in sc_lv 32 signal 6 } 
	{ control_flags sc_in sc_lv 3 signal 7 } 
	{ ref_reference_heading_error_address0 sc_out sc_lv 5 signal 8 } 
	{ ref_reference_heading_error_ce0 sc_out sc_logic 1 signal 8 } 
	{ ref_reference_heading_error_q0 sc_in sc_lv 32 signal 8 } 
	{ ref_reference_lateral_error_address0 sc_out sc_lv 5 signal 9 } 
	{ ref_reference_lateral_error_ce0 sc_out sc_logic 1 signal 9 } 
	{ ref_reference_lateral_error_q0 sc_in sc_lv 32 signal 9 } 
	{ ref_reference_velocity_address0 sc_out sc_lv 5 signal 10 } 
	{ ref_reference_velocity_ce0 sc_out sc_logic 1 signal 10 } 
	{ ref_reference_velocity_q0 sc_in sc_lv 32 signal 10 } 
	{ ref_reference_lateral_velocity_address0 sc_out sc_lv 5 signal 11 } 
	{ ref_reference_lateral_velocity_ce0 sc_out sc_logic 1 signal 11 } 
	{ ref_reference_lateral_velocity_q0 sc_in sc_lv 32 signal 11 } 
	{ ref_reference_yaw_rate_address0 sc_out sc_lv 5 signal 12 } 
	{ ref_reference_yaw_rate_ce0 sc_out sc_logic 1 signal 12 } 
	{ ref_reference_yaw_rate_q0 sc_in sc_lv 32 signal 12 } 
	{ ref_path_curvature_address0 sc_out sc_lv 5 signal 13 } 
	{ ref_path_curvature_ce0 sc_out sc_logic 1 signal 13 } 
	{ ref_path_curvature_q0 sc_in sc_lv 32 signal 13 } 
	{ ref_left_wall_bound_address0 sc_out sc_lv 5 signal 14 } 
	{ ref_left_wall_bound_ce0 sc_out sc_logic 1 signal 14 } 
	{ ref_left_wall_bound_q0 sc_in sc_lv 32 signal 14 } 
	{ ref_right_wall_bound_address0 sc_out sc_lv 5 signal 15 } 
	{ ref_right_wall_bound_ce0 sc_out sc_logic 1 signal 15 } 
	{ ref_right_wall_bound_q0 sc_in sc_lv 32 signal 15 } 
	{ out_steering sc_out sc_lv 32 signal 16 } 
	{ out_steering_ap_vld sc_out sc_logic 1 outvld 16 } 
	{ out_accel sc_out sc_lv 32 signal 17 } 
	{ out_accel_ap_vld sc_out sc_logic 1 outvld 17 } 
	{ out_status sc_out sc_lv 1 signal 18 } 
	{ out_status_ap_vld sc_out sc_logic 1 outvld 18 } 
	{ out_iters sc_out sc_lv 32 signal 19 } 
	{ out_iters_ap_vld sc_out sc_logic 1 outvld 19 } 
	{ out_steering_arg_index sc_in sc_lv 11 signal 20 } 
	{ out_accel_arg_index sc_in sc_lv 11 signal 21 } 
	{ out_status_arg_index sc_in sc_lv 11 signal 22 } 
	{ out_iters_arg_index sc_in sc_lv 11 signal 23 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "ey", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ey", "role": "default" }} , 
 	{ "name": "epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "epsi", "role": "default" }} , 
 	{ "name": "vx", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "vx", "role": "default" }} , 
 	{ "name": "vy", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "vy", "role": "default" }} , 
 	{ "name": "omega", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "omega", "role": "default" }} , 
 	{ "name": "steering", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "steering", "role": "default" }} , 
 	{ "name": "prev_accel", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "prev_accel", "role": "default" }} , 
 	{ "name": "control_flags", "direction": "in", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "control_flags", "role": "default" }} , 
 	{ "name": "ref_reference_heading_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "address0" }} , 
 	{ "name": "ref_reference_heading_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "ce0" }} , 
 	{ "name": "ref_reference_heading_error_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "q0" }} , 
 	{ "name": "ref_reference_lateral_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "address0" }} , 
 	{ "name": "ref_reference_lateral_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "ce0" }} , 
 	{ "name": "ref_reference_lateral_error_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "q0" }} , 
 	{ "name": "ref_reference_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "address0" }} , 
 	{ "name": "ref_reference_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "ce0" }} , 
 	{ "name": "ref_reference_velocity_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "q0" }} , 
 	{ "name": "ref_reference_lateral_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "address0" }} , 
 	{ "name": "ref_reference_lateral_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "ce0" }} , 
 	{ "name": "ref_reference_lateral_velocity_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "q0" }} , 
 	{ "name": "ref_reference_yaw_rate_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "address0" }} , 
 	{ "name": "ref_reference_yaw_rate_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "ce0" }} , 
 	{ "name": "ref_reference_yaw_rate_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "q0" }} , 
 	{ "name": "ref_path_curvature_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "address0" }} , 
 	{ "name": "ref_path_curvature_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "ce0" }} , 
 	{ "name": "ref_path_curvature_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "q0" }} , 
 	{ "name": "ref_left_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "address0" }} , 
 	{ "name": "ref_left_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "ce0" }} , 
 	{ "name": "ref_left_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "q0" }} , 
 	{ "name": "ref_right_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "address0" }} , 
 	{ "name": "ref_right_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "ce0" }} , 
 	{ "name": "ref_right_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "q0" }} , 
 	{ "name": "out_steering", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_steering", "role": "default" }} , 
 	{ "name": "out_steering_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "out_steering", "role": "ap_vld" }} , 
 	{ "name": "out_accel", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_accel", "role": "default" }} , 
 	{ "name": "out_accel_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "out_accel", "role": "ap_vld" }} , 
 	{ "name": "out_status", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "out_status", "role": "default" }} , 
 	{ "name": "out_status_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "out_status", "role": "ap_vld" }} , 
 	{ "name": "out_iters", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "out_iters", "role": "default" }} , 
 	{ "name": "out_iters_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "out_iters", "role": "ap_vld" }} , 
 	{ "name": "out_steering_arg_index", "direction": "in", "datatype": "sc_lv", "bitwidth":11, "type": "signal", "bundle":{"name": "out_steering_arg_index", "role": "default" }} , 
 	{ "name": "out_accel_arg_index", "direction": "in", "datatype": "sc_lv", "bitwidth":11, "type": "signal", "bundle":{"name": "out_accel_arg_index", "role": "default" }} , 
 	{ "name": "out_status_arg_index", "direction": "in", "datatype": "sc_lv", "bitwidth":11, "type": "signal", "bundle":{"name": "out_status_arg_index", "role": "default" }} , 
 	{ "name": "out_iters_arg_index", "direction": "in", "datatype": "sc_lv", "bitwidth":11, "type": "signal", "bundle":{"name": "out_iters_arg_index", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	p_anonymous_namespace_mpc_fpga_compute_core {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 0 FirstWrite -1}
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 0 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		steering {Type I LastRead 0 FirstWrite -1}
		prev_accel {Type I LastRead 0 FirstWrite -1}
		control_flags {Type I LastRead 0 FirstWrite -1}
		ref_reference_heading_error {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_yaw_rate {Type I LastRead 6 FirstWrite -1}
		ref_path_curvature {Type I LastRead 2 FirstWrite -1}
		ref_left_wall_bound {Type I LastRead 0 FirstWrite -1}
		ref_right_wall_bound {Type I LastRead 0 FirstWrite -1}
		out_steering {Type O LastRead -1 FirstWrite 10}
		out_accel {Type O LastRead -1 FirstWrite 10}
		out_status {Type O LastRead -1 FirstWrite 10}
		out_iters {Type O LastRead -1 FirstWrite 10}
		out_steering_arg_index {Type I LastRead 0 FirstWrite -1}
		out_accel_arg_index {Type I LastRead 0 FirstWrite -1}
		out_status_arg_index {Type I LastRead 0 FirstWrite -1}
		out_iters_arg_index {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_initialized {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_max_iter_streak {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_initialized {Type IO LastRead -1 FirstWrite -1}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	p_anonymous_namespace_reset_core_state_hls {
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_rho {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_initialized {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_max_iter_streak {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_initialized {Type O LastRead -1 FirstWrite 1}}
	p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_364_1 {
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_372_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_390_1 {
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_397_3 {
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_364_1 {
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_372_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls {
		state_ey {Type I LastRead 4 FirstWrite -1}
		state_epsi {Type I LastRead 4 FirstWrite -1}
		state_vx {Type I LastRead 3 FirstWrite -1}
		state_vy {Type I LastRead 4 FirstWrite -1}
		state_omega {Type I LastRead 4 FirstWrite -1}
		ref_reference_heading_error {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_yaw_rate {Type I LastRead 6 FirstWrite -1}
		ref_path_curvature {Type I LastRead 2 FirstWrite -1}
		ref_left_wall_bound {Type I LastRead 0 FirstWrite -1}
		ref_right_wall_bound {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type I LastRead 3 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type IO LastRead 4 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type IO LastRead 4 FirstWrite 0}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type IO LastRead 10 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type IO LastRead 10 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_initialized {Type IO LastRead 12 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type IO LastRead 27 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type IO LastRead 27 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type IO LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type IO LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho {Type IO LastRead 12 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type IO LastRead 12 FirstWrite 1}}
	riccati_pass_hls {
		step_data_0_0_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_0_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_1_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_2_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_3_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_4_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_5_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_5 {Type I LastRead 32 FirstWrite -1}
		step_data_1 {Type I LastRead 21 FirstWrite -1}
		step_data_2 {Type I LastRead 21 FirstWrite -1}
		B_sparse_0 {Type I LastRead 9 FirstWrite -1}
		B_sparse_1 {Type I LastRead 9 FirstWrite -1}
		B_sparse_2 {Type I LastRead 9 FirstWrite -1}
		B_sparse_3 {Type I LastRead 9 FirstWrite -1}
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 0 FirstWrite -1}
		p_read6 {Type I LastRead 2 FirstWrite -1}
		p_read7 {Type I LastRead 2 FirstWrite -1}
		p_read8 {Type I LastRead 2 FirstWrite -1}
		p_read9 {Type I LastRead 2 FirstWrite -1}
		p_read10 {Type I LastRead 2 FirstWrite -1}
		p_read11 {Type I LastRead 2 FirstWrite -1}
		p_read12 {Type I LastRead 2 FirstWrite -1}
		p_read13 {Type I LastRead 2 FirstWrite -1}
		rho {Type I LastRead 0 FirstWrite -1}
		rho_u {Type I LastRead 0 FirstWrite -1}
		z_x_0 {Type I LastRead 15 FirstWrite -1}
		z_x_5 {Type I LastRead 15 FirstWrite -1}
		y_x_0 {Type I LastRead 15 FirstWrite -1}
		y_x_5 {Type I LastRead 15 FirstWrite -1}
		z_u_0 {Type I LastRead 17 FirstWrite -1}
		z_u_1 {Type I LastRead 17 FirstWrite -1}
		y_u_0 {Type I LastRead 17 FirstWrite -1}
		y_u_1 {Type I LastRead 17 FirstWrite -1}
		K_0_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_0_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_0_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_0_1_7 {Type IO LastRead 35 FirstWrite -1}
		K_1_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_1_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_1_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_1_1_7 {Type IO LastRead 35 FirstWrite -1}
		K_2_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_2_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_2_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_2_1_7 {Type IO LastRead 35 FirstWrite -1}
		K_3_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_3_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_3_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_3_1_7 {Type IO LastRead 35 FirstWrite -1}
		kk_0 {Type IO LastRead 5 FirstWrite -1}
		kk_1 {Type IO LastRead 5 FirstWrite -1}
		x_out_0 {Type IO LastRead 1 FirstWrite 0}
		x_out_1 {Type IO LastRead 1 FirstWrite 0}
		x_out_2 {Type IO LastRead 1 FirstWrite 0}
		x_out_3 {Type IO LastRead 1 FirstWrite 0}
		x_out_4 {Type IO LastRead 1 FirstWrite 0}
		x_out_5 {Type IO LastRead 1 FirstWrite 0}
		x_out_6 {Type IO LastRead 1 FirstWrite 0}
		x_out_7 {Type IO LastRead 1 FirstWrite 0}
		u_out_0 {Type O LastRead -1 FirstWrite 6}
		u_out_1 {Type O LastRead -1 FirstWrite 6}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	riccati_backward_pass {
		step_data_0_0_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_0_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_1_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_2_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_3_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_4_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_5_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_5 {Type I LastRead 32 FirstWrite -1}
		step_data_1 {Type I LastRead 21 FirstWrite -1}
		step_data_2 {Type I LastRead 21 FirstWrite -1}
		B_sparse_0 {Type I LastRead 9 FirstWrite -1}
		B_sparse_1 {Type I LastRead 9 FirstWrite -1}
		B_sparse_2 {Type I LastRead 9 FirstWrite -1}
		B_sparse_3 {Type I LastRead 9 FirstWrite -1}
		p_read {Type I LastRead 7 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 7 FirstWrite -1}
		rho {Type I LastRead 4 FirstWrite -1}
		rho_u {Type I LastRead 7 FirstWrite -1}
		z_x_0 {Type I LastRead 15 FirstWrite -1}
		z_x_5 {Type I LastRead 15 FirstWrite -1}
		y_x_0 {Type I LastRead 15 FirstWrite -1}
		y_x_5 {Type I LastRead 15 FirstWrite -1}
		z_u_0 {Type I LastRead 17 FirstWrite -1}
		z_u_1 {Type I LastRead 17 FirstWrite -1}
		y_u_0 {Type I LastRead 17 FirstWrite -1}
		y_u_1 {Type I LastRead 17 FirstWrite -1}
		K_0_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_0_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_0_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_0_1_7 {Type IO LastRead 35 FirstWrite 3}
		K_1_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_1_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_1_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_1_1_7 {Type IO LastRead 35 FirstWrite 3}
		K_2_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_2_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_2_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_2_1_7 {Type IO LastRead 35 FirstWrite 3}
		K_3_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_3_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_3_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_3_1_7 {Type IO LastRead 35 FirstWrite 3}
		kk_0 {Type O LastRead -1 FirstWrite 38}
		kk_1 {Type O LastRead -1 FirstWrite 38}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	invert_2x2_qp_hls {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 1 FirstWrite -1}
		p_read4 {Type I LastRead 6 FirstWrite -1}
		p_read5 {Type I LastRead 6 FirstWrite -1}
		p_read6 {Type I LastRead 6 FirstWrite -1}
		p_read7 {Type I LastRead 6 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	fp_recip {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8 {
		sext_ln167_1 {Type I LastRead 0 FirstWrite -1}
		M {Type I LastRead 0 FirstWrite -1}
		sext_ln167_2 {Type I LastRead 0 FirstWrite -1}
		M_2 {Type I LastRead 0 FirstWrite -1}
		sext_ln167_3 {Type I LastRead 0 FirstWrite -1}
		M_12 {Type I LastRead 0 FirstWrite -1}
		sext_ln167_4 {Type I LastRead 0 FirstWrite -1}
		M_14 {Type I LastRead 0 FirstWrite -1}
		sext_ln167_5 {Type I LastRead 0 FirstWrite -1}
		M_16 {Type I LastRead 0 FirstWrite -1}
		sext_ln167_6 {Type I LastRead 0 FirstWrite -1}
		M_18 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_5_load {Type I LastRead 0 FirstWrite -1}
		G_13_out {Type O LastRead -1 FirstWrite 3}
		G_12_out {Type O LastRead -1 FirstWrite 3}
		G_11_out {Type O LastRead -1 FirstWrite 3}
		G_10_out {Type O LastRead -1 FirstWrite 3}
		G_9_out {Type O LastRead -1 FirstWrite 3}
		G_8_out {Type O LastRead -1 FirstWrite 3}
		G_7_out {Type O LastRead -1 FirstWrite 3}
		G_6_out {Type O LastRead -1 FirstWrite 3}
		G_5_out {Type O LastRead -1 FirstWrite 3}
		G_4_out {Type O LastRead -1 FirstWrite 3}
		G_3_out {Type O LastRead -1 FirstWrite 3}
		G_2_out {Type O LastRead -1 FirstWrite 3}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_766_10 {
		P {Type I LastRead 0 FirstWrite -1}
		sext_ln259 {Type I LastRead 0 FirstWrite -1}
		P_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln259_1 {Type I LastRead 0 FirstWrite -1}
		P_2 {Type I LastRead 0 FirstWrite -1}
		sext_ln259_2 {Type I LastRead 0 FirstWrite -1}
		P_3 {Type I LastRead 0 FirstWrite -1}
		sext_ln259_3 {Type I LastRead 0 FirstWrite -1}
		P_4 {Type I LastRead 0 FirstWrite -1}
		sext_ln766 {Type I LastRead 0 FirstWrite -1}
		P_5 {Type I LastRead 0 FirstWrite -1}
		sext_ln766_1 {Type I LastRead 0 FirstWrite -1}
		p_2_r {Type I LastRead 0 FirstWrite -1}
		p_3_r {Type I LastRead 0 FirstWrite -1}
		p_4_r {Type I LastRead 0 FirstWrite -1}
		p_5_r {Type I LastRead 0 FirstWrite -1}
		p_6 {Type I LastRead 0 FirstWrite -1}
		p_7 {Type I LastRead 0 FirstWrite -1}
		p_8 {Type I LastRead 0 FirstWrite -1}
		p_9 {Type I LastRead 0 FirstWrite -1}
		p_shift_7_out {Type O LastRead -1 FirstWrite 5}
		p_shift_6_out {Type O LastRead -1 FirstWrite 5}
		p_shift_5_out {Type O LastRead -1 FirstWrite 5}
		p_shift_4_out {Type O LastRead -1 FirstWrite 5}
		p_shift_3_out {Type O LastRead -1 FirstWrite 5}
		p_shift_2_out {Type O LastRead -1 FirstWrite 5}
		p_shift_1_out {Type O LastRead -1 FirstWrite 5}
		p_shift_out {Type O LastRead -1 FirstWrite 5}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_750_9 {
		K_0_0_0 {Type O LastRead -1 FirstWrite 3}
		zext_ln475 {Type I LastRead 0 FirstWrite -1}
		K_0_1_0 {Type O LastRead -1 FirstWrite 3}
		K_0_0_1 {Type O LastRead -1 FirstWrite 3}
		K_0_1_1 {Type O LastRead -1 FirstWrite 3}
		K_0_0_2 {Type O LastRead -1 FirstWrite 3}
		K_0_1_2 {Type O LastRead -1 FirstWrite 3}
		K_0_0_3 {Type O LastRead -1 FirstWrite 3}
		K_0_1_3 {Type O LastRead -1 FirstWrite 3}
		K_0_0_4 {Type O LastRead -1 FirstWrite 3}
		K_0_1_4 {Type O LastRead -1 FirstWrite 3}
		K_0_0_5 {Type O LastRead -1 FirstWrite 3}
		K_0_1_5 {Type O LastRead -1 FirstWrite 3}
		K_0_0_6 {Type O LastRead -1 FirstWrite 3}
		K_0_1_6 {Type O LastRead -1 FirstWrite 3}
		K_0_0_7 {Type O LastRead -1 FirstWrite 3}
		K_0_1_7 {Type O LastRead -1 FirstWrite 3}
		K_1_0_0 {Type O LastRead -1 FirstWrite 3}
		K_1_1_0 {Type O LastRead -1 FirstWrite 3}
		K_1_0_1 {Type O LastRead -1 FirstWrite 3}
		K_1_1_1 {Type O LastRead -1 FirstWrite 3}
		K_1_0_2 {Type O LastRead -1 FirstWrite 3}
		K_1_1_2 {Type O LastRead -1 FirstWrite 3}
		K_1_0_3 {Type O LastRead -1 FirstWrite 3}
		K_1_1_3 {Type O LastRead -1 FirstWrite 3}
		K_1_0_4 {Type O LastRead -1 FirstWrite 3}
		K_1_1_4 {Type O LastRead -1 FirstWrite 3}
		K_1_0_5 {Type O LastRead -1 FirstWrite 3}
		K_1_1_5 {Type O LastRead -1 FirstWrite 3}
		K_1_0_6 {Type O LastRead -1 FirstWrite 3}
		K_1_1_6 {Type O LastRead -1 FirstWrite 3}
		K_1_0_7 {Type O LastRead -1 FirstWrite 3}
		K_1_1_7 {Type O LastRead -1 FirstWrite 3}
		K_2_0_0 {Type O LastRead -1 FirstWrite 3}
		K_2_1_0 {Type O LastRead -1 FirstWrite 3}
		K_2_0_1 {Type O LastRead -1 FirstWrite 3}
		K_2_1_1 {Type O LastRead -1 FirstWrite 3}
		K_2_0_2 {Type O LastRead -1 FirstWrite 3}
		K_2_1_2 {Type O LastRead -1 FirstWrite 3}
		K_2_0_3 {Type O LastRead -1 FirstWrite 3}
		K_2_1_3 {Type O LastRead -1 FirstWrite 3}
		K_2_0_4 {Type O LastRead -1 FirstWrite 3}
		K_2_1_4 {Type O LastRead -1 FirstWrite 3}
		K_2_0_5 {Type O LastRead -1 FirstWrite 3}
		K_2_1_5 {Type O LastRead -1 FirstWrite 3}
		K_2_0_6 {Type O LastRead -1 FirstWrite 3}
		K_2_1_6 {Type O LastRead -1 FirstWrite 3}
		K_2_0_7 {Type O LastRead -1 FirstWrite 3}
		K_2_1_7 {Type O LastRead -1 FirstWrite 3}
		K_3_0_0 {Type O LastRead -1 FirstWrite 3}
		K_3_1_0 {Type O LastRead -1 FirstWrite 3}
		K_3_0_1 {Type O LastRead -1 FirstWrite 3}
		K_3_1_1 {Type O LastRead -1 FirstWrite 3}
		K_3_0_2 {Type O LastRead -1 FirstWrite 3}
		K_3_1_2 {Type O LastRead -1 FirstWrite 3}
		K_3_0_3 {Type O LastRead -1 FirstWrite 3}
		K_3_1_3 {Type O LastRead -1 FirstWrite 3}
		K_3_0_4 {Type O LastRead -1 FirstWrite 3}
		K_3_1_4 {Type O LastRead -1 FirstWrite 3}
		K_3_0_5 {Type O LastRead -1 FirstWrite 3}
		K_3_1_5 {Type O LastRead -1 FirstWrite 3}
		K_3_0_6 {Type O LastRead -1 FirstWrite 3}
		K_3_1_6 {Type O LastRead -1 FirstWrite 3}
		K_3_0_7 {Type O LastRead -1 FirstWrite 3}
		K_3_1_7 {Type O LastRead -1 FirstWrite 3}
		G_11_reload {Type I LastRead 0 FirstWrite -1}
		G_12_reload {Type I LastRead 0 FirstWrite -1}
		G_13_reload {Type I LastRead 0 FirstWrite -1}
		G_10_reload {Type I LastRead 0 FirstWrite -1}
		G_9_reload {Type I LastRead 0 FirstWrite -1}
		G_8_reload {Type I LastRead 0 FirstWrite -1}
		select_ln723 {Type I LastRead 0 FirstWrite -1}
		sext_ln283 {Type I LastRead 0 FirstWrite -1}
		G_7_reload {Type I LastRead 0 FirstWrite -1}
		G_6_reload {Type I LastRead 0 FirstWrite -1}
		G_5_reload {Type I LastRead 0 FirstWrite -1}
		G_4_reload {Type I LastRead 0 FirstWrite -1}
		G_3_reload {Type I LastRead 0 FirstWrite -1}
		G_2_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln283_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln283_2 {Type I LastRead 0 FirstWrite -1}
		sext_ln760 {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13 {
		P {Type I LastRead 0 FirstWrite -1}
		P_1 {Type I LastRead 0 FirstWrite -1}
		P_2 {Type I LastRead 0 FirstWrite -1}
		P_3 {Type I LastRead 0 FirstWrite -1}
		P_4 {Type I LastRead 0 FirstWrite -1}
		P_5 {Type I LastRead 0 FirstWrite -1}
		PA_5 {Type O LastRead -1 FirstWrite 5}
		PA_4 {Type O LastRead -1 FirstWrite 5}
		PA_3 {Type O LastRead -1 FirstWrite 5}
		PA_2 {Type O LastRead -1 FirstWrite 5}
		PA_1 {Type O LastRead -1 FirstWrite 5}
		PA {Type O LastRead -1 FirstWrite 5}
		a_60 {Type I LastRead 0 FirstWrite -1}
		a_68 {Type I LastRead 0 FirstWrite -1}
		a_76 {Type I LastRead 0 FirstWrite -1}
		a_84 {Type I LastRead 0 FirstWrite -1}
		a_92 {Type I LastRead 0 FirstWrite -1}
		a_61 {Type I LastRead 0 FirstWrite -1}
		a_69 {Type I LastRead 0 FirstWrite -1}
		a_77 {Type I LastRead 0 FirstWrite -1}
		a_85 {Type I LastRead 0 FirstWrite -1}
		a_93 {Type I LastRead 0 FirstWrite -1}
		a_62 {Type I LastRead 0 FirstWrite -1}
		a_70 {Type I LastRead 0 FirstWrite -1}
		a_78 {Type I LastRead 0 FirstWrite -1}
		a_86 {Type I LastRead 0 FirstWrite -1}
		a_94 {Type I LastRead 0 FirstWrite -1}
		a_63 {Type I LastRead 0 FirstWrite -1}
		a_71 {Type I LastRead 0 FirstWrite -1}
		a_79 {Type I LastRead 0 FirstWrite -1}
		a_87 {Type I LastRead 0 FirstWrite -1}
		a_95 {Type I LastRead 0 FirstWrite -1}
		a_64 {Type I LastRead 0 FirstWrite -1}
		a_72 {Type I LastRead 0 FirstWrite -1}
		a_80 {Type I LastRead 0 FirstWrite -1}
		a_88 {Type I LastRead 0 FirstWrite -1}
		a_96 {Type I LastRead 0 FirstWrite -1}
		a_65 {Type I LastRead 0 FirstWrite -1}
		a_73 {Type I LastRead 0 FirstWrite -1}
		a_81 {Type I LastRead 0 FirstWrite -1}
		a_89 {Type I LastRead 0 FirstWrite -1}
		a_97 {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37 {
		a_52 {Type I LastRead 0 FirstWrite -1}
		a_60 {Type I LastRead 0 FirstWrite -1}
		a_68 {Type I LastRead 0 FirstWrite -1}
		a_76 {Type I LastRead 0 FirstWrite -1}
		a_84 {Type I LastRead 0 FirstWrite -1}
		a_92 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_121 {Type I LastRead 0 FirstWrite -1}
		a_53 {Type I LastRead 0 FirstWrite -1}
		a_61 {Type I LastRead 0 FirstWrite -1}
		a_69 {Type I LastRead 0 FirstWrite -1}
		a_77 {Type I LastRead 0 FirstWrite -1}
		a_85 {Type I LastRead 0 FirstWrite -1}
		a_93 {Type I LastRead 0 FirstWrite -1}
		sext_ln291_41 {Type I LastRead 0 FirstWrite -1}
		a_54 {Type I LastRead 0 FirstWrite -1}
		a_62 {Type I LastRead 0 FirstWrite -1}
		a_70 {Type I LastRead 0 FirstWrite -1}
		a_78 {Type I LastRead 0 FirstWrite -1}
		a_86 {Type I LastRead 0 FirstWrite -1}
		a_94 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_47 {Type I LastRead 0 FirstWrite -1}
		a_55 {Type I LastRead 0 FirstWrite -1}
		a_63 {Type I LastRead 0 FirstWrite -1}
		a_71 {Type I LastRead 0 FirstWrite -1}
		a_79 {Type I LastRead 0 FirstWrite -1}
		a_87 {Type I LastRead 0 FirstWrite -1}
		a_95 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_49 {Type I LastRead 0 FirstWrite -1}
		a_56 {Type I LastRead 0 FirstWrite -1}
		a_64 {Type I LastRead 0 FirstWrite -1}
		a_72 {Type I LastRead 0 FirstWrite -1}
		a_80 {Type I LastRead 0 FirstWrite -1}
		a_88 {Type I LastRead 0 FirstWrite -1}
		a_96 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_51 {Type I LastRead 0 FirstWrite -1}
		a_57 {Type I LastRead 0 FirstWrite -1}
		a_65 {Type I LastRead 0 FirstWrite -1}
		a_73 {Type I LastRead 0 FirstWrite -1}
		a_81 {Type I LastRead 0 FirstWrite -1}
		a_89 {Type I LastRead 0 FirstWrite -1}
		a_97 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_45 {Type I LastRead 0 FirstWrite -1}
		G_11_reload {Type I LastRead 0 FirstWrite -1}
		G_12_reload {Type I LastRead 0 FirstWrite -1}
		G_13_reload {Type I LastRead 0 FirstWrite -1}
		G_10_reload {Type I LastRead 0 FirstWrite -1}
		G_9_reload {Type I LastRead 0 FirstWrite -1}
		G_8_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln291_43 {Type I LastRead 0 FirstWrite -1}
		G_7_reload {Type I LastRead 0 FirstWrite -1}
		G_6_reload {Type I LastRead 0 FirstWrite -1}
		G_5_reload {Type I LastRead 0 FirstWrite -1}
		G_4_reload {Type I LastRead 0 FirstWrite -1}
		G_3_reload {Type I LastRead 0 FirstWrite -1}
		G_2_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln1013_1 {Type I LastRead 0 FirstWrite -1}
		q_aug_linear_4 {Type I LastRead 0 FirstWrite -1}
		sext_ln572 {Type I LastRead 0 FirstWrite -1}
		sext_ln573 {Type I LastRead 0 FirstWrite -1}
		sext_ln574 {Type I LastRead 0 FirstWrite -1}
		sext_ln575 {Type I LastRead 0 FirstWrite -1}
		q_aug_linear_5 {Type I LastRead 0 FirstWrite -1}
		p_new_5_out {Type O LastRead -1 FirstWrite 0}
		p_new_4_out {Type O LastRead -1 FirstWrite 0}
		p_new_3_out {Type O LastRead -1 FirstWrite 0}
		p_new_2_out {Type O LastRead -1 FirstWrite 0}
		p_new_1_out {Type O LastRead -1 FirstWrite 0}
		p_new_out {Type O LastRead -1 FirstWrite 0}}
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_1029_38 {
		G_37 {Type I LastRead 0 FirstWrite -1}
		sext_ln291_42 {Type I LastRead 0 FirstWrite -1}
		sext_ln1013 {Type I LastRead 0 FirstWrite -1}
		p_new_14_out {Type IO LastRead 3 FirstWrite 0}
		p_new_13_out {Type IO LastRead 3 FirstWrite 0}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum6_P_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	riccati_forward_pass {
		step_data_0_0_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_5 {Type I LastRead 6 FirstWrite -1}
		step_data_1 {Type I LastRead 7 FirstWrite -1}
		B_sparse_0 {Type I LastRead 6 FirstWrite -1}
		B_sparse_1 {Type I LastRead 6 FirstWrite -1}
		B_sparse_2 {Type I LastRead 6 FirstWrite -1}
		B_sparse_3 {Type I LastRead 6 FirstWrite -1}
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 0 FirstWrite -1}
		p_read6 {Type I LastRead 0 FirstWrite -1}
		p_read7 {Type I LastRead 0 FirstWrite -1}
		K_0_0_0 {Type I LastRead 1 FirstWrite -1}
		K_0_0_1 {Type I LastRead 1 FirstWrite -1}
		K_0_0_2 {Type I LastRead 1 FirstWrite -1}
		K_0_0_3 {Type I LastRead 1 FirstWrite -1}
		K_0_0_4 {Type I LastRead 1 FirstWrite -1}
		K_0_0_5 {Type I LastRead 1 FirstWrite -1}
		K_0_0_6 {Type I LastRead 1 FirstWrite -1}
		K_0_0_7 {Type I LastRead 1 FirstWrite -1}
		K_0_1_0 {Type I LastRead 1 FirstWrite -1}
		K_0_1_1 {Type I LastRead 1 FirstWrite -1}
		K_0_1_2 {Type I LastRead 1 FirstWrite -1}
		K_0_1_3 {Type I LastRead 1 FirstWrite -1}
		K_0_1_4 {Type I LastRead 1 FirstWrite -1}
		K_0_1_5 {Type I LastRead 1 FirstWrite -1}
		K_0_1_6 {Type I LastRead 1 FirstWrite -1}
		K_0_1_7 {Type I LastRead 1 FirstWrite -1}
		K_1_0_0 {Type I LastRead 1 FirstWrite -1}
		K_1_0_1 {Type I LastRead 1 FirstWrite -1}
		K_1_0_2 {Type I LastRead 1 FirstWrite -1}
		K_1_0_3 {Type I LastRead 1 FirstWrite -1}
		K_1_0_4 {Type I LastRead 1 FirstWrite -1}
		K_1_0_5 {Type I LastRead 1 FirstWrite -1}
		K_1_0_6 {Type I LastRead 1 FirstWrite -1}
		K_1_0_7 {Type I LastRead 1 FirstWrite -1}
		K_1_1_0 {Type I LastRead 1 FirstWrite -1}
		K_1_1_1 {Type I LastRead 1 FirstWrite -1}
		K_1_1_2 {Type I LastRead 1 FirstWrite -1}
		K_1_1_3 {Type I LastRead 1 FirstWrite -1}
		K_1_1_4 {Type I LastRead 1 FirstWrite -1}
		K_1_1_5 {Type I LastRead 1 FirstWrite -1}
		K_1_1_6 {Type I LastRead 1 FirstWrite -1}
		K_1_1_7 {Type I LastRead 1 FirstWrite -1}
		K_2_0_0 {Type I LastRead 1 FirstWrite -1}
		K_2_0_1 {Type I LastRead 1 FirstWrite -1}
		K_2_0_2 {Type I LastRead 1 FirstWrite -1}
		K_2_0_3 {Type I LastRead 1 FirstWrite -1}
		K_2_0_4 {Type I LastRead 1 FirstWrite -1}
		K_2_0_5 {Type I LastRead 1 FirstWrite -1}
		K_2_0_6 {Type I LastRead 1 FirstWrite -1}
		K_2_0_7 {Type I LastRead 1 FirstWrite -1}
		K_2_1_0 {Type I LastRead 1 FirstWrite -1}
		K_2_1_1 {Type I LastRead 1 FirstWrite -1}
		K_2_1_2 {Type I LastRead 1 FirstWrite -1}
		K_2_1_3 {Type I LastRead 1 FirstWrite -1}
		K_2_1_4 {Type I LastRead 1 FirstWrite -1}
		K_2_1_5 {Type I LastRead 1 FirstWrite -1}
		K_2_1_6 {Type I LastRead 1 FirstWrite -1}
		K_2_1_7 {Type I LastRead 1 FirstWrite -1}
		K_3_0_0 {Type I LastRead 1 FirstWrite -1}
		K_3_0_1 {Type I LastRead 1 FirstWrite -1}
		K_3_0_2 {Type I LastRead 1 FirstWrite -1}
		K_3_0_3 {Type I LastRead 1 FirstWrite -1}
		K_3_0_4 {Type I LastRead 1 FirstWrite -1}
		K_3_0_5 {Type I LastRead 1 FirstWrite -1}
		K_3_0_6 {Type I LastRead 1 FirstWrite -1}
		K_3_0_7 {Type I LastRead 1 FirstWrite -1}
		K_3_1_0 {Type I LastRead 1 FirstWrite -1}
		K_3_1_1 {Type I LastRead 1 FirstWrite -1}
		K_3_1_2 {Type I LastRead 1 FirstWrite -1}
		K_3_1_3 {Type I LastRead 1 FirstWrite -1}
		K_3_1_4 {Type I LastRead 1 FirstWrite -1}
		K_3_1_5 {Type I LastRead 1 FirstWrite -1}
		K_3_1_6 {Type I LastRead 1 FirstWrite -1}
		K_3_1_7 {Type I LastRead 1 FirstWrite -1}
		kk_0 {Type I LastRead 5 FirstWrite -1}
		kk_1 {Type I LastRead 5 FirstWrite -1}
		x_out_0 {Type IO LastRead 1 FirstWrite 0}
		x_out_1 {Type IO LastRead 1 FirstWrite 0}
		x_out_2 {Type IO LastRead 1 FirstWrite 0}
		x_out_3 {Type IO LastRead 1 FirstWrite 0}
		x_out_4 {Type IO LastRead 1 FirstWrite 0}
		x_out_5 {Type IO LastRead 1 FirstWrite 0}
		x_out_6 {Type IO LastRead 1 FirstWrite 0}
		x_out_7 {Type IO LastRead 1 FirstWrite 0}
		u_out_0 {Type O LastRead -1 FirstWrite 6}
		u_out_1 {Type O LastRead -1 FirstWrite 6}}
	sum8_K_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_K_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	riccati_forward_pass_Pipeline_VITIS_LOOP_383_4 {
		x_out_0 {Type O LastRead -1 FirstWrite 4}
		zext_ln409 {Type I LastRead 0 FirstWrite -1}
		x_out_1 {Type O LastRead -1 FirstWrite 4}
		x_out_2 {Type O LastRead -1 FirstWrite 4}
		x_out_3 {Type O LastRead -1 FirstWrite 4}
		x_out_4 {Type O LastRead -1 FirstWrite 4}
		x_out_5 {Type O LastRead -1 FirstWrite 4}
		empty_519 {Type I LastRead 0 FirstWrite -1}
		empty_520 {Type I LastRead 0 FirstWrite -1}
		empty_521 {Type I LastRead 0 FirstWrite -1}
		empty_522 {Type I LastRead 0 FirstWrite -1}
		empty_523 {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_0_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_0_load {Type I LastRead 0 FirstWrite -1}
		sext_ln159 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_1_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_1_load {Type I LastRead 0 FirstWrite -1}
		sext_ln159_1 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_2_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_2_load {Type I LastRead 0 FirstWrite -1}
		sext_ln159_2 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_3_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_3_load {Type I LastRead 0 FirstWrite -1}
		sext_ln159_3 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_4_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_4_load {Type I LastRead 0 FirstWrite -1}
		sext_ln159_4 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_1_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_2_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_3_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_4_5_load {Type I LastRead 0 FirstWrite -1}
		step_data_0_5_5_load {Type I LastRead 0 FirstWrite -1}
		sext_ln406 {Type I LastRead 0 FirstWrite -1}
		sext_ln159_11 {Type I LastRead 0 FirstWrite -1}
		sext_ln420 {Type I LastRead 0 FirstWrite -1}
		sext_ln159_12 {Type I LastRead 0 FirstWrite -1}
		sext_ln159_13 {Type I LastRead 0 FirstWrite -1}
		sext_ln159_14 {Type I LastRead 0 FirstWrite -1}
		sext_ln420_1 {Type I LastRead 0 FirstWrite -1}}
	sum6_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_356_1 {
		ref_left_wall_bound {Type I LastRead 0 FirstWrite -1}
		ref_right_wall_bound {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144171_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145169_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144167_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145165_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144163_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145161_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144159_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145157_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144155_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145153_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144151_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145149_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144147_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145145_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144143_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145141_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144139_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145137_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144135_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145133_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144131_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145129_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144127_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145125_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144123_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145121_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144119_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145117_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144115_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145113_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144111_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145109_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144107_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145105_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144103_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145101_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014499_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014597_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014495_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014593_out {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_366_2 {
		p_0_0_010145165_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145161_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145157_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145153_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145149_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145145_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145141_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145137_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145133_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145129_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145125_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145121_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145117_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145113_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145109_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145105_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145101_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014597_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014593_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145169_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144167_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144163_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144159_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144155_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144151_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144147_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144143_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144139_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144135_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144131_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144127_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144123_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144119_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144115_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144111_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144107_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144103_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014499_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014495_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144171_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_19_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_18_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_17_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_16_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_15_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_14_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_13_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_12_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_11_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_10_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_9_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_8_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_7_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_6_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_5_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_4_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_3_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_2_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_1_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_19_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_18_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_17_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_16_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_15_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_14_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_13_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_12_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_11_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_10_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_9_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_8_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_7_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_6_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_5_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_4_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_3_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_2_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_1_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_out {Type O LastRead -1 FirstWrite 1}}
	compute_window_min_hls {
		ref_wall_0_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_1_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_2_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_3_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_4_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_5_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_6_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_7_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_8_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_9_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_10_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_11_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_12_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_13_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_14_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_15_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_16_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_17_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_18_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_19_val {Type I LastRead 0 FirstWrite -1}
		center_idx {Type I LastRead 0 FirstWrite -1}}
	compute_window_min_hls {
		ref_wall_0_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_1_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_2_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_3_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_4_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_5_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_6_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_7_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_8_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_9_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_10_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_11_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_12_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_13_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_14_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_15_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_16_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_17_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_18_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_19_val {Type I LastRead 0 FirstWrite -1}
		center_idx {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_371_3 {
		x0_1 {Type I LastRead 0 FirstWrite -1}
		state_omega {Type I LastRead 0 FirstWrite -1}
		state_vy {Type I LastRead 0 FirstWrite -1}
		zext_ln128 {Type I LastRead 0 FirstWrite -1}
		state_ey {Type I LastRead 0 FirstWrite -1}
		state_epsi {Type I LastRead 0 FirstWrite -1}
		sext_ln483 {Type I LastRead 0 FirstWrite -1}
		step_data_2 {Type O LastRead -1 FirstWrite 7}
		step_data_0_5_5 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_4 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_3 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_2 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_1 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_0 {Type O LastRead -1 FirstWrite 0}
		step_data_0_4_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_4 {Type O LastRead -1 FirstWrite 112}
		step_data_0_4_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_0 {Type O LastRead -1 FirstWrite 111}
		B_sparse_3 {Type O LastRead -1 FirstWrite 111}
		B_sparse_2 {Type O LastRead -1 FirstWrite 111}
		B_sparse_1 {Type O LastRead -1 FirstWrite 111}
		B_sparse {Type O LastRead -1 FirstWrite 0}
		step_data_3 {Type O LastRead -1 FirstWrite 1}
		step_data_4 {Type O LastRead -1 FirstWrite 1}
		step_data_5 {Type O LastRead -1 FirstWrite 30}
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
		step_data_1 {Type O LastRead -1 FirstWrite 28}
		ref_reference_heading_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_lateral_velocity {Type I LastRead 0 FirstWrite -1}
		ref_reference_yaw_rate {Type I LastRead 0 FirstWrite -1}
		out_144_out {Type O LastRead -1 FirstWrite 112}
		terminal_dff_raw_0153_0_0_0_out {Type O LastRead -1 FirstWrite 112}
		terminal_wall_x_ub_con_out {Type O LastRead -1 FirstWrite 112}
		terminal_wall_x_lb_con_out {Type O LastRead -1 FirstWrite 112}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
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
		ey {Type I LastRead 68 FirstWrite -1}
		epsi {Type I LastRead 65 FirstWrite -1}
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 0 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		delta {Type I LastRead 0 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		kappa {Type I LastRead 68 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_frenet_tire_hls {
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 4 FirstWrite -1}
		omega {Type I LastRead 9 FirstWrite -1}
		delta {Type I LastRead 24 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_slip_terms {
		vy {Type I LastRead 4 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		inv_vx {Type I LastRead 4 FirstWrite -1}}
	compute_front_tire_path_fn {
		conv_i_i_i_i1462_i {Type I LastRead 6 FirstWrite -1}
		p_0_0_02853_i {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		conv_i_i_i1476_i {Type I LastRead 31 FirstWrite -1}
		conv_i_i_i1991_i {Type I LastRead 38 FirstWrite -1}
		p_0_0_02849_i {Type I LastRead 38 FirstWrite -1}
		spec_select_i {Type I LastRead 38 FirstWrite -1}
		conv_i_i_i65_i {Type I LastRead 0 FirstWrite -1}
		cmp_i_i {Type I LastRead 47 FirstWrite -1}
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
		D_cb {Type I LastRead 3 FirstWrite -1}}
	fp_front_force_jacobians_fn {
		C_eff_f {Type I LastRead 4 FirstWrite -1}
		front_num {Type I LastRead 2 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_f {Type I LastRead 1 FirstWrite -1}}
	compute_rear_tire_path_fn {
		rear_ratio {Type I LastRead 0 FirstWrite -1}
		D_transfer {Type I LastRead 0 FirstWrite -1}
		D_pac_r {Type I LastRead 31 FirstWrite -1}
		C_min_r {Type I LastRead 38 FirstWrite -1}
		rear_num {Type I LastRead 38 FirstWrite -1}
		vx_safe {Type I LastRead 38 FirstWrite -1}
		D_r_fn {Type I LastRead 0 FirstWrite -1}
		low_speed {Type I LastRead 47 FirstWrite -1}
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
		D_cb {Type I LastRead 3 FirstWrite -1}}
	fp_rear_force_jacobians_fn {
		C_eff_r {Type I LastRead 4 FirstWrite -1}
		rear_num {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_r {Type I LastRead 0 FirstWrite -1}}
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
		tr_dFyr_dvx_val {Type I LastRead 8 FirstWrite -1}
		tr_dFyr_dvy_val {Type I LastRead 9 FirstWrite -1}
		tr_dFyr_dom_val {Type I LastRead 10 FirstWrite -1}
		p_0_0_04262 {Type I LastRead 0 FirstWrite -1}
		p_0_0_04261 {Type I LastRead 3 FirstWrite -1}
		conv_i_i_i_i425 {Type I LastRead 7 FirstWrite -1}
		conv_i_i_i_i574 {Type I LastRead 6 FirstWrite -1}}
	compute_B_accel_load_transfer {
		tr_C_eff_f_raw_val {Type I LastRead 8 FirstWrite -1}
		tr_C_eff_r_val {Type I LastRead 10 FirstWrite -1}
		tr_C_min_f_val {Type I LastRead 9 FirstWrite -1}
		tr_cos_delta_val {Type I LastRead 17 FirstWrite -1}
		tr_alpha_f_op_val {Type I LastRead 11 FirstWrite -1}
		tr_alpha_r_op_val {Type I LastRead 13 FirstWrite -1}
		tr_Fz_transfer_val {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_rollout_from_forces_fn {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 23 FirstWrite -1}
		sin_epsi {Type I LastRead 10 FirstWrite -1}
		cos_epsi {Type I LastRead 2 FirstWrite -1}
		vx_safe {Type I LastRead 2 FirstWrite -1}
		vy {Type I LastRead 8 FirstWrite -1}
		omega {Type I LastRead 7 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		Fx {Type I LastRead 9 FirstWrite -1}
		F_yf {Type I LastRead 1 FirstWrite -1}
		F_yr {Type I LastRead 4 FirstWrite -1}
		sin_delta {Type I LastRead 3 FirstWrite -1}
		cos_delta {Type I LastRead 1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_B_steering {
		tr_C_eff_f_raw_val {Type I LastRead 0 FirstWrite -1}
		tr_C_min_f_val {Type I LastRead 0 FirstWrite -1}
		tr_F_yf_val {Type I LastRead 2 FirstWrite -1}
		tr_sin_delta_val {Type I LastRead 0 FirstWrite -1}
		tr_cos_delta_val {Type I LastRead 2 FirstWrite -1}}
	fp_frenet_rows01_fn {
		sin_epsi {Type I LastRead 6 FirstWrite -1}
		cos_epsi {Type I LastRead 2 FirstWrite -1}
		vx {Type I LastRead 3 FirstWrite -1}
		vy {Type I LastRead 6 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 9 FirstWrite -1}}
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
		p_read26 {Type I LastRead 4 FirstWrite -1}
		p_read27 {Type I LastRead 8 FirstWrite -1}
		p_read28 {Type I LastRead 12 FirstWrite -1}
		p_read29 {Type I LastRead 16 FirstWrite -1}
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
		lin_delta {Type I LastRead 33 FirstWrite -1}
		uk0 {Type I LastRead 30 FirstWrite -1}
		uk1 {Type I LastRead 25 FirstWrite -1}
		lin_delta_k {Type I LastRead 0 FirstWrite -1}
		next_ey {Type I LastRead 5 FirstWrite -1}
		next_epsi {Type I LastRead 10 FirstWrite -1}
		next_vx {Type I LastRead 15 FirstWrite -1}
		next_vy {Type I LastRead 20 FirstWrite -1}
		next_omega {Type I LastRead 25 FirstWrite -1}
		next_delta {Type I LastRead 33 FirstWrite -1}
		sd_1 {Type O LastRead -1 FirstWrite 28}
		sd_1_offset {Type I LastRead 28 FirstWrite -1}}
	affine_b0_term_hls {
		b0_coeff_raw {Type I LastRead 0 FirstWrite -1}
		lin_delta_k_raw {Type I LastRead 0 FirstWrite -1}}
	reset_admm_state_hls {
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_rho {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_initialized {Type O LastRead -1 FirstWrite 1}}
	reset_admm_state_hls_Pipeline_VITIS_LOOP_364_1 {
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}}
	reset_admm_state_hls_Pipeline_VITIS_LOOP_372_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1183_1 {
		y_x_7 {Type O LastRead -1 FirstWrite 0}
		y_x_6 {Type O LastRead -1 FirstWrite 0}
		y_x_5 {Type O LastRead -1 FirstWrite 0}
		y_x_4 {Type O LastRead -1 FirstWrite 0}
		y_x_3 {Type O LastRead -1 FirstWrite 0}
		y_x_2 {Type O LastRead -1 FirstWrite 0}
		y_x_1 {Type O LastRead -1 FirstWrite 0}
		y_x {Type O LastRead -1 FirstWrite 0}
		z_x_7 {Type O LastRead -1 FirstWrite 0}
		z_x_6 {Type O LastRead -1 FirstWrite 0}
		z_x_5 {Type O LastRead -1 FirstWrite 0}
		z_x_4 {Type O LastRead -1 FirstWrite 0}
		z_x_3 {Type O LastRead -1 FirstWrite 0}
		z_x_2 {Type O LastRead -1 FirstWrite 0}
		z_x_1 {Type O LastRead -1 FirstWrite 0}
		z_x {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1195_4 {
		y_u_1 {Type O LastRead -1 FirstWrite 0}
		y_u {Type O LastRead -1 FirstWrite 0}
		z_u_1 {Type O LastRead -1 FirstWrite 0}
		z_u {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1203_6 {
		y_x_7 {Type O LastRead -1 FirstWrite 1}
		y_x_6 {Type O LastRead -1 FirstWrite 1}
		y_x_5 {Type O LastRead -1 FirstWrite 1}
		y_x_4 {Type O LastRead -1 FirstWrite 1}
		y_x_3 {Type O LastRead -1 FirstWrite 1}
		y_x_2 {Type O LastRead -1 FirstWrite 1}
		y_x_1 {Type O LastRead -1 FirstWrite 1}
		y_x {Type O LastRead -1 FirstWrite 1}
		z_x_7 {Type O LastRead -1 FirstWrite 1}
		z_x_6 {Type O LastRead -1 FirstWrite 1}
		z_x_5 {Type O LastRead -1 FirstWrite 1}
		z_x_4 {Type O LastRead -1 FirstWrite 1}
		z_x_3 {Type O LastRead -1 FirstWrite 1}
		z_x_2 {Type O LastRead -1 FirstWrite 1}
		z_x_1 {Type O LastRead -1 FirstWrite 1}
		z_x {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9 {
		y_u_1 {Type O LastRead -1 FirstWrite 1}
		y_u {Type O LastRead -1 FirstWrite 1}
		z_u_1 {Type O LastRead -1 FirstWrite 1}
		z_u {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19 {
		y_x_5 {Type IO LastRead 0 FirstWrite 2}
		y_x {Type IO LastRead 0 FirstWrite 2}
		z_x_7 {Type O LastRead -1 FirstWrite 1}
		z_x_6 {Type O LastRead -1 FirstWrite 1}
		z_x_5 {Type IO LastRead 1 FirstWrite 2}
		z_x_4 {Type O LastRead -1 FirstWrite 1}
		z_x_3 {Type O LastRead -1 FirstWrite 1}
		z_x_2 {Type O LastRead -1 FirstWrite 1}
		z_x_1 {Type O LastRead -1 FirstWrite 1}
		z_x {Type IO LastRead 1 FirstWrite 2}
		sol_x_1 {Type I LastRead 0 FirstWrite -1}
		sol_x_2 {Type I LastRead 0 FirstWrite -1}
		sol_x_3 {Type I LastRead 0 FirstWrite -1}
		sol_x_4 {Type I LastRead 0 FirstWrite -1}
		sol_x_6 {Type I LastRead 0 FirstWrite -1}
		sol_x_7 {Type I LastRead 0 FirstWrite -1}
		sol_x {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		step_data_3 {Type I LastRead 0 FirstWrite -1}
		step_data_4 {Type I LastRead 0 FirstWrite -1}
		terminal_wall_x_lb_con_reload {Type I LastRead 0 FirstWrite -1}
		terminal_wall_x_ub_con_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln1337 {Type I LastRead 0 FirstWrite -1}
		lnorm_da_out {Type O LastRead -1 FirstWrite 5}
		znorm_da_out {Type O LastRead -1 FirstWrite 5}
		dual_da_out {Type O LastRead -1 FirstWrite 5}
		primal_da_out {Type O LastRead -1 FirstWrite 5}
		lnorm_ey_out {Type O LastRead -1 FirstWrite 5}
		znorm_ey_out {Type O LastRead -1 FirstWrite 5}
		dual_ey_out {Type O LastRead -1 FirstWrite 5}
		primal_ey_out {Type O LastRead -1 FirstWrite 5}
		x_norm_out {Type O LastRead -1 FirstWrite 5}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20 {
		z_u_1 {Type IO LastRead 1 FirstWrite 2}
		z_u {Type IO LastRead 1 FirstWrite 2}
		y_u_1 {Type IO LastRead 0 FirstWrite 2}
		y_u {Type IO LastRead 0 FirstWrite 2}
		sol_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln1395 {Type I LastRead 0 FirstWrite -1}
		step_data_5 {Type I LastRead 0 FirstWrite -1}
		lnorm_u1_out {Type O LastRead -1 FirstWrite 5}
		znorm_u1_out {Type O LastRead -1 FirstWrite 5}
		dual_u1_out {Type O LastRead -1 FirstWrite 5}
		primal_u1_out {Type O LastRead -1 FirstWrite 5}
		lnorm_u0_out {Type O LastRead -1 FirstWrite 5}
		znorm_u0_out {Type O LastRead -1 FirstWrite 5}
		dual_u0_out {Type O LastRead -1 FirstWrite 5}
		primal_u0_out {Type O LastRead -1 FirstWrite 5}
		u_norm_out {Type O LastRead -1 FirstWrite 5}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1511_21 {
		y_x_5 {Type IO LastRead 1 FirstWrite 1}
		y_x {Type IO LastRead 0 FirstWrite 1}
		scale_rho_up {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1555_24 {
		z_x {Type I LastRead 0 FirstWrite -1}
		y_x {Type I LastRead 0 FirstWrite -1}
		z_x_1 {Type I LastRead 0 FirstWrite -1}
		y_x_1 {Type I LastRead 0 FirstWrite -1}
		z_x_2 {Type I LastRead 0 FirstWrite -1}
		y_x_2 {Type I LastRead 0 FirstWrite -1}
		z_x_3 {Type I LastRead 0 FirstWrite -1}
		y_x_3 {Type I LastRead 0 FirstWrite -1}
		z_x_4 {Type I LastRead 0 FirstWrite -1}
		y_x_4 {Type I LastRead 0 FirstWrite -1}
		z_x_5 {Type I LastRead 0 FirstWrite -1}
		y_x_5 {Type I LastRead 0 FirstWrite -1}
		z_x_6 {Type I LastRead 0 FirstWrite -1}
		y_x_6 {Type I LastRead 0 FirstWrite -1}
		z_x_7 {Type I LastRead 0 FirstWrite -1}
		y_x_7 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1562_26 {
		z_u {Type I LastRead 0 FirstWrite -1}
		y_u {Type I LastRead 0 FirstWrite -1}
		z_u_1 {Type I LastRead 0 FirstWrite -1}
		y_u_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1525_22 {
		y_u_1 {Type IO LastRead 0 FirstWrite 1}
		y_u {Type IO LastRead 0 FirstWrite 1}
		scale_rho_u_up {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1246_12 {
		z_x_7 {Type O LastRead -1 FirstWrite 1}
		z_x_6 {Type O LastRead -1 FirstWrite 1}
		z_x_5 {Type O LastRead -1 FirstWrite 1}
		z_x_4 {Type O LastRead -1 FirstWrite 1}
		z_x_3 {Type O LastRead -1 FirstWrite 1}
		z_x_2 {Type O LastRead -1 FirstWrite 1}
		z_x_1 {Type O LastRead -1 FirstWrite 1}
		z_x {Type O LastRead -1 FirstWrite 2}
		sol_x {Type I LastRead 1 FirstWrite -1}
		sol_x_1 {Type I LastRead 0 FirstWrite -1}
		sol_x_2 {Type I LastRead 0 FirstWrite -1}
		sol_x_3 {Type I LastRead 0 FirstWrite -1}
		sol_x_4 {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		sol_x_6 {Type I LastRead 0 FirstWrite -1}
		sol_x_7 {Type I LastRead 0 FirstWrite -1}
		step_data_3 {Type I LastRead 0 FirstWrite -1}
		step_data_4 {Type I LastRead 1 FirstWrite -1}
		terminal_wall_x_lb_con_reload {Type I LastRead 0 FirstWrite -1}
		terminal_wall_x_ub_con_reload {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1282_14 {
		z_u_1 {Type O LastRead -1 FirstWrite 1}
		z_u {Type O LastRead -1 FirstWrite 1}
		step_data_5 {Type I LastRead 0 FirstWrite -1}
		sol_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16 {
		y_x_5 {Type O LastRead -1 FirstWrite 1}
		y_x {Type O LastRead -1 FirstWrite 1}
		sol_x {Type I LastRead 0 FirstWrite -1}
		z_x {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		z_x_5 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1305_17 {
		y_u_1 {Type O LastRead -1 FirstWrite 1}
		y_u {Type O LastRead -1 FirstWrite 1}
		sol_u {Type I LastRead 0 FirstWrite -1}
		z_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}
		z_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_persist_writeback_hls {
		prev_steer_rate {Type I LastRead 0 FirstWrite -1}
		prev_accel {Type I LastRead 0 FirstWrite -1}
		prev_curvature {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type O LastRead -1 FirstWrite 0}}
	fp_recip {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "1", "Max" : "151086"}
	, {"Name" : "Interval", "Min" : "1", "Max" : "151086"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	ey { ap_none {  { ey in_data 0 32 } } }
	epsi { ap_none {  { epsi in_data 0 32 } } }
	vx { ap_none {  { vx in_data 0 32 } } }
	vy { ap_none {  { vy in_data 0 32 } } }
	omega { ap_none {  { omega in_data 0 32 } } }
	steering { ap_none {  { steering in_data 0 32 } } }
	prev_accel { ap_none {  { prev_accel in_data 0 32 } } }
	control_flags { ap_none {  { control_flags in_data 0 3 } } }
	ref_reference_heading_error { ap_memory {  { ref_reference_heading_error_address0 mem_address 1 5 }  { ref_reference_heading_error_ce0 mem_ce 1 1 }  { ref_reference_heading_error_q0 mem_dout 0 32 } } }
	ref_reference_lateral_error { ap_memory {  { ref_reference_lateral_error_address0 mem_address 1 5 }  { ref_reference_lateral_error_ce0 mem_ce 1 1 }  { ref_reference_lateral_error_q0 mem_dout 0 32 } } }
	ref_reference_velocity { ap_memory {  { ref_reference_velocity_address0 mem_address 1 5 }  { ref_reference_velocity_ce0 mem_ce 1 1 }  { ref_reference_velocity_q0 mem_dout 0 32 } } }
	ref_reference_lateral_velocity { ap_memory {  { ref_reference_lateral_velocity_address0 mem_address 1 5 }  { ref_reference_lateral_velocity_ce0 mem_ce 1 1 }  { ref_reference_lateral_velocity_q0 mem_dout 0 32 } } }
	ref_reference_yaw_rate { ap_memory {  { ref_reference_yaw_rate_address0 mem_address 1 5 }  { ref_reference_yaw_rate_ce0 mem_ce 1 1 }  { ref_reference_yaw_rate_q0 mem_dout 0 32 } } }
	ref_path_curvature { ap_memory {  { ref_path_curvature_address0 mem_address 1 5 }  { ref_path_curvature_ce0 mem_ce 1 1 }  { ref_path_curvature_q0 mem_dout 0 32 } } }
	ref_left_wall_bound { ap_memory {  { ref_left_wall_bound_address0 mem_address 1 5 }  { ref_left_wall_bound_ce0 mem_ce 1 1 }  { ref_left_wall_bound_q0 mem_dout 0 32 } } }
	ref_right_wall_bound { ap_memory {  { ref_right_wall_bound_address0 mem_address 1 5 }  { ref_right_wall_bound_ce0 mem_ce 1 1 }  { ref_right_wall_bound_q0 mem_dout 0 32 } } }
	out_steering { ap_vld {  { out_steering out_data 1 32 }  { out_steering_ap_vld out_vld 1 1 } } }
	out_accel { ap_vld {  { out_accel out_data 1 32 }  { out_accel_ap_vld out_vld 1 1 } } }
	out_status { ap_vld {  { out_status out_data 1 1 }  { out_status_ap_vld out_vld 1 1 } } }
	out_iters { ap_vld {  { out_iters out_data 1 32 }  { out_iters_ap_vld out_vld 1 1 } } }
	out_steering_arg_index { ap_none {  { out_steering_arg_index in_data 0 11 } } }
	out_accel_arg_index { ap_none {  { out_accel_arg_index in_data 0 11 } } }
	out_status_arg_index { ap_none {  { out_status_arg_index in_data 0 11 } } }
	out_iters_arg_index { ap_none {  { out_iters_arg_index in_data 0 11 } } }
}
