set moduleName mpc_compute_hls
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
set cdfgNum 74
set C_modelName {mpc_compute_hls}
set C_modelType { int 116 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict ref_reference_heading_error { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_lateral_error { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_velocity { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_lateral_velocity { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_reference_yaw_rate { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_path_curvature { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_left_wall_bound { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_right_wall_bound { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_0 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_0 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ state_ey int 26 regular  }
	{ state_epsi int 26 regular  }
	{ state_vx int 26 regular  }
	{ state_vy int 26 regular  }
	{ state_omega int 26 regular  }
	{ ref_reference_heading_error int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_lateral_error int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_velocity int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_lateral_velocity int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_reference_yaw_rate int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_path_curvature int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_left_wall_bound int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_right_wall_bound int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ p_anonymous_namespace_g_core_state_persist_actual_steering int 26 regular {pointer 0} {global 0}  }
	{ p_anonymous_namespace_g_core_state_persist_prev_steer_rate int 26 regular {pointer 2} {global 2}  }
	{ p_anonymous_namespace_g_core_state_persist_prev_accel int 26 regular {pointer 2} {global 2}  }
	{ p_anonymous_namespace_g_core_state_persist_prev_curvature int 26 regular {pointer 2} {global 2}  }
	{ p_anonymous_namespace_g_core_state_persist_prev_model_signature int 1 regular {pointer 2} {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_initialized int 1 regular {pointer 2} {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_0 int 26 regular {array 21 { 2 1 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_1 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_2 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_3 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_4 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_5 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_6 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_7 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_0 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_1 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_2 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_3 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_4 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_5 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_6 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_7 int 26 regular {array 21 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_u_0 int 26 regular {array 20 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_z_u_1 int 26 regular {array 20 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_0 int 26 regular {array 20 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_1 int 26 regular {array 20 { 2 3 } 1 1 bus  } {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_rho int 26 regular {pointer 2} {global 2}  }
	{ p_anonymous_namespace_g_core_state_admm_rho_u int 26 regular {pointer 2} {global 2}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "state_ey", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "state_epsi", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "state_vx", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "state_vy", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "state_omega", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_heading_error", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_lateral_error", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_velocity", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_lateral_velocity", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_reference_yaw_rate", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_path_curvature", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_left_wall_bound", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_right_wall_bound", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_actual_steering", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "interface" : "wire", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_accel", "interface" : "wire", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_curvature", "interface" : "wire", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "interface" : "wire", "bitwidth" : 1, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_initialized", "interface" : "wire", "bitwidth" : 1, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_rho", "interface" : "wire", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_rho_u", "interface" : "wire", "bitwidth" : 26, "direction" : "READWRITE", "extern" : 0} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 116} ]}
# RTL Port declarations: 
set portNum 164
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ state_ey sc_in sc_lv 26 signal 0 } 
	{ state_epsi sc_in sc_lv 26 signal 1 } 
	{ state_vx sc_in sc_lv 26 signal 2 } 
	{ state_vy sc_in sc_lv 26 signal 3 } 
	{ state_omega sc_in sc_lv 26 signal 4 } 
	{ ref_reference_heading_error_address0 sc_out sc_lv 5 signal 5 } 
	{ ref_reference_heading_error_ce0 sc_out sc_logic 1 signal 5 } 
	{ ref_reference_heading_error_q0 sc_in sc_lv 26 signal 5 } 
	{ ref_reference_lateral_error_address0 sc_out sc_lv 5 signal 6 } 
	{ ref_reference_lateral_error_ce0 sc_out sc_logic 1 signal 6 } 
	{ ref_reference_lateral_error_q0 sc_in sc_lv 26 signal 6 } 
	{ ref_reference_velocity_address0 sc_out sc_lv 5 signal 7 } 
	{ ref_reference_velocity_ce0 sc_out sc_logic 1 signal 7 } 
	{ ref_reference_velocity_q0 sc_in sc_lv 26 signal 7 } 
	{ ref_reference_lateral_velocity_address0 sc_out sc_lv 5 signal 8 } 
	{ ref_reference_lateral_velocity_ce0 sc_out sc_logic 1 signal 8 } 
	{ ref_reference_lateral_velocity_q0 sc_in sc_lv 26 signal 8 } 
	{ ref_reference_yaw_rate_address0 sc_out sc_lv 5 signal 9 } 
	{ ref_reference_yaw_rate_ce0 sc_out sc_logic 1 signal 9 } 
	{ ref_reference_yaw_rate_q0 sc_in sc_lv 26 signal 9 } 
	{ ref_path_curvature_address0 sc_out sc_lv 5 signal 10 } 
	{ ref_path_curvature_ce0 sc_out sc_logic 1 signal 10 } 
	{ ref_path_curvature_q0 sc_in sc_lv 26 signal 10 } 
	{ ref_left_wall_bound_address0 sc_out sc_lv 5 signal 11 } 
	{ ref_left_wall_bound_ce0 sc_out sc_logic 1 signal 11 } 
	{ ref_left_wall_bound_q0 sc_in sc_lv 26 signal 11 } 
	{ ref_right_wall_bound_address0 sc_out sc_lv 5 signal 12 } 
	{ ref_right_wall_bound_ce0 sc_out sc_logic 1 signal 12 } 
	{ ref_right_wall_bound_q0 sc_in sc_lv 26 signal 12 } 
	{ p_anonymous_namespace_g_core_state_persist_actual_steering sc_in sc_lv 26 signal 13 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_steer_rate_i sc_in sc_lv 26 signal 14 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o sc_out sc_lv 26 signal 14 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o_ap_vld sc_out sc_logic 1 outvld 14 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_accel_i sc_in sc_lv 26 signal 15 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_accel_o sc_out sc_lv 26 signal 15 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_accel_o_ap_vld sc_out sc_logic 1 outvld 15 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_curvature_i sc_in sc_lv 26 signal 16 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_curvature_o sc_out sc_lv 26 signal 16 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_curvature_o_ap_vld sc_out sc_logic 1 outvld 16 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_model_signature_i sc_in sc_lv 1 signal 17 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_model_signature_o sc_out sc_lv 1 signal 17 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_model_signature_o_ap_vld sc_out sc_logic 1 outvld 17 } 
	{ p_anonymous_namespace_g_core_state_admm_initialized_i sc_in sc_lv 1 signal 18 } 
	{ p_anonymous_namespace_g_core_state_admm_initialized_o sc_out sc_lv 1 signal 18 } 
	{ p_anonymous_namespace_g_core_state_admm_initialized_o_ap_vld sc_out sc_logic 1 outvld 18 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_address0 sc_out sc_lv 5 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_ce0 sc_out sc_logic 1 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_we0 sc_out sc_logic 1 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_d0 sc_out sc_lv 26 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_q0 sc_in sc_lv 26 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_address1 sc_out sc_lv 5 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_ce1 sc_out sc_logic 1 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_q1 sc_in sc_lv 26 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_address0 sc_out sc_lv 5 signal 20 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_ce0 sc_out sc_logic 1 signal 20 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_we0 sc_out sc_logic 1 signal 20 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_d0 sc_out sc_lv 26 signal 20 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_q0 sc_in sc_lv 26 signal 20 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_address0 sc_out sc_lv 5 signal 21 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_ce0 sc_out sc_logic 1 signal 21 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_we0 sc_out sc_logic 1 signal 21 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_d0 sc_out sc_lv 26 signal 21 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_q0 sc_in sc_lv 26 signal 21 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_address0 sc_out sc_lv 5 signal 22 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_ce0 sc_out sc_logic 1 signal 22 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_we0 sc_out sc_logic 1 signal 22 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_d0 sc_out sc_lv 26 signal 22 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_q0 sc_in sc_lv 26 signal 22 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_address0 sc_out sc_lv 5 signal 23 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_ce0 sc_out sc_logic 1 signal 23 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_we0 sc_out sc_logic 1 signal 23 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_d0 sc_out sc_lv 26 signal 23 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_q0 sc_in sc_lv 26 signal 23 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_address0 sc_out sc_lv 5 signal 24 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_ce0 sc_out sc_logic 1 signal 24 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_we0 sc_out sc_logic 1 signal 24 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_d0 sc_out sc_lv 26 signal 24 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_q0 sc_in sc_lv 26 signal 24 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_address0 sc_out sc_lv 5 signal 25 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_ce0 sc_out sc_logic 1 signal 25 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_we0 sc_out sc_logic 1 signal 25 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_d0 sc_out sc_lv 26 signal 25 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_q0 sc_in sc_lv 26 signal 25 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_address0 sc_out sc_lv 5 signal 26 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_ce0 sc_out sc_logic 1 signal 26 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_we0 sc_out sc_logic 1 signal 26 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_d0 sc_out sc_lv 26 signal 26 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_q0 sc_in sc_lv 26 signal 26 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_address0 sc_out sc_lv 5 signal 27 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_ce0 sc_out sc_logic 1 signal 27 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_we0 sc_out sc_logic 1 signal 27 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_d0 sc_out sc_lv 26 signal 27 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_q0 sc_in sc_lv 26 signal 27 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_address0 sc_out sc_lv 5 signal 28 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_ce0 sc_out sc_logic 1 signal 28 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_we0 sc_out sc_logic 1 signal 28 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_d0 sc_out sc_lv 26 signal 28 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_q0 sc_in sc_lv 26 signal 28 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_address0 sc_out sc_lv 5 signal 29 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_ce0 sc_out sc_logic 1 signal 29 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_we0 sc_out sc_logic 1 signal 29 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_d0 sc_out sc_lv 26 signal 29 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_q0 sc_in sc_lv 26 signal 29 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_address0 sc_out sc_lv 5 signal 30 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_ce0 sc_out sc_logic 1 signal 30 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_we0 sc_out sc_logic 1 signal 30 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_d0 sc_out sc_lv 26 signal 30 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_q0 sc_in sc_lv 26 signal 30 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_address0 sc_out sc_lv 5 signal 31 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_ce0 sc_out sc_logic 1 signal 31 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_we0 sc_out sc_logic 1 signal 31 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_d0 sc_out sc_lv 26 signal 31 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_q0 sc_in sc_lv 26 signal 31 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_address0 sc_out sc_lv 5 signal 32 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_ce0 sc_out sc_logic 1 signal 32 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_we0 sc_out sc_logic 1 signal 32 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_d0 sc_out sc_lv 26 signal 32 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_q0 sc_in sc_lv 26 signal 32 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_address0 sc_out sc_lv 5 signal 33 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_ce0 sc_out sc_logic 1 signal 33 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_we0 sc_out sc_logic 1 signal 33 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_d0 sc_out sc_lv 26 signal 33 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_q0 sc_in sc_lv 26 signal 33 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_address0 sc_out sc_lv 5 signal 34 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_ce0 sc_out sc_logic 1 signal 34 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_we0 sc_out sc_logic 1 signal 34 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_d0 sc_out sc_lv 26 signal 34 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_q0 sc_in sc_lv 26 signal 34 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_address0 sc_out sc_lv 5 signal 35 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 sc_out sc_logic 1 signal 35 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_we0 sc_out sc_logic 1 signal 35 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_d0 sc_out sc_lv 26 signal 35 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_q0 sc_in sc_lv 26 signal 35 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_address0 sc_out sc_lv 5 signal 36 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 sc_out sc_logic 1 signal 36 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_we0 sc_out sc_logic 1 signal 36 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_d0 sc_out sc_lv 26 signal 36 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_q0 sc_in sc_lv 26 signal 36 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_address0 sc_out sc_lv 5 signal 37 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 sc_out sc_logic 1 signal 37 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_we0 sc_out sc_logic 1 signal 37 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_d0 sc_out sc_lv 26 signal 37 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_q0 sc_in sc_lv 26 signal 37 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_address0 sc_out sc_lv 5 signal 38 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 sc_out sc_logic 1 signal 38 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_we0 sc_out sc_logic 1 signal 38 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_d0 sc_out sc_lv 26 signal 38 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_q0 sc_in sc_lv 26 signal 38 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_i sc_in sc_lv 26 signal 39 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_o sc_out sc_lv 26 signal 39 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_o_ap_vld sc_out sc_logic 1 outvld 39 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_u_i sc_in sc_lv 26 signal 40 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_u_o sc_out sc_lv 26 signal 40 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_u_o_ap_vld sc_out sc_logic 1 outvld 40 } 
	{ ap_return_0 sc_out sc_lv 26 signal -1 } 
	{ ap_return_1 sc_out sc_lv 26 signal -1 } 
	{ ap_return_2 sc_out sc_lv 32 signal -1 } 
	{ ap_return_3 sc_out sc_lv 32 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "state_ey", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_ey", "role": "default" }} , 
 	{ "name": "state_epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_epsi", "role": "default" }} , 
 	{ "name": "state_vx", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_vx", "role": "default" }} , 
 	{ "name": "state_vy", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_vy", "role": "default" }} , 
 	{ "name": "state_omega", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "state_omega", "role": "default" }} , 
 	{ "name": "ref_reference_heading_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "address0" }} , 
 	{ "name": "ref_reference_heading_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "ce0" }} , 
 	{ "name": "ref_reference_heading_error_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_heading_error", "role": "q0" }} , 
 	{ "name": "ref_reference_lateral_error_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "address0" }} , 
 	{ "name": "ref_reference_lateral_error_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "ce0" }} , 
 	{ "name": "ref_reference_lateral_error_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_lateral_error", "role": "q0" }} , 
 	{ "name": "ref_reference_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "address0" }} , 
 	{ "name": "ref_reference_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "ce0" }} , 
 	{ "name": "ref_reference_velocity_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_velocity", "role": "q0" }} , 
 	{ "name": "ref_reference_lateral_velocity_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "address0" }} , 
 	{ "name": "ref_reference_lateral_velocity_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "ce0" }} , 
 	{ "name": "ref_reference_lateral_velocity_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_lateral_velocity", "role": "q0" }} , 
 	{ "name": "ref_reference_yaw_rate_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "address0" }} , 
 	{ "name": "ref_reference_yaw_rate_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "ce0" }} , 
 	{ "name": "ref_reference_yaw_rate_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_reference_yaw_rate", "role": "q0" }} , 
 	{ "name": "ref_path_curvature_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "address0" }} , 
 	{ "name": "ref_path_curvature_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "ce0" }} , 
 	{ "name": "ref_path_curvature_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_path_curvature", "role": "q0" }} , 
 	{ "name": "ref_left_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "address0" }} , 
 	{ "name": "ref_left_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "ce0" }} , 
 	{ "name": "ref_left_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "q0" }} , 
 	{ "name": "ref_right_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "address0" }} , 
 	{ "name": "ref_right_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "ce0" }} , 
 	{ "name": "ref_right_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_actual_steering", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_actual_steering", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "role": "i" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "role": "o" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "role": "o_ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_accel_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_accel", "role": "i" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_accel_o", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_accel", "role": "o" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_accel_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_accel", "role": "o_ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_curvature_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_curvature", "role": "i" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_curvature_o", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_curvature", "role": "o" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_curvature_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_curvature", "role": "o_ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature_i", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "role": "i" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature_o", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "role": "o" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "role": "o_ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_initialized_i", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_initialized", "role": "i" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_initialized_o", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_initialized", "role": "o" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_initialized_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_initialized", "role": "o_ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "address1" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "ce1" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "q1" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho", "role": "i" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_o", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho", "role": "o" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho", "role": "o_ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_u_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho_u", "role": "i" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_u_o", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho_u", "role": "o" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_u_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho_u", "role": "o_ap_vld" }} , 
 	{ "name": "ap_return_0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_0", "role": "default" }} , 
 	{ "name": "ap_return_1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_1", "role": "default" }} , 
 	{ "name": "ap_return_2", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ap_return_2", "role": "default" }} , 
 	{ "name": "ap_return_3", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ap_return_3", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_compute_hls {
		state_ey {Type I LastRead 4 FirstWrite -1}
		state_epsi {Type I LastRead 4 FirstWrite -1}
		state_vx {Type I LastRead 4 FirstWrite -1}
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
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type I LastRead 4 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type IO LastRead 4 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type IO LastRead 4 FirstWrite 0}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type IO LastRead 9 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type IO LastRead 9 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_initialized {Type IO LastRead 11 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type IO LastRead 13 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type IO LastRead 24 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type IO LastRead 24 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type IO LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type IO LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho {Type IO LastRead 11 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type IO LastRead 11 FirstWrite 1}}
	riccati_pass_hls {
		step_data_0_0 {Type I LastRead 27 FirstWrite -1}
		step_data_0_1 {Type I LastRead 27 FirstWrite -1}
		step_data_0_2 {Type I LastRead 27 FirstWrite -1}
		step_data_0_3 {Type I LastRead 27 FirstWrite -1}
		step_data_0_4 {Type I LastRead 27 FirstWrite -1}
		step_data_0_5 {Type I LastRead 27 FirstWrite -1}
		step_data_1 {Type I LastRead 16 FirstWrite -1}
		step_data_2 {Type I LastRead 16 FirstWrite -1}
		B_sparse_0 {Type I LastRead 8 FirstWrite -1}
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
		z_x_0 {Type I LastRead 11 FirstWrite -1}
		z_x_5 {Type I LastRead 11 FirstWrite -1}
		y_x_0 {Type I LastRead 11 FirstWrite -1}
		y_x_5 {Type I LastRead 11 FirstWrite -1}
		z_u_0 {Type I LastRead 8 FirstWrite -1}
		z_u_1 {Type I LastRead 8 FirstWrite -1}
		y_u_0 {Type I LastRead 8 FirstWrite -1}
		y_u_1 {Type I LastRead 8 FirstWrite -1}
		K_0_0_0 {Type IO LastRead 33 FirstWrite -1}
		K_0_0_1 {Type IO LastRead 34 FirstWrite -1}
		K_0_0_2 {Type IO LastRead 34 FirstWrite -1}
		K_0_0_3 {Type IO LastRead 34 FirstWrite -1}
		K_0_0_4 {Type IO LastRead 34 FirstWrite -1}
		K_0_0_5 {Type IO LastRead 34 FirstWrite -1}
		K_0_0_6 {Type IO LastRead 33 FirstWrite -1}
		K_0_0_7 {Type IO LastRead 33 FirstWrite -1}
		K_0_1_0 {Type IO LastRead 33 FirstWrite -1}
		K_0_1_1 {Type IO LastRead 34 FirstWrite -1}
		K_0_1_2 {Type IO LastRead 34 FirstWrite -1}
		K_0_1_3 {Type IO LastRead 34 FirstWrite -1}
		K_0_1_4 {Type IO LastRead 34 FirstWrite -1}
		K_0_1_5 {Type IO LastRead 34 FirstWrite -1}
		K_0_1_6 {Type IO LastRead 33 FirstWrite -1}
		K_0_1_7 {Type IO LastRead 33 FirstWrite -1}
		K_1_0_0 {Type IO LastRead 33 FirstWrite -1}
		K_1_0_1 {Type IO LastRead 34 FirstWrite -1}
		K_1_0_2 {Type IO LastRead 34 FirstWrite -1}
		K_1_0_3 {Type IO LastRead 34 FirstWrite -1}
		K_1_0_4 {Type IO LastRead 34 FirstWrite -1}
		K_1_0_5 {Type IO LastRead 34 FirstWrite -1}
		K_1_0_6 {Type IO LastRead 33 FirstWrite -1}
		K_1_0_7 {Type IO LastRead 33 FirstWrite -1}
		K_1_1_0 {Type IO LastRead 33 FirstWrite -1}
		K_1_1_1 {Type IO LastRead 34 FirstWrite -1}
		K_1_1_2 {Type IO LastRead 34 FirstWrite -1}
		K_1_1_3 {Type IO LastRead 34 FirstWrite -1}
		K_1_1_4 {Type IO LastRead 34 FirstWrite -1}
		K_1_1_5 {Type IO LastRead 34 FirstWrite -1}
		K_1_1_6 {Type IO LastRead 33 FirstWrite -1}
		K_1_1_7 {Type IO LastRead 33 FirstWrite -1}
		K_2_0_0 {Type IO LastRead 33 FirstWrite -1}
		K_2_0_1 {Type IO LastRead 34 FirstWrite -1}
		K_2_0_2 {Type IO LastRead 34 FirstWrite -1}
		K_2_0_3 {Type IO LastRead 34 FirstWrite -1}
		K_2_0_4 {Type IO LastRead 34 FirstWrite -1}
		K_2_0_5 {Type IO LastRead 34 FirstWrite -1}
		K_2_0_6 {Type IO LastRead 33 FirstWrite -1}
		K_2_0_7 {Type IO LastRead 33 FirstWrite -1}
		K_2_1_0 {Type IO LastRead 33 FirstWrite -1}
		K_2_1_1 {Type IO LastRead 34 FirstWrite -1}
		K_2_1_2 {Type IO LastRead 34 FirstWrite -1}
		K_2_1_3 {Type IO LastRead 34 FirstWrite -1}
		K_2_1_4 {Type IO LastRead 34 FirstWrite -1}
		K_2_1_5 {Type IO LastRead 34 FirstWrite -1}
		K_2_1_6 {Type IO LastRead 33 FirstWrite -1}
		K_2_1_7 {Type IO LastRead 33 FirstWrite -1}
		K_3_0_0 {Type IO LastRead 33 FirstWrite -1}
		K_3_0_1 {Type IO LastRead 34 FirstWrite -1}
		K_3_0_2 {Type IO LastRead 34 FirstWrite -1}
		K_3_0_3 {Type IO LastRead 34 FirstWrite -1}
		K_3_0_4 {Type IO LastRead 34 FirstWrite -1}
		K_3_0_5 {Type IO LastRead 34 FirstWrite -1}
		K_3_0_6 {Type IO LastRead 33 FirstWrite -1}
		K_3_0_7 {Type IO LastRead 33 FirstWrite -1}
		K_3_1_0 {Type IO LastRead 33 FirstWrite -1}
		K_3_1_1 {Type IO LastRead 34 FirstWrite -1}
		K_3_1_2 {Type IO LastRead 34 FirstWrite -1}
		K_3_1_3 {Type IO LastRead 34 FirstWrite -1}
		K_3_1_4 {Type IO LastRead 34 FirstWrite -1}
		K_3_1_5 {Type IO LastRead 34 FirstWrite -1}
		K_3_1_6 {Type IO LastRead 33 FirstWrite -1}
		K_3_1_7 {Type IO LastRead 33 FirstWrite -1}
		kk_0 {Type IO LastRead 3 FirstWrite -1}
		kk_1 {Type IO LastRead 3 FirstWrite -1}
		x_out_0 {Type IO LastRead 1 FirstWrite 0}
		x_out_1 {Type IO LastRead 1 FirstWrite 0}
		x_out_2 {Type IO LastRead 1 FirstWrite 0}
		x_out_3 {Type IO LastRead 1 FirstWrite 0}
		x_out_4 {Type IO LastRead 1 FirstWrite 0}
		x_out_5 {Type IO LastRead 1 FirstWrite 0}
		x_out_6 {Type IO LastRead 1 FirstWrite 0}
		x_out_7 {Type IO LastRead 1 FirstWrite 0}
		u_out_0 {Type O LastRead -1 FirstWrite 4}
		u_out_1 {Type O LastRead -1 FirstWrite 4}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}}
	riccati_backward_pass {
		step_data_0_0 {Type I LastRead 27 FirstWrite -1}
		step_data_0_1 {Type I LastRead 27 FirstWrite -1}
		step_data_0_2 {Type I LastRead 27 FirstWrite -1}
		step_data_0_3 {Type I LastRead 27 FirstWrite -1}
		step_data_0_4 {Type I LastRead 27 FirstWrite -1}
		step_data_0_5 {Type I LastRead 27 FirstWrite -1}
		step_data_1 {Type I LastRead 16 FirstWrite -1}
		step_data_2 {Type I LastRead 16 FirstWrite -1}
		B_sparse_0 {Type I LastRead 8 FirstWrite -1}
		B_sparse_1 {Type I LastRead 9 FirstWrite -1}
		B_sparse_2 {Type I LastRead 9 FirstWrite -1}
		B_sparse_3 {Type I LastRead 9 FirstWrite -1}
		p_read {Type I LastRead 7 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 7 FirstWrite -1}
		rho {Type I LastRead 5 FirstWrite -1}
		rho_u {Type I LastRead 7 FirstWrite -1}
		z_x_0 {Type I LastRead 11 FirstWrite -1}
		z_x_5 {Type I LastRead 11 FirstWrite -1}
		y_x_0 {Type I LastRead 11 FirstWrite -1}
		y_x_5 {Type I LastRead 11 FirstWrite -1}
		z_u_0 {Type I LastRead 8 FirstWrite -1}
		z_u_1 {Type I LastRead 8 FirstWrite -1}
		y_u_0 {Type I LastRead 8 FirstWrite -1}
		y_u_1 {Type I LastRead 8 FirstWrite -1}
		K_0_0_0 {Type IO LastRead 33 FirstWrite 1}
		K_0_0_1 {Type IO LastRead 34 FirstWrite 1}
		K_0_0_2 {Type IO LastRead 34 FirstWrite 1}
		K_0_0_3 {Type IO LastRead 34 FirstWrite 1}
		K_0_0_4 {Type IO LastRead 34 FirstWrite 1}
		K_0_0_5 {Type IO LastRead 34 FirstWrite 1}
		K_0_0_6 {Type IO LastRead 33 FirstWrite 1}
		K_0_0_7 {Type IO LastRead 33 FirstWrite 1}
		K_0_1_0 {Type IO LastRead 33 FirstWrite 1}
		K_0_1_1 {Type IO LastRead 34 FirstWrite 1}
		K_0_1_2 {Type IO LastRead 34 FirstWrite 1}
		K_0_1_3 {Type IO LastRead 34 FirstWrite 1}
		K_0_1_4 {Type IO LastRead 34 FirstWrite 1}
		K_0_1_5 {Type IO LastRead 34 FirstWrite 1}
		K_0_1_6 {Type IO LastRead 33 FirstWrite 1}
		K_0_1_7 {Type IO LastRead 33 FirstWrite 1}
		K_1_0_0 {Type IO LastRead 33 FirstWrite 1}
		K_1_0_1 {Type IO LastRead 34 FirstWrite 1}
		K_1_0_2 {Type IO LastRead 34 FirstWrite 1}
		K_1_0_3 {Type IO LastRead 34 FirstWrite 1}
		K_1_0_4 {Type IO LastRead 34 FirstWrite 1}
		K_1_0_5 {Type IO LastRead 34 FirstWrite 1}
		K_1_0_6 {Type IO LastRead 33 FirstWrite 1}
		K_1_0_7 {Type IO LastRead 33 FirstWrite 1}
		K_1_1_0 {Type IO LastRead 33 FirstWrite 1}
		K_1_1_1 {Type IO LastRead 34 FirstWrite 1}
		K_1_1_2 {Type IO LastRead 34 FirstWrite 1}
		K_1_1_3 {Type IO LastRead 34 FirstWrite 1}
		K_1_1_4 {Type IO LastRead 34 FirstWrite 1}
		K_1_1_5 {Type IO LastRead 34 FirstWrite 1}
		K_1_1_6 {Type IO LastRead 33 FirstWrite 1}
		K_1_1_7 {Type IO LastRead 33 FirstWrite 1}
		K_2_0_0 {Type IO LastRead 33 FirstWrite 1}
		K_2_0_1 {Type IO LastRead 34 FirstWrite 1}
		K_2_0_2 {Type IO LastRead 34 FirstWrite 1}
		K_2_0_3 {Type IO LastRead 34 FirstWrite 1}
		K_2_0_4 {Type IO LastRead 34 FirstWrite 1}
		K_2_0_5 {Type IO LastRead 34 FirstWrite 1}
		K_2_0_6 {Type IO LastRead 33 FirstWrite 1}
		K_2_0_7 {Type IO LastRead 33 FirstWrite 1}
		K_2_1_0 {Type IO LastRead 33 FirstWrite 1}
		K_2_1_1 {Type IO LastRead 34 FirstWrite 1}
		K_2_1_2 {Type IO LastRead 34 FirstWrite 1}
		K_2_1_3 {Type IO LastRead 34 FirstWrite 1}
		K_2_1_4 {Type IO LastRead 34 FirstWrite 1}
		K_2_1_5 {Type IO LastRead 34 FirstWrite 1}
		K_2_1_6 {Type IO LastRead 33 FirstWrite 1}
		K_2_1_7 {Type IO LastRead 33 FirstWrite 1}
		K_3_0_0 {Type IO LastRead 33 FirstWrite 1}
		K_3_0_1 {Type IO LastRead 34 FirstWrite 1}
		K_3_0_2 {Type IO LastRead 34 FirstWrite 1}
		K_3_0_3 {Type IO LastRead 34 FirstWrite 1}
		K_3_0_4 {Type IO LastRead 34 FirstWrite 1}
		K_3_0_5 {Type IO LastRead 34 FirstWrite 1}
		K_3_0_6 {Type IO LastRead 33 FirstWrite 1}
		K_3_0_7 {Type IO LastRead 33 FirstWrite 1}
		K_3_1_0 {Type IO LastRead 33 FirstWrite 1}
		K_3_1_1 {Type IO LastRead 34 FirstWrite 1}
		K_3_1_2 {Type IO LastRead 34 FirstWrite 1}
		K_3_1_3 {Type IO LastRead 34 FirstWrite 1}
		K_3_1_4 {Type IO LastRead 34 FirstWrite 1}
		K_3_1_5 {Type IO LastRead 34 FirstWrite 1}
		K_3_1_6 {Type IO LastRead 33 FirstWrite 1}
		K_3_1_7 {Type IO LastRead 33 FirstWrite 1}
		kk_0 {Type O LastRead -1 FirstWrite 33}
		kk_1 {Type O LastRead -1 FirstWrite 33}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}}
	invert_2x2_qp_hls {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 4 FirstWrite -1}
		p_read5 {Type I LastRead 4 FirstWrite -1}
		p_read6 {Type I LastRead 4 FirstWrite -1}
		p_read7 {Type I LastRead 4 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}}
	fp_recip {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_742_9 {
		s1 {Type I LastRead 0 FirstWrite -1}
		s0 {Type I LastRead 0 FirstWrite -1}
		s1_1 {Type I LastRead 0 FirstWrite -1}
		s0_1 {Type I LastRead 0 FirstWrite -1}
		M_8 {Type I LastRead 0 FirstWrite -1}
		M {Type I LastRead 0 FirstWrite -1}
		M_10 {Type I LastRead 0 FirstWrite -1}
		M_9 {Type I LastRead 0 FirstWrite -1}
		M_12 {Type I LastRead 0 FirstWrite -1}
		M_11 {Type I LastRead 0 FirstWrite -1}
		M_14 {Type I LastRead 0 FirstWrite -1}
		M_13 {Type I LastRead 0 FirstWrite -1}
		b_215_cast {Type I LastRead 0 FirstWrite -1}
		b_216_cast {Type I LastRead 0 FirstWrite -1}
		b_217_cast {Type I LastRead 0 FirstWrite -1}
		b_218_cast {Type I LastRead 0 FirstWrite -1}
		b_219_cast {Type I LastRead 0 FirstWrite -1}
		b_220_cast {Type I LastRead 0 FirstWrite -1}
		b_158_cast {Type I LastRead 0 FirstWrite -1}
		b_159_cast {Type I LastRead 0 FirstWrite -1}
		b_160_cast {Type I LastRead 0 FirstWrite -1}
		b_161_cast {Type I LastRead 0 FirstWrite -1}
		b_162_cast {Type I LastRead 0 FirstWrite -1}
		b_163_cast {Type I LastRead 0 FirstWrite -1}
		b_164_cast {Type I LastRead 0 FirstWrite -1}
		b_165_cast {Type I LastRead 0 FirstWrite -1}
		b_166_cast {Type I LastRead 0 FirstWrite -1}
		b_167_cast {Type I LastRead 0 FirstWrite -1}
		b_168_cast {Type I LastRead 0 FirstWrite -1}
		b_169_cast {Type I LastRead 0 FirstWrite -1}
		b_170_cast {Type I LastRead 0 FirstWrite -1}
		b_171_cast {Type I LastRead 0 FirstWrite -1}
		b_172_cast {Type I LastRead 0 FirstWrite -1}
		b_173_cast {Type I LastRead 0 FirstWrite -1}
		b_174_cast {Type I LastRead 0 FirstWrite -1}
		b_175_cast {Type I LastRead 0 FirstWrite -1}
		b_176_cast {Type I LastRead 0 FirstWrite -1}
		b_177_cast {Type I LastRead 0 FirstWrite -1}
		b_178_cast {Type I LastRead 0 FirstWrite -1}
		b_179_cast {Type I LastRead 0 FirstWrite -1}
		b_180_cast {Type I LastRead 0 FirstWrite -1}
		b_181_cast {Type I LastRead 0 FirstWrite -1}
		b_182_cast {Type I LastRead 0 FirstWrite -1}
		b_183_cast {Type I LastRead 0 FirstWrite -1}
		b_184_cast {Type I LastRead 0 FirstWrite -1}
		b_185_cast {Type I LastRead 0 FirstWrite -1}
		b_186_cast {Type I LastRead 0 FirstWrite -1}
		sext_ln742 {Type I LastRead 0 FirstWrite -1}
		G_12_out {Type O LastRead -1 FirstWrite 0}
		G_11_out {Type O LastRead -1 FirstWrite 0}
		G_10_out {Type O LastRead -1 FirstWrite 0}
		G_9_out {Type O LastRead -1 FirstWrite 0}
		G_8_out {Type O LastRead -1 FirstWrite 0}
		G_7_out {Type O LastRead -1 FirstWrite 0}
		G_6_out {Type O LastRead -1 FirstWrite 0}
		G_5_out {Type O LastRead -1 FirstWrite 0}
		G_4_out {Type O LastRead -1 FirstWrite 0}
		G_3_out {Type O LastRead -1 FirstWrite 0}
		G_2_out {Type O LastRead -1 FirstWrite 0}
		G_1_out {Type O LastRead -1 FirstWrite 0}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_783_12 {
		P {Type I LastRead 0 FirstWrite -1}
		sext_ln256 {Type I LastRead 0 FirstWrite -1}
		P_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_1 {Type I LastRead 0 FirstWrite -1}
		P_2 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_2 {Type I LastRead 0 FirstWrite -1}
		P_3 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_3 {Type I LastRead 0 FirstWrite -1}
		P_4 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_4 {Type I LastRead 0 FirstWrite -1}
		P_5 {Type I LastRead 0 FirstWrite -1}
		sext_ln783 {Type I LastRead 0 FirstWrite -1}
		p_6 {Type I LastRead 0 FirstWrite -1}
		p_7 {Type I LastRead 0 FirstWrite -1}
		p_8 {Type I LastRead 0 FirstWrite -1}
		p_9 {Type I LastRead 0 FirstWrite -1}
		p_10 {Type I LastRead 0 FirstWrite -1}
		p_11 {Type I LastRead 0 FirstWrite -1}
		p_12 {Type I LastRead 0 FirstWrite -1}
		p_13 {Type I LastRead 0 FirstWrite -1}
		p_shift_7_out {Type O LastRead -1 FirstWrite 4}
		p_shift_6_out {Type O LastRead -1 FirstWrite 4}
		p_shift_5_out {Type O LastRead -1 FirstWrite 4}
		p_shift_4_out {Type O LastRead -1 FirstWrite 4}
		p_shift_3_out {Type O LastRead -1 FirstWrite 4}
		p_shift_2_out {Type O LastRead -1 FirstWrite 4}
		p_shift_1_out {Type O LastRead -1 FirstWrite 4}
		p_shift_out {Type O LastRead -1 FirstWrite 4}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_767_11 {
		K_0_0_0 {Type O LastRead -1 FirstWrite 1}
		zext_ln491 {Type I LastRead 0 FirstWrite -1}
		K_0_1_0 {Type O LastRead -1 FirstWrite 1}
		K_0_0_1 {Type O LastRead -1 FirstWrite 1}
		K_0_1_1 {Type O LastRead -1 FirstWrite 1}
		K_0_0_2 {Type O LastRead -1 FirstWrite 1}
		K_0_1_2 {Type O LastRead -1 FirstWrite 1}
		K_0_0_3 {Type O LastRead -1 FirstWrite 1}
		K_0_1_3 {Type O LastRead -1 FirstWrite 1}
		K_0_0_4 {Type O LastRead -1 FirstWrite 1}
		K_0_1_4 {Type O LastRead -1 FirstWrite 1}
		K_0_0_5 {Type O LastRead -1 FirstWrite 1}
		K_0_1_5 {Type O LastRead -1 FirstWrite 1}
		K_0_0_6 {Type O LastRead -1 FirstWrite 1}
		K_0_1_6 {Type O LastRead -1 FirstWrite 1}
		K_0_0_7 {Type O LastRead -1 FirstWrite 1}
		K_0_1_7 {Type O LastRead -1 FirstWrite 1}
		K_1_0_0 {Type O LastRead -1 FirstWrite 1}
		K_1_1_0 {Type O LastRead -1 FirstWrite 1}
		K_1_0_1 {Type O LastRead -1 FirstWrite 1}
		K_1_1_1 {Type O LastRead -1 FirstWrite 1}
		K_1_0_2 {Type O LastRead -1 FirstWrite 1}
		K_1_1_2 {Type O LastRead -1 FirstWrite 1}
		K_1_0_3 {Type O LastRead -1 FirstWrite 1}
		K_1_1_3 {Type O LastRead -1 FirstWrite 1}
		K_1_0_4 {Type O LastRead -1 FirstWrite 1}
		K_1_1_4 {Type O LastRead -1 FirstWrite 1}
		K_1_0_5 {Type O LastRead -1 FirstWrite 1}
		K_1_1_5 {Type O LastRead -1 FirstWrite 1}
		K_1_0_6 {Type O LastRead -1 FirstWrite 1}
		K_1_1_6 {Type O LastRead -1 FirstWrite 1}
		K_1_0_7 {Type O LastRead -1 FirstWrite 1}
		K_1_1_7 {Type O LastRead -1 FirstWrite 1}
		K_2_0_0 {Type O LastRead -1 FirstWrite 1}
		K_2_1_0 {Type O LastRead -1 FirstWrite 1}
		K_2_0_1 {Type O LastRead -1 FirstWrite 1}
		K_2_1_1 {Type O LastRead -1 FirstWrite 1}
		K_2_0_2 {Type O LastRead -1 FirstWrite 1}
		K_2_1_2 {Type O LastRead -1 FirstWrite 1}
		K_2_0_3 {Type O LastRead -1 FirstWrite 1}
		K_2_1_3 {Type O LastRead -1 FirstWrite 1}
		K_2_0_4 {Type O LastRead -1 FirstWrite 1}
		K_2_1_4 {Type O LastRead -1 FirstWrite 1}
		K_2_0_5 {Type O LastRead -1 FirstWrite 1}
		K_2_1_5 {Type O LastRead -1 FirstWrite 1}
		K_2_0_6 {Type O LastRead -1 FirstWrite 1}
		K_2_1_6 {Type O LastRead -1 FirstWrite 1}
		K_2_0_7 {Type O LastRead -1 FirstWrite 1}
		K_2_1_7 {Type O LastRead -1 FirstWrite 1}
		K_3_0_0 {Type O LastRead -1 FirstWrite 1}
		K_3_1_0 {Type O LastRead -1 FirstWrite 1}
		K_3_0_1 {Type O LastRead -1 FirstWrite 1}
		K_3_1_1 {Type O LastRead -1 FirstWrite 1}
		K_3_0_2 {Type O LastRead -1 FirstWrite 1}
		K_3_1_2 {Type O LastRead -1 FirstWrite 1}
		K_3_0_3 {Type O LastRead -1 FirstWrite 1}
		K_3_1_3 {Type O LastRead -1 FirstWrite 1}
		K_3_0_4 {Type O LastRead -1 FirstWrite 1}
		K_3_1_4 {Type O LastRead -1 FirstWrite 1}
		K_3_0_5 {Type O LastRead -1 FirstWrite 1}
		K_3_1_5 {Type O LastRead -1 FirstWrite 1}
		K_3_0_6 {Type O LastRead -1 FirstWrite 1}
		K_3_1_6 {Type O LastRead -1 FirstWrite 1}
		K_3_0_7 {Type O LastRead -1 FirstWrite 1}
		K_3_1_7 {Type O LastRead -1 FirstWrite 1}
		G_12_reload {Type I LastRead 0 FirstWrite -1}
		G_11_reload {Type I LastRead 0 FirstWrite -1}
		G_10_reload {Type I LastRead 0 FirstWrite -1}
		G_9_reload {Type I LastRead 0 FirstWrite -1}
		G_8_reload {Type I LastRead 0 FirstWrite -1}
		G_7_reload {Type I LastRead 0 FirstWrite -1}
		select_ln739 {Type I LastRead 0 FirstWrite -1}
		sext_ln280 {Type I LastRead 0 FirstWrite -1}
		G_6_reload {Type I LastRead 0 FirstWrite -1}
		G_5_reload {Type I LastRead 0 FirstWrite -1}
		G_4_reload {Type I LastRead 0 FirstWrite -1}
		G_3_reload {Type I LastRead 0 FirstWrite -1}
		G_2_reload {Type I LastRead 0 FirstWrite -1}
		G_1_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln280_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln280_2 {Type I LastRead 0 FirstWrite -1}
		sext_ln777 {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_848_14 {
		PA_5_load_4 {Type I LastRead 0 FirstWrite -1}
		PA_5_load_3 {Type I LastRead 0 FirstWrite -1}
		PA_5_load_2 {Type I LastRead 0 FirstWrite -1}
		PA_5_load_1 {Type I LastRead 0 FirstWrite -1}
		PA_5_load {Type I LastRead 0 FirstWrite -1}
		PA_load_4 {Type I LastRead 0 FirstWrite -1}
		PA_load_3 {Type I LastRead 0 FirstWrite -1}
		PA_load_2 {Type I LastRead 0 FirstWrite -1}
		PA_load_1 {Type I LastRead 0 FirstWrite -1}
		PA_load {Type I LastRead 0 FirstWrite -1}
		PA_1_load_4 {Type I LastRead 0 FirstWrite -1}
		PA_1_load_3 {Type I LastRead 0 FirstWrite -1}
		PA_1_load_2 {Type I LastRead 0 FirstWrite -1}
		PA_1_load_1 {Type I LastRead 0 FirstWrite -1}
		PA_1_load {Type I LastRead 0 FirstWrite -1}
		PA_2_load_4 {Type I LastRead 0 FirstWrite -1}
		PA_2_load_3 {Type I LastRead 0 FirstWrite -1}
		PA_2_load_2 {Type I LastRead 0 FirstWrite -1}
		PA_2_load_1 {Type I LastRead 0 FirstWrite -1}
		PA_2_load {Type I LastRead 0 FirstWrite -1}
		PA_3_load_4 {Type I LastRead 0 FirstWrite -1}
		PA_3_load_3 {Type I LastRead 0 FirstWrite -1}
		PA_3_load_2 {Type I LastRead 0 FirstWrite -1}
		PA_3_load_1 {Type I LastRead 0 FirstWrite -1}
		PA_3_load {Type I LastRead 0 FirstWrite -1}
		PA_4_load_4 {Type I LastRead 0 FirstWrite -1}
		PA_4_load_3 {Type I LastRead 0 FirstWrite -1}
		PA_4_load_2 {Type I LastRead 0 FirstWrite -1}
		PA_4_load_1 {Type I LastRead 0 FirstWrite -1}
		PA_4_load {Type I LastRead 0 FirstWrite -1}
		P {Type I LastRead 0 FirstWrite -1}
		P_1 {Type I LastRead 0 FirstWrite -1}
		P_2 {Type I LastRead 0 FirstWrite -1}
		P_3 {Type I LastRead 0 FirstWrite -1}
		P_4 {Type I LastRead 0 FirstWrite -1}
		P_5 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_5 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_7 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_9 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_11 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_13 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_15 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_17 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_19 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_21 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_23 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_25 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_27 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_29 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_31 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_33 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_35 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_37 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_39 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_41 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_43 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_45 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_47 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_49 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_51 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_53 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_55 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_57 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_59 {Type I LastRead 0 FirstWrite -1}
		sext_ln256_61 {Type I LastRead 0 FirstWrite -1}
		sext_ln848 {Type I LastRead 0 FirstWrite -1}
		conv_i_i13988_459_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_357_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_255_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_153_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i1398851_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_449_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_347_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_245_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_143_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i1398841_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_439_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_337_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_235_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_133_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i1398831_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_429_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_327_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_225_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_123_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i1398821_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_419_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_317_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_215_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_113_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i1398811_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_49_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_37_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_25_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i13988_13_out {Type O LastRead -1 FirstWrite 2}
		conv_i_i139881_out {Type O LastRead -1 FirstWrite 2}}
	sum6_P_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_P_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_P_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_P_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40 {
		G_36 {Type I LastRead 0 FirstWrite -1}
		sext_ln288_30 {Type I LastRead 0 FirstWrite -1}
		sext_ln261_2 {Type I LastRead 0 FirstWrite -1}
		p_new_6_out {Type IO LastRead 1 FirstWrite 0}
		p_new_out {Type IO LastRead 1 FirstWrite 0}}
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
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw {
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
		step_data_0_0 {Type I LastRead 0 FirstWrite -1}
		step_data_0_1 {Type I LastRead 0 FirstWrite -1}
		step_data_0_2 {Type I LastRead 0 FirstWrite -1}
		step_data_0_3 {Type I LastRead 0 FirstWrite -1}
		step_data_0_4 {Type I LastRead 0 FirstWrite -1}
		step_data_0_5 {Type I LastRead 0 FirstWrite -1}
		step_data_1 {Type I LastRead 5 FirstWrite -1}
		B_sparse_0 {Type I LastRead 4 FirstWrite -1}
		B_sparse_1 {Type I LastRead 4 FirstWrite -1}
		B_sparse_2 {Type I LastRead 4 FirstWrite -1}
		B_sparse_3 {Type I LastRead 4 FirstWrite -1}
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
		kk_0 {Type I LastRead 3 FirstWrite -1}
		kk_1 {Type I LastRead 3 FirstWrite -1}
		x_out_0 {Type IO LastRead 1 FirstWrite 0}
		x_out_1 {Type IO LastRead 1 FirstWrite 0}
		x_out_2 {Type IO LastRead 1 FirstWrite 0}
		x_out_3 {Type IO LastRead 1 FirstWrite 0}
		x_out_4 {Type IO LastRead 1 FirstWrite 0}
		x_out_5 {Type IO LastRead 1 FirstWrite 0}
		x_out_6 {Type IO LastRead 1 FirstWrite 0}
		x_out_7 {Type IO LastRead 1 FirstWrite 0}
		u_out_0 {Type O LastRead -1 FirstWrite 4}
		u_out_1 {Type O LastRead -1 FirstWrite 4}}
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
	riccati_forward_pass_Pipeline_VITIS_LOOP_399_4 {
		x_out_0 {Type O LastRead -1 FirstWrite 4}
		zext_ln428 {Type I LastRead 0 FirstWrite -1}
		x_out_1 {Type O LastRead -1 FirstWrite 4}
		x_out_2 {Type O LastRead -1 FirstWrite 4}
		x_out_3 {Type O LastRead -1 FirstWrite 4}
		x_out_4 {Type O LastRead -1 FirstWrite 4}
		x_out_5 {Type O LastRead -1 FirstWrite 4}
		mul_ln409_1 {Type I LastRead 0 FirstWrite -1}
		step_data_0_0 {Type I LastRead 0 FirstWrite -1}
		step_data_0_1 {Type I LastRead 0 FirstWrite -1}
		step_data_0_2 {Type I LastRead 0 FirstWrite -1}
		step_data_0_3 {Type I LastRead 0 FirstWrite -1}
		step_data_0_4 {Type I LastRead 0 FirstWrite -1}
		step_data_0_5 {Type I LastRead 0 FirstWrite -1}
		empty_564 {Type I LastRead 0 FirstWrite -1}
		empty_565 {Type I LastRead 0 FirstWrite -1}
		empty_566 {Type I LastRead 0 FirstWrite -1}
		empty_567 {Type I LastRead 0 FirstWrite -1}
		empty_568 {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		sext_ln156 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_2 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_3 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_4 {Type I LastRead 0 FirstWrite -1}
		sext_ln422 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_11 {Type I LastRead 0 FirstWrite -1}
		sext_ln425 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_12 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_13 {Type I LastRead 0 FirstWrite -1}
		sext_ln156_14 {Type I LastRead 0 FirstWrite -1}
		sext_ln436 {Type I LastRead 0 FirstWrite -1}}
	sum6_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_354_1 {
		ref_left_wall_bound {Type I LastRead 0 FirstWrite -1}
		ref_right_wall_bound {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170171_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171169_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170167_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171165_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170163_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171161_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170159_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171157_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170155_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171153_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170151_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171149_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170147_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171145_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170143_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171141_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170139_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171137_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170135_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171133_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170131_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171129_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170127_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171125_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170123_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171121_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170119_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171117_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170115_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171113_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170111_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171109_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170107_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171105_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010170103_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010171101_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01017099_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01017197_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01017095_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01017193_out {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_364_2 {
		p_0_0_010171165_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171161_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171157_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171153_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171149_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171145_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171141_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171137_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171133_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171129_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171125_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171121_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171117_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171113_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171109_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171105_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171101_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01017197_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01017193_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010171169_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170167_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170163_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170159_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170155_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170151_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170147_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170143_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170139_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170135_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170131_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170127_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170123_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170119_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170115_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170111_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170107_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170103_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01017099_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01017095_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010170171_reload {Type I LastRead 0 FirstWrite -1}
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
	reset_admm_state_hls_Pipeline_VITIS_LOOP_322_1 {
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
	reset_admm_state_hls_Pipeline_VITIS_LOOP_330_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1 {
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
	mpc_compute_hls_Pipeline_VITIS_LOOP_1214_4 {
		y_u_1 {Type O LastRead -1 FirstWrite 0}
		y_u {Type O LastRead -1 FirstWrite 0}
		z_u_1 {Type O LastRead -1 FirstWrite 0}
		z_u {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1222_6 {
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
	mpc_compute_hls_Pipeline_VITIS_LOOP_1234_9 {
		y_u_1 {Type O LastRead -1 FirstWrite 1}
		y_u {Type O LastRead -1 FirstWrite 1}
		z_u_1 {Type O LastRead -1 FirstWrite 1}
		z_u {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19 {
		z_u_1 {Type IO LastRead 1 FirstWrite 2}
		z_u {Type IO LastRead 1 FirstWrite 2}
		y_u_1 {Type IO LastRead 0 FirstWrite 2}
		y_u {Type IO LastRead 0 FirstWrite 2}
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
		sext_ln156_35 {Type I LastRead 0 FirstWrite -1}
		sol_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln1362 {Type I LastRead 0 FirstWrite -1}
		step_data_5 {Type I LastRead 0 FirstWrite -1}
		lnorm_u1_out {Type O LastRead -1 FirstWrite 4}
		znorm_u1_out {Type O LastRead -1 FirstWrite 4}
		dual_u1_out {Type O LastRead -1 FirstWrite 4}
		primal_u1_out {Type O LastRead -1 FirstWrite 4}
		lnorm_u0_out {Type O LastRead -1 FirstWrite 4}
		znorm_u0_out {Type O LastRead -1 FirstWrite 4}
		dual_u0_out {Type O LastRead -1 FirstWrite 4}
		primal_u0_out {Type O LastRead -1 FirstWrite 4}
		lnorm_da_out {Type O LastRead -1 FirstWrite 4}
		znorm_da_out {Type O LastRead -1 FirstWrite 4}
		dual_da_out {Type O LastRead -1 FirstWrite 4}
		primal_da_out {Type O LastRead -1 FirstWrite 4}
		lnorm_ey_out {Type O LastRead -1 FirstWrite 4}
		znorm_ey_out {Type O LastRead -1 FirstWrite 4}
		dual_ey_out {Type O LastRead -1 FirstWrite 4}
		primal_ey_out {Type O LastRead -1 FirstWrite 4}
		u_norm_out {Type O LastRead -1 FirstWrite 4}
		x_norm_out {Type O LastRead -1 FirstWrite 4}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1531_20 {
		y_x_5 {Type IO LastRead 1 FirstWrite 1}
		y_x {Type IO LastRead 0 FirstWrite 1}
		scale_rho_up {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1575_23 {
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
	mpc_compute_hls_Pipeline_VITIS_LOOP_1582_25 {
		z_u {Type I LastRead 0 FirstWrite -1}
		y_u {Type I LastRead 0 FirstWrite -1}
		z_u_1 {Type I LastRead 0 FirstWrite -1}
		y_u_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1545_21 {
		y_u_1 {Type IO LastRead 0 FirstWrite 1}
		y_u {Type IO LastRead 0 FirstWrite 1}
		scale_rho_u_up {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1265_12 {
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
	mpc_compute_hls_Pipeline_VITIS_LOOP_1301_14 {
		z_u_1 {Type O LastRead -1 FirstWrite 1}
		z_u {Type O LastRead -1 FirstWrite 1}
		step_data_5 {Type I LastRead 0 FirstWrite -1}
		sol_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1320_16 {
		y_x_5 {Type O LastRead -1 FirstWrite 1}
		y_x {Type O LastRead -1 FirstWrite 1}
		sol_x {Type I LastRead 0 FirstWrite -1}
		z_x {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		z_x_5 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1324_17 {
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
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "3282", "Max" : "101361"}
	, {"Name" : "Interval", "Min" : "3282", "Max" : "101361"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	state_ey { ap_none {  { state_ey in_data 0 26 } } }
	state_epsi { ap_none {  { state_epsi in_data 0 26 } } }
	state_vx { ap_none {  { state_vx in_data 0 26 } } }
	state_vy { ap_none {  { state_vy in_data 0 26 } } }
	state_omega { ap_none {  { state_omega in_data 0 26 } } }
	ref_reference_heading_error { ap_memory {  { ref_reference_heading_error_address0 mem_address 1 5 }  { ref_reference_heading_error_ce0 mem_ce 1 1 }  { ref_reference_heading_error_q0 mem_dout 0 26 } } }
	ref_reference_lateral_error { ap_memory {  { ref_reference_lateral_error_address0 mem_address 1 5 }  { ref_reference_lateral_error_ce0 mem_ce 1 1 }  { ref_reference_lateral_error_q0 mem_dout 0 26 } } }
	ref_reference_velocity { ap_memory {  { ref_reference_velocity_address0 mem_address 1 5 }  { ref_reference_velocity_ce0 mem_ce 1 1 }  { ref_reference_velocity_q0 mem_dout 0 26 } } }
	ref_reference_lateral_velocity { ap_memory {  { ref_reference_lateral_velocity_address0 mem_address 1 5 }  { ref_reference_lateral_velocity_ce0 mem_ce 1 1 }  { ref_reference_lateral_velocity_q0 mem_dout 0 26 } } }
	ref_reference_yaw_rate { ap_memory {  { ref_reference_yaw_rate_address0 mem_address 1 5 }  { ref_reference_yaw_rate_ce0 mem_ce 1 1 }  { ref_reference_yaw_rate_q0 mem_dout 0 26 } } }
	ref_path_curvature { ap_memory {  { ref_path_curvature_address0 mem_address 1 5 }  { ref_path_curvature_ce0 mem_ce 1 1 }  { ref_path_curvature_q0 mem_dout 0 26 } } }
	ref_left_wall_bound { ap_memory {  { ref_left_wall_bound_address0 mem_address 1 5 }  { ref_left_wall_bound_ce0 mem_ce 1 1 }  { ref_left_wall_bound_q0 mem_dout 0 26 } } }
	ref_right_wall_bound { ap_memory {  { ref_right_wall_bound_address0 mem_address 1 5 }  { ref_right_wall_bound_ce0 mem_ce 1 1 }  { ref_right_wall_bound_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_persist_actual_steering { ap_none {  { p_anonymous_namespace_g_core_state_persist_actual_steering in_data 0 26 } } }
	p_anonymous_namespace_g_core_state_persist_prev_steer_rate { ap_ovld {  { p_anonymous_namespace_g_core_state_persist_prev_steer_rate_i in_data 0 26 }  { p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o out_data 1 26 }  { p_anonymous_namespace_g_core_state_persist_prev_steer_rate_o_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_persist_prev_accel { ap_ovld {  { p_anonymous_namespace_g_core_state_persist_prev_accel_i in_data 0 26 }  { p_anonymous_namespace_g_core_state_persist_prev_accel_o out_data 1 26 }  { p_anonymous_namespace_g_core_state_persist_prev_accel_o_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_persist_prev_curvature { ap_ovld {  { p_anonymous_namespace_g_core_state_persist_prev_curvature_i in_data 0 26 }  { p_anonymous_namespace_g_core_state_persist_prev_curvature_o out_data 1 26 }  { p_anonymous_namespace_g_core_state_persist_prev_curvature_o_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_persist_prev_model_signature { ap_ovld {  { p_anonymous_namespace_g_core_state_persist_prev_model_signature_i in_data 0 1 }  { p_anonymous_namespace_g_core_state_persist_prev_model_signature_o out_data 1 1 }  { p_anonymous_namespace_g_core_state_persist_prev_model_signature_o_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_admm_initialized { ap_ovld {  { p_anonymous_namespace_g_core_state_admm_initialized_i in_data 0 1 }  { p_anonymous_namespace_g_core_state_admm_initialized_o out_data 1 1 }  { p_anonymous_namespace_g_core_state_admm_initialized_o_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_q0 mem_dout 0 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_address1 MemPortADDR2 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_ce1 MemPortCE2 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_q1 MemPortDOUT2 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_1_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_1_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_2 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_2_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_2_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_2_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_2_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_2_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_3 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_3_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_3_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_3_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_3_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_3_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_4 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_4_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_4_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_4_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_4_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_4_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_5 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_5_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_5_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_5_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_5_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_5_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_6 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_6_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_6_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_6_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_6_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_6_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_7 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_7_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_7_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_7_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_7_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_x_7_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_0_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_0_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_1_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_1_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_2 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_2_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_2_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_2_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_2_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_2_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_3 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_3_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_3_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_3_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_3_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_3_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_4 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_4_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_4_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_4_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_4_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_4_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_5 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_5_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_5_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_5_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_5_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_5_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_6 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_6_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_6_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_6_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_6_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_6_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_7 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_7_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_7_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_7_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_7_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_x_7_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_d0 mem_din 1 26 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_q0 mem_dout 0 26 } } }
	p_anonymous_namespace_g_core_state_admm_rho { ap_ovld {  { p_anonymous_namespace_g_core_state_admm_rho_i in_data 0 26 }  { p_anonymous_namespace_g_core_state_admm_rho_o out_data 1 26 }  { p_anonymous_namespace_g_core_state_admm_rho_o_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_admm_rho_u { ap_ovld {  { p_anonymous_namespace_g_core_state_admm_rho_u_i in_data 0 26 }  { p_anonymous_namespace_g_core_state_admm_rho_u_o out_data 1 26 }  { p_anonymous_namespace_g_core_state_admm_rho_u_o_ap_vld out_vld 1 1 } } }
}
