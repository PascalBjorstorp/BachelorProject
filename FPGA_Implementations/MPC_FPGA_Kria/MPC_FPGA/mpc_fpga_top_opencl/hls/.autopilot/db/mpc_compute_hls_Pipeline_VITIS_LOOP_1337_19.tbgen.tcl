set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19
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
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict y_x_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_7 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_6 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_4 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_3 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_2 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_1 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict sol_x_1 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_2 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_3 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_4 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_6 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_7 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_4 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ y_x_5 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ y_x int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ z_x_7 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_6 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_5 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ z_x_4 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_3 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_2 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_1 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ sol_x_1 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_2 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_3 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_4 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_6 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_7 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_5 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ step_data_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_4 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ terminal_wall_x_lb_con_reload int 32 regular  }
	{ terminal_wall_x_ub_con_reload int 32 regular  }
	{ sext_ln1337 int 32 regular  }
	{ lnorm_da_out int 32 regular {pointer 1}  }
	{ znorm_da_out int 31 regular {pointer 1}  }
	{ dual_da_out int 32 regular {pointer 1}  }
	{ primal_da_out int 32 regular {pointer 1}  }
	{ lnorm_ey_out int 32 regular {pointer 1}  }
	{ znorm_ey_out int 32 regular {pointer 1}  }
	{ dual_ey_out int 32 regular {pointer 1}  }
	{ primal_ey_out int 32 regular {pointer 1}  }
	{ x_norm_out int 32 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "y_x_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "y_x", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "z_x_7", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_6", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "z_x_4", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_3", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_2", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_1", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "sol_x_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_6", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_7", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "terminal_wall_x_lb_con_reload", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "terminal_wall_x_ub_con_reload", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln1337", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lnorm_da_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_da_out", "interface" : "wire", "bitwidth" : 31, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_da_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_da_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lnorm_ey_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_ey_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_ey_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_ey_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_norm_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 121
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ y_x_5_address0 sc_out sc_lv 5 signal 0 } 
	{ y_x_5_ce0 sc_out sc_logic 1 signal 0 } 
	{ y_x_5_q0 sc_in sc_lv 32 signal 0 } 
	{ y_x_5_address1 sc_out sc_lv 5 signal 0 } 
	{ y_x_5_ce1 sc_out sc_logic 1 signal 0 } 
	{ y_x_5_we1 sc_out sc_logic 1 signal 0 } 
	{ y_x_5_d1 sc_out sc_lv 32 signal 0 } 
	{ y_x_address0 sc_out sc_lv 5 signal 1 } 
	{ y_x_ce0 sc_out sc_logic 1 signal 1 } 
	{ y_x_q0 sc_in sc_lv 32 signal 1 } 
	{ y_x_address1 sc_out sc_lv 5 signal 1 } 
	{ y_x_ce1 sc_out sc_logic 1 signal 1 } 
	{ y_x_we1 sc_out sc_logic 1 signal 1 } 
	{ y_x_d1 sc_out sc_lv 32 signal 1 } 
	{ z_x_7_address1 sc_out sc_lv 5 signal 2 } 
	{ z_x_7_ce1 sc_out sc_logic 1 signal 2 } 
	{ z_x_7_we1 sc_out sc_logic 1 signal 2 } 
	{ z_x_7_d1 sc_out sc_lv 32 signal 2 } 
	{ z_x_6_address1 sc_out sc_lv 5 signal 3 } 
	{ z_x_6_ce1 sc_out sc_logic 1 signal 3 } 
	{ z_x_6_we1 sc_out sc_logic 1 signal 3 } 
	{ z_x_6_d1 sc_out sc_lv 32 signal 3 } 
	{ z_x_5_address0 sc_out sc_lv 5 signal 4 } 
	{ z_x_5_ce0 sc_out sc_logic 1 signal 4 } 
	{ z_x_5_q0 sc_in sc_lv 32 signal 4 } 
	{ z_x_5_address1 sc_out sc_lv 5 signal 4 } 
	{ z_x_5_ce1 sc_out sc_logic 1 signal 4 } 
	{ z_x_5_we1 sc_out sc_logic 1 signal 4 } 
	{ z_x_5_d1 sc_out sc_lv 32 signal 4 } 
	{ z_x_4_address1 sc_out sc_lv 5 signal 5 } 
	{ z_x_4_ce1 sc_out sc_logic 1 signal 5 } 
	{ z_x_4_we1 sc_out sc_logic 1 signal 5 } 
	{ z_x_4_d1 sc_out sc_lv 32 signal 5 } 
	{ z_x_3_address1 sc_out sc_lv 5 signal 6 } 
	{ z_x_3_ce1 sc_out sc_logic 1 signal 6 } 
	{ z_x_3_we1 sc_out sc_logic 1 signal 6 } 
	{ z_x_3_d1 sc_out sc_lv 32 signal 6 } 
	{ z_x_2_address1 sc_out sc_lv 5 signal 7 } 
	{ z_x_2_ce1 sc_out sc_logic 1 signal 7 } 
	{ z_x_2_we1 sc_out sc_logic 1 signal 7 } 
	{ z_x_2_d1 sc_out sc_lv 32 signal 7 } 
	{ z_x_1_address1 sc_out sc_lv 5 signal 8 } 
	{ z_x_1_ce1 sc_out sc_logic 1 signal 8 } 
	{ z_x_1_we1 sc_out sc_logic 1 signal 8 } 
	{ z_x_1_d1 sc_out sc_lv 32 signal 8 } 
	{ z_x_address0 sc_out sc_lv 5 signal 9 } 
	{ z_x_ce0 sc_out sc_logic 1 signal 9 } 
	{ z_x_q0 sc_in sc_lv 32 signal 9 } 
	{ z_x_address1 sc_out sc_lv 5 signal 9 } 
	{ z_x_ce1 sc_out sc_logic 1 signal 9 } 
	{ z_x_we1 sc_out sc_logic 1 signal 9 } 
	{ z_x_d1 sc_out sc_lv 32 signal 9 } 
	{ sol_x_1_address0 sc_out sc_lv 5 signal 10 } 
	{ sol_x_1_ce0 sc_out sc_logic 1 signal 10 } 
	{ sol_x_1_q0 sc_in sc_lv 32 signal 10 } 
	{ sol_x_2_address0 sc_out sc_lv 5 signal 11 } 
	{ sol_x_2_ce0 sc_out sc_logic 1 signal 11 } 
	{ sol_x_2_q0 sc_in sc_lv 32 signal 11 } 
	{ sol_x_3_address0 sc_out sc_lv 5 signal 12 } 
	{ sol_x_3_ce0 sc_out sc_logic 1 signal 12 } 
	{ sol_x_3_q0 sc_in sc_lv 32 signal 12 } 
	{ sol_x_4_address0 sc_out sc_lv 5 signal 13 } 
	{ sol_x_4_ce0 sc_out sc_logic 1 signal 13 } 
	{ sol_x_4_q0 sc_in sc_lv 32 signal 13 } 
	{ sol_x_6_address0 sc_out sc_lv 5 signal 14 } 
	{ sol_x_6_ce0 sc_out sc_logic 1 signal 14 } 
	{ sol_x_6_q0 sc_in sc_lv 32 signal 14 } 
	{ sol_x_7_address0 sc_out sc_lv 5 signal 15 } 
	{ sol_x_7_ce0 sc_out sc_logic 1 signal 15 } 
	{ sol_x_7_q0 sc_in sc_lv 32 signal 15 } 
	{ sol_x_address0 sc_out sc_lv 5 signal 16 } 
	{ sol_x_ce0 sc_out sc_logic 1 signal 16 } 
	{ sol_x_q0 sc_in sc_lv 32 signal 16 } 
	{ sol_x_5_address0 sc_out sc_lv 5 signal 17 } 
	{ sol_x_5_ce0 sc_out sc_logic 1 signal 17 } 
	{ sol_x_5_q0 sc_in sc_lv 32 signal 17 } 
	{ step_data_3_address0 sc_out sc_lv 5 signal 18 } 
	{ step_data_3_ce0 sc_out sc_logic 1 signal 18 } 
	{ step_data_3_q0 sc_in sc_lv 32 signal 18 } 
	{ step_data_4_address0 sc_out sc_lv 5 signal 19 } 
	{ step_data_4_ce0 sc_out sc_logic 1 signal 19 } 
	{ step_data_4_q0 sc_in sc_lv 32 signal 19 } 
	{ terminal_wall_x_lb_con_reload sc_in sc_lv 32 signal 20 } 
	{ terminal_wall_x_ub_con_reload sc_in sc_lv 32 signal 21 } 
	{ sext_ln1337 sc_in sc_lv 32 signal 22 } 
	{ lnorm_da_out sc_out sc_lv 32 signal 23 } 
	{ lnorm_da_out_ap_vld sc_out sc_logic 1 outvld 23 } 
	{ znorm_da_out sc_out sc_lv 31 signal 24 } 
	{ znorm_da_out_ap_vld sc_out sc_logic 1 outvld 24 } 
	{ dual_da_out sc_out sc_lv 32 signal 25 } 
	{ dual_da_out_ap_vld sc_out sc_logic 1 outvld 25 } 
	{ primal_da_out sc_out sc_lv 32 signal 26 } 
	{ primal_da_out_ap_vld sc_out sc_logic 1 outvld 26 } 
	{ lnorm_ey_out sc_out sc_lv 32 signal 27 } 
	{ lnorm_ey_out_ap_vld sc_out sc_logic 1 outvld 27 } 
	{ znorm_ey_out sc_out sc_lv 32 signal 28 } 
	{ znorm_ey_out_ap_vld sc_out sc_logic 1 outvld 28 } 
	{ dual_ey_out sc_out sc_lv 32 signal 29 } 
	{ dual_ey_out_ap_vld sc_out sc_logic 1 outvld 29 } 
	{ primal_ey_out sc_out sc_lv 32 signal 30 } 
	{ primal_ey_out_ap_vld sc_out sc_logic 1 outvld 30 } 
	{ x_norm_out sc_out sc_lv 32 signal 31 } 
	{ x_norm_out_ap_vld sc_out sc_logic 1 outvld 31 } 
	{ grp_fu_6164_p_din0 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_6164_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_6164_p_dout0 sc_in sc_lv 50 signal -1 } 
	{ grp_fu_6164_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_6156_p_din0 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_6156_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_6156_p_dout0 sc_in sc_lv 50 signal -1 } 
	{ grp_fu_6156_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_6160_p_din0 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_6160_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_6160_p_dout0 sc_in sc_lv 50 signal -1 } 
	{ grp_fu_6160_p_ce sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "y_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_5", "role": "address0" }} , 
 	{ "name": "y_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "ce0" }} , 
 	{ "name": "y_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_x_5", "role": "q0" }} , 
 	{ "name": "y_x_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_5", "role": "address1" }} , 
 	{ "name": "y_x_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "ce1" }} , 
 	{ "name": "y_x_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "we1" }} , 
 	{ "name": "y_x_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_x_5", "role": "d1" }} , 
 	{ "name": "y_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x", "role": "address0" }} , 
 	{ "name": "y_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "ce0" }} , 
 	{ "name": "y_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_x", "role": "q0" }} , 
 	{ "name": "y_x_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x", "role": "address1" }} , 
 	{ "name": "y_x_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "ce1" }} , 
 	{ "name": "y_x_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "we1" }} , 
 	{ "name": "y_x_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_x", "role": "d1" }} , 
 	{ "name": "z_x_7_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_7", "role": "address1" }} , 
 	{ "name": "z_x_7_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_7", "role": "ce1" }} , 
 	{ "name": "z_x_7_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_7", "role": "we1" }} , 
 	{ "name": "z_x_7_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_7", "role": "d1" }} , 
 	{ "name": "z_x_6_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_6", "role": "address1" }} , 
 	{ "name": "z_x_6_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_6", "role": "ce1" }} , 
 	{ "name": "z_x_6_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_6", "role": "we1" }} , 
 	{ "name": "z_x_6_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_6", "role": "d1" }} , 
 	{ "name": "z_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_5", "role": "address0" }} , 
 	{ "name": "z_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_5", "role": "ce0" }} , 
 	{ "name": "z_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_5", "role": "q0" }} , 
 	{ "name": "z_x_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_5", "role": "address1" }} , 
 	{ "name": "z_x_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_5", "role": "ce1" }} , 
 	{ "name": "z_x_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_5", "role": "we1" }} , 
 	{ "name": "z_x_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_5", "role": "d1" }} , 
 	{ "name": "z_x_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_4", "role": "address1" }} , 
 	{ "name": "z_x_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_4", "role": "ce1" }} , 
 	{ "name": "z_x_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_4", "role": "we1" }} , 
 	{ "name": "z_x_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_4", "role": "d1" }} , 
 	{ "name": "z_x_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_3", "role": "address1" }} , 
 	{ "name": "z_x_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_3", "role": "ce1" }} , 
 	{ "name": "z_x_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_3", "role": "we1" }} , 
 	{ "name": "z_x_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_3", "role": "d1" }} , 
 	{ "name": "z_x_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_2", "role": "address1" }} , 
 	{ "name": "z_x_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_2", "role": "ce1" }} , 
 	{ "name": "z_x_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_2", "role": "we1" }} , 
 	{ "name": "z_x_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_2", "role": "d1" }} , 
 	{ "name": "z_x_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_1", "role": "address1" }} , 
 	{ "name": "z_x_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_1", "role": "ce1" }} , 
 	{ "name": "z_x_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_1", "role": "we1" }} , 
 	{ "name": "z_x_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_1", "role": "d1" }} , 
 	{ "name": "z_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x", "role": "address0" }} , 
 	{ "name": "z_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "ce0" }} , 
 	{ "name": "z_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x", "role": "q0" }} , 
 	{ "name": "z_x_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x", "role": "address1" }} , 
 	{ "name": "z_x_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "ce1" }} , 
 	{ "name": "z_x_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "we1" }} , 
 	{ "name": "z_x_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x", "role": "d1" }} , 
 	{ "name": "sol_x_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_1", "role": "address0" }} , 
 	{ "name": "sol_x_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_1", "role": "ce0" }} , 
 	{ "name": "sol_x_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_1", "role": "q0" }} , 
 	{ "name": "sol_x_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_2", "role": "address0" }} , 
 	{ "name": "sol_x_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_2", "role": "ce0" }} , 
 	{ "name": "sol_x_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_2", "role": "q0" }} , 
 	{ "name": "sol_x_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_3", "role": "address0" }} , 
 	{ "name": "sol_x_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_3", "role": "ce0" }} , 
 	{ "name": "sol_x_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_3", "role": "q0" }} , 
 	{ "name": "sol_x_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_4", "role": "address0" }} , 
 	{ "name": "sol_x_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_4", "role": "ce0" }} , 
 	{ "name": "sol_x_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_4", "role": "q0" }} , 
 	{ "name": "sol_x_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_6", "role": "address0" }} , 
 	{ "name": "sol_x_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_6", "role": "ce0" }} , 
 	{ "name": "sol_x_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_6", "role": "q0" }} , 
 	{ "name": "sol_x_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_7", "role": "address0" }} , 
 	{ "name": "sol_x_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_7", "role": "ce0" }} , 
 	{ "name": "sol_x_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_7", "role": "q0" }} , 
 	{ "name": "sol_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x", "role": "address0" }} , 
 	{ "name": "sol_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x", "role": "ce0" }} , 
 	{ "name": "sol_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x", "role": "q0" }} , 
 	{ "name": "sol_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_5", "role": "address0" }} , 
 	{ "name": "sol_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_5", "role": "ce0" }} , 
 	{ "name": "sol_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_5", "role": "q0" }} , 
 	{ "name": "step_data_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_3", "role": "address0" }} , 
 	{ "name": "step_data_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_3", "role": "ce0" }} , 
 	{ "name": "step_data_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_3", "role": "q0" }} , 
 	{ "name": "step_data_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_4", "role": "address0" }} , 
 	{ "name": "step_data_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_4", "role": "ce0" }} , 
 	{ "name": "step_data_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_4", "role": "q0" }} , 
 	{ "name": "terminal_wall_x_lb_con_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "terminal_wall_x_lb_con_reload", "role": "default" }} , 
 	{ "name": "terminal_wall_x_ub_con_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "terminal_wall_x_ub_con_reload", "role": "default" }} , 
 	{ "name": "sext_ln1337", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln1337", "role": "default" }} , 
 	{ "name": "lnorm_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lnorm_da_out", "role": "default" }} , 
 	{ "name": "lnorm_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_da_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":31, "type": "signal", "bundle":{"name": "znorm_da_out", "role": "default" }} , 
 	{ "name": "znorm_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_da_out", "role": "ap_vld" }} , 
 	{ "name": "dual_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "dual_da_out", "role": "default" }} , 
 	{ "name": "dual_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_da_out", "role": "ap_vld" }} , 
 	{ "name": "primal_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "primal_da_out", "role": "default" }} , 
 	{ "name": "primal_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_da_out", "role": "ap_vld" }} , 
 	{ "name": "lnorm_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lnorm_ey_out", "role": "default" }} , 
 	{ "name": "lnorm_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_ey_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "znorm_ey_out", "role": "default" }} , 
 	{ "name": "znorm_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_ey_out", "role": "ap_vld" }} , 
 	{ "name": "dual_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "dual_ey_out", "role": "default" }} , 
 	{ "name": "dual_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_ey_out", "role": "ap_vld" }} , 
 	{ "name": "primal_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "primal_ey_out", "role": "default" }} , 
 	{ "name": "primal_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_ey_out", "role": "ap_vld" }} , 
 	{ "name": "x_norm_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_norm_out", "role": "default" }} , 
 	{ "name": "x_norm_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "x_norm_out", "role": "ap_vld" }} , 
 	{ "name": "grp_fu_6164_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_6164_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_6164_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_6164_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_6164_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":50, "type": "signal", "bundle":{"name": "grp_fu_6164_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_6164_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_6164_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_6156_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_6156_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_6156_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_6156_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_6156_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":50, "type": "signal", "bundle":{"name": "grp_fu_6156_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_6156_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_6156_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_6160_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_6160_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_6160_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_6160_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_6160_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":50, "type": "signal", "bundle":{"name": "grp_fu_6160_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_6160_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_6160_p_ce", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		x_norm_out {Type O LastRead -1 FirstWrite 5}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "28", "Max" : "28"}
	, {"Name" : "Interval", "Min" : "22", "Max" : "22"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	y_x_5 { ap_memory {  { y_x_5_address0 mem_address 1 5 }  { y_x_5_ce0 mem_ce 1 1 }  { y_x_5_q0 mem_dout 0 32 }  { y_x_5_address1 MemPortADDR2 1 5 }  { y_x_5_ce1 MemPortCE2 1 1 }  { y_x_5_we1 MemPortWE2 1 1 }  { y_x_5_d1 MemPortDIN2 1 32 } } }
	y_x { ap_memory {  { y_x_address0 mem_address 1 5 }  { y_x_ce0 mem_ce 1 1 }  { y_x_q0 mem_dout 0 32 }  { y_x_address1 MemPortADDR2 1 5 }  { y_x_ce1 MemPortCE2 1 1 }  { y_x_we1 MemPortWE2 1 1 }  { y_x_d1 MemPortDIN2 1 32 } } }
	z_x_7 { ap_memory {  { z_x_7_address1 MemPortADDR2 1 5 }  { z_x_7_ce1 MemPortCE2 1 1 }  { z_x_7_we1 MemPortWE2 1 1 }  { z_x_7_d1 MemPortDIN2 1 32 } } }
	z_x_6 { ap_memory {  { z_x_6_address1 MemPortADDR2 1 5 }  { z_x_6_ce1 MemPortCE2 1 1 }  { z_x_6_we1 MemPortWE2 1 1 }  { z_x_6_d1 MemPortDIN2 1 32 } } }
	z_x_5 { ap_memory {  { z_x_5_address0 mem_address 1 5 }  { z_x_5_ce0 mem_ce 1 1 }  { z_x_5_q0 mem_dout 0 32 }  { z_x_5_address1 MemPortADDR2 1 5 }  { z_x_5_ce1 MemPortCE2 1 1 }  { z_x_5_we1 MemPortWE2 1 1 }  { z_x_5_d1 MemPortDIN2 1 32 } } }
	z_x_4 { ap_memory {  { z_x_4_address1 MemPortADDR2 1 5 }  { z_x_4_ce1 MemPortCE2 1 1 }  { z_x_4_we1 MemPortWE2 1 1 }  { z_x_4_d1 MemPortDIN2 1 32 } } }
	z_x_3 { ap_memory {  { z_x_3_address1 MemPortADDR2 1 5 }  { z_x_3_ce1 MemPortCE2 1 1 }  { z_x_3_we1 MemPortWE2 1 1 }  { z_x_3_d1 MemPortDIN2 1 32 } } }
	z_x_2 { ap_memory {  { z_x_2_address1 MemPortADDR2 1 5 }  { z_x_2_ce1 MemPortCE2 1 1 }  { z_x_2_we1 MemPortWE2 1 1 }  { z_x_2_d1 MemPortDIN2 1 32 } } }
	z_x_1 { ap_memory {  { z_x_1_address1 MemPortADDR2 1 5 }  { z_x_1_ce1 MemPortCE2 1 1 }  { z_x_1_we1 MemPortWE2 1 1 }  { z_x_1_d1 MemPortDIN2 1 32 } } }
	z_x { ap_memory {  { z_x_address0 mem_address 1 5 }  { z_x_ce0 mem_ce 1 1 }  { z_x_q0 mem_dout 0 32 }  { z_x_address1 MemPortADDR2 1 5 }  { z_x_ce1 MemPortCE2 1 1 }  { z_x_we1 MemPortWE2 1 1 }  { z_x_d1 MemPortDIN2 1 32 } } }
	sol_x_1 { ap_memory {  { sol_x_1_address0 mem_address 1 5 }  { sol_x_1_ce0 mem_ce 1 1 }  { sol_x_1_q0 mem_dout 0 32 } } }
	sol_x_2 { ap_memory {  { sol_x_2_address0 mem_address 1 5 }  { sol_x_2_ce0 mem_ce 1 1 }  { sol_x_2_q0 mem_dout 0 32 } } }
	sol_x_3 { ap_memory {  { sol_x_3_address0 mem_address 1 5 }  { sol_x_3_ce0 mem_ce 1 1 }  { sol_x_3_q0 mem_dout 0 32 } } }
	sol_x_4 { ap_memory {  { sol_x_4_address0 mem_address 1 5 }  { sol_x_4_ce0 mem_ce 1 1 }  { sol_x_4_q0 mem_dout 0 32 } } }
	sol_x_6 { ap_memory {  { sol_x_6_address0 mem_address 1 5 }  { sol_x_6_ce0 mem_ce 1 1 }  { sol_x_6_q0 mem_dout 0 32 } } }
	sol_x_7 { ap_memory {  { sol_x_7_address0 mem_address 1 5 }  { sol_x_7_ce0 mem_ce 1 1 }  { sol_x_7_q0 mem_dout 0 32 } } }
	sol_x { ap_memory {  { sol_x_address0 mem_address 1 5 }  { sol_x_ce0 mem_ce 1 1 }  { sol_x_q0 mem_dout 0 32 } } }
	sol_x_5 { ap_memory {  { sol_x_5_address0 mem_address 1 5 }  { sol_x_5_ce0 mem_ce 1 1 }  { sol_x_5_q0 mem_dout 0 32 } } }
	step_data_3 { ap_memory {  { step_data_3_address0 mem_address 1 5 }  { step_data_3_ce0 mem_ce 1 1 }  { step_data_3_q0 mem_dout 0 32 } } }
	step_data_4 { ap_memory {  { step_data_4_address0 mem_address 1 5 }  { step_data_4_ce0 mem_ce 1 1 }  { step_data_4_q0 mem_dout 0 32 } } }
	terminal_wall_x_lb_con_reload { ap_none {  { terminal_wall_x_lb_con_reload in_data 0 32 } } }
	terminal_wall_x_ub_con_reload { ap_none {  { terminal_wall_x_ub_con_reload in_data 0 32 } } }
	sext_ln1337 { ap_none {  { sext_ln1337 in_data 0 32 } } }
	lnorm_da_out { ap_vld {  { lnorm_da_out out_data 1 32 }  { lnorm_da_out_ap_vld out_vld 1 1 } } }
	znorm_da_out { ap_vld {  { znorm_da_out out_data 1 31 }  { znorm_da_out_ap_vld out_vld 1 1 } } }
	dual_da_out { ap_vld {  { dual_da_out out_data 1 32 }  { dual_da_out_ap_vld out_vld 1 1 } } }
	primal_da_out { ap_vld {  { primal_da_out out_data 1 32 }  { primal_da_out_ap_vld out_vld 1 1 } } }
	lnorm_ey_out { ap_vld {  { lnorm_ey_out out_data 1 32 }  { lnorm_ey_out_ap_vld out_vld 1 1 } } }
	znorm_ey_out { ap_vld {  { znorm_ey_out out_data 1 32 }  { znorm_ey_out_ap_vld out_vld 1 1 } } }
	dual_ey_out { ap_vld {  { dual_ey_out out_data 1 32 }  { dual_ey_out_ap_vld out_vld 1 1 } } }
	primal_ey_out { ap_vld {  { primal_ey_out out_data 1 32 }  { primal_ey_out_ap_vld out_vld 1 1 } } }
	x_norm_out { ap_vld {  { x_norm_out out_data 1 32 }  { x_norm_out_ap_vld out_vld 1 1 } } }
}
