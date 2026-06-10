set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20
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
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict z_u_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_u { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_u_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_u { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict sol_u { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_u_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_5 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ z_u_1 int 32 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ z_u int 32 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ y_u_1 int 32 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ y_u int 32 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ sol_u int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ sol_u_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ sext_ln1395 int 32 regular  }
	{ step_data_5 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ lnorm_u1_out int 32 regular {pointer 1}  }
	{ znorm_u1_out int 32 regular {pointer 1}  }
	{ dual_u1_out int 32 regular {pointer 1}  }
	{ primal_u1_out int 32 regular {pointer 1}  }
	{ lnorm_u0_out int 32 regular {pointer 1}  }
	{ znorm_u0_out int 31 regular {pointer 1}  }
	{ dual_u0_out int 32 regular {pointer 1}  }
	{ primal_u0_out int 32 regular {pointer 1}  }
	{ u_norm_out int 32 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "z_u_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "z_u", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "y_u_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "y_u", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "sol_u", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_u_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln1395", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "lnorm_u1_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_u1_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_u1_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_u1_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lnorm_u0_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_u0_out", "interface" : "wire", "bitwidth" : 31, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_u0_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_u0_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "u_norm_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 62
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ z_u_1_address0 sc_out sc_lv 5 signal 0 } 
	{ z_u_1_ce0 sc_out sc_logic 1 signal 0 } 
	{ z_u_1_we0 sc_out sc_logic 1 signal 0 } 
	{ z_u_1_d0 sc_out sc_lv 32 signal 0 } 
	{ z_u_1_address1 sc_out sc_lv 5 signal 0 } 
	{ z_u_1_ce1 sc_out sc_logic 1 signal 0 } 
	{ z_u_1_q1 sc_in sc_lv 32 signal 0 } 
	{ z_u_address0 sc_out sc_lv 5 signal 1 } 
	{ z_u_ce0 sc_out sc_logic 1 signal 1 } 
	{ z_u_we0 sc_out sc_logic 1 signal 1 } 
	{ z_u_d0 sc_out sc_lv 32 signal 1 } 
	{ z_u_address1 sc_out sc_lv 5 signal 1 } 
	{ z_u_ce1 sc_out sc_logic 1 signal 1 } 
	{ z_u_q1 sc_in sc_lv 32 signal 1 } 
	{ y_u_1_address0 sc_out sc_lv 5 signal 2 } 
	{ y_u_1_ce0 sc_out sc_logic 1 signal 2 } 
	{ y_u_1_we0 sc_out sc_logic 1 signal 2 } 
	{ y_u_1_d0 sc_out sc_lv 32 signal 2 } 
	{ y_u_1_address1 sc_out sc_lv 5 signal 2 } 
	{ y_u_1_ce1 sc_out sc_logic 1 signal 2 } 
	{ y_u_1_q1 sc_in sc_lv 32 signal 2 } 
	{ y_u_address0 sc_out sc_lv 5 signal 3 } 
	{ y_u_ce0 sc_out sc_logic 1 signal 3 } 
	{ y_u_we0 sc_out sc_logic 1 signal 3 } 
	{ y_u_d0 sc_out sc_lv 32 signal 3 } 
	{ y_u_address1 sc_out sc_lv 5 signal 3 } 
	{ y_u_ce1 sc_out sc_logic 1 signal 3 } 
	{ y_u_q1 sc_in sc_lv 32 signal 3 } 
	{ sol_u_address0 sc_out sc_lv 5 signal 4 } 
	{ sol_u_ce0 sc_out sc_logic 1 signal 4 } 
	{ sol_u_q0 sc_in sc_lv 32 signal 4 } 
	{ sol_u_1_address0 sc_out sc_lv 5 signal 5 } 
	{ sol_u_1_ce0 sc_out sc_logic 1 signal 5 } 
	{ sol_u_1_q0 sc_in sc_lv 32 signal 5 } 
	{ sext_ln1395 sc_in sc_lv 32 signal 6 } 
	{ step_data_5_address0 sc_out sc_lv 5 signal 7 } 
	{ step_data_5_ce0 sc_out sc_logic 1 signal 7 } 
	{ step_data_5_q0 sc_in sc_lv 32 signal 7 } 
	{ lnorm_u1_out sc_out sc_lv 32 signal 8 } 
	{ lnorm_u1_out_ap_vld sc_out sc_logic 1 outvld 8 } 
	{ znorm_u1_out sc_out sc_lv 32 signal 9 } 
	{ znorm_u1_out_ap_vld sc_out sc_logic 1 outvld 9 } 
	{ dual_u1_out sc_out sc_lv 32 signal 10 } 
	{ dual_u1_out_ap_vld sc_out sc_logic 1 outvld 10 } 
	{ primal_u1_out sc_out sc_lv 32 signal 11 } 
	{ primal_u1_out_ap_vld sc_out sc_logic 1 outvld 11 } 
	{ lnorm_u0_out sc_out sc_lv 32 signal 12 } 
	{ lnorm_u0_out_ap_vld sc_out sc_logic 1 outvld 12 } 
	{ znorm_u0_out sc_out sc_lv 31 signal 13 } 
	{ znorm_u0_out_ap_vld sc_out sc_logic 1 outvld 13 } 
	{ dual_u0_out sc_out sc_lv 32 signal 14 } 
	{ dual_u0_out_ap_vld sc_out sc_logic 1 outvld 14 } 
	{ primal_u0_out sc_out sc_lv 32 signal 15 } 
	{ primal_u0_out_ap_vld sc_out sc_logic 1 outvld 15 } 
	{ u_norm_out sc_out sc_lv 32 signal 16 } 
	{ u_norm_out_ap_vld sc_out sc_logic 1 outvld 16 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "z_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u_1", "role": "address0" }} , 
 	{ "name": "z_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "ce0" }} , 
 	{ "name": "z_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "we0" }} , 
 	{ "name": "z_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_u_1", "role": "d0" }} , 
 	{ "name": "z_u_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u_1", "role": "address1" }} , 
 	{ "name": "z_u_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "ce1" }} , 
 	{ "name": "z_u_1_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_u_1", "role": "q1" }} , 
 	{ "name": "z_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u", "role": "address0" }} , 
 	{ "name": "z_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "ce0" }} , 
 	{ "name": "z_u_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "we0" }} , 
 	{ "name": "z_u_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_u", "role": "d0" }} , 
 	{ "name": "z_u_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u", "role": "address1" }} , 
 	{ "name": "z_u_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "ce1" }} , 
 	{ "name": "z_u_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_u", "role": "q1" }} , 
 	{ "name": "y_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u_1", "role": "address0" }} , 
 	{ "name": "y_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "ce0" }} , 
 	{ "name": "y_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "we0" }} , 
 	{ "name": "y_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_u_1", "role": "d0" }} , 
 	{ "name": "y_u_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u_1", "role": "address1" }} , 
 	{ "name": "y_u_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "ce1" }} , 
 	{ "name": "y_u_1_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_u_1", "role": "q1" }} , 
 	{ "name": "y_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u", "role": "address0" }} , 
 	{ "name": "y_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "ce0" }} , 
 	{ "name": "y_u_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "we0" }} , 
 	{ "name": "y_u_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_u", "role": "d0" }} , 
 	{ "name": "y_u_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u", "role": "address1" }} , 
 	{ "name": "y_u_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "ce1" }} , 
 	{ "name": "y_u_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_u", "role": "q1" }} , 
 	{ "name": "sol_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_u", "role": "address0" }} , 
 	{ "name": "sol_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_u", "role": "ce0" }} , 
 	{ "name": "sol_u_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_u", "role": "q0" }} , 
 	{ "name": "sol_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_u_1", "role": "address0" }} , 
 	{ "name": "sol_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_u_1", "role": "ce0" }} , 
 	{ "name": "sol_u_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_u_1", "role": "q0" }} , 
 	{ "name": "sext_ln1395", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln1395", "role": "default" }} , 
 	{ "name": "step_data_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_5", "role": "address0" }} , 
 	{ "name": "step_data_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_5", "role": "ce0" }} , 
 	{ "name": "step_data_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_5", "role": "q0" }} , 
 	{ "name": "lnorm_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lnorm_u1_out", "role": "default" }} , 
 	{ "name": "lnorm_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_u1_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "znorm_u1_out", "role": "default" }} , 
 	{ "name": "znorm_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_u1_out", "role": "ap_vld" }} , 
 	{ "name": "dual_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "dual_u1_out", "role": "default" }} , 
 	{ "name": "dual_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_u1_out", "role": "ap_vld" }} , 
 	{ "name": "primal_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "primal_u1_out", "role": "default" }} , 
 	{ "name": "primal_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_u1_out", "role": "ap_vld" }} , 
 	{ "name": "lnorm_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lnorm_u0_out", "role": "default" }} , 
 	{ "name": "lnorm_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_u0_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":31, "type": "signal", "bundle":{"name": "znorm_u0_out", "role": "default" }} , 
 	{ "name": "znorm_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_u0_out", "role": "ap_vld" }} , 
 	{ "name": "dual_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "dual_u0_out", "role": "default" }} , 
 	{ "name": "dual_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_u0_out", "role": "ap_vld" }} , 
 	{ "name": "primal_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "primal_u0_out", "role": "default" }} , 
 	{ "name": "primal_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_u0_out", "role": "ap_vld" }} , 
 	{ "name": "u_norm_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "u_norm_out", "role": "default" }} , 
 	{ "name": "u_norm_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "u_norm_out", "role": "ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		u_norm_out {Type O LastRead -1 FirstWrite 5}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "27", "Max" : "27"}
	, {"Name" : "Interval", "Min" : "21", "Max" : "21"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	z_u_1 { ap_memory {  { z_u_1_address0 mem_address 1 5 }  { z_u_1_ce0 mem_ce 1 1 }  { z_u_1_we0 mem_we 1 1 }  { z_u_1_d0 mem_din 1 32 }  { z_u_1_address1 MemPortADDR2 1 5 }  { z_u_1_ce1 MemPortCE2 1 1 }  { z_u_1_q1 MemPortDOUT2 0 32 } } }
	z_u { ap_memory {  { z_u_address0 mem_address 1 5 }  { z_u_ce0 mem_ce 1 1 }  { z_u_we0 mem_we 1 1 }  { z_u_d0 mem_din 1 32 }  { z_u_address1 MemPortADDR2 1 5 }  { z_u_ce1 MemPortCE2 1 1 }  { z_u_q1 MemPortDOUT2 0 32 } } }
	y_u_1 { ap_memory {  { y_u_1_address0 mem_address 1 5 }  { y_u_1_ce0 mem_ce 1 1 }  { y_u_1_we0 mem_we 1 1 }  { y_u_1_d0 mem_din 1 32 }  { y_u_1_address1 MemPortADDR2 1 5 }  { y_u_1_ce1 MemPortCE2 1 1 }  { y_u_1_q1 MemPortDOUT2 0 32 } } }
	y_u { ap_memory {  { y_u_address0 mem_address 1 5 }  { y_u_ce0 mem_ce 1 1 }  { y_u_we0 mem_we 1 1 }  { y_u_d0 mem_din 1 32 }  { y_u_address1 MemPortADDR2 1 5 }  { y_u_ce1 MemPortCE2 1 1 }  { y_u_q1 MemPortDOUT2 0 32 } } }
	sol_u { ap_memory {  { sol_u_address0 mem_address 1 5 }  { sol_u_ce0 mem_ce 1 1 }  { sol_u_q0 mem_dout 0 32 } } }
	sol_u_1 { ap_memory {  { sol_u_1_address0 mem_address 1 5 }  { sol_u_1_ce0 mem_ce 1 1 }  { sol_u_1_q0 mem_dout 0 32 } } }
	sext_ln1395 { ap_none {  { sext_ln1395 in_data 0 32 } } }
	step_data_5 { ap_memory {  { step_data_5_address0 mem_address 1 5 }  { step_data_5_ce0 mem_ce 1 1 }  { step_data_5_q0 mem_dout 0 32 } } }
	lnorm_u1_out { ap_vld {  { lnorm_u1_out out_data 1 32 }  { lnorm_u1_out_ap_vld out_vld 1 1 } } }
	znorm_u1_out { ap_vld {  { znorm_u1_out out_data 1 32 }  { znorm_u1_out_ap_vld out_vld 1 1 } } }
	dual_u1_out { ap_vld {  { dual_u1_out out_data 1 32 }  { dual_u1_out_ap_vld out_vld 1 1 } } }
	primal_u1_out { ap_vld {  { primal_u1_out out_data 1 32 }  { primal_u1_out_ap_vld out_vld 1 1 } } }
	lnorm_u0_out { ap_vld {  { lnorm_u0_out out_data 1 32 }  { lnorm_u0_out_ap_vld out_vld 1 1 } } }
	znorm_u0_out { ap_vld {  { znorm_u0_out out_data 1 31 }  { znorm_u0_out_ap_vld out_vld 1 1 } } }
	dual_u0_out { ap_vld {  { dual_u0_out out_data 1 32 }  { dual_u0_out_ap_vld out_vld 1 1 } } }
	primal_u0_out { ap_vld {  { primal_u0_out out_data 1 32 }  { primal_u0_out_ap_vld out_vld 1 1 } } }
	u_norm_out { ap_vld {  { u_norm_out out_data 1 32 }  { u_norm_out_ap_vld out_vld 1 1 } } }
}
