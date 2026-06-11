set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_354_1
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
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_354_1}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict ref_left_wall_bound { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_right_wall_bound { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ ref_left_wall_bound int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_right_wall_bound int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ p_0_0_010170171_out int 26 regular {pointer 1}  }
	{ p_0_0_010171169_out int 26 regular {pointer 1}  }
	{ p_0_0_010170167_out int 26 regular {pointer 1}  }
	{ p_0_0_010171165_out int 26 regular {pointer 1}  }
	{ p_0_0_010170163_out int 26 regular {pointer 1}  }
	{ p_0_0_010171161_out int 26 regular {pointer 1}  }
	{ p_0_0_010170159_out int 26 regular {pointer 1}  }
	{ p_0_0_010171157_out int 26 regular {pointer 1}  }
	{ p_0_0_010170155_out int 26 regular {pointer 1}  }
	{ p_0_0_010171153_out int 26 regular {pointer 1}  }
	{ p_0_0_010170151_out int 26 regular {pointer 1}  }
	{ p_0_0_010171149_out int 26 regular {pointer 1}  }
	{ p_0_0_010170147_out int 26 regular {pointer 1}  }
	{ p_0_0_010171145_out int 26 regular {pointer 1}  }
	{ p_0_0_010170143_out int 26 regular {pointer 1}  }
	{ p_0_0_010171141_out int 26 regular {pointer 1}  }
	{ p_0_0_010170139_out int 26 regular {pointer 1}  }
	{ p_0_0_010171137_out int 26 regular {pointer 1}  }
	{ p_0_0_010170135_out int 26 regular {pointer 1}  }
	{ p_0_0_010171133_out int 26 regular {pointer 1}  }
	{ p_0_0_010170131_out int 26 regular {pointer 1}  }
	{ p_0_0_010171129_out int 26 regular {pointer 1}  }
	{ p_0_0_010170127_out int 26 regular {pointer 1}  }
	{ p_0_0_010171125_out int 26 regular {pointer 1}  }
	{ p_0_0_010170123_out int 26 regular {pointer 1}  }
	{ p_0_0_010171121_out int 26 regular {pointer 1}  }
	{ p_0_0_010170119_out int 26 regular {pointer 1}  }
	{ p_0_0_010171117_out int 26 regular {pointer 1}  }
	{ p_0_0_010170115_out int 26 regular {pointer 1}  }
	{ p_0_0_010171113_out int 26 regular {pointer 1}  }
	{ p_0_0_010170111_out int 26 regular {pointer 1}  }
	{ p_0_0_010171109_out int 26 regular {pointer 1}  }
	{ p_0_0_010170107_out int 26 regular {pointer 1}  }
	{ p_0_0_010171105_out int 26 regular {pointer 1}  }
	{ p_0_0_010170103_out int 26 regular {pointer 1}  }
	{ p_0_0_010171101_out int 26 regular {pointer 1}  }
	{ p_0_0_01017099_out int 26 regular {pointer 1}  }
	{ p_0_0_01017197_out int 26 regular {pointer 1}  }
	{ p_0_0_01017095_out int 26 regular {pointer 1}  }
	{ p_0_0_01017193_out int 26 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "ref_left_wall_bound", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_right_wall_bound", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_0_0_010170171_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171169_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170167_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171165_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170163_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171161_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170159_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171157_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170155_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171153_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170151_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171149_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170147_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171145_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170143_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171141_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170139_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171137_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170135_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171133_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170131_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171129_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170127_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171125_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170123_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171121_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170119_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171117_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170115_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171113_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170111_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171109_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170107_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171105_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010170103_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010171101_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01017099_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01017197_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01017095_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01017193_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 92
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ ref_left_wall_bound_address0 sc_out sc_lv 5 signal 0 } 
	{ ref_left_wall_bound_ce0 sc_out sc_logic 1 signal 0 } 
	{ ref_left_wall_bound_q0 sc_in sc_lv 26 signal 0 } 
	{ ref_right_wall_bound_address0 sc_out sc_lv 5 signal 1 } 
	{ ref_right_wall_bound_ce0 sc_out sc_logic 1 signal 1 } 
	{ ref_right_wall_bound_q0 sc_in sc_lv 26 signal 1 } 
	{ p_0_0_010170171_out sc_out sc_lv 26 signal 2 } 
	{ p_0_0_010170171_out_ap_vld sc_out sc_logic 1 outvld 2 } 
	{ p_0_0_010171169_out sc_out sc_lv 26 signal 3 } 
	{ p_0_0_010171169_out_ap_vld sc_out sc_logic 1 outvld 3 } 
	{ p_0_0_010170167_out sc_out sc_lv 26 signal 4 } 
	{ p_0_0_010170167_out_ap_vld sc_out sc_logic 1 outvld 4 } 
	{ p_0_0_010171165_out sc_out sc_lv 26 signal 5 } 
	{ p_0_0_010171165_out_ap_vld sc_out sc_logic 1 outvld 5 } 
	{ p_0_0_010170163_out sc_out sc_lv 26 signal 6 } 
	{ p_0_0_010170163_out_ap_vld sc_out sc_logic 1 outvld 6 } 
	{ p_0_0_010171161_out sc_out sc_lv 26 signal 7 } 
	{ p_0_0_010171161_out_ap_vld sc_out sc_logic 1 outvld 7 } 
	{ p_0_0_010170159_out sc_out sc_lv 26 signal 8 } 
	{ p_0_0_010170159_out_ap_vld sc_out sc_logic 1 outvld 8 } 
	{ p_0_0_010171157_out sc_out sc_lv 26 signal 9 } 
	{ p_0_0_010171157_out_ap_vld sc_out sc_logic 1 outvld 9 } 
	{ p_0_0_010170155_out sc_out sc_lv 26 signal 10 } 
	{ p_0_0_010170155_out_ap_vld sc_out sc_logic 1 outvld 10 } 
	{ p_0_0_010171153_out sc_out sc_lv 26 signal 11 } 
	{ p_0_0_010171153_out_ap_vld sc_out sc_logic 1 outvld 11 } 
	{ p_0_0_010170151_out sc_out sc_lv 26 signal 12 } 
	{ p_0_0_010170151_out_ap_vld sc_out sc_logic 1 outvld 12 } 
	{ p_0_0_010171149_out sc_out sc_lv 26 signal 13 } 
	{ p_0_0_010171149_out_ap_vld sc_out sc_logic 1 outvld 13 } 
	{ p_0_0_010170147_out sc_out sc_lv 26 signal 14 } 
	{ p_0_0_010170147_out_ap_vld sc_out sc_logic 1 outvld 14 } 
	{ p_0_0_010171145_out sc_out sc_lv 26 signal 15 } 
	{ p_0_0_010171145_out_ap_vld sc_out sc_logic 1 outvld 15 } 
	{ p_0_0_010170143_out sc_out sc_lv 26 signal 16 } 
	{ p_0_0_010170143_out_ap_vld sc_out sc_logic 1 outvld 16 } 
	{ p_0_0_010171141_out sc_out sc_lv 26 signal 17 } 
	{ p_0_0_010171141_out_ap_vld sc_out sc_logic 1 outvld 17 } 
	{ p_0_0_010170139_out sc_out sc_lv 26 signal 18 } 
	{ p_0_0_010170139_out_ap_vld sc_out sc_logic 1 outvld 18 } 
	{ p_0_0_010171137_out sc_out sc_lv 26 signal 19 } 
	{ p_0_0_010171137_out_ap_vld sc_out sc_logic 1 outvld 19 } 
	{ p_0_0_010170135_out sc_out sc_lv 26 signal 20 } 
	{ p_0_0_010170135_out_ap_vld sc_out sc_logic 1 outvld 20 } 
	{ p_0_0_010171133_out sc_out sc_lv 26 signal 21 } 
	{ p_0_0_010171133_out_ap_vld sc_out sc_logic 1 outvld 21 } 
	{ p_0_0_010170131_out sc_out sc_lv 26 signal 22 } 
	{ p_0_0_010170131_out_ap_vld sc_out sc_logic 1 outvld 22 } 
	{ p_0_0_010171129_out sc_out sc_lv 26 signal 23 } 
	{ p_0_0_010171129_out_ap_vld sc_out sc_logic 1 outvld 23 } 
	{ p_0_0_010170127_out sc_out sc_lv 26 signal 24 } 
	{ p_0_0_010170127_out_ap_vld sc_out sc_logic 1 outvld 24 } 
	{ p_0_0_010171125_out sc_out sc_lv 26 signal 25 } 
	{ p_0_0_010171125_out_ap_vld sc_out sc_logic 1 outvld 25 } 
	{ p_0_0_010170123_out sc_out sc_lv 26 signal 26 } 
	{ p_0_0_010170123_out_ap_vld sc_out sc_logic 1 outvld 26 } 
	{ p_0_0_010171121_out sc_out sc_lv 26 signal 27 } 
	{ p_0_0_010171121_out_ap_vld sc_out sc_logic 1 outvld 27 } 
	{ p_0_0_010170119_out sc_out sc_lv 26 signal 28 } 
	{ p_0_0_010170119_out_ap_vld sc_out sc_logic 1 outvld 28 } 
	{ p_0_0_010171117_out sc_out sc_lv 26 signal 29 } 
	{ p_0_0_010171117_out_ap_vld sc_out sc_logic 1 outvld 29 } 
	{ p_0_0_010170115_out sc_out sc_lv 26 signal 30 } 
	{ p_0_0_010170115_out_ap_vld sc_out sc_logic 1 outvld 30 } 
	{ p_0_0_010171113_out sc_out sc_lv 26 signal 31 } 
	{ p_0_0_010171113_out_ap_vld sc_out sc_logic 1 outvld 31 } 
	{ p_0_0_010170111_out sc_out sc_lv 26 signal 32 } 
	{ p_0_0_010170111_out_ap_vld sc_out sc_logic 1 outvld 32 } 
	{ p_0_0_010171109_out sc_out sc_lv 26 signal 33 } 
	{ p_0_0_010171109_out_ap_vld sc_out sc_logic 1 outvld 33 } 
	{ p_0_0_010170107_out sc_out sc_lv 26 signal 34 } 
	{ p_0_0_010170107_out_ap_vld sc_out sc_logic 1 outvld 34 } 
	{ p_0_0_010171105_out sc_out sc_lv 26 signal 35 } 
	{ p_0_0_010171105_out_ap_vld sc_out sc_logic 1 outvld 35 } 
	{ p_0_0_010170103_out sc_out sc_lv 26 signal 36 } 
	{ p_0_0_010170103_out_ap_vld sc_out sc_logic 1 outvld 36 } 
	{ p_0_0_010171101_out sc_out sc_lv 26 signal 37 } 
	{ p_0_0_010171101_out_ap_vld sc_out sc_logic 1 outvld 37 } 
	{ p_0_0_01017099_out sc_out sc_lv 26 signal 38 } 
	{ p_0_0_01017099_out_ap_vld sc_out sc_logic 1 outvld 38 } 
	{ p_0_0_01017197_out sc_out sc_lv 26 signal 39 } 
	{ p_0_0_01017197_out_ap_vld sc_out sc_logic 1 outvld 39 } 
	{ p_0_0_01017095_out sc_out sc_lv 26 signal 40 } 
	{ p_0_0_01017095_out_ap_vld sc_out sc_logic 1 outvld 40 } 
	{ p_0_0_01017193_out sc_out sc_lv 26 signal 41 } 
	{ p_0_0_01017193_out_ap_vld sc_out sc_logic 1 outvld 41 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "ref_left_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "address0" }} , 
 	{ "name": "ref_left_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "ce0" }} , 
 	{ "name": "ref_left_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "q0" }} , 
 	{ "name": "ref_right_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "address0" }} , 
 	{ "name": "ref_right_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "ce0" }} , 
 	{ "name": "ref_right_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "q0" }} , 
 	{ "name": "p_0_0_010170171_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170171_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170171_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170171_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171169_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171169_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171169_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171169_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170167_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170167_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170167_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170167_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171165_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171165_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171165_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171165_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170163_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170163_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170163_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170163_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171161_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171161_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171161_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171161_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170159_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170159_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170159_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170159_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171157_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171157_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171157_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171157_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170155_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170155_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170155_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170155_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171153_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171153_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171153_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171153_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170151_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170151_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170151_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170151_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171149_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171149_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171149_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171149_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170147_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170147_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170147_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170147_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171145_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171145_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171145_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171145_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170143_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170143_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170143_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170143_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171141_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171141_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171141_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171141_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170139_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170139_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170139_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170139_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171137_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171137_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171137_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171137_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170135_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170135_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170135_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170135_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171133_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171133_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171133_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171133_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170131_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170131_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170131_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170131_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171129_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171129_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171129_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171129_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170127_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170127_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170127_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170127_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171125_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171125_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171125_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171125_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170123_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170123_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170123_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170123_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171121_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171121_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171121_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171121_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170119_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170119_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170119_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170119_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171117_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171117_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171117_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171117_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170115_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170115_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170115_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170115_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171113_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171113_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171113_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171113_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170111_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170111_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170111_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170111_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171109_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171109_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171109_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171109_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170107_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170107_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170107_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170107_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171105_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171105_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171105_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171105_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010170103_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010170103_out", "role": "default" }} , 
 	{ "name": "p_0_0_010170103_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010170103_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010171101_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_010171101_out", "role": "default" }} , 
 	{ "name": "p_0_0_010171101_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010171101_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01017099_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_01017099_out", "role": "default" }} , 
 	{ "name": "p_0_0_01017099_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01017099_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01017197_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_01017197_out", "role": "default" }} , 
 	{ "name": "p_0_0_01017197_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01017197_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01017095_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_01017095_out", "role": "default" }} , 
 	{ "name": "p_0_0_01017095_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01017095_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01017193_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_01017193_out", "role": "default" }} , 
 	{ "name": "p_0_0_01017193_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01017193_out", "role": "ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		p_0_0_01017193_out {Type O LastRead -1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "22", "Max" : "22"}
	, {"Name" : "Interval", "Min" : "21", "Max" : "21"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	ref_left_wall_bound { ap_memory {  { ref_left_wall_bound_address0 mem_address 1 5 }  { ref_left_wall_bound_ce0 mem_ce 1 1 }  { ref_left_wall_bound_q0 mem_dout 0 26 } } }
	ref_right_wall_bound { ap_memory {  { ref_right_wall_bound_address0 mem_address 1 5 }  { ref_right_wall_bound_ce0 mem_ce 1 1 }  { ref_right_wall_bound_q0 mem_dout 0 26 } } }
	p_0_0_010170171_out { ap_vld {  { p_0_0_010170171_out out_data 1 26 }  { p_0_0_010170171_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171169_out { ap_vld {  { p_0_0_010171169_out out_data 1 26 }  { p_0_0_010171169_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170167_out { ap_vld {  { p_0_0_010170167_out out_data 1 26 }  { p_0_0_010170167_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171165_out { ap_vld {  { p_0_0_010171165_out out_data 1 26 }  { p_0_0_010171165_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170163_out { ap_vld {  { p_0_0_010170163_out out_data 1 26 }  { p_0_0_010170163_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171161_out { ap_vld {  { p_0_0_010171161_out out_data 1 26 }  { p_0_0_010171161_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170159_out { ap_vld {  { p_0_0_010170159_out out_data 1 26 }  { p_0_0_010170159_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171157_out { ap_vld {  { p_0_0_010171157_out out_data 1 26 }  { p_0_0_010171157_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170155_out { ap_vld {  { p_0_0_010170155_out out_data 1 26 }  { p_0_0_010170155_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171153_out { ap_vld {  { p_0_0_010171153_out out_data 1 26 }  { p_0_0_010171153_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170151_out { ap_vld {  { p_0_0_010170151_out out_data 1 26 }  { p_0_0_010170151_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171149_out { ap_vld {  { p_0_0_010171149_out out_data 1 26 }  { p_0_0_010171149_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170147_out { ap_vld {  { p_0_0_010170147_out out_data 1 26 }  { p_0_0_010170147_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171145_out { ap_vld {  { p_0_0_010171145_out out_data 1 26 }  { p_0_0_010171145_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170143_out { ap_vld {  { p_0_0_010170143_out out_data 1 26 }  { p_0_0_010170143_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171141_out { ap_vld {  { p_0_0_010171141_out out_data 1 26 }  { p_0_0_010171141_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170139_out { ap_vld {  { p_0_0_010170139_out out_data 1 26 }  { p_0_0_010170139_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171137_out { ap_vld {  { p_0_0_010171137_out out_data 1 26 }  { p_0_0_010171137_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170135_out { ap_vld {  { p_0_0_010170135_out out_data 1 26 }  { p_0_0_010170135_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171133_out { ap_vld {  { p_0_0_010171133_out out_data 1 26 }  { p_0_0_010171133_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170131_out { ap_vld {  { p_0_0_010170131_out out_data 1 26 }  { p_0_0_010170131_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171129_out { ap_vld {  { p_0_0_010171129_out out_data 1 26 }  { p_0_0_010171129_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170127_out { ap_vld {  { p_0_0_010170127_out out_data 1 26 }  { p_0_0_010170127_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171125_out { ap_vld {  { p_0_0_010171125_out out_data 1 26 }  { p_0_0_010171125_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170123_out { ap_vld {  { p_0_0_010170123_out out_data 1 26 }  { p_0_0_010170123_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171121_out { ap_vld {  { p_0_0_010171121_out out_data 1 26 }  { p_0_0_010171121_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170119_out { ap_vld {  { p_0_0_010170119_out out_data 1 26 }  { p_0_0_010170119_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171117_out { ap_vld {  { p_0_0_010171117_out out_data 1 26 }  { p_0_0_010171117_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170115_out { ap_vld {  { p_0_0_010170115_out out_data 1 26 }  { p_0_0_010170115_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171113_out { ap_vld {  { p_0_0_010171113_out out_data 1 26 }  { p_0_0_010171113_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170111_out { ap_vld {  { p_0_0_010170111_out out_data 1 26 }  { p_0_0_010170111_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171109_out { ap_vld {  { p_0_0_010171109_out out_data 1 26 }  { p_0_0_010171109_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170107_out { ap_vld {  { p_0_0_010170107_out out_data 1 26 }  { p_0_0_010170107_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171105_out { ap_vld {  { p_0_0_010171105_out out_data 1 26 }  { p_0_0_010171105_out_ap_vld out_vld 1 1 } } }
	p_0_0_010170103_out { ap_vld {  { p_0_0_010170103_out out_data 1 26 }  { p_0_0_010170103_out_ap_vld out_vld 1 1 } } }
	p_0_0_010171101_out { ap_vld {  { p_0_0_010171101_out out_data 1 26 }  { p_0_0_010171101_out_ap_vld out_vld 1 1 } } }
	p_0_0_01017099_out { ap_vld {  { p_0_0_01017099_out out_data 1 26 }  { p_0_0_01017099_out_ap_vld out_vld 1 1 } } }
	p_0_0_01017197_out { ap_vld {  { p_0_0_01017197_out out_data 1 26 }  { p_0_0_01017197_out_ap_vld out_vld 1 1 } } }
	p_0_0_01017095_out { ap_vld {  { p_0_0_01017095_out out_data 1 26 }  { p_0_0_01017095_out_ap_vld out_vld 1 1 } } }
	p_0_0_01017193_out { ap_vld {  { p_0_0_01017193_out out_data 1 26 }  { p_0_0_01017193_out_ap_vld out_vld 1 1 } } }
}
