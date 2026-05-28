set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_356_1
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
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_356_1}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict ref_left_wall_bound { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict ref_right_wall_bound { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ ref_left_wall_bound int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ ref_right_wall_bound int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ p_0_0_010144171_out int 32 regular {pointer 1}  }
	{ p_0_0_010145169_out int 32 regular {pointer 1}  }
	{ p_0_0_010144167_out int 32 regular {pointer 1}  }
	{ p_0_0_010145165_out int 32 regular {pointer 1}  }
	{ p_0_0_010144163_out int 32 regular {pointer 1}  }
	{ p_0_0_010145161_out int 32 regular {pointer 1}  }
	{ p_0_0_010144159_out int 32 regular {pointer 1}  }
	{ p_0_0_010145157_out int 32 regular {pointer 1}  }
	{ p_0_0_010144155_out int 32 regular {pointer 1}  }
	{ p_0_0_010145153_out int 32 regular {pointer 1}  }
	{ p_0_0_010144151_out int 32 regular {pointer 1}  }
	{ p_0_0_010145149_out int 32 regular {pointer 1}  }
	{ p_0_0_010144147_out int 32 regular {pointer 1}  }
	{ p_0_0_010145145_out int 32 regular {pointer 1}  }
	{ p_0_0_010144143_out int 32 regular {pointer 1}  }
	{ p_0_0_010145141_out int 32 regular {pointer 1}  }
	{ p_0_0_010144139_out int 32 regular {pointer 1}  }
	{ p_0_0_010145137_out int 32 regular {pointer 1}  }
	{ p_0_0_010144135_out int 32 regular {pointer 1}  }
	{ p_0_0_010145133_out int 32 regular {pointer 1}  }
	{ p_0_0_010144131_out int 32 regular {pointer 1}  }
	{ p_0_0_010145129_out int 32 regular {pointer 1}  }
	{ p_0_0_010144127_out int 32 regular {pointer 1}  }
	{ p_0_0_010145125_out int 32 regular {pointer 1}  }
	{ p_0_0_010144123_out int 32 regular {pointer 1}  }
	{ p_0_0_010145121_out int 32 regular {pointer 1}  }
	{ p_0_0_010144119_out int 32 regular {pointer 1}  }
	{ p_0_0_010145117_out int 32 regular {pointer 1}  }
	{ p_0_0_010144115_out int 32 regular {pointer 1}  }
	{ p_0_0_010145113_out int 32 regular {pointer 1}  }
	{ p_0_0_010144111_out int 32 regular {pointer 1}  }
	{ p_0_0_010145109_out int 32 regular {pointer 1}  }
	{ p_0_0_010144107_out int 32 regular {pointer 1}  }
	{ p_0_0_010145105_out int 32 regular {pointer 1}  }
	{ p_0_0_010144103_out int 32 regular {pointer 1}  }
	{ p_0_0_010145101_out int 32 regular {pointer 1}  }
	{ p_0_0_01014499_out int 32 regular {pointer 1}  }
	{ p_0_0_01014597_out int 32 regular {pointer 1}  }
	{ p_0_0_01014495_out int 32 regular {pointer 1}  }
	{ p_0_0_01014593_out int 32 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "ref_left_wall_bound", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ref_right_wall_bound", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_0_0_010144171_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145169_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144167_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145165_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144163_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145161_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144159_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145157_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144155_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145153_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144151_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145149_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144147_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145145_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144143_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145141_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144139_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145137_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144135_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145133_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144131_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145129_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144127_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145125_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144123_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145121_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144119_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145117_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144115_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145113_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144111_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145109_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144107_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145105_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010144103_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_010145101_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01014499_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01014597_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01014495_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_0_0_01014593_out", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY"} ]}
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
	{ ref_left_wall_bound_q0 sc_in sc_lv 32 signal 0 } 
	{ ref_right_wall_bound_address0 sc_out sc_lv 5 signal 1 } 
	{ ref_right_wall_bound_ce0 sc_out sc_logic 1 signal 1 } 
	{ ref_right_wall_bound_q0 sc_in sc_lv 32 signal 1 } 
	{ p_0_0_010144171_out sc_out sc_lv 32 signal 2 } 
	{ p_0_0_010144171_out_ap_vld sc_out sc_logic 1 outvld 2 } 
	{ p_0_0_010145169_out sc_out sc_lv 32 signal 3 } 
	{ p_0_0_010145169_out_ap_vld sc_out sc_logic 1 outvld 3 } 
	{ p_0_0_010144167_out sc_out sc_lv 32 signal 4 } 
	{ p_0_0_010144167_out_ap_vld sc_out sc_logic 1 outvld 4 } 
	{ p_0_0_010145165_out sc_out sc_lv 32 signal 5 } 
	{ p_0_0_010145165_out_ap_vld sc_out sc_logic 1 outvld 5 } 
	{ p_0_0_010144163_out sc_out sc_lv 32 signal 6 } 
	{ p_0_0_010144163_out_ap_vld sc_out sc_logic 1 outvld 6 } 
	{ p_0_0_010145161_out sc_out sc_lv 32 signal 7 } 
	{ p_0_0_010145161_out_ap_vld sc_out sc_logic 1 outvld 7 } 
	{ p_0_0_010144159_out sc_out sc_lv 32 signal 8 } 
	{ p_0_0_010144159_out_ap_vld sc_out sc_logic 1 outvld 8 } 
	{ p_0_0_010145157_out sc_out sc_lv 32 signal 9 } 
	{ p_0_0_010145157_out_ap_vld sc_out sc_logic 1 outvld 9 } 
	{ p_0_0_010144155_out sc_out sc_lv 32 signal 10 } 
	{ p_0_0_010144155_out_ap_vld sc_out sc_logic 1 outvld 10 } 
	{ p_0_0_010145153_out sc_out sc_lv 32 signal 11 } 
	{ p_0_0_010145153_out_ap_vld sc_out sc_logic 1 outvld 11 } 
	{ p_0_0_010144151_out sc_out sc_lv 32 signal 12 } 
	{ p_0_0_010144151_out_ap_vld sc_out sc_logic 1 outvld 12 } 
	{ p_0_0_010145149_out sc_out sc_lv 32 signal 13 } 
	{ p_0_0_010145149_out_ap_vld sc_out sc_logic 1 outvld 13 } 
	{ p_0_0_010144147_out sc_out sc_lv 32 signal 14 } 
	{ p_0_0_010144147_out_ap_vld sc_out sc_logic 1 outvld 14 } 
	{ p_0_0_010145145_out sc_out sc_lv 32 signal 15 } 
	{ p_0_0_010145145_out_ap_vld sc_out sc_logic 1 outvld 15 } 
	{ p_0_0_010144143_out sc_out sc_lv 32 signal 16 } 
	{ p_0_0_010144143_out_ap_vld sc_out sc_logic 1 outvld 16 } 
	{ p_0_0_010145141_out sc_out sc_lv 32 signal 17 } 
	{ p_0_0_010145141_out_ap_vld sc_out sc_logic 1 outvld 17 } 
	{ p_0_0_010144139_out sc_out sc_lv 32 signal 18 } 
	{ p_0_0_010144139_out_ap_vld sc_out sc_logic 1 outvld 18 } 
	{ p_0_0_010145137_out sc_out sc_lv 32 signal 19 } 
	{ p_0_0_010145137_out_ap_vld sc_out sc_logic 1 outvld 19 } 
	{ p_0_0_010144135_out sc_out sc_lv 32 signal 20 } 
	{ p_0_0_010144135_out_ap_vld sc_out sc_logic 1 outvld 20 } 
	{ p_0_0_010145133_out sc_out sc_lv 32 signal 21 } 
	{ p_0_0_010145133_out_ap_vld sc_out sc_logic 1 outvld 21 } 
	{ p_0_0_010144131_out sc_out sc_lv 32 signal 22 } 
	{ p_0_0_010144131_out_ap_vld sc_out sc_logic 1 outvld 22 } 
	{ p_0_0_010145129_out sc_out sc_lv 32 signal 23 } 
	{ p_0_0_010145129_out_ap_vld sc_out sc_logic 1 outvld 23 } 
	{ p_0_0_010144127_out sc_out sc_lv 32 signal 24 } 
	{ p_0_0_010144127_out_ap_vld sc_out sc_logic 1 outvld 24 } 
	{ p_0_0_010145125_out sc_out sc_lv 32 signal 25 } 
	{ p_0_0_010145125_out_ap_vld sc_out sc_logic 1 outvld 25 } 
	{ p_0_0_010144123_out sc_out sc_lv 32 signal 26 } 
	{ p_0_0_010144123_out_ap_vld sc_out sc_logic 1 outvld 26 } 
	{ p_0_0_010145121_out sc_out sc_lv 32 signal 27 } 
	{ p_0_0_010145121_out_ap_vld sc_out sc_logic 1 outvld 27 } 
	{ p_0_0_010144119_out sc_out sc_lv 32 signal 28 } 
	{ p_0_0_010144119_out_ap_vld sc_out sc_logic 1 outvld 28 } 
	{ p_0_0_010145117_out sc_out sc_lv 32 signal 29 } 
	{ p_0_0_010145117_out_ap_vld sc_out sc_logic 1 outvld 29 } 
	{ p_0_0_010144115_out sc_out sc_lv 32 signal 30 } 
	{ p_0_0_010144115_out_ap_vld sc_out sc_logic 1 outvld 30 } 
	{ p_0_0_010145113_out sc_out sc_lv 32 signal 31 } 
	{ p_0_0_010145113_out_ap_vld sc_out sc_logic 1 outvld 31 } 
	{ p_0_0_010144111_out sc_out sc_lv 32 signal 32 } 
	{ p_0_0_010144111_out_ap_vld sc_out sc_logic 1 outvld 32 } 
	{ p_0_0_010145109_out sc_out sc_lv 32 signal 33 } 
	{ p_0_0_010145109_out_ap_vld sc_out sc_logic 1 outvld 33 } 
	{ p_0_0_010144107_out sc_out sc_lv 32 signal 34 } 
	{ p_0_0_010144107_out_ap_vld sc_out sc_logic 1 outvld 34 } 
	{ p_0_0_010145105_out sc_out sc_lv 32 signal 35 } 
	{ p_0_0_010145105_out_ap_vld sc_out sc_logic 1 outvld 35 } 
	{ p_0_0_010144103_out sc_out sc_lv 32 signal 36 } 
	{ p_0_0_010144103_out_ap_vld sc_out sc_logic 1 outvld 36 } 
	{ p_0_0_010145101_out sc_out sc_lv 32 signal 37 } 
	{ p_0_0_010145101_out_ap_vld sc_out sc_logic 1 outvld 37 } 
	{ p_0_0_01014499_out sc_out sc_lv 32 signal 38 } 
	{ p_0_0_01014499_out_ap_vld sc_out sc_logic 1 outvld 38 } 
	{ p_0_0_01014597_out sc_out sc_lv 32 signal 39 } 
	{ p_0_0_01014597_out_ap_vld sc_out sc_logic 1 outvld 39 } 
	{ p_0_0_01014495_out sc_out sc_lv 32 signal 40 } 
	{ p_0_0_01014495_out_ap_vld sc_out sc_logic 1 outvld 40 } 
	{ p_0_0_01014593_out sc_out sc_lv 32 signal 41 } 
	{ p_0_0_01014593_out_ap_vld sc_out sc_logic 1 outvld 41 } 
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
 	{ "name": "ref_left_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_left_wall_bound", "role": "q0" }} , 
 	{ "name": "ref_right_wall_bound_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "address0" }} , 
 	{ "name": "ref_right_wall_bound_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "ce0" }} , 
 	{ "name": "ref_right_wall_bound_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ref_right_wall_bound", "role": "q0" }} , 
 	{ "name": "p_0_0_010144171_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144171_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144171_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144171_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145169_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145169_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145169_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145169_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144167_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144167_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144167_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144167_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145165_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145165_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145165_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145165_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144163_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144163_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144163_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144163_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145161_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145161_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145161_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145161_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144159_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144159_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144159_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144159_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145157_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145157_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145157_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145157_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144155_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144155_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144155_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144155_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145153_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145153_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145153_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145153_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144151_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144151_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144151_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144151_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145149_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145149_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145149_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145149_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144147_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144147_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144147_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144147_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145145_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145145_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145145_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145145_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144143_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144143_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144143_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144143_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145141_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145141_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145141_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145141_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144139_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144139_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144139_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144139_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145137_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145137_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145137_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145137_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144135_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144135_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144135_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144135_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145133_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145133_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145133_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145133_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144131_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144131_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144131_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144131_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145129_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145129_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145129_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145129_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144127_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144127_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144127_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144127_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145125_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145125_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145125_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145125_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144123_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144123_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144123_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144123_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145121_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145121_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145121_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145121_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144119_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144119_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144119_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144119_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145117_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145117_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145117_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145117_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144115_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144115_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144115_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144115_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145113_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145113_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145113_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145113_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144111_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144111_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144111_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144111_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145109_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145109_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145109_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145109_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144107_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144107_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144107_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144107_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145105_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145105_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145105_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145105_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010144103_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010144103_out", "role": "default" }} , 
 	{ "name": "p_0_0_010144103_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010144103_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_010145101_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_010145101_out", "role": "default" }} , 
 	{ "name": "p_0_0_010145101_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_010145101_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01014499_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_01014499_out", "role": "default" }} , 
 	{ "name": "p_0_0_01014499_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01014499_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01014597_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_01014597_out", "role": "default" }} , 
 	{ "name": "p_0_0_01014597_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01014597_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01014495_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_01014495_out", "role": "default" }} , 
 	{ "name": "p_0_0_01014495_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01014495_out", "role": "ap_vld" }} , 
 	{ "name": "p_0_0_01014593_out", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_0_0_01014593_out", "role": "default" }} , 
 	{ "name": "p_0_0_01014593_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_0_0_01014593_out", "role": "ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		p_0_0_01014593_out {Type O LastRead -1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "22", "Max" : "22"}
	, {"Name" : "Interval", "Min" : "21", "Max" : "21"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	ref_left_wall_bound { ap_memory {  { ref_left_wall_bound_address0 mem_address 1 5 }  { ref_left_wall_bound_ce0 mem_ce 1 1 }  { ref_left_wall_bound_q0 mem_dout 0 32 } } }
	ref_right_wall_bound { ap_memory {  { ref_right_wall_bound_address0 mem_address 1 5 }  { ref_right_wall_bound_ce0 mem_ce 1 1 }  { ref_right_wall_bound_q0 mem_dout 0 32 } } }
	p_0_0_010144171_out { ap_vld {  { p_0_0_010144171_out out_data 1 32 }  { p_0_0_010144171_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145169_out { ap_vld {  { p_0_0_010145169_out out_data 1 32 }  { p_0_0_010145169_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144167_out { ap_vld {  { p_0_0_010144167_out out_data 1 32 }  { p_0_0_010144167_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145165_out { ap_vld {  { p_0_0_010145165_out out_data 1 32 }  { p_0_0_010145165_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144163_out { ap_vld {  { p_0_0_010144163_out out_data 1 32 }  { p_0_0_010144163_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145161_out { ap_vld {  { p_0_0_010145161_out out_data 1 32 }  { p_0_0_010145161_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144159_out { ap_vld {  { p_0_0_010144159_out out_data 1 32 }  { p_0_0_010144159_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145157_out { ap_vld {  { p_0_0_010145157_out out_data 1 32 }  { p_0_0_010145157_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144155_out { ap_vld {  { p_0_0_010144155_out out_data 1 32 }  { p_0_0_010144155_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145153_out { ap_vld {  { p_0_0_010145153_out out_data 1 32 }  { p_0_0_010145153_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144151_out { ap_vld {  { p_0_0_010144151_out out_data 1 32 }  { p_0_0_010144151_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145149_out { ap_vld {  { p_0_0_010145149_out out_data 1 32 }  { p_0_0_010145149_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144147_out { ap_vld {  { p_0_0_010144147_out out_data 1 32 }  { p_0_0_010144147_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145145_out { ap_vld {  { p_0_0_010145145_out out_data 1 32 }  { p_0_0_010145145_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144143_out { ap_vld {  { p_0_0_010144143_out out_data 1 32 }  { p_0_0_010144143_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145141_out { ap_vld {  { p_0_0_010145141_out out_data 1 32 }  { p_0_0_010145141_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144139_out { ap_vld {  { p_0_0_010144139_out out_data 1 32 }  { p_0_0_010144139_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145137_out { ap_vld {  { p_0_0_010145137_out out_data 1 32 }  { p_0_0_010145137_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144135_out { ap_vld {  { p_0_0_010144135_out out_data 1 32 }  { p_0_0_010144135_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145133_out { ap_vld {  { p_0_0_010145133_out out_data 1 32 }  { p_0_0_010145133_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144131_out { ap_vld {  { p_0_0_010144131_out out_data 1 32 }  { p_0_0_010144131_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145129_out { ap_vld {  { p_0_0_010145129_out out_data 1 32 }  { p_0_0_010145129_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144127_out { ap_vld {  { p_0_0_010144127_out out_data 1 32 }  { p_0_0_010144127_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145125_out { ap_vld {  { p_0_0_010145125_out out_data 1 32 }  { p_0_0_010145125_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144123_out { ap_vld {  { p_0_0_010144123_out out_data 1 32 }  { p_0_0_010144123_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145121_out { ap_vld {  { p_0_0_010145121_out out_data 1 32 }  { p_0_0_010145121_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144119_out { ap_vld {  { p_0_0_010144119_out out_data 1 32 }  { p_0_0_010144119_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145117_out { ap_vld {  { p_0_0_010145117_out out_data 1 32 }  { p_0_0_010145117_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144115_out { ap_vld {  { p_0_0_010144115_out out_data 1 32 }  { p_0_0_010144115_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145113_out { ap_vld {  { p_0_0_010145113_out out_data 1 32 }  { p_0_0_010145113_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144111_out { ap_vld {  { p_0_0_010144111_out out_data 1 32 }  { p_0_0_010144111_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145109_out { ap_vld {  { p_0_0_010145109_out out_data 1 32 }  { p_0_0_010145109_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144107_out { ap_vld {  { p_0_0_010144107_out out_data 1 32 }  { p_0_0_010144107_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145105_out { ap_vld {  { p_0_0_010145105_out out_data 1 32 }  { p_0_0_010145105_out_ap_vld out_vld 1 1 } } }
	p_0_0_010144103_out { ap_vld {  { p_0_0_010144103_out out_data 1 32 }  { p_0_0_010144103_out_ap_vld out_vld 1 1 } } }
	p_0_0_010145101_out { ap_vld {  { p_0_0_010145101_out out_data 1 32 }  { p_0_0_010145101_out_ap_vld out_vld 1 1 } } }
	p_0_0_01014499_out { ap_vld {  { p_0_0_01014499_out out_data 1 32 }  { p_0_0_01014499_out_ap_vld out_vld 1 1 } } }
	p_0_0_01014597_out { ap_vld {  { p_0_0_01014597_out out_data 1 32 }  { p_0_0_01014597_out_ap_vld out_vld 1 1 } } }
	p_0_0_01014495_out { ap_vld {  { p_0_0_01014495_out out_data 1 32 }  { p_0_0_01014495_out_ap_vld out_vld 1 1 } } }
	p_0_0_01014593_out { ap_vld {  { p_0_0_01014593_out out_data 1 32 }  { p_0_0_01014593_out_ap_vld out_vld 1 1 } } }
}
