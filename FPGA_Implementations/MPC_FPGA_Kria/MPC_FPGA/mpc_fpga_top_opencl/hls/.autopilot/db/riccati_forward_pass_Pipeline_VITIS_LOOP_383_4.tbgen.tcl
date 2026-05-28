set moduleName riccati_forward_pass_Pipeline_VITIS_LOOP_383_4
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
set C_modelName {riccati_forward_pass_Pipeline_VITIS_LOOP_383_4}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict x_out_0 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_1 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_2 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_3 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_4 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ x_out_0 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ zext_ln409 int 5 regular  }
	{ x_out_1 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_2 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_3 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_4 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_5 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ empty_519 int 27 regular  }
	{ empty_520 int 27 regular  }
	{ empty_521 int 27 regular  }
	{ empty_522 int 27 regular  }
	{ empty_523 int 27 regular  }
	{ empty int 27 regular  }
	{ step_data_0_0_0_load int 32 regular  }
	{ step_data_0_1_0_load int 32 regular  }
	{ step_data_0_2_0_load int 32 regular  }
	{ step_data_0_3_0_load int 32 regular  }
	{ step_data_0_4_0_load int 32 regular  }
	{ step_data_0_5_0_load int 32 regular  }
	{ sext_ln159 int 32 regular  }
	{ step_data_0_0_1_load int 32 regular  }
	{ step_data_0_1_1_load int 32 regular  }
	{ step_data_0_2_1_load int 32 regular  }
	{ step_data_0_3_1_load int 32 regular  }
	{ step_data_0_4_1_load int 32 regular  }
	{ step_data_0_5_1_load int 32 regular  }
	{ sext_ln159_1 int 32 regular  }
	{ step_data_0_0_2_load int 32 regular  }
	{ step_data_0_1_2_load int 32 regular  }
	{ step_data_0_2_2_load int 32 regular  }
	{ step_data_0_3_2_load int 32 regular  }
	{ step_data_0_4_2_load int 32 regular  }
	{ step_data_0_5_2_load int 32 regular  }
	{ sext_ln159_2 int 32 regular  }
	{ step_data_0_0_3_load int 32 regular  }
	{ step_data_0_1_3_load int 32 regular  }
	{ step_data_0_2_3_load int 32 regular  }
	{ step_data_0_3_3_load int 32 regular  }
	{ step_data_0_4_3_load int 32 regular  }
	{ step_data_0_5_3_load int 32 regular  }
	{ sext_ln159_3 int 32 regular  }
	{ step_data_0_0_4_load int 32 regular  }
	{ step_data_0_1_4_load int 32 regular  }
	{ step_data_0_2_4_load int 32 regular  }
	{ step_data_0_3_4_load int 32 regular  }
	{ step_data_0_4_4_load int 32 regular  }
	{ step_data_0_5_4_load int 32 regular  }
	{ sext_ln159_4 int 32 regular  }
	{ step_data_0_0_5_load int 32 regular  }
	{ step_data_0_1_5_load int 32 regular  }
	{ step_data_0_2_5_load int 32 regular  }
	{ step_data_0_3_5_load int 32 regular  }
	{ step_data_0_4_5_load int 32 regular  }
	{ step_data_0_5_5_load int 32 regular  }
	{ sext_ln406 int 32 regular  }
	{ sext_ln159_11 int 32 regular  }
	{ sext_ln420 int 27 regular  }
	{ sext_ln159_12 int 32 regular  }
	{ sext_ln159_13 int 32 regular  }
	{ sext_ln159_14 int 32 regular  }
	{ sext_ln420_1 int 27 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "x_out_0", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "zext_ln409", "interface" : "wire", "bitwidth" : 5, "direction" : "READONLY"} , 
 	{ "Name" : "x_out_1", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_2", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_3", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_4", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_5", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "empty_519", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "empty_520", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "empty_521", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "empty_522", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "empty_523", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "empty", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_1", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_2", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_3", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_4", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln406", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_11", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln420", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_12", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_13", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln159_14", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln420_1", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} ]}
# RTL Port declarations: 
set portNum 85
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ x_out_0_address1 sc_out sc_lv 5 signal 0 } 
	{ x_out_0_ce1 sc_out sc_logic 1 signal 0 } 
	{ x_out_0_we1 sc_out sc_logic 1 signal 0 } 
	{ x_out_0_d1 sc_out sc_lv 32 signal 0 } 
	{ zext_ln409 sc_in sc_lv 5 signal 1 } 
	{ x_out_1_address1 sc_out sc_lv 5 signal 2 } 
	{ x_out_1_ce1 sc_out sc_logic 1 signal 2 } 
	{ x_out_1_we1 sc_out sc_logic 1 signal 2 } 
	{ x_out_1_d1 sc_out sc_lv 32 signal 2 } 
	{ x_out_2_address1 sc_out sc_lv 5 signal 3 } 
	{ x_out_2_ce1 sc_out sc_logic 1 signal 3 } 
	{ x_out_2_we1 sc_out sc_logic 1 signal 3 } 
	{ x_out_2_d1 sc_out sc_lv 32 signal 3 } 
	{ x_out_3_address1 sc_out sc_lv 5 signal 4 } 
	{ x_out_3_ce1 sc_out sc_logic 1 signal 4 } 
	{ x_out_3_we1 sc_out sc_logic 1 signal 4 } 
	{ x_out_3_d1 sc_out sc_lv 32 signal 4 } 
	{ x_out_4_address1 sc_out sc_lv 5 signal 5 } 
	{ x_out_4_ce1 sc_out sc_logic 1 signal 5 } 
	{ x_out_4_we1 sc_out sc_logic 1 signal 5 } 
	{ x_out_4_d1 sc_out sc_lv 32 signal 5 } 
	{ x_out_5_address1 sc_out sc_lv 5 signal 6 } 
	{ x_out_5_ce1 sc_out sc_logic 1 signal 6 } 
	{ x_out_5_we1 sc_out sc_logic 1 signal 6 } 
	{ x_out_5_d1 sc_out sc_lv 32 signal 6 } 
	{ empty_519 sc_in sc_lv 27 signal 7 } 
	{ empty_520 sc_in sc_lv 27 signal 8 } 
	{ empty_521 sc_in sc_lv 27 signal 9 } 
	{ empty_522 sc_in sc_lv 27 signal 10 } 
	{ empty_523 sc_in sc_lv 27 signal 11 } 
	{ empty sc_in sc_lv 27 signal 12 } 
	{ step_data_0_0_0_load sc_in sc_lv 32 signal 13 } 
	{ step_data_0_1_0_load sc_in sc_lv 32 signal 14 } 
	{ step_data_0_2_0_load sc_in sc_lv 32 signal 15 } 
	{ step_data_0_3_0_load sc_in sc_lv 32 signal 16 } 
	{ step_data_0_4_0_load sc_in sc_lv 32 signal 17 } 
	{ step_data_0_5_0_load sc_in sc_lv 32 signal 18 } 
	{ sext_ln159 sc_in sc_lv 32 signal 19 } 
	{ step_data_0_0_1_load sc_in sc_lv 32 signal 20 } 
	{ step_data_0_1_1_load sc_in sc_lv 32 signal 21 } 
	{ step_data_0_2_1_load sc_in sc_lv 32 signal 22 } 
	{ step_data_0_3_1_load sc_in sc_lv 32 signal 23 } 
	{ step_data_0_4_1_load sc_in sc_lv 32 signal 24 } 
	{ step_data_0_5_1_load sc_in sc_lv 32 signal 25 } 
	{ sext_ln159_1 sc_in sc_lv 32 signal 26 } 
	{ step_data_0_0_2_load sc_in sc_lv 32 signal 27 } 
	{ step_data_0_1_2_load sc_in sc_lv 32 signal 28 } 
	{ step_data_0_2_2_load sc_in sc_lv 32 signal 29 } 
	{ step_data_0_3_2_load sc_in sc_lv 32 signal 30 } 
	{ step_data_0_4_2_load sc_in sc_lv 32 signal 31 } 
	{ step_data_0_5_2_load sc_in sc_lv 32 signal 32 } 
	{ sext_ln159_2 sc_in sc_lv 32 signal 33 } 
	{ step_data_0_0_3_load sc_in sc_lv 32 signal 34 } 
	{ step_data_0_1_3_load sc_in sc_lv 32 signal 35 } 
	{ step_data_0_2_3_load sc_in sc_lv 32 signal 36 } 
	{ step_data_0_3_3_load sc_in sc_lv 32 signal 37 } 
	{ step_data_0_4_3_load sc_in sc_lv 32 signal 38 } 
	{ step_data_0_5_3_load sc_in sc_lv 32 signal 39 } 
	{ sext_ln159_3 sc_in sc_lv 32 signal 40 } 
	{ step_data_0_0_4_load sc_in sc_lv 32 signal 41 } 
	{ step_data_0_1_4_load sc_in sc_lv 32 signal 42 } 
	{ step_data_0_2_4_load sc_in sc_lv 32 signal 43 } 
	{ step_data_0_3_4_load sc_in sc_lv 32 signal 44 } 
	{ step_data_0_4_4_load sc_in sc_lv 32 signal 45 } 
	{ step_data_0_5_4_load sc_in sc_lv 32 signal 46 } 
	{ sext_ln159_4 sc_in sc_lv 32 signal 47 } 
	{ step_data_0_0_5_load sc_in sc_lv 32 signal 48 } 
	{ step_data_0_1_5_load sc_in sc_lv 32 signal 49 } 
	{ step_data_0_2_5_load sc_in sc_lv 32 signal 50 } 
	{ step_data_0_3_5_load sc_in sc_lv 32 signal 51 } 
	{ step_data_0_4_5_load sc_in sc_lv 32 signal 52 } 
	{ step_data_0_5_5_load sc_in sc_lv 32 signal 53 } 
	{ sext_ln406 sc_in sc_lv 32 signal 54 } 
	{ sext_ln159_11 sc_in sc_lv 32 signal 55 } 
	{ sext_ln420 sc_in sc_lv 27 signal 56 } 
	{ sext_ln159_12 sc_in sc_lv 32 signal 57 } 
	{ sext_ln159_13 sc_in sc_lv 32 signal 58 } 
	{ sext_ln159_14 sc_in sc_lv 32 signal 59 } 
	{ sext_ln420_1 sc_in sc_lv 27 signal 60 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "x_out_0_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_0", "role": "address1" }} , 
 	{ "name": "x_out_0_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "ce1" }} , 
 	{ "name": "x_out_0_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "we1" }} , 
 	{ "name": "x_out_0_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_0", "role": "d1" }} , 
 	{ "name": "zext_ln409", "direction": "in", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "zext_ln409", "role": "default" }} , 
 	{ "name": "x_out_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_1", "role": "address1" }} , 
 	{ "name": "x_out_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "ce1" }} , 
 	{ "name": "x_out_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "we1" }} , 
 	{ "name": "x_out_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_1", "role": "d1" }} , 
 	{ "name": "x_out_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_2", "role": "address1" }} , 
 	{ "name": "x_out_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "ce1" }} , 
 	{ "name": "x_out_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "we1" }} , 
 	{ "name": "x_out_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_2", "role": "d1" }} , 
 	{ "name": "x_out_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_3", "role": "address1" }} , 
 	{ "name": "x_out_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "ce1" }} , 
 	{ "name": "x_out_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "we1" }} , 
 	{ "name": "x_out_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_3", "role": "d1" }} , 
 	{ "name": "x_out_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_4", "role": "address1" }} , 
 	{ "name": "x_out_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "ce1" }} , 
 	{ "name": "x_out_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "we1" }} , 
 	{ "name": "x_out_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_4", "role": "d1" }} , 
 	{ "name": "x_out_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_5", "role": "address1" }} , 
 	{ "name": "x_out_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "ce1" }} , 
 	{ "name": "x_out_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "we1" }} , 
 	{ "name": "x_out_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_5", "role": "d1" }} , 
 	{ "name": "empty_519", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "empty_519", "role": "default" }} , 
 	{ "name": "empty_520", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "empty_520", "role": "default" }} , 
 	{ "name": "empty_521", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "empty_521", "role": "default" }} , 
 	{ "name": "empty_522", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "empty_522", "role": "default" }} , 
 	{ "name": "empty_523", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "empty_523", "role": "default" }} , 
 	{ "name": "empty", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "empty", "role": "default" }} , 
 	{ "name": "step_data_0_0_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_0_load", "role": "default" }} , 
 	{ "name": "sext_ln159", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159", "role": "default" }} , 
 	{ "name": "step_data_0_0_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_1_load", "role": "default" }} , 
 	{ "name": "sext_ln159_1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_1", "role": "default" }} , 
 	{ "name": "step_data_0_0_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_2_load", "role": "default" }} , 
 	{ "name": "sext_ln159_2", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_2", "role": "default" }} , 
 	{ "name": "step_data_0_0_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_3_load", "role": "default" }} , 
 	{ "name": "sext_ln159_3", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_3", "role": "default" }} , 
 	{ "name": "step_data_0_0_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_4_load", "role": "default" }} , 
 	{ "name": "sext_ln159_4", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_4", "role": "default" }} , 
 	{ "name": "step_data_0_0_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_5_load", "role": "default" }} , 
 	{ "name": "sext_ln406", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln406", "role": "default" }} , 
 	{ "name": "sext_ln159_11", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_11", "role": "default" }} , 
 	{ "name": "sext_ln420", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "sext_ln420", "role": "default" }} , 
 	{ "name": "sext_ln159_12", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_12", "role": "default" }} , 
 	{ "name": "sext_ln159_13", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_13", "role": "default" }} , 
 	{ "name": "sext_ln159_14", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln159_14", "role": "default" }} , 
 	{ "name": "sext_ln420_1", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "sext_ln420_1", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		a5 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "11", "Max" : "11"}
	, {"Name" : "Interval", "Min" : "7", "Max" : "7"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	x_out_0 { ap_memory {  { x_out_0_address1 MemPortADDR2 1 5 }  { x_out_0_ce1 MemPortCE2 1 1 }  { x_out_0_we1 MemPortWE2 1 1 }  { x_out_0_d1 MemPortDIN2 1 32 } } }
	zext_ln409 { ap_none {  { zext_ln409 in_data 0 5 } } }
	x_out_1 { ap_memory {  { x_out_1_address1 MemPortADDR2 1 5 }  { x_out_1_ce1 MemPortCE2 1 1 }  { x_out_1_we1 MemPortWE2 1 1 }  { x_out_1_d1 MemPortDIN2 1 32 } } }
	x_out_2 { ap_memory {  { x_out_2_address1 MemPortADDR2 1 5 }  { x_out_2_ce1 MemPortCE2 1 1 }  { x_out_2_we1 MemPortWE2 1 1 }  { x_out_2_d1 MemPortDIN2 1 32 } } }
	x_out_3 { ap_memory {  { x_out_3_address1 MemPortADDR2 1 5 }  { x_out_3_ce1 MemPortCE2 1 1 }  { x_out_3_we1 MemPortWE2 1 1 }  { x_out_3_d1 MemPortDIN2 1 32 } } }
	x_out_4 { ap_memory {  { x_out_4_address1 MemPortADDR2 1 5 }  { x_out_4_ce1 MemPortCE2 1 1 }  { x_out_4_we1 MemPortWE2 1 1 }  { x_out_4_d1 MemPortDIN2 1 32 } } }
	x_out_5 { ap_memory {  { x_out_5_address1 MemPortADDR2 1 5 }  { x_out_5_ce1 MemPortCE2 1 1 }  { x_out_5_we1 MemPortWE2 1 1 }  { x_out_5_d1 MemPortDIN2 1 32 } } }
	empty_519 { ap_none {  { empty_519 in_data 0 27 } } }
	empty_520 { ap_none {  { empty_520 in_data 0 27 } } }
	empty_521 { ap_none {  { empty_521 in_data 0 27 } } }
	empty_522 { ap_none {  { empty_522 in_data 0 27 } } }
	empty_523 { ap_none {  { empty_523 in_data 0 27 } } }
	empty { ap_none {  { empty in_data 0 27 } } }
	step_data_0_0_0_load { ap_none {  { step_data_0_0_0_load in_data 0 32 } } }
	step_data_0_1_0_load { ap_none {  { step_data_0_1_0_load in_data 0 32 } } }
	step_data_0_2_0_load { ap_none {  { step_data_0_2_0_load in_data 0 32 } } }
	step_data_0_3_0_load { ap_none {  { step_data_0_3_0_load in_data 0 32 } } }
	step_data_0_4_0_load { ap_none {  { step_data_0_4_0_load in_data 0 32 } } }
	step_data_0_5_0_load { ap_none {  { step_data_0_5_0_load in_data 0 32 } } }
	sext_ln159 { ap_none {  { sext_ln159 in_data 0 32 } } }
	step_data_0_0_1_load { ap_none {  { step_data_0_0_1_load in_data 0 32 } } }
	step_data_0_1_1_load { ap_none {  { step_data_0_1_1_load in_data 0 32 } } }
	step_data_0_2_1_load { ap_none {  { step_data_0_2_1_load in_data 0 32 } } }
	step_data_0_3_1_load { ap_none {  { step_data_0_3_1_load in_data 0 32 } } }
	step_data_0_4_1_load { ap_none {  { step_data_0_4_1_load in_data 0 32 } } }
	step_data_0_5_1_load { ap_none {  { step_data_0_5_1_load in_data 0 32 } } }
	sext_ln159_1 { ap_none {  { sext_ln159_1 in_data 0 32 } } }
	step_data_0_0_2_load { ap_none {  { step_data_0_0_2_load in_data 0 32 } } }
	step_data_0_1_2_load { ap_none {  { step_data_0_1_2_load in_data 0 32 } } }
	step_data_0_2_2_load { ap_none {  { step_data_0_2_2_load in_data 0 32 } } }
	step_data_0_3_2_load { ap_none {  { step_data_0_3_2_load in_data 0 32 } } }
	step_data_0_4_2_load { ap_none {  { step_data_0_4_2_load in_data 0 32 } } }
	step_data_0_5_2_load { ap_none {  { step_data_0_5_2_load in_data 0 32 } } }
	sext_ln159_2 { ap_none {  { sext_ln159_2 in_data 0 32 } } }
	step_data_0_0_3_load { ap_none {  { step_data_0_0_3_load in_data 0 32 } } }
	step_data_0_1_3_load { ap_none {  { step_data_0_1_3_load in_data 0 32 } } }
	step_data_0_2_3_load { ap_none {  { step_data_0_2_3_load in_data 0 32 } } }
	step_data_0_3_3_load { ap_none {  { step_data_0_3_3_load in_data 0 32 } } }
	step_data_0_4_3_load { ap_none {  { step_data_0_4_3_load in_data 0 32 } } }
	step_data_0_5_3_load { ap_none {  { step_data_0_5_3_load in_data 0 32 } } }
	sext_ln159_3 { ap_none {  { sext_ln159_3 in_data 0 32 } } }
	step_data_0_0_4_load { ap_none {  { step_data_0_0_4_load in_data 0 32 } } }
	step_data_0_1_4_load { ap_none {  { step_data_0_1_4_load in_data 0 32 } } }
	step_data_0_2_4_load { ap_none {  { step_data_0_2_4_load in_data 0 32 } } }
	step_data_0_3_4_load { ap_none {  { step_data_0_3_4_load in_data 0 32 } } }
	step_data_0_4_4_load { ap_none {  { step_data_0_4_4_load in_data 0 32 } } }
	step_data_0_5_4_load { ap_none {  { step_data_0_5_4_load in_data 0 32 } } }
	sext_ln159_4 { ap_none {  { sext_ln159_4 in_data 0 32 } } }
	step_data_0_0_5_load { ap_none {  { step_data_0_0_5_load in_data 0 32 } } }
	step_data_0_1_5_load { ap_none {  { step_data_0_1_5_load in_data 0 32 } } }
	step_data_0_2_5_load { ap_none {  { step_data_0_2_5_load in_data 0 32 } } }
	step_data_0_3_5_load { ap_none {  { step_data_0_3_5_load in_data 0 32 } } }
	step_data_0_4_5_load { ap_none {  { step_data_0_4_5_load in_data 0 32 } } }
	step_data_0_5_5_load { ap_none {  { step_data_0_5_5_load in_data 0 32 } } }
	sext_ln406 { ap_none {  { sext_ln406 in_data 0 32 } } }
	sext_ln159_11 { ap_none {  { sext_ln159_11 in_data 0 32 } } }
	sext_ln420 { ap_none {  { sext_ln420 in_data 0 27 } } }
	sext_ln159_12 { ap_none {  { sext_ln159_12 in_data 0 32 } } }
	sext_ln159_13 { ap_none {  { sext_ln159_13 in_data 0 32 } } }
	sext_ln159_14 { ap_none {  { sext_ln159_14 in_data 0 32 } } }
	sext_ln420_1 { ap_none {  { sext_ln420_1 in_data 0 27 } } }
}
