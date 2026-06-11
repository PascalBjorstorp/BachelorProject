set moduleName riccati_forward_pass_Pipeline_VITIS_LOOP_399_4
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
set C_modelName {riccati_forward_pass_Pipeline_VITIS_LOOP_399_4}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict x_out_0 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict step_data_0_0 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ x_out_0 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ zext_ln428 int 5 regular  }
	{ x_out_1 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_2 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_3 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_4 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ x_out_5 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ mul_ln409_1 int 7 regular  }
	{ step_data_0_0 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ empty_564 int 25 regular  }
	{ empty_565 int 25 regular  }
	{ empty_566 int 25 regular  }
	{ empty_567 int 25 regular  }
	{ empty_568 int 25 regular  }
	{ empty int 25 regular  }
	{ sext_ln156 int 26 regular  }
	{ sext_ln156_1 int 26 regular  }
	{ sext_ln156_2 int 26 regular  }
	{ sext_ln156_3 int 26 regular  }
	{ sext_ln156_4 int 26 regular  }
	{ sext_ln422 int 26 regular  }
	{ sext_ln156_11 int 26 regular  }
	{ sext_ln425 int 25 regular  }
	{ sext_ln156_12 int 26 regular  }
	{ sext_ln156_13 int 26 regular  }
	{ sext_ln156_14 int 26 regular  }
	{ sext_ln436 int 25 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "x_out_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "zext_ln428", "interface" : "wire", "bitwidth" : 5, "direction" : "READONLY"} , 
 	{ "Name" : "x_out_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_out_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "mul_ln409_1", "interface" : "wire", "bitwidth" : 7, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "empty_564", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "empty_565", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "empty_566", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "empty_567", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "empty_568", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "empty", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_1", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_2", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_3", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_4", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln422", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_11", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln425", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_12", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_13", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_14", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln436", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} ]}
# RTL Port declarations: 
set portNum 72
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
	{ x_out_0_d1 sc_out sc_lv 26 signal 0 } 
	{ zext_ln428 sc_in sc_lv 5 signal 1 } 
	{ x_out_1_address1 sc_out sc_lv 5 signal 2 } 
	{ x_out_1_ce1 sc_out sc_logic 1 signal 2 } 
	{ x_out_1_we1 sc_out sc_logic 1 signal 2 } 
	{ x_out_1_d1 sc_out sc_lv 26 signal 2 } 
	{ x_out_2_address1 sc_out sc_lv 5 signal 3 } 
	{ x_out_2_ce1 sc_out sc_logic 1 signal 3 } 
	{ x_out_2_we1 sc_out sc_logic 1 signal 3 } 
	{ x_out_2_d1 sc_out sc_lv 26 signal 3 } 
	{ x_out_3_address1 sc_out sc_lv 5 signal 4 } 
	{ x_out_3_ce1 sc_out sc_logic 1 signal 4 } 
	{ x_out_3_we1 sc_out sc_logic 1 signal 4 } 
	{ x_out_3_d1 sc_out sc_lv 26 signal 4 } 
	{ x_out_4_address1 sc_out sc_lv 5 signal 5 } 
	{ x_out_4_ce1 sc_out sc_logic 1 signal 5 } 
	{ x_out_4_we1 sc_out sc_logic 1 signal 5 } 
	{ x_out_4_d1 sc_out sc_lv 26 signal 5 } 
	{ x_out_5_address1 sc_out sc_lv 5 signal 6 } 
	{ x_out_5_ce1 sc_out sc_logic 1 signal 6 } 
	{ x_out_5_we1 sc_out sc_logic 1 signal 6 } 
	{ x_out_5_d1 sc_out sc_lv 26 signal 6 } 
	{ mul_ln409_1 sc_in sc_lv 7 signal 7 } 
	{ step_data_0_0_address0 sc_out sc_lv 7 signal 8 } 
	{ step_data_0_0_ce0 sc_out sc_logic 1 signal 8 } 
	{ step_data_0_0_q0 sc_in sc_lv 26 signal 8 } 
	{ step_data_0_1_address0 sc_out sc_lv 7 signal 9 } 
	{ step_data_0_1_ce0 sc_out sc_logic 1 signal 9 } 
	{ step_data_0_1_q0 sc_in sc_lv 26 signal 9 } 
	{ step_data_0_2_address0 sc_out sc_lv 7 signal 10 } 
	{ step_data_0_2_ce0 sc_out sc_logic 1 signal 10 } 
	{ step_data_0_2_q0 sc_in sc_lv 26 signal 10 } 
	{ step_data_0_3_address0 sc_out sc_lv 7 signal 11 } 
	{ step_data_0_3_ce0 sc_out sc_logic 1 signal 11 } 
	{ step_data_0_3_q0 sc_in sc_lv 26 signal 11 } 
	{ step_data_0_4_address0 sc_out sc_lv 7 signal 12 } 
	{ step_data_0_4_ce0 sc_out sc_logic 1 signal 12 } 
	{ step_data_0_4_q0 sc_in sc_lv 26 signal 12 } 
	{ step_data_0_5_address0 sc_out sc_lv 7 signal 13 } 
	{ step_data_0_5_ce0 sc_out sc_logic 1 signal 13 } 
	{ step_data_0_5_q0 sc_in sc_lv 26 signal 13 } 
	{ empty_564 sc_in sc_lv 25 signal 14 } 
	{ empty_565 sc_in sc_lv 25 signal 15 } 
	{ empty_566 sc_in sc_lv 25 signal 16 } 
	{ empty_567 sc_in sc_lv 25 signal 17 } 
	{ empty_568 sc_in sc_lv 25 signal 18 } 
	{ empty sc_in sc_lv 25 signal 19 } 
	{ sext_ln156 sc_in sc_lv 26 signal 20 } 
	{ sext_ln156_1 sc_in sc_lv 26 signal 21 } 
	{ sext_ln156_2 sc_in sc_lv 26 signal 22 } 
	{ sext_ln156_3 sc_in sc_lv 26 signal 23 } 
	{ sext_ln156_4 sc_in sc_lv 26 signal 24 } 
	{ sext_ln422 sc_in sc_lv 26 signal 25 } 
	{ sext_ln156_11 sc_in sc_lv 26 signal 26 } 
	{ sext_ln425 sc_in sc_lv 25 signal 27 } 
	{ sext_ln156_12 sc_in sc_lv 26 signal 28 } 
	{ sext_ln156_13 sc_in sc_lv 26 signal 29 } 
	{ sext_ln156_14 sc_in sc_lv 26 signal 30 } 
	{ sext_ln436 sc_in sc_lv 25 signal 31 } 
	{ grp_fu_3394_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_3394_p_din1 sc_out sc_lv 25 signal -1 } 
	{ grp_fu_3394_p_dout0 sc_in sc_lv 51 signal -1 } 
	{ grp_fu_3394_p_ce sc_out sc_logic 1 signal -1 } 
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
 	{ "name": "x_out_0_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_0", "role": "d1" }} , 
 	{ "name": "zext_ln428", "direction": "in", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "zext_ln428", "role": "default" }} , 
 	{ "name": "x_out_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_1", "role": "address1" }} , 
 	{ "name": "x_out_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "ce1" }} , 
 	{ "name": "x_out_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "we1" }} , 
 	{ "name": "x_out_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_1", "role": "d1" }} , 
 	{ "name": "x_out_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_2", "role": "address1" }} , 
 	{ "name": "x_out_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "ce1" }} , 
 	{ "name": "x_out_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "we1" }} , 
 	{ "name": "x_out_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_2", "role": "d1" }} , 
 	{ "name": "x_out_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_3", "role": "address1" }} , 
 	{ "name": "x_out_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "ce1" }} , 
 	{ "name": "x_out_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "we1" }} , 
 	{ "name": "x_out_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_3", "role": "d1" }} , 
 	{ "name": "x_out_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_4", "role": "address1" }} , 
 	{ "name": "x_out_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "ce1" }} , 
 	{ "name": "x_out_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "we1" }} , 
 	{ "name": "x_out_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_4", "role": "d1" }} , 
 	{ "name": "x_out_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_5", "role": "address1" }} , 
 	{ "name": "x_out_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "ce1" }} , 
 	{ "name": "x_out_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "we1" }} , 
 	{ "name": "x_out_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_5", "role": "d1" }} , 
 	{ "name": "mul_ln409_1", "direction": "in", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "mul_ln409_1", "role": "default" }} , 
 	{ "name": "step_data_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_0", "role": "address0" }} , 
 	{ "name": "step_data_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0", "role": "ce0" }} , 
 	{ "name": "step_data_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_0", "role": "q0" }} , 
 	{ "name": "step_data_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_1", "role": "address0" }} , 
 	{ "name": "step_data_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1", "role": "ce0" }} , 
 	{ "name": "step_data_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_1", "role": "q0" }} , 
 	{ "name": "step_data_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_2", "role": "address0" }} , 
 	{ "name": "step_data_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2", "role": "ce0" }} , 
 	{ "name": "step_data_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_2", "role": "q0" }} , 
 	{ "name": "step_data_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_3", "role": "address0" }} , 
 	{ "name": "step_data_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3", "role": "ce0" }} , 
 	{ "name": "step_data_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_3", "role": "q0" }} , 
 	{ "name": "step_data_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_4", "role": "address0" }} , 
 	{ "name": "step_data_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4", "role": "ce0" }} , 
 	{ "name": "step_data_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_4", "role": "q0" }} , 
 	{ "name": "step_data_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_0_5", "role": "address0" }} , 
 	{ "name": "step_data_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5", "role": "ce0" }} , 
 	{ "name": "step_data_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_0_5", "role": "q0" }} , 
 	{ "name": "empty_564", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "empty_564", "role": "default" }} , 
 	{ "name": "empty_565", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "empty_565", "role": "default" }} , 
 	{ "name": "empty_566", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "empty_566", "role": "default" }} , 
 	{ "name": "empty_567", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "empty_567", "role": "default" }} , 
 	{ "name": "empty_568", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "empty_568", "role": "default" }} , 
 	{ "name": "empty", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "empty", "role": "default" }} , 
 	{ "name": "sext_ln156", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156", "role": "default" }} , 
 	{ "name": "sext_ln156_1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_1", "role": "default" }} , 
 	{ "name": "sext_ln156_2", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_2", "role": "default" }} , 
 	{ "name": "sext_ln156_3", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_3", "role": "default" }} , 
 	{ "name": "sext_ln156_4", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_4", "role": "default" }} , 
 	{ "name": "sext_ln422", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln422", "role": "default" }} , 
 	{ "name": "sext_ln156_11", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_11", "role": "default" }} , 
 	{ "name": "sext_ln425", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "sext_ln425", "role": "default" }} , 
 	{ "name": "sext_ln156_12", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_12", "role": "default" }} , 
 	{ "name": "sext_ln156_13", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_13", "role": "default" }} , 
 	{ "name": "sext_ln156_14", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_14", "role": "default" }} , 
 	{ "name": "sext_ln436", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "sext_ln436", "role": "default" }} , 
 	{ "name": "grp_fu_3394_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_3394_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_3394_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "grp_fu_3394_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_3394_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":51, "type": "signal", "bundle":{"name": "grp_fu_3394_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_3394_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_3394_p_ce", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
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
	x_out_0 { ap_memory {  { x_out_0_address1 MemPortADDR2 1 5 }  { x_out_0_ce1 MemPortCE2 1 1 }  { x_out_0_we1 MemPortWE2 1 1 }  { x_out_0_d1 MemPortDIN2 1 26 } } }
	zext_ln428 { ap_none {  { zext_ln428 in_data 0 5 } } }
	x_out_1 { ap_memory {  { x_out_1_address1 MemPortADDR2 1 5 }  { x_out_1_ce1 MemPortCE2 1 1 }  { x_out_1_we1 MemPortWE2 1 1 }  { x_out_1_d1 MemPortDIN2 1 26 } } }
	x_out_2 { ap_memory {  { x_out_2_address1 MemPortADDR2 1 5 }  { x_out_2_ce1 MemPortCE2 1 1 }  { x_out_2_we1 MemPortWE2 1 1 }  { x_out_2_d1 MemPortDIN2 1 26 } } }
	x_out_3 { ap_memory {  { x_out_3_address1 MemPortADDR2 1 5 }  { x_out_3_ce1 MemPortCE2 1 1 }  { x_out_3_we1 MemPortWE2 1 1 }  { x_out_3_d1 MemPortDIN2 1 26 } } }
	x_out_4 { ap_memory {  { x_out_4_address1 MemPortADDR2 1 5 }  { x_out_4_ce1 MemPortCE2 1 1 }  { x_out_4_we1 MemPortWE2 1 1 }  { x_out_4_d1 MemPortDIN2 1 26 } } }
	x_out_5 { ap_memory {  { x_out_5_address1 MemPortADDR2 1 5 }  { x_out_5_ce1 MemPortCE2 1 1 }  { x_out_5_we1 MemPortWE2 1 1 }  { x_out_5_d1 MemPortDIN2 1 26 } } }
	mul_ln409_1 { ap_none {  { mul_ln409_1 in_data 0 7 } } }
	step_data_0_0 { ap_memory {  { step_data_0_0_address0 mem_address 1 7 }  { step_data_0_0_ce0 mem_ce 1 1 }  { step_data_0_0_q0 mem_dout 0 26 } } }
	step_data_0_1 { ap_memory {  { step_data_0_1_address0 mem_address 1 7 }  { step_data_0_1_ce0 mem_ce 1 1 }  { step_data_0_1_q0 mem_dout 0 26 } } }
	step_data_0_2 { ap_memory {  { step_data_0_2_address0 mem_address 1 7 }  { step_data_0_2_ce0 mem_ce 1 1 }  { step_data_0_2_q0 mem_dout 0 26 } } }
	step_data_0_3 { ap_memory {  { step_data_0_3_address0 mem_address 1 7 }  { step_data_0_3_ce0 mem_ce 1 1 }  { step_data_0_3_q0 mem_dout 0 26 } } }
	step_data_0_4 { ap_memory {  { step_data_0_4_address0 mem_address 1 7 }  { step_data_0_4_ce0 mem_ce 1 1 }  { step_data_0_4_q0 mem_dout 0 26 } } }
	step_data_0_5 { ap_memory {  { step_data_0_5_address0 mem_address 1 7 }  { step_data_0_5_ce0 mem_ce 1 1 }  { step_data_0_5_q0 mem_dout 0 26 } } }
	empty_564 { ap_none {  { empty_564 in_data 0 25 } } }
	empty_565 { ap_none {  { empty_565 in_data 0 25 } } }
	empty_566 { ap_none {  { empty_566 in_data 0 25 } } }
	empty_567 { ap_none {  { empty_567 in_data 0 25 } } }
	empty_568 { ap_none {  { empty_568 in_data 0 25 } } }
	empty { ap_none {  { empty in_data 0 25 } } }
	sext_ln156 { ap_none {  { sext_ln156 in_data 0 26 } } }
	sext_ln156_1 { ap_none {  { sext_ln156_1 in_data 0 26 } } }
	sext_ln156_2 { ap_none {  { sext_ln156_2 in_data 0 26 } } }
	sext_ln156_3 { ap_none {  { sext_ln156_3 in_data 0 26 } } }
	sext_ln156_4 { ap_none {  { sext_ln156_4 in_data 0 26 } } }
	sext_ln422 { ap_none {  { sext_ln422 in_data 0 26 } } }
	sext_ln156_11 { ap_none {  { sext_ln156_11 in_data 0 26 } } }
	sext_ln425 { ap_none {  { sext_ln425 in_data 0 25 } } }
	sext_ln156_12 { ap_none {  { sext_ln156_12 in_data 0 26 } } }
	sext_ln156_13 { ap_none {  { sext_ln156_13 in_data 0 26 } } }
	sext_ln156_14 { ap_none {  { sext_ln156_14 in_data 0 26 } } }
	sext_ln436 { ap_none {  { sext_ln436 in_data 0 25 } } }
}
