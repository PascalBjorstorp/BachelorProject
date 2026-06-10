set moduleName riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8
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
set C_modelName {riccati_backward_pass_Pipeline_VITIS_LOOP_726_7_VITIS_LOOP_727_8}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ sext_ln167_1 int 33 regular  }
	{ M int 34 regular  }
	{ sext_ln167_2 int 33 regular  }
	{ M_2 int 34 regular  }
	{ sext_ln167_3 int 33 regular  }
	{ M_12 int 34 regular  }
	{ sext_ln167_4 int 33 regular  }
	{ M_14 int 34 regular  }
	{ sext_ln167_5 int 33 regular  }
	{ M_16 int 34 regular  }
	{ sext_ln167_6 int 33 regular  }
	{ M_18 int 34 regular  }
	{ step_data_0_0_0_load int 32 regular  }
	{ step_data_0_0_1_load int 32 regular  }
	{ step_data_0_0_2_load int 32 regular  }
	{ step_data_0_0_3_load int 32 regular  }
	{ step_data_0_0_4_load int 32 regular  }
	{ step_data_0_0_5_load int 32 regular  }
	{ step_data_0_1_0_load int 32 regular  }
	{ step_data_0_1_1_load int 32 regular  }
	{ step_data_0_1_2_load int 32 regular  }
	{ step_data_0_1_3_load int 32 regular  }
	{ step_data_0_1_4_load int 32 regular  }
	{ step_data_0_1_5_load int 32 regular  }
	{ step_data_0_2_0_load int 32 regular  }
	{ step_data_0_2_1_load int 32 regular  }
	{ step_data_0_2_2_load int 32 regular  }
	{ step_data_0_2_3_load int 32 regular  }
	{ step_data_0_2_4_load int 32 regular  }
	{ step_data_0_2_5_load int 32 regular  }
	{ step_data_0_3_0_load int 32 regular  }
	{ step_data_0_3_1_load int 32 regular  }
	{ step_data_0_3_2_load int 32 regular  }
	{ step_data_0_3_3_load int 32 regular  }
	{ step_data_0_3_4_load int 32 regular  }
	{ step_data_0_3_5_load int 32 regular  }
	{ step_data_0_4_0_load int 32 regular  }
	{ step_data_0_4_1_load int 32 regular  }
	{ step_data_0_4_2_load int 32 regular  }
	{ step_data_0_4_3_load int 32 regular  }
	{ step_data_0_4_4_load int 32 regular  }
	{ step_data_0_4_5_load int 32 regular  }
	{ step_data_0_5_0_load int 32 regular  }
	{ step_data_0_5_1_load int 32 regular  }
	{ step_data_0_5_2_load int 32 regular  }
	{ step_data_0_5_3_load int 32 regular  }
	{ step_data_0_5_4_load int 32 regular  }
	{ step_data_0_5_5_load int 32 regular  }
	{ G_13_out int 34 regular {pointer 1}  }
	{ G_12_out int 34 regular {pointer 1}  }
	{ G_11_out int 34 regular {pointer 1}  }
	{ G_10_out int 34 regular {pointer 1}  }
	{ G_9_out int 34 regular {pointer 1}  }
	{ G_8_out int 34 regular {pointer 1}  }
	{ G_7_out int 34 regular {pointer 1}  }
	{ G_6_out int 34 regular {pointer 1}  }
	{ G_5_out int 34 regular {pointer 1}  }
	{ G_4_out int 34 regular {pointer 1}  }
	{ G_3_out int 34 regular {pointer 1}  }
	{ G_2_out int 34 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "sext_ln167_1", "interface" : "wire", "bitwidth" : 33, "direction" : "READONLY"} , 
 	{ "Name" : "M", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln167_2", "interface" : "wire", "bitwidth" : 33, "direction" : "READONLY"} , 
 	{ "Name" : "M_2", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln167_3", "interface" : "wire", "bitwidth" : 33, "direction" : "READONLY"} , 
 	{ "Name" : "M_12", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln167_4", "interface" : "wire", "bitwidth" : 33, "direction" : "READONLY"} , 
 	{ "Name" : "M_14", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln167_5", "interface" : "wire", "bitwidth" : 33, "direction" : "READONLY"} , 
 	{ "Name" : "M_16", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln167_6", "interface" : "wire", "bitwidth" : 33, "direction" : "READONLY"} , 
 	{ "Name" : "M_18", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_0_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_1_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_2_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_3_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_4_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_5_load", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "G_13_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_12_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_11_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_10_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_9_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_8_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_7_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_6_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_5_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_4_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_3_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_2_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 78
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ sext_ln167_1 sc_in sc_lv 33 signal 0 } 
	{ M sc_in sc_lv 34 signal 1 } 
	{ sext_ln167_2 sc_in sc_lv 33 signal 2 } 
	{ M_2 sc_in sc_lv 34 signal 3 } 
	{ sext_ln167_3 sc_in sc_lv 33 signal 4 } 
	{ M_12 sc_in sc_lv 34 signal 5 } 
	{ sext_ln167_4 sc_in sc_lv 33 signal 6 } 
	{ M_14 sc_in sc_lv 34 signal 7 } 
	{ sext_ln167_5 sc_in sc_lv 33 signal 8 } 
	{ M_16 sc_in sc_lv 34 signal 9 } 
	{ sext_ln167_6 sc_in sc_lv 33 signal 10 } 
	{ M_18 sc_in sc_lv 34 signal 11 } 
	{ step_data_0_0_0_load sc_in sc_lv 32 signal 12 } 
	{ step_data_0_0_1_load sc_in sc_lv 32 signal 13 } 
	{ step_data_0_0_2_load sc_in sc_lv 32 signal 14 } 
	{ step_data_0_0_3_load sc_in sc_lv 32 signal 15 } 
	{ step_data_0_0_4_load sc_in sc_lv 32 signal 16 } 
	{ step_data_0_0_5_load sc_in sc_lv 32 signal 17 } 
	{ step_data_0_1_0_load sc_in sc_lv 32 signal 18 } 
	{ step_data_0_1_1_load sc_in sc_lv 32 signal 19 } 
	{ step_data_0_1_2_load sc_in sc_lv 32 signal 20 } 
	{ step_data_0_1_3_load sc_in sc_lv 32 signal 21 } 
	{ step_data_0_1_4_load sc_in sc_lv 32 signal 22 } 
	{ step_data_0_1_5_load sc_in sc_lv 32 signal 23 } 
	{ step_data_0_2_0_load sc_in sc_lv 32 signal 24 } 
	{ step_data_0_2_1_load sc_in sc_lv 32 signal 25 } 
	{ step_data_0_2_2_load sc_in sc_lv 32 signal 26 } 
	{ step_data_0_2_3_load sc_in sc_lv 32 signal 27 } 
	{ step_data_0_2_4_load sc_in sc_lv 32 signal 28 } 
	{ step_data_0_2_5_load sc_in sc_lv 32 signal 29 } 
	{ step_data_0_3_0_load sc_in sc_lv 32 signal 30 } 
	{ step_data_0_3_1_load sc_in sc_lv 32 signal 31 } 
	{ step_data_0_3_2_load sc_in sc_lv 32 signal 32 } 
	{ step_data_0_3_3_load sc_in sc_lv 32 signal 33 } 
	{ step_data_0_3_4_load sc_in sc_lv 32 signal 34 } 
	{ step_data_0_3_5_load sc_in sc_lv 32 signal 35 } 
	{ step_data_0_4_0_load sc_in sc_lv 32 signal 36 } 
	{ step_data_0_4_1_load sc_in sc_lv 32 signal 37 } 
	{ step_data_0_4_2_load sc_in sc_lv 32 signal 38 } 
	{ step_data_0_4_3_load sc_in sc_lv 32 signal 39 } 
	{ step_data_0_4_4_load sc_in sc_lv 32 signal 40 } 
	{ step_data_0_4_5_load sc_in sc_lv 32 signal 41 } 
	{ step_data_0_5_0_load sc_in sc_lv 32 signal 42 } 
	{ step_data_0_5_1_load sc_in sc_lv 32 signal 43 } 
	{ step_data_0_5_2_load sc_in sc_lv 32 signal 44 } 
	{ step_data_0_5_3_load sc_in sc_lv 32 signal 45 } 
	{ step_data_0_5_4_load sc_in sc_lv 32 signal 46 } 
	{ step_data_0_5_5_load sc_in sc_lv 32 signal 47 } 
	{ G_13_out sc_out sc_lv 34 signal 48 } 
	{ G_13_out_ap_vld sc_out sc_logic 1 outvld 48 } 
	{ G_12_out sc_out sc_lv 34 signal 49 } 
	{ G_12_out_ap_vld sc_out sc_logic 1 outvld 49 } 
	{ G_11_out sc_out sc_lv 34 signal 50 } 
	{ G_11_out_ap_vld sc_out sc_logic 1 outvld 50 } 
	{ G_10_out sc_out sc_lv 34 signal 51 } 
	{ G_10_out_ap_vld sc_out sc_logic 1 outvld 51 } 
	{ G_9_out sc_out sc_lv 34 signal 52 } 
	{ G_9_out_ap_vld sc_out sc_logic 1 outvld 52 } 
	{ G_8_out sc_out sc_lv 34 signal 53 } 
	{ G_8_out_ap_vld sc_out sc_logic 1 outvld 53 } 
	{ G_7_out sc_out sc_lv 34 signal 54 } 
	{ G_7_out_ap_vld sc_out sc_logic 1 outvld 54 } 
	{ G_6_out sc_out sc_lv 34 signal 55 } 
	{ G_6_out_ap_vld sc_out sc_logic 1 outvld 55 } 
	{ G_5_out sc_out sc_lv 34 signal 56 } 
	{ G_5_out_ap_vld sc_out sc_logic 1 outvld 56 } 
	{ G_4_out sc_out sc_lv 34 signal 57 } 
	{ G_4_out_ap_vld sc_out sc_logic 1 outvld 57 } 
	{ G_3_out sc_out sc_lv 34 signal 58 } 
	{ G_3_out_ap_vld sc_out sc_logic 1 outvld 58 } 
	{ G_2_out sc_out sc_lv 34 signal 59 } 
	{ G_2_out_ap_vld sc_out sc_logic 1 outvld 59 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "sext_ln167_1", "direction": "in", "datatype": "sc_lv", "bitwidth":33, "type": "signal", "bundle":{"name": "sext_ln167_1", "role": "default" }} , 
 	{ "name": "M", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "M", "role": "default" }} , 
 	{ "name": "sext_ln167_2", "direction": "in", "datatype": "sc_lv", "bitwidth":33, "type": "signal", "bundle":{"name": "sext_ln167_2", "role": "default" }} , 
 	{ "name": "M_2", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "M_2", "role": "default" }} , 
 	{ "name": "sext_ln167_3", "direction": "in", "datatype": "sc_lv", "bitwidth":33, "type": "signal", "bundle":{"name": "sext_ln167_3", "role": "default" }} , 
 	{ "name": "M_12", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "M_12", "role": "default" }} , 
 	{ "name": "sext_ln167_4", "direction": "in", "datatype": "sc_lv", "bitwidth":33, "type": "signal", "bundle":{"name": "sext_ln167_4", "role": "default" }} , 
 	{ "name": "M_14", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "M_14", "role": "default" }} , 
 	{ "name": "sext_ln167_5", "direction": "in", "datatype": "sc_lv", "bitwidth":33, "type": "signal", "bundle":{"name": "sext_ln167_5", "role": "default" }} , 
 	{ "name": "M_16", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "M_16", "role": "default" }} , 
 	{ "name": "sext_ln167_6", "direction": "in", "datatype": "sc_lv", "bitwidth":33, "type": "signal", "bundle":{"name": "sext_ln167_6", "role": "default" }} , 
 	{ "name": "M_18", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "M_18", "role": "default" }} , 
 	{ "name": "step_data_0_0_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_0_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_0_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_0_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_0_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_0_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_1_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_2_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_3_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_4_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_5_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_0_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_0_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_1_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_2_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_3_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_4_load", "role": "default" }} , 
 	{ "name": "step_data_0_5_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_5_load", "role": "default" }} , 
 	{ "name": "G_13_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_13_out", "role": "default" }} , 
 	{ "name": "G_13_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_13_out", "role": "ap_vld" }} , 
 	{ "name": "G_12_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_12_out", "role": "default" }} , 
 	{ "name": "G_12_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_12_out", "role": "ap_vld" }} , 
 	{ "name": "G_11_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_11_out", "role": "default" }} , 
 	{ "name": "G_11_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_11_out", "role": "ap_vld" }} , 
 	{ "name": "G_10_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_10_out", "role": "default" }} , 
 	{ "name": "G_10_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_10_out", "role": "ap_vld" }} , 
 	{ "name": "G_9_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_9_out", "role": "default" }} , 
 	{ "name": "G_9_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_9_out", "role": "ap_vld" }} , 
 	{ "name": "G_8_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_8_out", "role": "default" }} , 
 	{ "name": "G_8_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_8_out", "role": "ap_vld" }} , 
 	{ "name": "G_7_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_7_out", "role": "default" }} , 
 	{ "name": "G_7_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_7_out", "role": "ap_vld" }} , 
 	{ "name": "G_6_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_6_out", "role": "default" }} , 
 	{ "name": "G_6_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_6_out", "role": "ap_vld" }} , 
 	{ "name": "G_5_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_5_out", "role": "default" }} , 
 	{ "name": "G_5_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_5_out", "role": "ap_vld" }} , 
 	{ "name": "G_4_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_4_out", "role": "default" }} , 
 	{ "name": "G_4_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_4_out", "role": "ap_vld" }} , 
 	{ "name": "G_3_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_3_out", "role": "default" }} , 
 	{ "name": "G_3_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_3_out", "role": "ap_vld" }} , 
 	{ "name": "G_2_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_2_out", "role": "default" }} , 
 	{ "name": "G_2_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_2_out", "role": "ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		a5 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "17", "Max" : "17"}
	, {"Name" : "Interval", "Min" : "13", "Max" : "13"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	sext_ln167_1 { ap_none {  { sext_ln167_1 in_data 0 33 } } }
	M { ap_none {  { M in_data 0 34 } } }
	sext_ln167_2 { ap_none {  { sext_ln167_2 in_data 0 33 } } }
	M_2 { ap_none {  { M_2 in_data 0 34 } } }
	sext_ln167_3 { ap_none {  { sext_ln167_3 in_data 0 33 } } }
	M_12 { ap_none {  { M_12 in_data 0 34 } } }
	sext_ln167_4 { ap_none {  { sext_ln167_4 in_data 0 33 } } }
	M_14 { ap_none {  { M_14 in_data 0 34 } } }
	sext_ln167_5 { ap_none {  { sext_ln167_5 in_data 0 33 } } }
	M_16 { ap_none {  { M_16 in_data 0 34 } } }
	sext_ln167_6 { ap_none {  { sext_ln167_6 in_data 0 33 } } }
	M_18 { ap_none {  { M_18 in_data 0 34 } } }
	step_data_0_0_0_load { ap_none {  { step_data_0_0_0_load in_data 0 32 } } }
	step_data_0_0_1_load { ap_none {  { step_data_0_0_1_load in_data 0 32 } } }
	step_data_0_0_2_load { ap_none {  { step_data_0_0_2_load in_data 0 32 } } }
	step_data_0_0_3_load { ap_none {  { step_data_0_0_3_load in_data 0 32 } } }
	step_data_0_0_4_load { ap_none {  { step_data_0_0_4_load in_data 0 32 } } }
	step_data_0_0_5_load { ap_none {  { step_data_0_0_5_load in_data 0 32 } } }
	step_data_0_1_0_load { ap_none {  { step_data_0_1_0_load in_data 0 32 } } }
	step_data_0_1_1_load { ap_none {  { step_data_0_1_1_load in_data 0 32 } } }
	step_data_0_1_2_load { ap_none {  { step_data_0_1_2_load in_data 0 32 } } }
	step_data_0_1_3_load { ap_none {  { step_data_0_1_3_load in_data 0 32 } } }
	step_data_0_1_4_load { ap_none {  { step_data_0_1_4_load in_data 0 32 } } }
	step_data_0_1_5_load { ap_none {  { step_data_0_1_5_load in_data 0 32 } } }
	step_data_0_2_0_load { ap_none {  { step_data_0_2_0_load in_data 0 32 } } }
	step_data_0_2_1_load { ap_none {  { step_data_0_2_1_load in_data 0 32 } } }
	step_data_0_2_2_load { ap_none {  { step_data_0_2_2_load in_data 0 32 } } }
	step_data_0_2_3_load { ap_none {  { step_data_0_2_3_load in_data 0 32 } } }
	step_data_0_2_4_load { ap_none {  { step_data_0_2_4_load in_data 0 32 } } }
	step_data_0_2_5_load { ap_none {  { step_data_0_2_5_load in_data 0 32 } } }
	step_data_0_3_0_load { ap_none {  { step_data_0_3_0_load in_data 0 32 } } }
	step_data_0_3_1_load { ap_none {  { step_data_0_3_1_load in_data 0 32 } } }
	step_data_0_3_2_load { ap_none {  { step_data_0_3_2_load in_data 0 32 } } }
	step_data_0_3_3_load { ap_none {  { step_data_0_3_3_load in_data 0 32 } } }
	step_data_0_3_4_load { ap_none {  { step_data_0_3_4_load in_data 0 32 } } }
	step_data_0_3_5_load { ap_none {  { step_data_0_3_5_load in_data 0 32 } } }
	step_data_0_4_0_load { ap_none {  { step_data_0_4_0_load in_data 0 32 } } }
	step_data_0_4_1_load { ap_none {  { step_data_0_4_1_load in_data 0 32 } } }
	step_data_0_4_2_load { ap_none {  { step_data_0_4_2_load in_data 0 32 } } }
	step_data_0_4_3_load { ap_none {  { step_data_0_4_3_load in_data 0 32 } } }
	step_data_0_4_4_load { ap_none {  { step_data_0_4_4_load in_data 0 32 } } }
	step_data_0_4_5_load { ap_none {  { step_data_0_4_5_load in_data 0 32 } } }
	step_data_0_5_0_load { ap_none {  { step_data_0_5_0_load in_data 0 32 } } }
	step_data_0_5_1_load { ap_none {  { step_data_0_5_1_load in_data 0 32 } } }
	step_data_0_5_2_load { ap_none {  { step_data_0_5_2_load in_data 0 32 } } }
	step_data_0_5_3_load { ap_none {  { step_data_0_5_3_load in_data 0 32 } } }
	step_data_0_5_4_load { ap_none {  { step_data_0_5_4_load in_data 0 32 } } }
	step_data_0_5_5_load { ap_none {  { step_data_0_5_5_load in_data 0 32 } } }
	G_13_out { ap_vld {  { G_13_out out_data 1 34 }  { G_13_out_ap_vld out_vld 1 1 } } }
	G_12_out { ap_vld {  { G_12_out out_data 1 34 }  { G_12_out_ap_vld out_vld 1 1 } } }
	G_11_out { ap_vld {  { G_11_out out_data 1 34 }  { G_11_out_ap_vld out_vld 1 1 } } }
	G_10_out { ap_vld {  { G_10_out out_data 1 34 }  { G_10_out_ap_vld out_vld 1 1 } } }
	G_9_out { ap_vld {  { G_9_out out_data 1 34 }  { G_9_out_ap_vld out_vld 1 1 } } }
	G_8_out { ap_vld {  { G_8_out out_data 1 34 }  { G_8_out_ap_vld out_vld 1 1 } } }
	G_7_out { ap_vld {  { G_7_out out_data 1 34 }  { G_7_out_ap_vld out_vld 1 1 } } }
	G_6_out { ap_vld {  { G_6_out out_data 1 34 }  { G_6_out_ap_vld out_vld 1 1 } } }
	G_5_out { ap_vld {  { G_5_out out_data 1 34 }  { G_5_out_ap_vld out_vld 1 1 } } }
	G_4_out { ap_vld {  { G_4_out out_data 1 34 }  { G_4_out_ap_vld out_vld 1 1 } } }
	G_3_out { ap_vld {  { G_3_out out_data 1 34 }  { G_3_out_ap_vld out_vld 1 1 } } }
	G_2_out { ap_vld {  { G_2_out out_data 1 34 }  { G_2_out_ap_vld out_vld 1 1 } } }
}
