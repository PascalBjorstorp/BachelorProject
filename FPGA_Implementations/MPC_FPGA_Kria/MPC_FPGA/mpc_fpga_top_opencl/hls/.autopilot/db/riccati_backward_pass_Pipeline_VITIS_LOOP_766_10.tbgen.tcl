set moduleName riccati_backward_pass_Pipeline_VITIS_LOOP_766_10
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
set C_modelName {riccati_backward_pass_Pipeline_VITIS_LOOP_766_10}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict P { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_1 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_2 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_3 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_4 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_5 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ P int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ sext_ln259 int 32 regular  }
	{ P_1 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ sext_ln259_1 int 32 regular  }
	{ P_2 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ sext_ln259_2 int 32 regular  }
	{ P_3 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ sext_ln259_3 int 32 regular  }
	{ P_4 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ sext_ln766 int 32 regular  }
	{ P_5 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ sext_ln766_1 int 32 regular  }
	{ p_2_r int 40 regular  }
	{ p_3_r int 40 regular  }
	{ p_4_r int 40 regular  }
	{ p_5_r int 40 regular  }
	{ p_6 int 40 regular  }
	{ p_7 int 40 regular  }
	{ p_8 int 40 regular  }
	{ p_9 int 40 regular  }
	{ p_shift_7_out int 40 regular {pointer 1}  }
	{ p_shift_6_out int 40 regular {pointer 1}  }
	{ p_shift_5_out int 40 regular {pointer 1}  }
	{ p_shift_4_out int 40 regular {pointer 1}  }
	{ p_shift_3_out int 40 regular {pointer 1}  }
	{ p_shift_2_out int 40 regular {pointer 1}  }
	{ p_shift_1_out int 34 regular {pointer 1}  }
	{ p_shift_out int 33 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "P", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln259", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "P_1", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln259_1", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "P_2", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln259_2", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "P_3", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln259_3", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "P_4", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln766", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "P_5", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln766_1", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_2_r", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_3_r", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_4_r", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_5_r", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_6", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_7", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_8", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_9", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "p_shift_7_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_shift_6_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_shift_5_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_shift_4_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_shift_3_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_shift_2_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_shift_1_out", "interface" : "wire", "bitwidth" : 34, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_shift_out", "interface" : "wire", "bitwidth" : 33, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 86
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ P_address0 sc_out sc_lv 3 signal 0 } 
	{ P_ce0 sc_out sc_logic 1 signal 0 } 
	{ P_q0 sc_in sc_lv 40 signal 0 } 
	{ sext_ln259 sc_in sc_lv 32 signal 1 } 
	{ P_1_address0 sc_out sc_lv 3 signal 2 } 
	{ P_1_ce0 sc_out sc_logic 1 signal 2 } 
	{ P_1_q0 sc_in sc_lv 40 signal 2 } 
	{ sext_ln259_1 sc_in sc_lv 32 signal 3 } 
	{ P_2_address0 sc_out sc_lv 3 signal 4 } 
	{ P_2_ce0 sc_out sc_logic 1 signal 4 } 
	{ P_2_q0 sc_in sc_lv 40 signal 4 } 
	{ sext_ln259_2 sc_in sc_lv 32 signal 5 } 
	{ P_3_address0 sc_out sc_lv 3 signal 6 } 
	{ P_3_ce0 sc_out sc_logic 1 signal 6 } 
	{ P_3_q0 sc_in sc_lv 40 signal 6 } 
	{ sext_ln259_3 sc_in sc_lv 32 signal 7 } 
	{ P_4_address0 sc_out sc_lv 3 signal 8 } 
	{ P_4_ce0 sc_out sc_logic 1 signal 8 } 
	{ P_4_q0 sc_in sc_lv 40 signal 8 } 
	{ sext_ln766 sc_in sc_lv 32 signal 9 } 
	{ P_5_address0 sc_out sc_lv 3 signal 10 } 
	{ P_5_ce0 sc_out sc_logic 1 signal 10 } 
	{ P_5_q0 sc_in sc_lv 40 signal 10 } 
	{ sext_ln766_1 sc_in sc_lv 32 signal 11 } 
	{ p_2_r sc_in sc_lv 40 signal 12 } 
	{ p_3_r sc_in sc_lv 40 signal 13 } 
	{ p_4_r sc_in sc_lv 40 signal 14 } 
	{ p_5_r sc_in sc_lv 40 signal 15 } 
	{ p_6 sc_in sc_lv 40 signal 16 } 
	{ p_7 sc_in sc_lv 40 signal 17 } 
	{ p_8 sc_in sc_lv 40 signal 18 } 
	{ p_9 sc_in sc_lv 40 signal 19 } 
	{ p_shift_7_out sc_out sc_lv 40 signal 20 } 
	{ p_shift_7_out_ap_vld sc_out sc_logic 1 outvld 20 } 
	{ p_shift_6_out sc_out sc_lv 40 signal 21 } 
	{ p_shift_6_out_ap_vld sc_out sc_logic 1 outvld 21 } 
	{ p_shift_5_out sc_out sc_lv 40 signal 22 } 
	{ p_shift_5_out_ap_vld sc_out sc_logic 1 outvld 22 } 
	{ p_shift_4_out sc_out sc_lv 40 signal 23 } 
	{ p_shift_4_out_ap_vld sc_out sc_logic 1 outvld 23 } 
	{ p_shift_3_out sc_out sc_lv 40 signal 24 } 
	{ p_shift_3_out_ap_vld sc_out sc_logic 1 outvld 24 } 
	{ p_shift_2_out sc_out sc_lv 40 signal 25 } 
	{ p_shift_2_out_ap_vld sc_out sc_logic 1 outvld 25 } 
	{ p_shift_1_out sc_out sc_lv 34 signal 26 } 
	{ p_shift_1_out_ap_vld sc_out sc_logic 1 outvld 26 } 
	{ p_shift_out sc_out sc_lv 33 signal 27 } 
	{ p_shift_out_ap_vld sc_out sc_logic 1 outvld 27 } 
	{ grp_fu_5073_p_din0 sc_out sc_lv 40 signal -1 } 
	{ grp_fu_5073_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_5073_p_dout0 sc_in sc_lv 58 signal -1 } 
	{ grp_fu_5073_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_5077_p_din0 sc_out sc_lv 40 signal -1 } 
	{ grp_fu_5077_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_5077_p_dout0 sc_in sc_lv 58 signal -1 } 
	{ grp_fu_5077_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_5081_p_din0 sc_out sc_lv 40 signal -1 } 
	{ grp_fu_5081_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_5081_p_dout0 sc_in sc_lv 58 signal -1 } 
	{ grp_fu_5081_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_5085_p_din0 sc_out sc_lv 40 signal -1 } 
	{ grp_fu_5085_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_5085_p_dout0 sc_in sc_lv 58 signal -1 } 
	{ grp_fu_5085_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_5089_p_din0 sc_out sc_lv 40 signal -1 } 
	{ grp_fu_5089_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_5089_p_dout0 sc_in sc_lv 58 signal -1 } 
	{ grp_fu_5089_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_5093_p_din0 sc_out sc_lv 40 signal -1 } 
	{ grp_fu_5093_p_din1 sc_out sc_lv 32 signal -1 } 
	{ grp_fu_5093_p_dout0 sc_in sc_lv 58 signal -1 } 
	{ grp_fu_5093_p_ce sc_out sc_logic 1 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_din1 sc_out sc_lv 58 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_din2 sc_out sc_lv 58 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_din3 sc_out sc_lv 58 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_din4 sc_out sc_lv 58 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_din5 sc_out sc_lv 58 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_din6 sc_out sc_lv 58 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_dout0 sc_in sc_lv 58 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_13849_p_ready sc_in sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "P_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P", "role": "address0" }} , 
 	{ "name": "P_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P", "role": "ce0" }} , 
 	{ "name": "P_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P", "role": "q0" }} , 
 	{ "name": "sext_ln259", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln259", "role": "default" }} , 
 	{ "name": "P_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_1", "role": "address0" }} , 
 	{ "name": "P_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_1", "role": "ce0" }} , 
 	{ "name": "P_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_1", "role": "q0" }} , 
 	{ "name": "sext_ln259_1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln259_1", "role": "default" }} , 
 	{ "name": "P_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_2", "role": "address0" }} , 
 	{ "name": "P_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_2", "role": "ce0" }} , 
 	{ "name": "P_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_2", "role": "q0" }} , 
 	{ "name": "sext_ln259_2", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln259_2", "role": "default" }} , 
 	{ "name": "P_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_3", "role": "address0" }} , 
 	{ "name": "P_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_3", "role": "ce0" }} , 
 	{ "name": "P_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_3", "role": "q0" }} , 
 	{ "name": "sext_ln259_3", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln259_3", "role": "default" }} , 
 	{ "name": "P_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_4", "role": "address0" }} , 
 	{ "name": "P_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_4", "role": "ce0" }} , 
 	{ "name": "P_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_4", "role": "q0" }} , 
 	{ "name": "sext_ln766", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln766", "role": "default" }} , 
 	{ "name": "P_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_5", "role": "address0" }} , 
 	{ "name": "P_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_5", "role": "ce0" }} , 
 	{ "name": "P_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_5", "role": "q0" }} , 
 	{ "name": "sext_ln766_1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln766_1", "role": "default" }} , 
 	{ "name": "p_2_r", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_2_r", "role": "default" }} , 
 	{ "name": "p_3_r", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_3_r", "role": "default" }} , 
 	{ "name": "p_4_r", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_4_r", "role": "default" }} , 
 	{ "name": "p_5_r", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_5_r", "role": "default" }} , 
 	{ "name": "p_6", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_6", "role": "default" }} , 
 	{ "name": "p_7", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_7", "role": "default" }} , 
 	{ "name": "p_8", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_8", "role": "default" }} , 
 	{ "name": "p_9", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_9", "role": "default" }} , 
 	{ "name": "p_shift_7_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_shift_7_out", "role": "default" }} , 
 	{ "name": "p_shift_7_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_7_out", "role": "ap_vld" }} , 
 	{ "name": "p_shift_6_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_shift_6_out", "role": "default" }} , 
 	{ "name": "p_shift_6_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_6_out", "role": "ap_vld" }} , 
 	{ "name": "p_shift_5_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_shift_5_out", "role": "default" }} , 
 	{ "name": "p_shift_5_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_5_out", "role": "ap_vld" }} , 
 	{ "name": "p_shift_4_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_shift_4_out", "role": "default" }} , 
 	{ "name": "p_shift_4_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_4_out", "role": "ap_vld" }} , 
 	{ "name": "p_shift_3_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_shift_3_out", "role": "default" }} , 
 	{ "name": "p_shift_3_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_3_out", "role": "ap_vld" }} , 
 	{ "name": "p_shift_2_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_shift_2_out", "role": "default" }} , 
 	{ "name": "p_shift_2_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_2_out", "role": "ap_vld" }} , 
 	{ "name": "p_shift_1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "p_shift_1_out", "role": "default" }} , 
 	{ "name": "p_shift_1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_1_out", "role": "ap_vld" }} , 
 	{ "name": "p_shift_out", "direction": "out", "datatype": "sc_lv", "bitwidth":33, "type": "signal", "bundle":{"name": "p_shift_out", "role": "default" }} , 
 	{ "name": "p_shift_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_shift_out", "role": "ap_vld" }} , 
 	{ "name": "grp_fu_5073_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5073_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5073_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_5073_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5073_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "grp_fu_5073_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5073_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5073_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_5077_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5077_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5077_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_5077_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5077_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "grp_fu_5077_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5077_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5077_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_5081_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5081_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5081_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_5081_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5081_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "grp_fu_5081_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5081_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5081_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_5085_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5085_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5085_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_5085_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5085_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "grp_fu_5085_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5085_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5085_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_5089_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5089_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5089_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_5089_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5089_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "grp_fu_5089_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5089_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5089_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_5093_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5093_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5093_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "grp_fu_5093_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5093_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "grp_fu_5093_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5093_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5093_p_ce", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din1", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din2", "direction": "out", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din2", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din3", "direction": "out", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din3", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din4", "direction": "out", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din4", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din5", "direction": "out", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din5", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din6", "direction": "out", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_din6", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_dout0", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_13849_p_ready", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_13849_p_ready", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		p_shift_out {Type O LastRead -1 FirstWrite 5}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "15", "Max" : "15"}
	, {"Name" : "Interval", "Min" : "9", "Max" : "9"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	P { ap_memory {  { P_address0 mem_address 1 3 }  { P_ce0 mem_ce 1 1 }  { P_q0 mem_dout 0 40 } } }
	sext_ln259 { ap_none {  { sext_ln259 in_data 0 32 } } }
	P_1 { ap_memory {  { P_1_address0 mem_address 1 3 }  { P_1_ce0 mem_ce 1 1 }  { P_1_q0 mem_dout 0 40 } } }
	sext_ln259_1 { ap_none {  { sext_ln259_1 in_data 0 32 } } }
	P_2 { ap_memory {  { P_2_address0 mem_address 1 3 }  { P_2_ce0 mem_ce 1 1 }  { P_2_q0 mem_dout 0 40 } } }
	sext_ln259_2 { ap_none {  { sext_ln259_2 in_data 0 32 } } }
	P_3 { ap_memory {  { P_3_address0 mem_address 1 3 }  { P_3_ce0 mem_ce 1 1 }  { P_3_q0 mem_dout 0 40 } } }
	sext_ln259_3 { ap_none {  { sext_ln259_3 in_data 0 32 } } }
	P_4 { ap_memory {  { P_4_address0 mem_address 1 3 }  { P_4_ce0 mem_ce 1 1 }  { P_4_q0 mem_dout 0 40 } } }
	sext_ln766 { ap_none {  { sext_ln766 in_data 0 32 } } }
	P_5 { ap_memory {  { P_5_address0 mem_address 1 3 }  { P_5_ce0 mem_ce 1 1 }  { P_5_q0 mem_dout 0 40 } } }
	sext_ln766_1 { ap_none {  { sext_ln766_1 in_data 0 32 } } }
	p_2_r { ap_none {  { p_2_r in_data 0 40 } } }
	p_3_r { ap_none {  { p_3_r in_data 0 40 } } }
	p_4_r { ap_none {  { p_4_r in_data 0 40 } } }
	p_5_r { ap_none {  { p_5_r in_data 0 40 } } }
	p_6 { ap_none {  { p_6 in_data 0 40 } } }
	p_7 { ap_none {  { p_7 in_data 0 40 } } }
	p_8 { ap_none {  { p_8 in_data 0 40 } } }
	p_9 { ap_none {  { p_9 in_data 0 40 } } }
	p_shift_7_out { ap_vld {  { p_shift_7_out out_data 1 40 }  { p_shift_7_out_ap_vld out_vld 1 1 } } }
	p_shift_6_out { ap_vld {  { p_shift_6_out out_data 1 40 }  { p_shift_6_out_ap_vld out_vld 1 1 } } }
	p_shift_5_out { ap_vld {  { p_shift_5_out out_data 1 40 }  { p_shift_5_out_ap_vld out_vld 1 1 } } }
	p_shift_4_out { ap_vld {  { p_shift_4_out out_data 1 40 }  { p_shift_4_out_ap_vld out_vld 1 1 } } }
	p_shift_3_out { ap_vld {  { p_shift_3_out out_data 1 40 }  { p_shift_3_out_ap_vld out_vld 1 1 } } }
	p_shift_2_out { ap_vld {  { p_shift_2_out out_data 1 40 }  { p_shift_2_out_ap_vld out_vld 1 1 } } }
	p_shift_1_out { ap_vld {  { p_shift_1_out out_data 1 34 }  { p_shift_1_out_ap_vld out_vld 1 1 } } }
	p_shift_out { ap_vld {  { p_shift_out out_data 1 33 }  { p_shift_out_ap_vld out_vld 1 1 } } }
}
