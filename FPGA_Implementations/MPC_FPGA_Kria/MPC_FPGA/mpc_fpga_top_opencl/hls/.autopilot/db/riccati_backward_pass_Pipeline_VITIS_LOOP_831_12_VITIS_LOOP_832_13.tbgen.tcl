set moduleName riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13
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
set C_modelName {riccati_backward_pass_Pipeline_VITIS_LOOP_831_12_VITIS_LOOP_832_13}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict P { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_1 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_2 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_3 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_4 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_5 { MEM_WIDTH 40 MEM_SIZE 40 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict PA_5 { MEM_WIDTH 40 MEM_SIZE 30 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict PA_4 { MEM_WIDTH 40 MEM_SIZE 30 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict PA_3 { MEM_WIDTH 40 MEM_SIZE 30 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict PA_2 { MEM_WIDTH 40 MEM_SIZE 30 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict PA_1 { MEM_WIDTH 40 MEM_SIZE 30 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict PA { MEM_WIDTH 40 MEM_SIZE 30 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ P int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_1 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_2 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_3 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_4 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_5 int 40 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ PA_5 int 40 regular {array 6 { 3 0 } 0 1 bus  }  }
	{ PA_4 int 40 regular {array 6 { 3 0 } 0 1 bus  }  }
	{ PA_3 int 40 regular {array 6 { 3 0 } 0 1 bus  }  }
	{ PA_2 int 40 regular {array 6 { 3 0 } 0 1 bus  }  }
	{ PA_1 int 40 regular {array 6 { 3 0 } 0 1 bus  }  }
	{ PA int 40 regular {array 6 { 3 0 } 0 1 bus  }  }
	{ a_60 int 32 regular  }
	{ a_68 int 32 regular  }
	{ a_76 int 32 regular  }
	{ a_84 int 32 regular  }
	{ a_92 int 32 regular  }
	{ a_61 int 32 regular  }
	{ a_69 int 32 regular  }
	{ a_77 int 32 regular  }
	{ a_85 int 32 regular  }
	{ a_93 int 32 regular  }
	{ a_62 int 32 regular  }
	{ a_70 int 32 regular  }
	{ a_78 int 32 regular  }
	{ a_86 int 32 regular  }
	{ a_94 int 32 regular  }
	{ a_63 int 32 regular  }
	{ a_71 int 32 regular  }
	{ a_79 int 32 regular  }
	{ a_87 int 32 regular  }
	{ a_95 int 32 regular  }
	{ a_64 int 32 regular  }
	{ a_72 int 32 regular  }
	{ a_80 int 32 regular  }
	{ a_88 int 32 regular  }
	{ a_96 int 32 regular  }
	{ a_65 int 32 regular  }
	{ a_73 int 32 regular  }
	{ a_81 int 32 regular  }
	{ a_89 int 32 regular  }
	{ a_97 int 32 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "P", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "P_1", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "P_2", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "P_3", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "P_4", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "P_5", "interface" : "memory", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "PA_5", "interface" : "memory", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "PA_4", "interface" : "memory", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "PA_3", "interface" : "memory", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "PA_2", "interface" : "memory", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "PA_1", "interface" : "memory", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "PA", "interface" : "memory", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "a_60", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_68", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_76", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_84", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_92", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_61", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_69", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_77", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_85", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_93", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_62", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_70", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_78", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_86", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_94", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_63", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_71", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_79", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_87", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_95", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_64", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_72", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_80", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_88", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_96", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_65", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_73", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_81", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_89", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_97", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} ]}
# RTL Port declarations: 
set portNum 110
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
	{ P_1_address0 sc_out sc_lv 3 signal 1 } 
	{ P_1_ce0 sc_out sc_logic 1 signal 1 } 
	{ P_1_q0 sc_in sc_lv 40 signal 1 } 
	{ P_2_address0 sc_out sc_lv 3 signal 2 } 
	{ P_2_ce0 sc_out sc_logic 1 signal 2 } 
	{ P_2_q0 sc_in sc_lv 40 signal 2 } 
	{ P_3_address0 sc_out sc_lv 3 signal 3 } 
	{ P_3_ce0 sc_out sc_logic 1 signal 3 } 
	{ P_3_q0 sc_in sc_lv 40 signal 3 } 
	{ P_4_address0 sc_out sc_lv 3 signal 4 } 
	{ P_4_ce0 sc_out sc_logic 1 signal 4 } 
	{ P_4_q0 sc_in sc_lv 40 signal 4 } 
	{ P_5_address0 sc_out sc_lv 3 signal 5 } 
	{ P_5_ce0 sc_out sc_logic 1 signal 5 } 
	{ P_5_q0 sc_in sc_lv 40 signal 5 } 
	{ PA_5_address1 sc_out sc_lv 3 signal 6 } 
	{ PA_5_ce1 sc_out sc_logic 1 signal 6 } 
	{ PA_5_we1 sc_out sc_logic 1 signal 6 } 
	{ PA_5_d1 sc_out sc_lv 40 signal 6 } 
	{ PA_4_address1 sc_out sc_lv 3 signal 7 } 
	{ PA_4_ce1 sc_out sc_logic 1 signal 7 } 
	{ PA_4_we1 sc_out sc_logic 1 signal 7 } 
	{ PA_4_d1 sc_out sc_lv 40 signal 7 } 
	{ PA_3_address1 sc_out sc_lv 3 signal 8 } 
	{ PA_3_ce1 sc_out sc_logic 1 signal 8 } 
	{ PA_3_we1 sc_out sc_logic 1 signal 8 } 
	{ PA_3_d1 sc_out sc_lv 40 signal 8 } 
	{ PA_2_address1 sc_out sc_lv 3 signal 9 } 
	{ PA_2_ce1 sc_out sc_logic 1 signal 9 } 
	{ PA_2_we1 sc_out sc_logic 1 signal 9 } 
	{ PA_2_d1 sc_out sc_lv 40 signal 9 } 
	{ PA_1_address1 sc_out sc_lv 3 signal 10 } 
	{ PA_1_ce1 sc_out sc_logic 1 signal 10 } 
	{ PA_1_we1 sc_out sc_logic 1 signal 10 } 
	{ PA_1_d1 sc_out sc_lv 40 signal 10 } 
	{ PA_address1 sc_out sc_lv 3 signal 11 } 
	{ PA_ce1 sc_out sc_logic 1 signal 11 } 
	{ PA_we1 sc_out sc_logic 1 signal 11 } 
	{ PA_d1 sc_out sc_lv 40 signal 11 } 
	{ a_60 sc_in sc_lv 32 signal 12 } 
	{ a_68 sc_in sc_lv 32 signal 13 } 
	{ a_76 sc_in sc_lv 32 signal 14 } 
	{ a_84 sc_in sc_lv 32 signal 15 } 
	{ a_92 sc_in sc_lv 32 signal 16 } 
	{ a_61 sc_in sc_lv 32 signal 17 } 
	{ a_69 sc_in sc_lv 32 signal 18 } 
	{ a_77 sc_in sc_lv 32 signal 19 } 
	{ a_85 sc_in sc_lv 32 signal 20 } 
	{ a_93 sc_in sc_lv 32 signal 21 } 
	{ a_62 sc_in sc_lv 32 signal 22 } 
	{ a_70 sc_in sc_lv 32 signal 23 } 
	{ a_78 sc_in sc_lv 32 signal 24 } 
	{ a_86 sc_in sc_lv 32 signal 25 } 
	{ a_94 sc_in sc_lv 32 signal 26 } 
	{ a_63 sc_in sc_lv 32 signal 27 } 
	{ a_71 sc_in sc_lv 32 signal 28 } 
	{ a_79 sc_in sc_lv 32 signal 29 } 
	{ a_87 sc_in sc_lv 32 signal 30 } 
	{ a_95 sc_in sc_lv 32 signal 31 } 
	{ a_64 sc_in sc_lv 32 signal 32 } 
	{ a_72 sc_in sc_lv 32 signal 33 } 
	{ a_80 sc_in sc_lv 32 signal 34 } 
	{ a_88 sc_in sc_lv 32 signal 35 } 
	{ a_96 sc_in sc_lv 32 signal 36 } 
	{ a_65 sc_in sc_lv 32 signal 37 } 
	{ a_73 sc_in sc_lv 32 signal 38 } 
	{ a_81 sc_in sc_lv 32 signal 39 } 
	{ a_89 sc_in sc_lv 32 signal 40 } 
	{ a_97 sc_in sc_lv 32 signal 41 } 
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
 	{ "name": "P_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_1", "role": "address0" }} , 
 	{ "name": "P_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_1", "role": "ce0" }} , 
 	{ "name": "P_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_1", "role": "q0" }} , 
 	{ "name": "P_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_2", "role": "address0" }} , 
 	{ "name": "P_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_2", "role": "ce0" }} , 
 	{ "name": "P_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_2", "role": "q0" }} , 
 	{ "name": "P_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_3", "role": "address0" }} , 
 	{ "name": "P_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_3", "role": "ce0" }} , 
 	{ "name": "P_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_3", "role": "q0" }} , 
 	{ "name": "P_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_4", "role": "address0" }} , 
 	{ "name": "P_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_4", "role": "ce0" }} , 
 	{ "name": "P_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_4", "role": "q0" }} , 
 	{ "name": "P_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_5", "role": "address0" }} , 
 	{ "name": "P_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_5", "role": "ce0" }} , 
 	{ "name": "P_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "P_5", "role": "q0" }} , 
 	{ "name": "PA_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "PA_5", "role": "address1" }} , 
 	{ "name": "PA_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_5", "role": "ce1" }} , 
 	{ "name": "PA_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_5", "role": "we1" }} , 
 	{ "name": "PA_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "PA_5", "role": "d1" }} , 
 	{ "name": "PA_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "PA_4", "role": "address1" }} , 
 	{ "name": "PA_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_4", "role": "ce1" }} , 
 	{ "name": "PA_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_4", "role": "we1" }} , 
 	{ "name": "PA_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "PA_4", "role": "d1" }} , 
 	{ "name": "PA_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "PA_3", "role": "address1" }} , 
 	{ "name": "PA_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_3", "role": "ce1" }} , 
 	{ "name": "PA_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_3", "role": "we1" }} , 
 	{ "name": "PA_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "PA_3", "role": "d1" }} , 
 	{ "name": "PA_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "PA_2", "role": "address1" }} , 
 	{ "name": "PA_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_2", "role": "ce1" }} , 
 	{ "name": "PA_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_2", "role": "we1" }} , 
 	{ "name": "PA_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "PA_2", "role": "d1" }} , 
 	{ "name": "PA_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "PA_1", "role": "address1" }} , 
 	{ "name": "PA_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_1", "role": "ce1" }} , 
 	{ "name": "PA_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA_1", "role": "we1" }} , 
 	{ "name": "PA_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "PA_1", "role": "d1" }} , 
 	{ "name": "PA_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "PA", "role": "address1" }} , 
 	{ "name": "PA_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA", "role": "ce1" }} , 
 	{ "name": "PA_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "PA", "role": "we1" }} , 
 	{ "name": "PA_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "PA", "role": "d1" }} , 
 	{ "name": "a_60", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_60", "role": "default" }} , 
 	{ "name": "a_68", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_68", "role": "default" }} , 
 	{ "name": "a_76", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_76", "role": "default" }} , 
 	{ "name": "a_84", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_84", "role": "default" }} , 
 	{ "name": "a_92", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_92", "role": "default" }} , 
 	{ "name": "a_61", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_61", "role": "default" }} , 
 	{ "name": "a_69", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_69", "role": "default" }} , 
 	{ "name": "a_77", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_77", "role": "default" }} , 
 	{ "name": "a_85", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_85", "role": "default" }} , 
 	{ "name": "a_93", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_93", "role": "default" }} , 
 	{ "name": "a_62", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_62", "role": "default" }} , 
 	{ "name": "a_70", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_70", "role": "default" }} , 
 	{ "name": "a_78", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_78", "role": "default" }} , 
 	{ "name": "a_86", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_86", "role": "default" }} , 
 	{ "name": "a_94", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_94", "role": "default" }} , 
 	{ "name": "a_63", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_63", "role": "default" }} , 
 	{ "name": "a_71", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_71", "role": "default" }} , 
 	{ "name": "a_79", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_79", "role": "default" }} , 
 	{ "name": "a_87", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_87", "role": "default" }} , 
 	{ "name": "a_95", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_95", "role": "default" }} , 
 	{ "name": "a_64", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_64", "role": "default" }} , 
 	{ "name": "a_72", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_72", "role": "default" }} , 
 	{ "name": "a_80", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_80", "role": "default" }} , 
 	{ "name": "a_88", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_88", "role": "default" }} , 
 	{ "name": "a_96", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_96", "role": "default" }} , 
 	{ "name": "a_65", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_65", "role": "default" }} , 
 	{ "name": "a_73", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_73", "role": "default" }} , 
 	{ "name": "a_81", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_81", "role": "default" }} , 
 	{ "name": "a_89", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_89", "role": "default" }} , 
 	{ "name": "a_97", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_97", "role": "default" }} , 
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
		a_97 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "36", "Max" : "36"}
	, {"Name" : "Interval", "Min" : "31", "Max" : "31"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	P { ap_memory {  { P_address0 mem_address 1 3 }  { P_ce0 mem_ce 1 1 }  { P_q0 mem_dout 0 40 } } }
	P_1 { ap_memory {  { P_1_address0 mem_address 1 3 }  { P_1_ce0 mem_ce 1 1 }  { P_1_q0 mem_dout 0 40 } } }
	P_2 { ap_memory {  { P_2_address0 mem_address 1 3 }  { P_2_ce0 mem_ce 1 1 }  { P_2_q0 mem_dout 0 40 } } }
	P_3 { ap_memory {  { P_3_address0 mem_address 1 3 }  { P_3_ce0 mem_ce 1 1 }  { P_3_q0 mem_dout 0 40 } } }
	P_4 { ap_memory {  { P_4_address0 mem_address 1 3 }  { P_4_ce0 mem_ce 1 1 }  { P_4_q0 mem_dout 0 40 } } }
	P_5 { ap_memory {  { P_5_address0 mem_address 1 3 }  { P_5_ce0 mem_ce 1 1 }  { P_5_q0 mem_dout 0 40 } } }
	PA_5 { ap_memory {  { PA_5_address1 MemPortADDR2 1 3 }  { PA_5_ce1 MemPortCE2 1 1 }  { PA_5_we1 MemPortWE2 1 1 }  { PA_5_d1 MemPortDIN2 1 40 } } }
	PA_4 { ap_memory {  { PA_4_address1 MemPortADDR2 1 3 }  { PA_4_ce1 MemPortCE2 1 1 }  { PA_4_we1 MemPortWE2 1 1 }  { PA_4_d1 MemPortDIN2 1 40 } } }
	PA_3 { ap_memory {  { PA_3_address1 MemPortADDR2 1 3 }  { PA_3_ce1 MemPortCE2 1 1 }  { PA_3_we1 MemPortWE2 1 1 }  { PA_3_d1 MemPortDIN2 1 40 } } }
	PA_2 { ap_memory {  { PA_2_address1 MemPortADDR2 1 3 }  { PA_2_ce1 MemPortCE2 1 1 }  { PA_2_we1 MemPortWE2 1 1 }  { PA_2_d1 MemPortDIN2 1 40 } } }
	PA_1 { ap_memory {  { PA_1_address1 MemPortADDR2 1 3 }  { PA_1_ce1 MemPortCE2 1 1 }  { PA_1_we1 MemPortWE2 1 1 }  { PA_1_d1 MemPortDIN2 1 40 } } }
	PA { ap_memory {  { PA_address1 MemPortADDR2 1 3 }  { PA_ce1 MemPortCE2 1 1 }  { PA_we1 MemPortWE2 1 1 }  { PA_d1 MemPortDIN2 1 40 } } }
	a_60 { ap_none {  { a_60 in_data 0 32 } } }
	a_68 { ap_none {  { a_68 in_data 0 32 } } }
	a_76 { ap_none {  { a_76 in_data 0 32 } } }
	a_84 { ap_none {  { a_84 in_data 0 32 } } }
	a_92 { ap_none {  { a_92 in_data 0 32 } } }
	a_61 { ap_none {  { a_61 in_data 0 32 } } }
	a_69 { ap_none {  { a_69 in_data 0 32 } } }
	a_77 { ap_none {  { a_77 in_data 0 32 } } }
	a_85 { ap_none {  { a_85 in_data 0 32 } } }
	a_93 { ap_none {  { a_93 in_data 0 32 } } }
	a_62 { ap_none {  { a_62 in_data 0 32 } } }
	a_70 { ap_none {  { a_70 in_data 0 32 } } }
	a_78 { ap_none {  { a_78 in_data 0 32 } } }
	a_86 { ap_none {  { a_86 in_data 0 32 } } }
	a_94 { ap_none {  { a_94 in_data 0 32 } } }
	a_63 { ap_none {  { a_63 in_data 0 32 } } }
	a_71 { ap_none {  { a_71 in_data 0 32 } } }
	a_79 { ap_none {  { a_79 in_data 0 32 } } }
	a_87 { ap_none {  { a_87 in_data 0 32 } } }
	a_95 { ap_none {  { a_95 in_data 0 32 } } }
	a_64 { ap_none {  { a_64 in_data 0 32 } } }
	a_72 { ap_none {  { a_72 in_data 0 32 } } }
	a_80 { ap_none {  { a_80 in_data 0 32 } } }
	a_88 { ap_none {  { a_88 in_data 0 32 } } }
	a_96 { ap_none {  { a_96 in_data 0 32 } } }
	a_65 { ap_none {  { a_65 in_data 0 32 } } }
	a_73 { ap_none {  { a_73 in_data 0 32 } } }
	a_81 { ap_none {  { a_81 in_data 0 32 } } }
	a_89 { ap_none {  { a_89 in_data 0 32 } } }
	a_97 { ap_none {  { a_97 in_data 0 32 } } }
}
