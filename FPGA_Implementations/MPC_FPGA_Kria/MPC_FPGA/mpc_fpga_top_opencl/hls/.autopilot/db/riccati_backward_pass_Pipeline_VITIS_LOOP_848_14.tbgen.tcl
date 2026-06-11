set moduleName riccati_backward_pass_Pipeline_VITIS_LOOP_848_14
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
set C_modelName {riccati_backward_pass_Pipeline_VITIS_LOOP_848_14}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict P { MEM_WIDTH 27 MEM_SIZE 32 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_1 { MEM_WIDTH 27 MEM_SIZE 32 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_2 { MEM_WIDTH 27 MEM_SIZE 32 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_3 { MEM_WIDTH 27 MEM_SIZE 32 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_4 { MEM_WIDTH 27 MEM_SIZE 32 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict P_5 { MEM_WIDTH 27 MEM_SIZE 32 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ PA_5_load_4 int 27 regular  }
	{ PA_5_load_3 int 27 regular  }
	{ PA_5_load_2 int 27 regular  }
	{ PA_5_load_1 int 27 regular  }
	{ PA_5_load int 27 regular  }
	{ PA_load_4 int 27 regular  }
	{ PA_load_3 int 27 regular  }
	{ PA_load_2 int 27 regular  }
	{ PA_load_1 int 27 regular  }
	{ PA_load int 27 regular  }
	{ PA_1_load_4 int 27 regular  }
	{ PA_1_load_3 int 27 regular  }
	{ PA_1_load_2 int 27 regular  }
	{ PA_1_load_1 int 27 regular  }
	{ PA_1_load int 27 regular  }
	{ PA_2_load_4 int 27 regular  }
	{ PA_2_load_3 int 27 regular  }
	{ PA_2_load_2 int 27 regular  }
	{ PA_2_load_1 int 27 regular  }
	{ PA_2_load int 27 regular  }
	{ PA_3_load_4 int 27 regular  }
	{ PA_3_load_3 int 27 regular  }
	{ PA_3_load_2 int 27 regular  }
	{ PA_3_load_1 int 27 regular  }
	{ PA_3_load int 27 regular  }
	{ PA_4_load_4 int 27 regular  }
	{ PA_4_load_3 int 27 regular  }
	{ PA_4_load_2 int 27 regular  }
	{ PA_4_load_1 int 27 regular  }
	{ PA_4_load int 27 regular  }
	{ P int 27 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_1 int 27 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_2 int 27 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_3 int 27 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_4 int 27 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ P_5 int 27 regular {array 8 { 1 3 } 1 1 bus  }  }
	{ sext_ln256_5 int 26 regular  }
	{ sext_ln256_7 int 26 regular  }
	{ sext_ln256_9 int 26 regular  }
	{ sext_ln256_11 int 26 regular  }
	{ sext_ln256_13 int 26 regular  }
	{ sext_ln256_15 int 26 regular  }
	{ sext_ln256_17 int 26 regular  }
	{ sext_ln256_19 int 26 regular  }
	{ sext_ln256_21 int 26 regular  }
	{ sext_ln256_23 int 26 regular  }
	{ sext_ln256_25 int 26 regular  }
	{ sext_ln256_27 int 26 regular  }
	{ sext_ln256_29 int 26 regular  }
	{ sext_ln256_31 int 26 regular  }
	{ sext_ln256_33 int 26 regular  }
	{ sext_ln256_35 int 26 regular  }
	{ sext_ln256_37 int 26 regular  }
	{ sext_ln256_39 int 26 regular  }
	{ sext_ln256_41 int 26 regular  }
	{ sext_ln256_43 int 26 regular  }
	{ sext_ln256_45 int 26 regular  }
	{ sext_ln256_47 int 26 regular  }
	{ sext_ln256_49 int 26 regular  }
	{ sext_ln256_51 int 26 regular  }
	{ sext_ln256_53 int 26 regular  }
	{ sext_ln256_55 int 26 regular  }
	{ sext_ln256_57 int 26 regular  }
	{ sext_ln256_59 int 26 regular  }
	{ sext_ln256_61 int 26 regular  }
	{ sext_ln848 int 26 regular  }
	{ conv_i_i13988_459_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_357_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_255_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_153_out int 27 regular {pointer 1}  }
	{ conv_i_i1398851_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_449_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_347_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_245_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_143_out int 27 regular {pointer 1}  }
	{ conv_i_i1398841_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_439_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_337_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_235_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_133_out int 27 regular {pointer 1}  }
	{ conv_i_i1398831_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_429_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_327_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_225_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_123_out int 27 regular {pointer 1}  }
	{ conv_i_i1398821_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_419_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_317_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_215_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_113_out int 27 regular {pointer 1}  }
	{ conv_i_i1398811_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_49_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_37_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_25_out int 27 regular {pointer 1}  }
	{ conv_i_i13988_13_out int 27 regular {pointer 1}  }
	{ conv_i_i139881_out int 27 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "PA_5_load_4", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_5_load_3", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_5_load_2", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_5_load_1", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_5_load", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_load_4", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_load_3", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_load_2", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_load_1", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_load", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_1_load_4", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_1_load_3", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_1_load_2", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_1_load_1", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_1_load", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_2_load_4", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_2_load_3", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_2_load_2", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_2_load_1", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_2_load", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_3_load_4", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_3_load_3", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_3_load_2", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_3_load_1", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_3_load", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_4_load_4", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_4_load_3", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_4_load_2", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_4_load_1", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "PA_4_load", "interface" : "wire", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "P", "interface" : "memory", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "P_1", "interface" : "memory", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "P_2", "interface" : "memory", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "P_3", "interface" : "memory", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "P_4", "interface" : "memory", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "P_5", "interface" : "memory", "bitwidth" : 27, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_5", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_7", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_9", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_11", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_13", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_15", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_17", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_19", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_21", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_23", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_25", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_27", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_29", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_31", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_33", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_35", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_37", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_39", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_41", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_43", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_45", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_47", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_49", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_51", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_53", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_55", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_57", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_59", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln256_61", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln848", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "conv_i_i13988_459_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_357_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_255_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_153_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i1398851_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_449_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_347_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_245_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_143_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i1398841_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_439_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_337_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_235_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_133_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i1398831_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_429_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_327_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_225_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_123_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i1398821_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_419_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_317_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_215_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_113_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i1398811_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_49_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_37_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_25_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i13988_13_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} , 
 	{ "Name" : "conv_i_i139881_out", "interface" : "wire", "bitwidth" : 27, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 176
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ PA_5_load_4 sc_in sc_lv 27 signal 0 } 
	{ PA_5_load_3 sc_in sc_lv 27 signal 1 } 
	{ PA_5_load_2 sc_in sc_lv 27 signal 2 } 
	{ PA_5_load_1 sc_in sc_lv 27 signal 3 } 
	{ PA_5_load sc_in sc_lv 27 signal 4 } 
	{ PA_load_4 sc_in sc_lv 27 signal 5 } 
	{ PA_load_3 sc_in sc_lv 27 signal 6 } 
	{ PA_load_2 sc_in sc_lv 27 signal 7 } 
	{ PA_load_1 sc_in sc_lv 27 signal 8 } 
	{ PA_load sc_in sc_lv 27 signal 9 } 
	{ PA_1_load_4 sc_in sc_lv 27 signal 10 } 
	{ PA_1_load_3 sc_in sc_lv 27 signal 11 } 
	{ PA_1_load_2 sc_in sc_lv 27 signal 12 } 
	{ PA_1_load_1 sc_in sc_lv 27 signal 13 } 
	{ PA_1_load sc_in sc_lv 27 signal 14 } 
	{ PA_2_load_4 sc_in sc_lv 27 signal 15 } 
	{ PA_2_load_3 sc_in sc_lv 27 signal 16 } 
	{ PA_2_load_2 sc_in sc_lv 27 signal 17 } 
	{ PA_2_load_1 sc_in sc_lv 27 signal 18 } 
	{ PA_2_load sc_in sc_lv 27 signal 19 } 
	{ PA_3_load_4 sc_in sc_lv 27 signal 20 } 
	{ PA_3_load_3 sc_in sc_lv 27 signal 21 } 
	{ PA_3_load_2 sc_in sc_lv 27 signal 22 } 
	{ PA_3_load_1 sc_in sc_lv 27 signal 23 } 
	{ PA_3_load sc_in sc_lv 27 signal 24 } 
	{ PA_4_load_4 sc_in sc_lv 27 signal 25 } 
	{ PA_4_load_3 sc_in sc_lv 27 signal 26 } 
	{ PA_4_load_2 sc_in sc_lv 27 signal 27 } 
	{ PA_4_load_1 sc_in sc_lv 27 signal 28 } 
	{ PA_4_load sc_in sc_lv 27 signal 29 } 
	{ P_address0 sc_out sc_lv 3 signal 30 } 
	{ P_ce0 sc_out sc_logic 1 signal 30 } 
	{ P_q0 sc_in sc_lv 27 signal 30 } 
	{ P_1_address0 sc_out sc_lv 3 signal 31 } 
	{ P_1_ce0 sc_out sc_logic 1 signal 31 } 
	{ P_1_q0 sc_in sc_lv 27 signal 31 } 
	{ P_2_address0 sc_out sc_lv 3 signal 32 } 
	{ P_2_ce0 sc_out sc_logic 1 signal 32 } 
	{ P_2_q0 sc_in sc_lv 27 signal 32 } 
	{ P_3_address0 sc_out sc_lv 3 signal 33 } 
	{ P_3_ce0 sc_out sc_logic 1 signal 33 } 
	{ P_3_q0 sc_in sc_lv 27 signal 33 } 
	{ P_4_address0 sc_out sc_lv 3 signal 34 } 
	{ P_4_ce0 sc_out sc_logic 1 signal 34 } 
	{ P_4_q0 sc_in sc_lv 27 signal 34 } 
	{ P_5_address0 sc_out sc_lv 3 signal 35 } 
	{ P_5_ce0 sc_out sc_logic 1 signal 35 } 
	{ P_5_q0 sc_in sc_lv 27 signal 35 } 
	{ sext_ln256_5 sc_in sc_lv 26 signal 36 } 
	{ sext_ln256_7 sc_in sc_lv 26 signal 37 } 
	{ sext_ln256_9 sc_in sc_lv 26 signal 38 } 
	{ sext_ln256_11 sc_in sc_lv 26 signal 39 } 
	{ sext_ln256_13 sc_in sc_lv 26 signal 40 } 
	{ sext_ln256_15 sc_in sc_lv 26 signal 41 } 
	{ sext_ln256_17 sc_in sc_lv 26 signal 42 } 
	{ sext_ln256_19 sc_in sc_lv 26 signal 43 } 
	{ sext_ln256_21 sc_in sc_lv 26 signal 44 } 
	{ sext_ln256_23 sc_in sc_lv 26 signal 45 } 
	{ sext_ln256_25 sc_in sc_lv 26 signal 46 } 
	{ sext_ln256_27 sc_in sc_lv 26 signal 47 } 
	{ sext_ln256_29 sc_in sc_lv 26 signal 48 } 
	{ sext_ln256_31 sc_in sc_lv 26 signal 49 } 
	{ sext_ln256_33 sc_in sc_lv 26 signal 50 } 
	{ sext_ln256_35 sc_in sc_lv 26 signal 51 } 
	{ sext_ln256_37 sc_in sc_lv 26 signal 52 } 
	{ sext_ln256_39 sc_in sc_lv 26 signal 53 } 
	{ sext_ln256_41 sc_in sc_lv 26 signal 54 } 
	{ sext_ln256_43 sc_in sc_lv 26 signal 55 } 
	{ sext_ln256_45 sc_in sc_lv 26 signal 56 } 
	{ sext_ln256_47 sc_in sc_lv 26 signal 57 } 
	{ sext_ln256_49 sc_in sc_lv 26 signal 58 } 
	{ sext_ln256_51 sc_in sc_lv 26 signal 59 } 
	{ sext_ln256_53 sc_in sc_lv 26 signal 60 } 
	{ sext_ln256_55 sc_in sc_lv 26 signal 61 } 
	{ sext_ln256_57 sc_in sc_lv 26 signal 62 } 
	{ sext_ln256_59 sc_in sc_lv 26 signal 63 } 
	{ sext_ln256_61 sc_in sc_lv 26 signal 64 } 
	{ sext_ln848 sc_in sc_lv 26 signal 65 } 
	{ conv_i_i13988_459_out sc_out sc_lv 27 signal 66 } 
	{ conv_i_i13988_459_out_ap_vld sc_out sc_logic 1 outvld 66 } 
	{ conv_i_i13988_357_out sc_out sc_lv 27 signal 67 } 
	{ conv_i_i13988_357_out_ap_vld sc_out sc_logic 1 outvld 67 } 
	{ conv_i_i13988_255_out sc_out sc_lv 27 signal 68 } 
	{ conv_i_i13988_255_out_ap_vld sc_out sc_logic 1 outvld 68 } 
	{ conv_i_i13988_153_out sc_out sc_lv 27 signal 69 } 
	{ conv_i_i13988_153_out_ap_vld sc_out sc_logic 1 outvld 69 } 
	{ conv_i_i1398851_out sc_out sc_lv 27 signal 70 } 
	{ conv_i_i1398851_out_ap_vld sc_out sc_logic 1 outvld 70 } 
	{ conv_i_i13988_449_out sc_out sc_lv 27 signal 71 } 
	{ conv_i_i13988_449_out_ap_vld sc_out sc_logic 1 outvld 71 } 
	{ conv_i_i13988_347_out sc_out sc_lv 27 signal 72 } 
	{ conv_i_i13988_347_out_ap_vld sc_out sc_logic 1 outvld 72 } 
	{ conv_i_i13988_245_out sc_out sc_lv 27 signal 73 } 
	{ conv_i_i13988_245_out_ap_vld sc_out sc_logic 1 outvld 73 } 
	{ conv_i_i13988_143_out sc_out sc_lv 27 signal 74 } 
	{ conv_i_i13988_143_out_ap_vld sc_out sc_logic 1 outvld 74 } 
	{ conv_i_i1398841_out sc_out sc_lv 27 signal 75 } 
	{ conv_i_i1398841_out_ap_vld sc_out sc_logic 1 outvld 75 } 
	{ conv_i_i13988_439_out sc_out sc_lv 27 signal 76 } 
	{ conv_i_i13988_439_out_ap_vld sc_out sc_logic 1 outvld 76 } 
	{ conv_i_i13988_337_out sc_out sc_lv 27 signal 77 } 
	{ conv_i_i13988_337_out_ap_vld sc_out sc_logic 1 outvld 77 } 
	{ conv_i_i13988_235_out sc_out sc_lv 27 signal 78 } 
	{ conv_i_i13988_235_out_ap_vld sc_out sc_logic 1 outvld 78 } 
	{ conv_i_i13988_133_out sc_out sc_lv 27 signal 79 } 
	{ conv_i_i13988_133_out_ap_vld sc_out sc_logic 1 outvld 79 } 
	{ conv_i_i1398831_out sc_out sc_lv 27 signal 80 } 
	{ conv_i_i1398831_out_ap_vld sc_out sc_logic 1 outvld 80 } 
	{ conv_i_i13988_429_out sc_out sc_lv 27 signal 81 } 
	{ conv_i_i13988_429_out_ap_vld sc_out sc_logic 1 outvld 81 } 
	{ conv_i_i13988_327_out sc_out sc_lv 27 signal 82 } 
	{ conv_i_i13988_327_out_ap_vld sc_out sc_logic 1 outvld 82 } 
	{ conv_i_i13988_225_out sc_out sc_lv 27 signal 83 } 
	{ conv_i_i13988_225_out_ap_vld sc_out sc_logic 1 outvld 83 } 
	{ conv_i_i13988_123_out sc_out sc_lv 27 signal 84 } 
	{ conv_i_i13988_123_out_ap_vld sc_out sc_logic 1 outvld 84 } 
	{ conv_i_i1398821_out sc_out sc_lv 27 signal 85 } 
	{ conv_i_i1398821_out_ap_vld sc_out sc_logic 1 outvld 85 } 
	{ conv_i_i13988_419_out sc_out sc_lv 27 signal 86 } 
	{ conv_i_i13988_419_out_ap_vld sc_out sc_logic 1 outvld 86 } 
	{ conv_i_i13988_317_out sc_out sc_lv 27 signal 87 } 
	{ conv_i_i13988_317_out_ap_vld sc_out sc_logic 1 outvld 87 } 
	{ conv_i_i13988_215_out sc_out sc_lv 27 signal 88 } 
	{ conv_i_i13988_215_out_ap_vld sc_out sc_logic 1 outvld 88 } 
	{ conv_i_i13988_113_out sc_out sc_lv 27 signal 89 } 
	{ conv_i_i13988_113_out_ap_vld sc_out sc_logic 1 outvld 89 } 
	{ conv_i_i1398811_out sc_out sc_lv 27 signal 90 } 
	{ conv_i_i1398811_out_ap_vld sc_out sc_logic 1 outvld 90 } 
	{ conv_i_i13988_49_out sc_out sc_lv 27 signal 91 } 
	{ conv_i_i13988_49_out_ap_vld sc_out sc_logic 1 outvld 91 } 
	{ conv_i_i13988_37_out sc_out sc_lv 27 signal 92 } 
	{ conv_i_i13988_37_out_ap_vld sc_out sc_logic 1 outvld 92 } 
	{ conv_i_i13988_25_out sc_out sc_lv 27 signal 93 } 
	{ conv_i_i13988_25_out_ap_vld sc_out sc_logic 1 outvld 93 } 
	{ conv_i_i13988_13_out sc_out sc_lv 27 signal 94 } 
	{ conv_i_i13988_13_out_ap_vld sc_out sc_logic 1 outvld 94 } 
	{ conv_i_i139881_out sc_out sc_lv 27 signal 95 } 
	{ conv_i_i139881_out_ap_vld sc_out sc_logic 1 outvld 95 } 
	{ grp_fu_15628_p_din0 sc_out sc_lv 27 signal -1 } 
	{ grp_fu_15628_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15628_p_dout0 sc_in sc_lv 43 signal -1 } 
	{ grp_fu_15628_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_15632_p_din0 sc_out sc_lv 27 signal -1 } 
	{ grp_fu_15632_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15632_p_dout0 sc_in sc_lv 43 signal -1 } 
	{ grp_fu_15632_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_15636_p_din0 sc_out sc_lv 27 signal -1 } 
	{ grp_fu_15636_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15636_p_dout0 sc_in sc_lv 43 signal -1 } 
	{ grp_fu_15636_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_15640_p_din0 sc_out sc_lv 27 signal -1 } 
	{ grp_fu_15640_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15640_p_dout0 sc_in sc_lv 43 signal -1 } 
	{ grp_fu_15640_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_15644_p_din0 sc_out sc_lv 27 signal -1 } 
	{ grp_fu_15644_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15644_p_dout0 sc_in sc_lv 43 signal -1 } 
	{ grp_fu_15644_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_15648_p_din0 sc_out sc_lv 27 signal -1 } 
	{ grp_fu_15648_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15648_p_dout0 sc_in sc_lv 43 signal -1 } 
	{ grp_fu_15648_p_ce sc_out sc_logic 1 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_din1 sc_out sc_lv 41 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_din2 sc_out sc_lv 41 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_din3 sc_out sc_lv 41 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_din4 sc_out sc_lv 41 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_din5 sc_out sc_lv 41 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_din6 sc_out sc_lv 41 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_dout0 sc_in sc_lv 41 signal -1 } 
	{ pd_sum_sum6_P_QP_raw_fu_15652_p_ready sc_in sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "PA_5_load_4", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_5_load_4", "role": "default" }} , 
 	{ "name": "PA_5_load_3", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_5_load_3", "role": "default" }} , 
 	{ "name": "PA_5_load_2", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_5_load_2", "role": "default" }} , 
 	{ "name": "PA_5_load_1", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_5_load_1", "role": "default" }} , 
 	{ "name": "PA_5_load", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_5_load", "role": "default" }} , 
 	{ "name": "PA_load_4", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_load_4", "role": "default" }} , 
 	{ "name": "PA_load_3", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_load_3", "role": "default" }} , 
 	{ "name": "PA_load_2", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_load_2", "role": "default" }} , 
 	{ "name": "PA_load_1", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_load_1", "role": "default" }} , 
 	{ "name": "PA_load", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_load", "role": "default" }} , 
 	{ "name": "PA_1_load_4", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_1_load_4", "role": "default" }} , 
 	{ "name": "PA_1_load_3", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_1_load_3", "role": "default" }} , 
 	{ "name": "PA_1_load_2", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_1_load_2", "role": "default" }} , 
 	{ "name": "PA_1_load_1", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_1_load_1", "role": "default" }} , 
 	{ "name": "PA_1_load", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_1_load", "role": "default" }} , 
 	{ "name": "PA_2_load_4", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_2_load_4", "role": "default" }} , 
 	{ "name": "PA_2_load_3", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_2_load_3", "role": "default" }} , 
 	{ "name": "PA_2_load_2", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_2_load_2", "role": "default" }} , 
 	{ "name": "PA_2_load_1", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_2_load_1", "role": "default" }} , 
 	{ "name": "PA_2_load", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_2_load", "role": "default" }} , 
 	{ "name": "PA_3_load_4", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_3_load_4", "role": "default" }} , 
 	{ "name": "PA_3_load_3", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_3_load_3", "role": "default" }} , 
 	{ "name": "PA_3_load_2", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_3_load_2", "role": "default" }} , 
 	{ "name": "PA_3_load_1", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_3_load_1", "role": "default" }} , 
 	{ "name": "PA_3_load", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_3_load", "role": "default" }} , 
 	{ "name": "PA_4_load_4", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_4_load_4", "role": "default" }} , 
 	{ "name": "PA_4_load_3", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_4_load_3", "role": "default" }} , 
 	{ "name": "PA_4_load_2", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_4_load_2", "role": "default" }} , 
 	{ "name": "PA_4_load_1", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_4_load_1", "role": "default" }} , 
 	{ "name": "PA_4_load", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "PA_4_load", "role": "default" }} , 
 	{ "name": "P_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P", "role": "address0" }} , 
 	{ "name": "P_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P", "role": "ce0" }} , 
 	{ "name": "P_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "P", "role": "q0" }} , 
 	{ "name": "P_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_1", "role": "address0" }} , 
 	{ "name": "P_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_1", "role": "ce0" }} , 
 	{ "name": "P_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "P_1", "role": "q0" }} , 
 	{ "name": "P_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_2", "role": "address0" }} , 
 	{ "name": "P_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_2", "role": "ce0" }} , 
 	{ "name": "P_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "P_2", "role": "q0" }} , 
 	{ "name": "P_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_3", "role": "address0" }} , 
 	{ "name": "P_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_3", "role": "ce0" }} , 
 	{ "name": "P_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "P_3", "role": "q0" }} , 
 	{ "name": "P_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_4", "role": "address0" }} , 
 	{ "name": "P_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_4", "role": "ce0" }} , 
 	{ "name": "P_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "P_4", "role": "q0" }} , 
 	{ "name": "P_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "P_5", "role": "address0" }} , 
 	{ "name": "P_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "P_5", "role": "ce0" }} , 
 	{ "name": "P_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "P_5", "role": "q0" }} , 
 	{ "name": "sext_ln256_5", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_5", "role": "default" }} , 
 	{ "name": "sext_ln256_7", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_7", "role": "default" }} , 
 	{ "name": "sext_ln256_9", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_9", "role": "default" }} , 
 	{ "name": "sext_ln256_11", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_11", "role": "default" }} , 
 	{ "name": "sext_ln256_13", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_13", "role": "default" }} , 
 	{ "name": "sext_ln256_15", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_15", "role": "default" }} , 
 	{ "name": "sext_ln256_17", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_17", "role": "default" }} , 
 	{ "name": "sext_ln256_19", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_19", "role": "default" }} , 
 	{ "name": "sext_ln256_21", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_21", "role": "default" }} , 
 	{ "name": "sext_ln256_23", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_23", "role": "default" }} , 
 	{ "name": "sext_ln256_25", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_25", "role": "default" }} , 
 	{ "name": "sext_ln256_27", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_27", "role": "default" }} , 
 	{ "name": "sext_ln256_29", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_29", "role": "default" }} , 
 	{ "name": "sext_ln256_31", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_31", "role": "default" }} , 
 	{ "name": "sext_ln256_33", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_33", "role": "default" }} , 
 	{ "name": "sext_ln256_35", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_35", "role": "default" }} , 
 	{ "name": "sext_ln256_37", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_37", "role": "default" }} , 
 	{ "name": "sext_ln256_39", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_39", "role": "default" }} , 
 	{ "name": "sext_ln256_41", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_41", "role": "default" }} , 
 	{ "name": "sext_ln256_43", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_43", "role": "default" }} , 
 	{ "name": "sext_ln256_45", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_45", "role": "default" }} , 
 	{ "name": "sext_ln256_47", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_47", "role": "default" }} , 
 	{ "name": "sext_ln256_49", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_49", "role": "default" }} , 
 	{ "name": "sext_ln256_51", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_51", "role": "default" }} , 
 	{ "name": "sext_ln256_53", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_53", "role": "default" }} , 
 	{ "name": "sext_ln256_55", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_55", "role": "default" }} , 
 	{ "name": "sext_ln256_57", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_57", "role": "default" }} , 
 	{ "name": "sext_ln256_59", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_59", "role": "default" }} , 
 	{ "name": "sext_ln256_61", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln256_61", "role": "default" }} , 
 	{ "name": "sext_ln848", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln848", "role": "default" }} , 
 	{ "name": "conv_i_i13988_459_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_459_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_459_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_459_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_357_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_357_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_357_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_357_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_255_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_255_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_255_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_255_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_153_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_153_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_153_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_153_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i1398851_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i1398851_out", "role": "default" }} , 
 	{ "name": "conv_i_i1398851_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i1398851_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_449_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_449_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_449_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_449_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_347_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_347_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_347_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_347_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_245_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_245_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_245_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_245_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_143_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_143_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_143_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_143_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i1398841_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i1398841_out", "role": "default" }} , 
 	{ "name": "conv_i_i1398841_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i1398841_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_439_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_439_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_439_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_439_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_337_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_337_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_337_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_337_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_235_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_235_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_235_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_235_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_133_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_133_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_133_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_133_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i1398831_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i1398831_out", "role": "default" }} , 
 	{ "name": "conv_i_i1398831_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i1398831_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_429_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_429_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_429_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_429_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_327_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_327_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_327_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_327_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_225_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_225_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_225_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_225_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_123_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_123_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_123_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_123_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i1398821_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i1398821_out", "role": "default" }} , 
 	{ "name": "conv_i_i1398821_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i1398821_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_419_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_419_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_419_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_419_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_317_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_317_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_317_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_317_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_215_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_215_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_215_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_215_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_113_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_113_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_113_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_113_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i1398811_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i1398811_out", "role": "default" }} , 
 	{ "name": "conv_i_i1398811_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i1398811_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_49_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_49_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_49_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_49_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_37_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_37_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_37_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_37_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_25_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_25_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_25_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_25_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i13988_13_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i13988_13_out", "role": "default" }} , 
 	{ "name": "conv_i_i13988_13_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i13988_13_out", "role": "ap_vld" }} , 
 	{ "name": "conv_i_i139881_out", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "conv_i_i139881_out", "role": "default" }} , 
 	{ "name": "conv_i_i139881_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "conv_i_i139881_out", "role": "ap_vld" }} , 
 	{ "name": "grp_fu_15628_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "grp_fu_15628_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15628_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15628_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15628_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":43, "type": "signal", "bundle":{"name": "grp_fu_15628_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15628_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15628_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_15632_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "grp_fu_15632_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15632_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15632_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15632_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":43, "type": "signal", "bundle":{"name": "grp_fu_15632_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15632_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15632_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_15636_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "grp_fu_15636_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15636_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15636_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15636_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":43, "type": "signal", "bundle":{"name": "grp_fu_15636_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15636_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15636_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_15640_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "grp_fu_15640_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15640_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15640_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15640_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":43, "type": "signal", "bundle":{"name": "grp_fu_15640_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15640_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15640_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_15644_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "grp_fu_15644_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15644_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15644_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15644_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":43, "type": "signal", "bundle":{"name": "grp_fu_15644_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15644_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15644_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_15648_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "grp_fu_15648_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15648_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15648_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15648_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":43, "type": "signal", "bundle":{"name": "grp_fu_15648_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15648_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15648_p_ce", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din1", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din2", "direction": "out", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din2", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din3", "direction": "out", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din3", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din4", "direction": "out", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din4", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din5", "direction": "out", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din5", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din6", "direction": "out", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_din6", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_dout0", "role": "default" }} , 
 	{ "name": "pd_sum_sum6_P_QP_raw_fu_15652_p_ready", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "pd_sum_sum6_P_QP_raw_fu_15652_p_ready", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		a5 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "10", "Max" : "10"}
	, {"Name" : "Interval", "Min" : "7", "Max" : "7"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	PA_5_load_4 { ap_none {  { PA_5_load_4 in_data 0 27 } } }
	PA_5_load_3 { ap_none {  { PA_5_load_3 in_data 0 27 } } }
	PA_5_load_2 { ap_none {  { PA_5_load_2 in_data 0 27 } } }
	PA_5_load_1 { ap_none {  { PA_5_load_1 in_data 0 27 } } }
	PA_5_load { ap_none {  { PA_5_load in_data 0 27 } } }
	PA_load_4 { ap_none {  { PA_load_4 in_data 0 27 } } }
	PA_load_3 { ap_none {  { PA_load_3 in_data 0 27 } } }
	PA_load_2 { ap_none {  { PA_load_2 in_data 0 27 } } }
	PA_load_1 { ap_none {  { PA_load_1 in_data 0 27 } } }
	PA_load { ap_none {  { PA_load in_data 0 27 } } }
	PA_1_load_4 { ap_none {  { PA_1_load_4 in_data 0 27 } } }
	PA_1_load_3 { ap_none {  { PA_1_load_3 in_data 0 27 } } }
	PA_1_load_2 { ap_none {  { PA_1_load_2 in_data 0 27 } } }
	PA_1_load_1 { ap_none {  { PA_1_load_1 in_data 0 27 } } }
	PA_1_load { ap_none {  { PA_1_load in_data 0 27 } } }
	PA_2_load_4 { ap_none {  { PA_2_load_4 in_data 0 27 } } }
	PA_2_load_3 { ap_none {  { PA_2_load_3 in_data 0 27 } } }
	PA_2_load_2 { ap_none {  { PA_2_load_2 in_data 0 27 } } }
	PA_2_load_1 { ap_none {  { PA_2_load_1 in_data 0 27 } } }
	PA_2_load { ap_none {  { PA_2_load in_data 0 27 } } }
	PA_3_load_4 { ap_none {  { PA_3_load_4 in_data 0 27 } } }
	PA_3_load_3 { ap_none {  { PA_3_load_3 in_data 0 27 } } }
	PA_3_load_2 { ap_none {  { PA_3_load_2 in_data 0 27 } } }
	PA_3_load_1 { ap_none {  { PA_3_load_1 in_data 0 27 } } }
	PA_3_load { ap_none {  { PA_3_load in_data 0 27 } } }
	PA_4_load_4 { ap_none {  { PA_4_load_4 in_data 0 27 } } }
	PA_4_load_3 { ap_none {  { PA_4_load_3 in_data 0 27 } } }
	PA_4_load_2 { ap_none {  { PA_4_load_2 in_data 0 27 } } }
	PA_4_load_1 { ap_none {  { PA_4_load_1 in_data 0 27 } } }
	PA_4_load { ap_none {  { PA_4_load in_data 0 27 } } }
	P { ap_memory {  { P_address0 mem_address 1 3 }  { P_ce0 mem_ce 1 1 }  { P_q0 mem_dout 0 27 } } }
	P_1 { ap_memory {  { P_1_address0 mem_address 1 3 }  { P_1_ce0 mem_ce 1 1 }  { P_1_q0 mem_dout 0 27 } } }
	P_2 { ap_memory {  { P_2_address0 mem_address 1 3 }  { P_2_ce0 mem_ce 1 1 }  { P_2_q0 mem_dout 0 27 } } }
	P_3 { ap_memory {  { P_3_address0 mem_address 1 3 }  { P_3_ce0 mem_ce 1 1 }  { P_3_q0 mem_dout 0 27 } } }
	P_4 { ap_memory {  { P_4_address0 mem_address 1 3 }  { P_4_ce0 mem_ce 1 1 }  { P_4_q0 mem_dout 0 27 } } }
	P_5 { ap_memory {  { P_5_address0 mem_address 1 3 }  { P_5_ce0 mem_ce 1 1 }  { P_5_q0 mem_dout 0 27 } } }
	sext_ln256_5 { ap_none {  { sext_ln256_5 in_data 0 26 } } }
	sext_ln256_7 { ap_none {  { sext_ln256_7 in_data 0 26 } } }
	sext_ln256_9 { ap_none {  { sext_ln256_9 in_data 0 26 } } }
	sext_ln256_11 { ap_none {  { sext_ln256_11 in_data 0 26 } } }
	sext_ln256_13 { ap_none {  { sext_ln256_13 in_data 0 26 } } }
	sext_ln256_15 { ap_none {  { sext_ln256_15 in_data 0 26 } } }
	sext_ln256_17 { ap_none {  { sext_ln256_17 in_data 0 26 } } }
	sext_ln256_19 { ap_none {  { sext_ln256_19 in_data 0 26 } } }
	sext_ln256_21 { ap_none {  { sext_ln256_21 in_data 0 26 } } }
	sext_ln256_23 { ap_none {  { sext_ln256_23 in_data 0 26 } } }
	sext_ln256_25 { ap_none {  { sext_ln256_25 in_data 0 26 } } }
	sext_ln256_27 { ap_none {  { sext_ln256_27 in_data 0 26 } } }
	sext_ln256_29 { ap_none {  { sext_ln256_29 in_data 0 26 } } }
	sext_ln256_31 { ap_none {  { sext_ln256_31 in_data 0 26 } } }
	sext_ln256_33 { ap_none {  { sext_ln256_33 in_data 0 26 } } }
	sext_ln256_35 { ap_none {  { sext_ln256_35 in_data 0 26 } } }
	sext_ln256_37 { ap_none {  { sext_ln256_37 in_data 0 26 } } }
	sext_ln256_39 { ap_none {  { sext_ln256_39 in_data 0 26 } } }
	sext_ln256_41 { ap_none {  { sext_ln256_41 in_data 0 26 } } }
	sext_ln256_43 { ap_none {  { sext_ln256_43 in_data 0 26 } } }
	sext_ln256_45 { ap_none {  { sext_ln256_45 in_data 0 26 } } }
	sext_ln256_47 { ap_none {  { sext_ln256_47 in_data 0 26 } } }
	sext_ln256_49 { ap_none {  { sext_ln256_49 in_data 0 26 } } }
	sext_ln256_51 { ap_none {  { sext_ln256_51 in_data 0 26 } } }
	sext_ln256_53 { ap_none {  { sext_ln256_53 in_data 0 26 } } }
	sext_ln256_55 { ap_none {  { sext_ln256_55 in_data 0 26 } } }
	sext_ln256_57 { ap_none {  { sext_ln256_57 in_data 0 26 } } }
	sext_ln256_59 { ap_none {  { sext_ln256_59 in_data 0 26 } } }
	sext_ln256_61 { ap_none {  { sext_ln256_61 in_data 0 26 } } }
	sext_ln848 { ap_none {  { sext_ln848 in_data 0 26 } } }
	conv_i_i13988_459_out { ap_vld {  { conv_i_i13988_459_out out_data 1 27 }  { conv_i_i13988_459_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_357_out { ap_vld {  { conv_i_i13988_357_out out_data 1 27 }  { conv_i_i13988_357_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_255_out { ap_vld {  { conv_i_i13988_255_out out_data 1 27 }  { conv_i_i13988_255_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_153_out { ap_vld {  { conv_i_i13988_153_out out_data 1 27 }  { conv_i_i13988_153_out_ap_vld out_vld 1 1 } } }
	conv_i_i1398851_out { ap_vld {  { conv_i_i1398851_out out_data 1 27 }  { conv_i_i1398851_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_449_out { ap_vld {  { conv_i_i13988_449_out out_data 1 27 }  { conv_i_i13988_449_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_347_out { ap_vld {  { conv_i_i13988_347_out out_data 1 27 }  { conv_i_i13988_347_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_245_out { ap_vld {  { conv_i_i13988_245_out out_data 1 27 }  { conv_i_i13988_245_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_143_out { ap_vld {  { conv_i_i13988_143_out out_data 1 27 }  { conv_i_i13988_143_out_ap_vld out_vld 1 1 } } }
	conv_i_i1398841_out { ap_vld {  { conv_i_i1398841_out out_data 1 27 }  { conv_i_i1398841_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_439_out { ap_vld {  { conv_i_i13988_439_out out_data 1 27 }  { conv_i_i13988_439_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_337_out { ap_vld {  { conv_i_i13988_337_out out_data 1 27 }  { conv_i_i13988_337_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_235_out { ap_vld {  { conv_i_i13988_235_out out_data 1 27 }  { conv_i_i13988_235_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_133_out { ap_vld {  { conv_i_i13988_133_out out_data 1 27 }  { conv_i_i13988_133_out_ap_vld out_vld 1 1 } } }
	conv_i_i1398831_out { ap_vld {  { conv_i_i1398831_out out_data 1 27 }  { conv_i_i1398831_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_429_out { ap_vld {  { conv_i_i13988_429_out out_data 1 27 }  { conv_i_i13988_429_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_327_out { ap_vld {  { conv_i_i13988_327_out out_data 1 27 }  { conv_i_i13988_327_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_225_out { ap_vld {  { conv_i_i13988_225_out out_data 1 27 }  { conv_i_i13988_225_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_123_out { ap_vld {  { conv_i_i13988_123_out out_data 1 27 }  { conv_i_i13988_123_out_ap_vld out_vld 1 1 } } }
	conv_i_i1398821_out { ap_vld {  { conv_i_i1398821_out out_data 1 27 }  { conv_i_i1398821_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_419_out { ap_vld {  { conv_i_i13988_419_out out_data 1 27 }  { conv_i_i13988_419_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_317_out { ap_vld {  { conv_i_i13988_317_out out_data 1 27 }  { conv_i_i13988_317_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_215_out { ap_vld {  { conv_i_i13988_215_out out_data 1 27 }  { conv_i_i13988_215_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_113_out { ap_vld {  { conv_i_i13988_113_out out_data 1 27 }  { conv_i_i13988_113_out_ap_vld out_vld 1 1 } } }
	conv_i_i1398811_out { ap_vld {  { conv_i_i1398811_out out_data 1 27 }  { conv_i_i1398811_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_49_out { ap_vld {  { conv_i_i13988_49_out out_data 1 27 }  { conv_i_i13988_49_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_37_out { ap_vld {  { conv_i_i13988_37_out out_data 1 27 }  { conv_i_i13988_37_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_25_out { ap_vld {  { conv_i_i13988_25_out out_data 1 27 }  { conv_i_i13988_25_out_ap_vld out_vld 1 1 } } }
	conv_i_i13988_13_out { ap_vld {  { conv_i_i13988_13_out out_data 1 27 }  { conv_i_i13988_13_out_ap_vld out_vld 1 1 } } }
	conv_i_i139881_out { ap_vld {  { conv_i_i139881_out out_data 1 27 }  { conv_i_i139881_out_ap_vld out_vld 1 1 } } }
}
