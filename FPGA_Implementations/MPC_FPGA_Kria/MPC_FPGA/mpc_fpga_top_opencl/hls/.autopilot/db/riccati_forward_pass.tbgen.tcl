set moduleName riccati_forward_pass
set isTopModule 0
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set isPipelined_legacy 0
set pipeline_type none
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
set C_modelName {riccati_forward_pass}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict step_data_0_0_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_0_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_0_2 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_0_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_0_4 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_0_5 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1_2 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1_4 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1_5 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2_2 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2_4 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2_5 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3_2 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3_4 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3_5 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4_2 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4_4 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4_5 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5_2 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5_4 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5_5 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_1 { MEM_WIDTH 32 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_2 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_3 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_0 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_1 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_2 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_3 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_4 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_5 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_6 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_7 { MEM_WIDTH 26 MEM_SIZE 20 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict kk_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict kk_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_0 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_1 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_2 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_3 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_4 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_6 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_7 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict u_out_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict u_out_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ step_data_0_0_0 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_0_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_0_2 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_0_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_0_4 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_0_5 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1_0 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1_2 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1_4 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1_5 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2_0 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2_2 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2_4 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2_5 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3_0 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3_2 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3_4 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3_5 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4_0 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4_2 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4_4 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4_5 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5_0 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5_2 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5_4 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5_5 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_1 int 32 regular {array 120 { 1 1 } 1 1 bus  }  }
	{ B_sparse_0 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ B_sparse_1 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ B_sparse_2 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ B_sparse_3 int 32 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ p_read int 32 regular  }
	{ p_read1 int 32 regular  }
	{ p_read2 int 32 regular  }
	{ p_read3 int 32 regular  }
	{ p_read4 int 32 regular  }
	{ p_read5 int 32 regular  }
	{ p_read6 int 32 regular  }
	{ p_read7 int 32 regular  }
	{ K_0_0_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_0 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_1 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_2 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_3 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_4 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_5 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_6 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_7 int 26 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ kk_0 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ kk_1 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ x_out_0 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_1 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_2 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_3 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_4 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_5 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_6 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_7 int 32 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ u_out_0 int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ u_out_1 int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "step_data_0_0_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_0_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read1", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read2", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read3", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read4", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read5", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read6", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_read7", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "kk_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "kk_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "x_out_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_2", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_3", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_4", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_6", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_7", "interface" : "memory", "bitwidth" : 32, "direction" : "READWRITE"} , 
 	{ "Name" : "u_out_0", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "u_out_1", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 402
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ step_data_0_0_0_address0 sc_out sc_lv 5 signal 0 } 
	{ step_data_0_0_0_ce0 sc_out sc_logic 1 signal 0 } 
	{ step_data_0_0_0_q0 sc_in sc_lv 32 signal 0 } 
	{ step_data_0_0_1_address0 sc_out sc_lv 5 signal 1 } 
	{ step_data_0_0_1_ce0 sc_out sc_logic 1 signal 1 } 
	{ step_data_0_0_1_q0 sc_in sc_lv 32 signal 1 } 
	{ step_data_0_0_2_address0 sc_out sc_lv 5 signal 2 } 
	{ step_data_0_0_2_ce0 sc_out sc_logic 1 signal 2 } 
	{ step_data_0_0_2_q0 sc_in sc_lv 32 signal 2 } 
	{ step_data_0_0_3_address0 sc_out sc_lv 5 signal 3 } 
	{ step_data_0_0_3_ce0 sc_out sc_logic 1 signal 3 } 
	{ step_data_0_0_3_q0 sc_in sc_lv 32 signal 3 } 
	{ step_data_0_0_4_address0 sc_out sc_lv 5 signal 4 } 
	{ step_data_0_0_4_ce0 sc_out sc_logic 1 signal 4 } 
	{ step_data_0_0_4_q0 sc_in sc_lv 32 signal 4 } 
	{ step_data_0_0_5_address0 sc_out sc_lv 5 signal 5 } 
	{ step_data_0_0_5_ce0 sc_out sc_logic 1 signal 5 } 
	{ step_data_0_0_5_q0 sc_in sc_lv 32 signal 5 } 
	{ step_data_0_1_0_address0 sc_out sc_lv 5 signal 6 } 
	{ step_data_0_1_0_ce0 sc_out sc_logic 1 signal 6 } 
	{ step_data_0_1_0_q0 sc_in sc_lv 32 signal 6 } 
	{ step_data_0_1_1_address0 sc_out sc_lv 5 signal 7 } 
	{ step_data_0_1_1_ce0 sc_out sc_logic 1 signal 7 } 
	{ step_data_0_1_1_q0 sc_in sc_lv 32 signal 7 } 
	{ step_data_0_1_2_address0 sc_out sc_lv 5 signal 8 } 
	{ step_data_0_1_2_ce0 sc_out sc_logic 1 signal 8 } 
	{ step_data_0_1_2_q0 sc_in sc_lv 32 signal 8 } 
	{ step_data_0_1_3_address0 sc_out sc_lv 5 signal 9 } 
	{ step_data_0_1_3_ce0 sc_out sc_logic 1 signal 9 } 
	{ step_data_0_1_3_q0 sc_in sc_lv 32 signal 9 } 
	{ step_data_0_1_4_address0 sc_out sc_lv 5 signal 10 } 
	{ step_data_0_1_4_ce0 sc_out sc_logic 1 signal 10 } 
	{ step_data_0_1_4_q0 sc_in sc_lv 32 signal 10 } 
	{ step_data_0_1_5_address0 sc_out sc_lv 5 signal 11 } 
	{ step_data_0_1_5_ce0 sc_out sc_logic 1 signal 11 } 
	{ step_data_0_1_5_q0 sc_in sc_lv 32 signal 11 } 
	{ step_data_0_2_0_address0 sc_out sc_lv 5 signal 12 } 
	{ step_data_0_2_0_ce0 sc_out sc_logic 1 signal 12 } 
	{ step_data_0_2_0_q0 sc_in sc_lv 32 signal 12 } 
	{ step_data_0_2_1_address0 sc_out sc_lv 5 signal 13 } 
	{ step_data_0_2_1_ce0 sc_out sc_logic 1 signal 13 } 
	{ step_data_0_2_1_q0 sc_in sc_lv 32 signal 13 } 
	{ step_data_0_2_2_address0 sc_out sc_lv 5 signal 14 } 
	{ step_data_0_2_2_ce0 sc_out sc_logic 1 signal 14 } 
	{ step_data_0_2_2_q0 sc_in sc_lv 32 signal 14 } 
	{ step_data_0_2_3_address0 sc_out sc_lv 5 signal 15 } 
	{ step_data_0_2_3_ce0 sc_out sc_logic 1 signal 15 } 
	{ step_data_0_2_3_q0 sc_in sc_lv 32 signal 15 } 
	{ step_data_0_2_4_address0 sc_out sc_lv 5 signal 16 } 
	{ step_data_0_2_4_ce0 sc_out sc_logic 1 signal 16 } 
	{ step_data_0_2_4_q0 sc_in sc_lv 32 signal 16 } 
	{ step_data_0_2_5_address0 sc_out sc_lv 5 signal 17 } 
	{ step_data_0_2_5_ce0 sc_out sc_logic 1 signal 17 } 
	{ step_data_0_2_5_q0 sc_in sc_lv 32 signal 17 } 
	{ step_data_0_3_0_address0 sc_out sc_lv 5 signal 18 } 
	{ step_data_0_3_0_ce0 sc_out sc_logic 1 signal 18 } 
	{ step_data_0_3_0_q0 sc_in sc_lv 32 signal 18 } 
	{ step_data_0_3_1_address0 sc_out sc_lv 5 signal 19 } 
	{ step_data_0_3_1_ce0 sc_out sc_logic 1 signal 19 } 
	{ step_data_0_3_1_q0 sc_in sc_lv 32 signal 19 } 
	{ step_data_0_3_2_address0 sc_out sc_lv 5 signal 20 } 
	{ step_data_0_3_2_ce0 sc_out sc_logic 1 signal 20 } 
	{ step_data_0_3_2_q0 sc_in sc_lv 32 signal 20 } 
	{ step_data_0_3_3_address0 sc_out sc_lv 5 signal 21 } 
	{ step_data_0_3_3_ce0 sc_out sc_logic 1 signal 21 } 
	{ step_data_0_3_3_q0 sc_in sc_lv 32 signal 21 } 
	{ step_data_0_3_4_address0 sc_out sc_lv 5 signal 22 } 
	{ step_data_0_3_4_ce0 sc_out sc_logic 1 signal 22 } 
	{ step_data_0_3_4_q0 sc_in sc_lv 32 signal 22 } 
	{ step_data_0_3_5_address0 sc_out sc_lv 5 signal 23 } 
	{ step_data_0_3_5_ce0 sc_out sc_logic 1 signal 23 } 
	{ step_data_0_3_5_q0 sc_in sc_lv 32 signal 23 } 
	{ step_data_0_4_0_address0 sc_out sc_lv 5 signal 24 } 
	{ step_data_0_4_0_ce0 sc_out sc_logic 1 signal 24 } 
	{ step_data_0_4_0_q0 sc_in sc_lv 32 signal 24 } 
	{ step_data_0_4_1_address0 sc_out sc_lv 5 signal 25 } 
	{ step_data_0_4_1_ce0 sc_out sc_logic 1 signal 25 } 
	{ step_data_0_4_1_q0 sc_in sc_lv 32 signal 25 } 
	{ step_data_0_4_2_address0 sc_out sc_lv 5 signal 26 } 
	{ step_data_0_4_2_ce0 sc_out sc_logic 1 signal 26 } 
	{ step_data_0_4_2_q0 sc_in sc_lv 32 signal 26 } 
	{ step_data_0_4_3_address0 sc_out sc_lv 5 signal 27 } 
	{ step_data_0_4_3_ce0 sc_out sc_logic 1 signal 27 } 
	{ step_data_0_4_3_q0 sc_in sc_lv 32 signal 27 } 
	{ step_data_0_4_4_address0 sc_out sc_lv 5 signal 28 } 
	{ step_data_0_4_4_ce0 sc_out sc_logic 1 signal 28 } 
	{ step_data_0_4_4_q0 sc_in sc_lv 32 signal 28 } 
	{ step_data_0_4_5_address0 sc_out sc_lv 5 signal 29 } 
	{ step_data_0_4_5_ce0 sc_out sc_logic 1 signal 29 } 
	{ step_data_0_4_5_q0 sc_in sc_lv 32 signal 29 } 
	{ step_data_0_5_0_address0 sc_out sc_lv 5 signal 30 } 
	{ step_data_0_5_0_ce0 sc_out sc_logic 1 signal 30 } 
	{ step_data_0_5_0_q0 sc_in sc_lv 32 signal 30 } 
	{ step_data_0_5_1_address0 sc_out sc_lv 5 signal 31 } 
	{ step_data_0_5_1_ce0 sc_out sc_logic 1 signal 31 } 
	{ step_data_0_5_1_q0 sc_in sc_lv 32 signal 31 } 
	{ step_data_0_5_2_address0 sc_out sc_lv 5 signal 32 } 
	{ step_data_0_5_2_ce0 sc_out sc_logic 1 signal 32 } 
	{ step_data_0_5_2_q0 sc_in sc_lv 32 signal 32 } 
	{ step_data_0_5_3_address0 sc_out sc_lv 5 signal 33 } 
	{ step_data_0_5_3_ce0 sc_out sc_logic 1 signal 33 } 
	{ step_data_0_5_3_q0 sc_in sc_lv 32 signal 33 } 
	{ step_data_0_5_4_address0 sc_out sc_lv 5 signal 34 } 
	{ step_data_0_5_4_ce0 sc_out sc_logic 1 signal 34 } 
	{ step_data_0_5_4_q0 sc_in sc_lv 32 signal 34 } 
	{ step_data_0_5_5_address0 sc_out sc_lv 5 signal 35 } 
	{ step_data_0_5_5_ce0 sc_out sc_logic 1 signal 35 } 
	{ step_data_0_5_5_q0 sc_in sc_lv 32 signal 35 } 
	{ step_data_1_address0 sc_out sc_lv 7 signal 36 } 
	{ step_data_1_ce0 sc_out sc_logic 1 signal 36 } 
	{ step_data_1_q0 sc_in sc_lv 32 signal 36 } 
	{ step_data_1_address1 sc_out sc_lv 7 signal 36 } 
	{ step_data_1_ce1 sc_out sc_logic 1 signal 36 } 
	{ step_data_1_q1 sc_in sc_lv 32 signal 36 } 
	{ B_sparse_0_address0 sc_out sc_lv 5 signal 37 } 
	{ B_sparse_0_ce0 sc_out sc_logic 1 signal 37 } 
	{ B_sparse_0_q0 sc_in sc_lv 32 signal 37 } 
	{ B_sparse_1_address0 sc_out sc_lv 5 signal 38 } 
	{ B_sparse_1_ce0 sc_out sc_logic 1 signal 38 } 
	{ B_sparse_1_q0 sc_in sc_lv 32 signal 38 } 
	{ B_sparse_2_address0 sc_out sc_lv 5 signal 39 } 
	{ B_sparse_2_ce0 sc_out sc_logic 1 signal 39 } 
	{ B_sparse_2_q0 sc_in sc_lv 32 signal 39 } 
	{ B_sparse_3_address0 sc_out sc_lv 5 signal 40 } 
	{ B_sparse_3_ce0 sc_out sc_logic 1 signal 40 } 
	{ B_sparse_3_q0 sc_in sc_lv 32 signal 40 } 
	{ p_read sc_in sc_lv 32 signal 41 } 
	{ p_read1 sc_in sc_lv 32 signal 42 } 
	{ p_read2 sc_in sc_lv 32 signal 43 } 
	{ p_read3 sc_in sc_lv 32 signal 44 } 
	{ p_read4 sc_in sc_lv 32 signal 45 } 
	{ p_read5 sc_in sc_lv 32 signal 46 } 
	{ p_read6 sc_in sc_lv 32 signal 47 } 
	{ p_read7 sc_in sc_lv 32 signal 48 } 
	{ K_0_0_0_address0 sc_out sc_lv 3 signal 49 } 
	{ K_0_0_0_ce0 sc_out sc_logic 1 signal 49 } 
	{ K_0_0_0_q0 sc_in sc_lv 26 signal 49 } 
	{ K_0_0_1_address0 sc_out sc_lv 3 signal 50 } 
	{ K_0_0_1_ce0 sc_out sc_logic 1 signal 50 } 
	{ K_0_0_1_q0 sc_in sc_lv 26 signal 50 } 
	{ K_0_0_2_address0 sc_out sc_lv 3 signal 51 } 
	{ K_0_0_2_ce0 sc_out sc_logic 1 signal 51 } 
	{ K_0_0_2_q0 sc_in sc_lv 26 signal 51 } 
	{ K_0_0_3_address0 sc_out sc_lv 3 signal 52 } 
	{ K_0_0_3_ce0 sc_out sc_logic 1 signal 52 } 
	{ K_0_0_3_q0 sc_in sc_lv 26 signal 52 } 
	{ K_0_0_4_address0 sc_out sc_lv 3 signal 53 } 
	{ K_0_0_4_ce0 sc_out sc_logic 1 signal 53 } 
	{ K_0_0_4_q0 sc_in sc_lv 26 signal 53 } 
	{ K_0_0_5_address0 sc_out sc_lv 3 signal 54 } 
	{ K_0_0_5_ce0 sc_out sc_logic 1 signal 54 } 
	{ K_0_0_5_q0 sc_in sc_lv 26 signal 54 } 
	{ K_0_0_6_address0 sc_out sc_lv 3 signal 55 } 
	{ K_0_0_6_ce0 sc_out sc_logic 1 signal 55 } 
	{ K_0_0_6_q0 sc_in sc_lv 26 signal 55 } 
	{ K_0_0_7_address0 sc_out sc_lv 3 signal 56 } 
	{ K_0_0_7_ce0 sc_out sc_logic 1 signal 56 } 
	{ K_0_0_7_q0 sc_in sc_lv 26 signal 56 } 
	{ K_0_1_0_address0 sc_out sc_lv 3 signal 57 } 
	{ K_0_1_0_ce0 sc_out sc_logic 1 signal 57 } 
	{ K_0_1_0_q0 sc_in sc_lv 26 signal 57 } 
	{ K_0_1_1_address0 sc_out sc_lv 3 signal 58 } 
	{ K_0_1_1_ce0 sc_out sc_logic 1 signal 58 } 
	{ K_0_1_1_q0 sc_in sc_lv 26 signal 58 } 
	{ K_0_1_2_address0 sc_out sc_lv 3 signal 59 } 
	{ K_0_1_2_ce0 sc_out sc_logic 1 signal 59 } 
	{ K_0_1_2_q0 sc_in sc_lv 26 signal 59 } 
	{ K_0_1_3_address0 sc_out sc_lv 3 signal 60 } 
	{ K_0_1_3_ce0 sc_out sc_logic 1 signal 60 } 
	{ K_0_1_3_q0 sc_in sc_lv 26 signal 60 } 
	{ K_0_1_4_address0 sc_out sc_lv 3 signal 61 } 
	{ K_0_1_4_ce0 sc_out sc_logic 1 signal 61 } 
	{ K_0_1_4_q0 sc_in sc_lv 26 signal 61 } 
	{ K_0_1_5_address0 sc_out sc_lv 3 signal 62 } 
	{ K_0_1_5_ce0 sc_out sc_logic 1 signal 62 } 
	{ K_0_1_5_q0 sc_in sc_lv 26 signal 62 } 
	{ K_0_1_6_address0 sc_out sc_lv 3 signal 63 } 
	{ K_0_1_6_ce0 sc_out sc_logic 1 signal 63 } 
	{ K_0_1_6_q0 sc_in sc_lv 26 signal 63 } 
	{ K_0_1_7_address0 sc_out sc_lv 3 signal 64 } 
	{ K_0_1_7_ce0 sc_out sc_logic 1 signal 64 } 
	{ K_0_1_7_q0 sc_in sc_lv 26 signal 64 } 
	{ K_1_0_0_address0 sc_out sc_lv 3 signal 65 } 
	{ K_1_0_0_ce0 sc_out sc_logic 1 signal 65 } 
	{ K_1_0_0_q0 sc_in sc_lv 26 signal 65 } 
	{ K_1_0_1_address0 sc_out sc_lv 3 signal 66 } 
	{ K_1_0_1_ce0 sc_out sc_logic 1 signal 66 } 
	{ K_1_0_1_q0 sc_in sc_lv 26 signal 66 } 
	{ K_1_0_2_address0 sc_out sc_lv 3 signal 67 } 
	{ K_1_0_2_ce0 sc_out sc_logic 1 signal 67 } 
	{ K_1_0_2_q0 sc_in sc_lv 26 signal 67 } 
	{ K_1_0_3_address0 sc_out sc_lv 3 signal 68 } 
	{ K_1_0_3_ce0 sc_out sc_logic 1 signal 68 } 
	{ K_1_0_3_q0 sc_in sc_lv 26 signal 68 } 
	{ K_1_0_4_address0 sc_out sc_lv 3 signal 69 } 
	{ K_1_0_4_ce0 sc_out sc_logic 1 signal 69 } 
	{ K_1_0_4_q0 sc_in sc_lv 26 signal 69 } 
	{ K_1_0_5_address0 sc_out sc_lv 3 signal 70 } 
	{ K_1_0_5_ce0 sc_out sc_logic 1 signal 70 } 
	{ K_1_0_5_q0 sc_in sc_lv 26 signal 70 } 
	{ K_1_0_6_address0 sc_out sc_lv 3 signal 71 } 
	{ K_1_0_6_ce0 sc_out sc_logic 1 signal 71 } 
	{ K_1_0_6_q0 sc_in sc_lv 26 signal 71 } 
	{ K_1_0_7_address0 sc_out sc_lv 3 signal 72 } 
	{ K_1_0_7_ce0 sc_out sc_logic 1 signal 72 } 
	{ K_1_0_7_q0 sc_in sc_lv 26 signal 72 } 
	{ K_1_1_0_address0 sc_out sc_lv 3 signal 73 } 
	{ K_1_1_0_ce0 sc_out sc_logic 1 signal 73 } 
	{ K_1_1_0_q0 sc_in sc_lv 26 signal 73 } 
	{ K_1_1_1_address0 sc_out sc_lv 3 signal 74 } 
	{ K_1_1_1_ce0 sc_out sc_logic 1 signal 74 } 
	{ K_1_1_1_q0 sc_in sc_lv 26 signal 74 } 
	{ K_1_1_2_address0 sc_out sc_lv 3 signal 75 } 
	{ K_1_1_2_ce0 sc_out sc_logic 1 signal 75 } 
	{ K_1_1_2_q0 sc_in sc_lv 26 signal 75 } 
	{ K_1_1_3_address0 sc_out sc_lv 3 signal 76 } 
	{ K_1_1_3_ce0 sc_out sc_logic 1 signal 76 } 
	{ K_1_1_3_q0 sc_in sc_lv 26 signal 76 } 
	{ K_1_1_4_address0 sc_out sc_lv 3 signal 77 } 
	{ K_1_1_4_ce0 sc_out sc_logic 1 signal 77 } 
	{ K_1_1_4_q0 sc_in sc_lv 26 signal 77 } 
	{ K_1_1_5_address0 sc_out sc_lv 3 signal 78 } 
	{ K_1_1_5_ce0 sc_out sc_logic 1 signal 78 } 
	{ K_1_1_5_q0 sc_in sc_lv 26 signal 78 } 
	{ K_1_1_6_address0 sc_out sc_lv 3 signal 79 } 
	{ K_1_1_6_ce0 sc_out sc_logic 1 signal 79 } 
	{ K_1_1_6_q0 sc_in sc_lv 26 signal 79 } 
	{ K_1_1_7_address0 sc_out sc_lv 3 signal 80 } 
	{ K_1_1_7_ce0 sc_out sc_logic 1 signal 80 } 
	{ K_1_1_7_q0 sc_in sc_lv 26 signal 80 } 
	{ K_2_0_0_address0 sc_out sc_lv 3 signal 81 } 
	{ K_2_0_0_ce0 sc_out sc_logic 1 signal 81 } 
	{ K_2_0_0_q0 sc_in sc_lv 26 signal 81 } 
	{ K_2_0_1_address0 sc_out sc_lv 3 signal 82 } 
	{ K_2_0_1_ce0 sc_out sc_logic 1 signal 82 } 
	{ K_2_0_1_q0 sc_in sc_lv 26 signal 82 } 
	{ K_2_0_2_address0 sc_out sc_lv 3 signal 83 } 
	{ K_2_0_2_ce0 sc_out sc_logic 1 signal 83 } 
	{ K_2_0_2_q0 sc_in sc_lv 26 signal 83 } 
	{ K_2_0_3_address0 sc_out sc_lv 3 signal 84 } 
	{ K_2_0_3_ce0 sc_out sc_logic 1 signal 84 } 
	{ K_2_0_3_q0 sc_in sc_lv 26 signal 84 } 
	{ K_2_0_4_address0 sc_out sc_lv 3 signal 85 } 
	{ K_2_0_4_ce0 sc_out sc_logic 1 signal 85 } 
	{ K_2_0_4_q0 sc_in sc_lv 26 signal 85 } 
	{ K_2_0_5_address0 sc_out sc_lv 3 signal 86 } 
	{ K_2_0_5_ce0 sc_out sc_logic 1 signal 86 } 
	{ K_2_0_5_q0 sc_in sc_lv 26 signal 86 } 
	{ K_2_0_6_address0 sc_out sc_lv 3 signal 87 } 
	{ K_2_0_6_ce0 sc_out sc_logic 1 signal 87 } 
	{ K_2_0_6_q0 sc_in sc_lv 26 signal 87 } 
	{ K_2_0_7_address0 sc_out sc_lv 3 signal 88 } 
	{ K_2_0_7_ce0 sc_out sc_logic 1 signal 88 } 
	{ K_2_0_7_q0 sc_in sc_lv 26 signal 88 } 
	{ K_2_1_0_address0 sc_out sc_lv 3 signal 89 } 
	{ K_2_1_0_ce0 sc_out sc_logic 1 signal 89 } 
	{ K_2_1_0_q0 sc_in sc_lv 26 signal 89 } 
	{ K_2_1_1_address0 sc_out sc_lv 3 signal 90 } 
	{ K_2_1_1_ce0 sc_out sc_logic 1 signal 90 } 
	{ K_2_1_1_q0 sc_in sc_lv 26 signal 90 } 
	{ K_2_1_2_address0 sc_out sc_lv 3 signal 91 } 
	{ K_2_1_2_ce0 sc_out sc_logic 1 signal 91 } 
	{ K_2_1_2_q0 sc_in sc_lv 26 signal 91 } 
	{ K_2_1_3_address0 sc_out sc_lv 3 signal 92 } 
	{ K_2_1_3_ce0 sc_out sc_logic 1 signal 92 } 
	{ K_2_1_3_q0 sc_in sc_lv 26 signal 92 } 
	{ K_2_1_4_address0 sc_out sc_lv 3 signal 93 } 
	{ K_2_1_4_ce0 sc_out sc_logic 1 signal 93 } 
	{ K_2_1_4_q0 sc_in sc_lv 26 signal 93 } 
	{ K_2_1_5_address0 sc_out sc_lv 3 signal 94 } 
	{ K_2_1_5_ce0 sc_out sc_logic 1 signal 94 } 
	{ K_2_1_5_q0 sc_in sc_lv 26 signal 94 } 
	{ K_2_1_6_address0 sc_out sc_lv 3 signal 95 } 
	{ K_2_1_6_ce0 sc_out sc_logic 1 signal 95 } 
	{ K_2_1_6_q0 sc_in sc_lv 26 signal 95 } 
	{ K_2_1_7_address0 sc_out sc_lv 3 signal 96 } 
	{ K_2_1_7_ce0 sc_out sc_logic 1 signal 96 } 
	{ K_2_1_7_q0 sc_in sc_lv 26 signal 96 } 
	{ K_3_0_0_address0 sc_out sc_lv 3 signal 97 } 
	{ K_3_0_0_ce0 sc_out sc_logic 1 signal 97 } 
	{ K_3_0_0_q0 sc_in sc_lv 26 signal 97 } 
	{ K_3_0_1_address0 sc_out sc_lv 3 signal 98 } 
	{ K_3_0_1_ce0 sc_out sc_logic 1 signal 98 } 
	{ K_3_0_1_q0 sc_in sc_lv 26 signal 98 } 
	{ K_3_0_2_address0 sc_out sc_lv 3 signal 99 } 
	{ K_3_0_2_ce0 sc_out sc_logic 1 signal 99 } 
	{ K_3_0_2_q0 sc_in sc_lv 26 signal 99 } 
	{ K_3_0_3_address0 sc_out sc_lv 3 signal 100 } 
	{ K_3_0_3_ce0 sc_out sc_logic 1 signal 100 } 
	{ K_3_0_3_q0 sc_in sc_lv 26 signal 100 } 
	{ K_3_0_4_address0 sc_out sc_lv 3 signal 101 } 
	{ K_3_0_4_ce0 sc_out sc_logic 1 signal 101 } 
	{ K_3_0_4_q0 sc_in sc_lv 26 signal 101 } 
	{ K_3_0_5_address0 sc_out sc_lv 3 signal 102 } 
	{ K_3_0_5_ce0 sc_out sc_logic 1 signal 102 } 
	{ K_3_0_5_q0 sc_in sc_lv 26 signal 102 } 
	{ K_3_0_6_address0 sc_out sc_lv 3 signal 103 } 
	{ K_3_0_6_ce0 sc_out sc_logic 1 signal 103 } 
	{ K_3_0_6_q0 sc_in sc_lv 26 signal 103 } 
	{ K_3_0_7_address0 sc_out sc_lv 3 signal 104 } 
	{ K_3_0_7_ce0 sc_out sc_logic 1 signal 104 } 
	{ K_3_0_7_q0 sc_in sc_lv 26 signal 104 } 
	{ K_3_1_0_address0 sc_out sc_lv 3 signal 105 } 
	{ K_3_1_0_ce0 sc_out sc_logic 1 signal 105 } 
	{ K_3_1_0_q0 sc_in sc_lv 26 signal 105 } 
	{ K_3_1_1_address0 sc_out sc_lv 3 signal 106 } 
	{ K_3_1_1_ce0 sc_out sc_logic 1 signal 106 } 
	{ K_3_1_1_q0 sc_in sc_lv 26 signal 106 } 
	{ K_3_1_2_address0 sc_out sc_lv 3 signal 107 } 
	{ K_3_1_2_ce0 sc_out sc_logic 1 signal 107 } 
	{ K_3_1_2_q0 sc_in sc_lv 26 signal 107 } 
	{ K_3_1_3_address0 sc_out sc_lv 3 signal 108 } 
	{ K_3_1_3_ce0 sc_out sc_logic 1 signal 108 } 
	{ K_3_1_3_q0 sc_in sc_lv 26 signal 108 } 
	{ K_3_1_4_address0 sc_out sc_lv 3 signal 109 } 
	{ K_3_1_4_ce0 sc_out sc_logic 1 signal 109 } 
	{ K_3_1_4_q0 sc_in sc_lv 26 signal 109 } 
	{ K_3_1_5_address0 sc_out sc_lv 3 signal 110 } 
	{ K_3_1_5_ce0 sc_out sc_logic 1 signal 110 } 
	{ K_3_1_5_q0 sc_in sc_lv 26 signal 110 } 
	{ K_3_1_6_address0 sc_out sc_lv 3 signal 111 } 
	{ K_3_1_6_ce0 sc_out sc_logic 1 signal 111 } 
	{ K_3_1_6_q0 sc_in sc_lv 26 signal 111 } 
	{ K_3_1_7_address0 sc_out sc_lv 3 signal 112 } 
	{ K_3_1_7_ce0 sc_out sc_logic 1 signal 112 } 
	{ K_3_1_7_q0 sc_in sc_lv 26 signal 112 } 
	{ kk_0_address0 sc_out sc_lv 5 signal 113 } 
	{ kk_0_ce0 sc_out sc_logic 1 signal 113 } 
	{ kk_0_q0 sc_in sc_lv 26 signal 113 } 
	{ kk_1_address0 sc_out sc_lv 5 signal 114 } 
	{ kk_1_ce0 sc_out sc_logic 1 signal 114 } 
	{ kk_1_q0 sc_in sc_lv 26 signal 114 } 
	{ x_out_0_address0 sc_out sc_lv 5 signal 115 } 
	{ x_out_0_ce0 sc_out sc_logic 1 signal 115 } 
	{ x_out_0_q0 sc_in sc_lv 32 signal 115 } 
	{ x_out_0_address1 sc_out sc_lv 5 signal 115 } 
	{ x_out_0_ce1 sc_out sc_logic 1 signal 115 } 
	{ x_out_0_we1 sc_out sc_logic 1 signal 115 } 
	{ x_out_0_d1 sc_out sc_lv 32 signal 115 } 
	{ x_out_1_address0 sc_out sc_lv 5 signal 116 } 
	{ x_out_1_ce0 sc_out sc_logic 1 signal 116 } 
	{ x_out_1_q0 sc_in sc_lv 32 signal 116 } 
	{ x_out_1_address1 sc_out sc_lv 5 signal 116 } 
	{ x_out_1_ce1 sc_out sc_logic 1 signal 116 } 
	{ x_out_1_we1 sc_out sc_logic 1 signal 116 } 
	{ x_out_1_d1 sc_out sc_lv 32 signal 116 } 
	{ x_out_2_address0 sc_out sc_lv 5 signal 117 } 
	{ x_out_2_ce0 sc_out sc_logic 1 signal 117 } 
	{ x_out_2_q0 sc_in sc_lv 32 signal 117 } 
	{ x_out_2_address1 sc_out sc_lv 5 signal 117 } 
	{ x_out_2_ce1 sc_out sc_logic 1 signal 117 } 
	{ x_out_2_we1 sc_out sc_logic 1 signal 117 } 
	{ x_out_2_d1 sc_out sc_lv 32 signal 117 } 
	{ x_out_3_address0 sc_out sc_lv 5 signal 118 } 
	{ x_out_3_ce0 sc_out sc_logic 1 signal 118 } 
	{ x_out_3_q0 sc_in sc_lv 32 signal 118 } 
	{ x_out_3_address1 sc_out sc_lv 5 signal 118 } 
	{ x_out_3_ce1 sc_out sc_logic 1 signal 118 } 
	{ x_out_3_we1 sc_out sc_logic 1 signal 118 } 
	{ x_out_3_d1 sc_out sc_lv 32 signal 118 } 
	{ x_out_4_address0 sc_out sc_lv 5 signal 119 } 
	{ x_out_4_ce0 sc_out sc_logic 1 signal 119 } 
	{ x_out_4_q0 sc_in sc_lv 32 signal 119 } 
	{ x_out_4_address1 sc_out sc_lv 5 signal 119 } 
	{ x_out_4_ce1 sc_out sc_logic 1 signal 119 } 
	{ x_out_4_we1 sc_out sc_logic 1 signal 119 } 
	{ x_out_4_d1 sc_out sc_lv 32 signal 119 } 
	{ x_out_5_address0 sc_out sc_lv 5 signal 120 } 
	{ x_out_5_ce0 sc_out sc_logic 1 signal 120 } 
	{ x_out_5_q0 sc_in sc_lv 32 signal 120 } 
	{ x_out_5_address1 sc_out sc_lv 5 signal 120 } 
	{ x_out_5_ce1 sc_out sc_logic 1 signal 120 } 
	{ x_out_5_we1 sc_out sc_logic 1 signal 120 } 
	{ x_out_5_d1 sc_out sc_lv 32 signal 120 } 
	{ x_out_6_address0 sc_out sc_lv 5 signal 121 } 
	{ x_out_6_ce0 sc_out sc_logic 1 signal 121 } 
	{ x_out_6_q0 sc_in sc_lv 32 signal 121 } 
	{ x_out_6_address1 sc_out sc_lv 5 signal 121 } 
	{ x_out_6_ce1 sc_out sc_logic 1 signal 121 } 
	{ x_out_6_we1 sc_out sc_logic 1 signal 121 } 
	{ x_out_6_d1 sc_out sc_lv 32 signal 121 } 
	{ x_out_7_address0 sc_out sc_lv 5 signal 122 } 
	{ x_out_7_ce0 sc_out sc_logic 1 signal 122 } 
	{ x_out_7_q0 sc_in sc_lv 32 signal 122 } 
	{ x_out_7_address1 sc_out sc_lv 5 signal 122 } 
	{ x_out_7_ce1 sc_out sc_logic 1 signal 122 } 
	{ x_out_7_we1 sc_out sc_logic 1 signal 122 } 
	{ x_out_7_d1 sc_out sc_lv 32 signal 122 } 
	{ u_out_0_address0 sc_out sc_lv 5 signal 123 } 
	{ u_out_0_ce0 sc_out sc_logic 1 signal 123 } 
	{ u_out_0_we0 sc_out sc_logic 1 signal 123 } 
	{ u_out_0_d0 sc_out sc_lv 32 signal 123 } 
	{ u_out_1_address0 sc_out sc_lv 5 signal 124 } 
	{ u_out_1_ce0 sc_out sc_logic 1 signal 124 } 
	{ u_out_1_we0 sc_out sc_logic 1 signal 124 } 
	{ u_out_1_d0 sc_out sc_lv 32 signal 124 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "step_data_0_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_0_0", "role": "address0" }} , 
 	{ "name": "step_data_0_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0_0", "role": "ce0" }} , 
 	{ "name": "step_data_0_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_0", "role": "q0" }} , 
 	{ "name": "step_data_0_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_0_1", "role": "address0" }} , 
 	{ "name": "step_data_0_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0_1", "role": "ce0" }} , 
 	{ "name": "step_data_0_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_1", "role": "q0" }} , 
 	{ "name": "step_data_0_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_0_2", "role": "address0" }} , 
 	{ "name": "step_data_0_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0_2", "role": "ce0" }} , 
 	{ "name": "step_data_0_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_2", "role": "q0" }} , 
 	{ "name": "step_data_0_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_0_3", "role": "address0" }} , 
 	{ "name": "step_data_0_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0_3", "role": "ce0" }} , 
 	{ "name": "step_data_0_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_3", "role": "q0" }} , 
 	{ "name": "step_data_0_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_0_4", "role": "address0" }} , 
 	{ "name": "step_data_0_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0_4", "role": "ce0" }} , 
 	{ "name": "step_data_0_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_4", "role": "q0" }} , 
 	{ "name": "step_data_0_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_0_5", "role": "address0" }} , 
 	{ "name": "step_data_0_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_0_5", "role": "ce0" }} , 
 	{ "name": "step_data_0_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_0_5", "role": "q0" }} , 
 	{ "name": "step_data_0_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_1_0", "role": "address0" }} , 
 	{ "name": "step_data_0_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1_0", "role": "ce0" }} , 
 	{ "name": "step_data_0_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_0", "role": "q0" }} , 
 	{ "name": "step_data_0_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_1_1", "role": "address0" }} , 
 	{ "name": "step_data_0_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1_1", "role": "ce0" }} , 
 	{ "name": "step_data_0_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_1", "role": "q0" }} , 
 	{ "name": "step_data_0_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_1_2", "role": "address0" }} , 
 	{ "name": "step_data_0_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1_2", "role": "ce0" }} , 
 	{ "name": "step_data_0_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_2", "role": "q0" }} , 
 	{ "name": "step_data_0_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_1_3", "role": "address0" }} , 
 	{ "name": "step_data_0_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1_3", "role": "ce0" }} , 
 	{ "name": "step_data_0_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_3", "role": "q0" }} , 
 	{ "name": "step_data_0_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_1_4", "role": "address0" }} , 
 	{ "name": "step_data_0_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1_4", "role": "ce0" }} , 
 	{ "name": "step_data_0_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_4", "role": "q0" }} , 
 	{ "name": "step_data_0_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_1_5", "role": "address0" }} , 
 	{ "name": "step_data_0_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_1_5", "role": "ce0" }} , 
 	{ "name": "step_data_0_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_1_5", "role": "q0" }} , 
 	{ "name": "step_data_0_2_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_2_0", "role": "address0" }} , 
 	{ "name": "step_data_0_2_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2_0", "role": "ce0" }} , 
 	{ "name": "step_data_0_2_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_0", "role": "q0" }} , 
 	{ "name": "step_data_0_2_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_2_1", "role": "address0" }} , 
 	{ "name": "step_data_0_2_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2_1", "role": "ce0" }} , 
 	{ "name": "step_data_0_2_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_1", "role": "q0" }} , 
 	{ "name": "step_data_0_2_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_2_2", "role": "address0" }} , 
 	{ "name": "step_data_0_2_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2_2", "role": "ce0" }} , 
 	{ "name": "step_data_0_2_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_2", "role": "q0" }} , 
 	{ "name": "step_data_0_2_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_2_3", "role": "address0" }} , 
 	{ "name": "step_data_0_2_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2_3", "role": "ce0" }} , 
 	{ "name": "step_data_0_2_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_3", "role": "q0" }} , 
 	{ "name": "step_data_0_2_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_2_4", "role": "address0" }} , 
 	{ "name": "step_data_0_2_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2_4", "role": "ce0" }} , 
 	{ "name": "step_data_0_2_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_4", "role": "q0" }} , 
 	{ "name": "step_data_0_2_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_2_5", "role": "address0" }} , 
 	{ "name": "step_data_0_2_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_2_5", "role": "ce0" }} , 
 	{ "name": "step_data_0_2_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_2_5", "role": "q0" }} , 
 	{ "name": "step_data_0_3_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_3_0", "role": "address0" }} , 
 	{ "name": "step_data_0_3_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3_0", "role": "ce0" }} , 
 	{ "name": "step_data_0_3_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_0", "role": "q0" }} , 
 	{ "name": "step_data_0_3_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_3_1", "role": "address0" }} , 
 	{ "name": "step_data_0_3_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3_1", "role": "ce0" }} , 
 	{ "name": "step_data_0_3_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_1", "role": "q0" }} , 
 	{ "name": "step_data_0_3_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_3_2", "role": "address0" }} , 
 	{ "name": "step_data_0_3_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3_2", "role": "ce0" }} , 
 	{ "name": "step_data_0_3_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_2", "role": "q0" }} , 
 	{ "name": "step_data_0_3_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_3_3", "role": "address0" }} , 
 	{ "name": "step_data_0_3_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3_3", "role": "ce0" }} , 
 	{ "name": "step_data_0_3_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_3", "role": "q0" }} , 
 	{ "name": "step_data_0_3_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_3_4", "role": "address0" }} , 
 	{ "name": "step_data_0_3_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3_4", "role": "ce0" }} , 
 	{ "name": "step_data_0_3_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_4", "role": "q0" }} , 
 	{ "name": "step_data_0_3_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_3_5", "role": "address0" }} , 
 	{ "name": "step_data_0_3_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_3_5", "role": "ce0" }} , 
 	{ "name": "step_data_0_3_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_3_5", "role": "q0" }} , 
 	{ "name": "step_data_0_4_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_4_0", "role": "address0" }} , 
 	{ "name": "step_data_0_4_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4_0", "role": "ce0" }} , 
 	{ "name": "step_data_0_4_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_0", "role": "q0" }} , 
 	{ "name": "step_data_0_4_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_4_1", "role": "address0" }} , 
 	{ "name": "step_data_0_4_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4_1", "role": "ce0" }} , 
 	{ "name": "step_data_0_4_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_1", "role": "q0" }} , 
 	{ "name": "step_data_0_4_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_4_2", "role": "address0" }} , 
 	{ "name": "step_data_0_4_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4_2", "role": "ce0" }} , 
 	{ "name": "step_data_0_4_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_2", "role": "q0" }} , 
 	{ "name": "step_data_0_4_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_4_3", "role": "address0" }} , 
 	{ "name": "step_data_0_4_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4_3", "role": "ce0" }} , 
 	{ "name": "step_data_0_4_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_3", "role": "q0" }} , 
 	{ "name": "step_data_0_4_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_4_4", "role": "address0" }} , 
 	{ "name": "step_data_0_4_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4_4", "role": "ce0" }} , 
 	{ "name": "step_data_0_4_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_4", "role": "q0" }} , 
 	{ "name": "step_data_0_4_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_4_5", "role": "address0" }} , 
 	{ "name": "step_data_0_4_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_4_5", "role": "ce0" }} , 
 	{ "name": "step_data_0_4_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_4_5", "role": "q0" }} , 
 	{ "name": "step_data_0_5_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_5_0", "role": "address0" }} , 
 	{ "name": "step_data_0_5_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5_0", "role": "ce0" }} , 
 	{ "name": "step_data_0_5_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_0", "role": "q0" }} , 
 	{ "name": "step_data_0_5_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_5_1", "role": "address0" }} , 
 	{ "name": "step_data_0_5_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5_1", "role": "ce0" }} , 
 	{ "name": "step_data_0_5_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_1", "role": "q0" }} , 
 	{ "name": "step_data_0_5_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_5_2", "role": "address0" }} , 
 	{ "name": "step_data_0_5_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5_2", "role": "ce0" }} , 
 	{ "name": "step_data_0_5_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_2", "role": "q0" }} , 
 	{ "name": "step_data_0_5_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_5_3", "role": "address0" }} , 
 	{ "name": "step_data_0_5_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5_3", "role": "ce0" }} , 
 	{ "name": "step_data_0_5_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_3", "role": "q0" }} , 
 	{ "name": "step_data_0_5_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_5_4", "role": "address0" }} , 
 	{ "name": "step_data_0_5_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5_4", "role": "ce0" }} , 
 	{ "name": "step_data_0_5_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_4", "role": "q0" }} , 
 	{ "name": "step_data_0_5_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_0_5_5", "role": "address0" }} , 
 	{ "name": "step_data_0_5_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_0_5_5", "role": "ce0" }} , 
 	{ "name": "step_data_0_5_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_0_5_5", "role": "q0" }} , 
 	{ "name": "step_data_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_1", "role": "address0" }} , 
 	{ "name": "step_data_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_1", "role": "ce0" }} , 
 	{ "name": "step_data_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_1", "role": "q0" }} , 
 	{ "name": "step_data_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_1", "role": "address1" }} , 
 	{ "name": "step_data_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_1", "role": "ce1" }} , 
 	{ "name": "step_data_1_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "step_data_1", "role": "q1" }} , 
 	{ "name": "B_sparse_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_0", "role": "address0" }} , 
 	{ "name": "B_sparse_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_0", "role": "ce0" }} , 
 	{ "name": "B_sparse_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "B_sparse_0", "role": "q0" }} , 
 	{ "name": "B_sparse_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "address0" }} , 
 	{ "name": "B_sparse_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "ce0" }} , 
 	{ "name": "B_sparse_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "q0" }} , 
 	{ "name": "B_sparse_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "address0" }} , 
 	{ "name": "B_sparse_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "ce0" }} , 
 	{ "name": "B_sparse_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "q0" }} , 
 	{ "name": "B_sparse_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "address0" }} , 
 	{ "name": "B_sparse_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "ce0" }} , 
 	{ "name": "B_sparse_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "q0" }} , 
 	{ "name": "p_read", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read", "role": "default" }} , 
 	{ "name": "p_read1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read1", "role": "default" }} , 
 	{ "name": "p_read2", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read2", "role": "default" }} , 
 	{ "name": "p_read3", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read3", "role": "default" }} , 
 	{ "name": "p_read4", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read4", "role": "default" }} , 
 	{ "name": "p_read5", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read5", "role": "default" }} , 
 	{ "name": "p_read6", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read6", "role": "default" }} , 
 	{ "name": "p_read7", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_read7", "role": "default" }} , 
 	{ "name": "K_0_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_0", "role": "address0" }} , 
 	{ "name": "K_0_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_0", "role": "ce0" }} , 
 	{ "name": "K_0_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_0", "role": "q0" }} , 
 	{ "name": "K_0_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_1", "role": "address0" }} , 
 	{ "name": "K_0_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_1", "role": "ce0" }} , 
 	{ "name": "K_0_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_1", "role": "q0" }} , 
 	{ "name": "K_0_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_2", "role": "address0" }} , 
 	{ "name": "K_0_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_2", "role": "ce0" }} , 
 	{ "name": "K_0_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_2", "role": "q0" }} , 
 	{ "name": "K_0_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_3", "role": "address0" }} , 
 	{ "name": "K_0_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_3", "role": "ce0" }} , 
 	{ "name": "K_0_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_3", "role": "q0" }} , 
 	{ "name": "K_0_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_4", "role": "address0" }} , 
 	{ "name": "K_0_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_4", "role": "ce0" }} , 
 	{ "name": "K_0_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_4", "role": "q0" }} , 
 	{ "name": "K_0_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_5", "role": "address0" }} , 
 	{ "name": "K_0_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_5", "role": "ce0" }} , 
 	{ "name": "K_0_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_5", "role": "q0" }} , 
 	{ "name": "K_0_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_6", "role": "address0" }} , 
 	{ "name": "K_0_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_6", "role": "ce0" }} , 
 	{ "name": "K_0_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_6", "role": "q0" }} , 
 	{ "name": "K_0_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_7", "role": "address0" }} , 
 	{ "name": "K_0_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_7", "role": "ce0" }} , 
 	{ "name": "K_0_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_0_7", "role": "q0" }} , 
 	{ "name": "K_0_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_0", "role": "address0" }} , 
 	{ "name": "K_0_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_0", "role": "ce0" }} , 
 	{ "name": "K_0_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_0", "role": "q0" }} , 
 	{ "name": "K_0_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_1", "role": "address0" }} , 
 	{ "name": "K_0_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_1", "role": "ce0" }} , 
 	{ "name": "K_0_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_1", "role": "q0" }} , 
 	{ "name": "K_0_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_2", "role": "address0" }} , 
 	{ "name": "K_0_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_2", "role": "ce0" }} , 
 	{ "name": "K_0_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_2", "role": "q0" }} , 
 	{ "name": "K_0_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_3", "role": "address0" }} , 
 	{ "name": "K_0_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_3", "role": "ce0" }} , 
 	{ "name": "K_0_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_3", "role": "q0" }} , 
 	{ "name": "K_0_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_4", "role": "address0" }} , 
 	{ "name": "K_0_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_4", "role": "ce0" }} , 
 	{ "name": "K_0_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_4", "role": "q0" }} , 
 	{ "name": "K_0_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_5", "role": "address0" }} , 
 	{ "name": "K_0_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_5", "role": "ce0" }} , 
 	{ "name": "K_0_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_5", "role": "q0" }} , 
 	{ "name": "K_0_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_6", "role": "address0" }} , 
 	{ "name": "K_0_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_6", "role": "ce0" }} , 
 	{ "name": "K_0_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_6", "role": "q0" }} , 
 	{ "name": "K_0_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_7", "role": "address0" }} , 
 	{ "name": "K_0_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_7", "role": "ce0" }} , 
 	{ "name": "K_0_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_0_1_7", "role": "q0" }} , 
 	{ "name": "K_1_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_0", "role": "address0" }} , 
 	{ "name": "K_1_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_0", "role": "ce0" }} , 
 	{ "name": "K_1_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_0", "role": "q0" }} , 
 	{ "name": "K_1_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_1", "role": "address0" }} , 
 	{ "name": "K_1_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_1", "role": "ce0" }} , 
 	{ "name": "K_1_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_1", "role": "q0" }} , 
 	{ "name": "K_1_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_2", "role": "address0" }} , 
 	{ "name": "K_1_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_2", "role": "ce0" }} , 
 	{ "name": "K_1_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_2", "role": "q0" }} , 
 	{ "name": "K_1_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_3", "role": "address0" }} , 
 	{ "name": "K_1_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_3", "role": "ce0" }} , 
 	{ "name": "K_1_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_3", "role": "q0" }} , 
 	{ "name": "K_1_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_4", "role": "address0" }} , 
 	{ "name": "K_1_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_4", "role": "ce0" }} , 
 	{ "name": "K_1_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_4", "role": "q0" }} , 
 	{ "name": "K_1_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_5", "role": "address0" }} , 
 	{ "name": "K_1_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_5", "role": "ce0" }} , 
 	{ "name": "K_1_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_5", "role": "q0" }} , 
 	{ "name": "K_1_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_6", "role": "address0" }} , 
 	{ "name": "K_1_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_6", "role": "ce0" }} , 
 	{ "name": "K_1_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_6", "role": "q0" }} , 
 	{ "name": "K_1_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_7", "role": "address0" }} , 
 	{ "name": "K_1_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_7", "role": "ce0" }} , 
 	{ "name": "K_1_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_0_7", "role": "q0" }} , 
 	{ "name": "K_1_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_0", "role": "address0" }} , 
 	{ "name": "K_1_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_0", "role": "ce0" }} , 
 	{ "name": "K_1_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_0", "role": "q0" }} , 
 	{ "name": "K_1_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_1", "role": "address0" }} , 
 	{ "name": "K_1_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_1", "role": "ce0" }} , 
 	{ "name": "K_1_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_1", "role": "q0" }} , 
 	{ "name": "K_1_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_2", "role": "address0" }} , 
 	{ "name": "K_1_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_2", "role": "ce0" }} , 
 	{ "name": "K_1_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_2", "role": "q0" }} , 
 	{ "name": "K_1_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_3", "role": "address0" }} , 
 	{ "name": "K_1_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_3", "role": "ce0" }} , 
 	{ "name": "K_1_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_3", "role": "q0" }} , 
 	{ "name": "K_1_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_4", "role": "address0" }} , 
 	{ "name": "K_1_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_4", "role": "ce0" }} , 
 	{ "name": "K_1_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_4", "role": "q0" }} , 
 	{ "name": "K_1_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_5", "role": "address0" }} , 
 	{ "name": "K_1_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_5", "role": "ce0" }} , 
 	{ "name": "K_1_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_5", "role": "q0" }} , 
 	{ "name": "K_1_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_6", "role": "address0" }} , 
 	{ "name": "K_1_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_6", "role": "ce0" }} , 
 	{ "name": "K_1_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_6", "role": "q0" }} , 
 	{ "name": "K_1_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_7", "role": "address0" }} , 
 	{ "name": "K_1_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_7", "role": "ce0" }} , 
 	{ "name": "K_1_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_1_1_7", "role": "q0" }} , 
 	{ "name": "K_2_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_0", "role": "address0" }} , 
 	{ "name": "K_2_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_0", "role": "ce0" }} , 
 	{ "name": "K_2_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_0", "role": "q0" }} , 
 	{ "name": "K_2_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_1", "role": "address0" }} , 
 	{ "name": "K_2_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_1", "role": "ce0" }} , 
 	{ "name": "K_2_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_1", "role": "q0" }} , 
 	{ "name": "K_2_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_2", "role": "address0" }} , 
 	{ "name": "K_2_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_2", "role": "ce0" }} , 
 	{ "name": "K_2_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_2", "role": "q0" }} , 
 	{ "name": "K_2_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_3", "role": "address0" }} , 
 	{ "name": "K_2_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_3", "role": "ce0" }} , 
 	{ "name": "K_2_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_3", "role": "q0" }} , 
 	{ "name": "K_2_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_4", "role": "address0" }} , 
 	{ "name": "K_2_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_4", "role": "ce0" }} , 
 	{ "name": "K_2_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_4", "role": "q0" }} , 
 	{ "name": "K_2_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_5", "role": "address0" }} , 
 	{ "name": "K_2_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_5", "role": "ce0" }} , 
 	{ "name": "K_2_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_5", "role": "q0" }} , 
 	{ "name": "K_2_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_6", "role": "address0" }} , 
 	{ "name": "K_2_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_6", "role": "ce0" }} , 
 	{ "name": "K_2_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_6", "role": "q0" }} , 
 	{ "name": "K_2_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_7", "role": "address0" }} , 
 	{ "name": "K_2_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_7", "role": "ce0" }} , 
 	{ "name": "K_2_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_0_7", "role": "q0" }} , 
 	{ "name": "K_2_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_0", "role": "address0" }} , 
 	{ "name": "K_2_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_0", "role": "ce0" }} , 
 	{ "name": "K_2_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_0", "role": "q0" }} , 
 	{ "name": "K_2_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_1", "role": "address0" }} , 
 	{ "name": "K_2_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_1", "role": "ce0" }} , 
 	{ "name": "K_2_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_1", "role": "q0" }} , 
 	{ "name": "K_2_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_2", "role": "address0" }} , 
 	{ "name": "K_2_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_2", "role": "ce0" }} , 
 	{ "name": "K_2_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_2", "role": "q0" }} , 
 	{ "name": "K_2_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_3", "role": "address0" }} , 
 	{ "name": "K_2_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_3", "role": "ce0" }} , 
 	{ "name": "K_2_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_3", "role": "q0" }} , 
 	{ "name": "K_2_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_4", "role": "address0" }} , 
 	{ "name": "K_2_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_4", "role": "ce0" }} , 
 	{ "name": "K_2_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_4", "role": "q0" }} , 
 	{ "name": "K_2_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_5", "role": "address0" }} , 
 	{ "name": "K_2_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_5", "role": "ce0" }} , 
 	{ "name": "K_2_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_5", "role": "q0" }} , 
 	{ "name": "K_2_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_6", "role": "address0" }} , 
 	{ "name": "K_2_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_6", "role": "ce0" }} , 
 	{ "name": "K_2_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_6", "role": "q0" }} , 
 	{ "name": "K_2_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_7", "role": "address0" }} , 
 	{ "name": "K_2_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_7", "role": "ce0" }} , 
 	{ "name": "K_2_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_2_1_7", "role": "q0" }} , 
 	{ "name": "K_3_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_0", "role": "address0" }} , 
 	{ "name": "K_3_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_0", "role": "ce0" }} , 
 	{ "name": "K_3_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_0", "role": "q0" }} , 
 	{ "name": "K_3_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_1", "role": "address0" }} , 
 	{ "name": "K_3_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_1", "role": "ce0" }} , 
 	{ "name": "K_3_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_1", "role": "q0" }} , 
 	{ "name": "K_3_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_2", "role": "address0" }} , 
 	{ "name": "K_3_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_2", "role": "ce0" }} , 
 	{ "name": "K_3_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_2", "role": "q0" }} , 
 	{ "name": "K_3_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_3", "role": "address0" }} , 
 	{ "name": "K_3_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_3", "role": "ce0" }} , 
 	{ "name": "K_3_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_3", "role": "q0" }} , 
 	{ "name": "K_3_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_4", "role": "address0" }} , 
 	{ "name": "K_3_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_4", "role": "ce0" }} , 
 	{ "name": "K_3_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_4", "role": "q0" }} , 
 	{ "name": "K_3_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_5", "role": "address0" }} , 
 	{ "name": "K_3_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_5", "role": "ce0" }} , 
 	{ "name": "K_3_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_5", "role": "q0" }} , 
 	{ "name": "K_3_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_6", "role": "address0" }} , 
 	{ "name": "K_3_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_6", "role": "ce0" }} , 
 	{ "name": "K_3_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_6", "role": "q0" }} , 
 	{ "name": "K_3_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_7", "role": "address0" }} , 
 	{ "name": "K_3_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_7", "role": "ce0" }} , 
 	{ "name": "K_3_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_0_7", "role": "q0" }} , 
 	{ "name": "K_3_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_0", "role": "address0" }} , 
 	{ "name": "K_3_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_0", "role": "ce0" }} , 
 	{ "name": "K_3_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_0", "role": "q0" }} , 
 	{ "name": "K_3_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_1", "role": "address0" }} , 
 	{ "name": "K_3_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_1", "role": "ce0" }} , 
 	{ "name": "K_3_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_1", "role": "q0" }} , 
 	{ "name": "K_3_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_2", "role": "address0" }} , 
 	{ "name": "K_3_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_2", "role": "ce0" }} , 
 	{ "name": "K_3_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_2", "role": "q0" }} , 
 	{ "name": "K_3_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_3", "role": "address0" }} , 
 	{ "name": "K_3_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_3", "role": "ce0" }} , 
 	{ "name": "K_3_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_3", "role": "q0" }} , 
 	{ "name": "K_3_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_4", "role": "address0" }} , 
 	{ "name": "K_3_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_4", "role": "ce0" }} , 
 	{ "name": "K_3_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_4", "role": "q0" }} , 
 	{ "name": "K_3_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_5", "role": "address0" }} , 
 	{ "name": "K_3_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_5", "role": "ce0" }} , 
 	{ "name": "K_3_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_5", "role": "q0" }} , 
 	{ "name": "K_3_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_6", "role": "address0" }} , 
 	{ "name": "K_3_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_6", "role": "ce0" }} , 
 	{ "name": "K_3_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_6", "role": "q0" }} , 
 	{ "name": "K_3_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_7", "role": "address0" }} , 
 	{ "name": "K_3_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_7", "role": "ce0" }} , 
 	{ "name": "K_3_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "K_3_1_7", "role": "q0" }} , 
 	{ "name": "kk_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "kk_0", "role": "address0" }} , 
 	{ "name": "kk_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "kk_0", "role": "ce0" }} , 
 	{ "name": "kk_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "kk_0", "role": "q0" }} , 
 	{ "name": "kk_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "kk_1", "role": "address0" }} , 
 	{ "name": "kk_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "kk_1", "role": "ce0" }} , 
 	{ "name": "kk_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "kk_1", "role": "q0" }} , 
 	{ "name": "x_out_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_0", "role": "address0" }} , 
 	{ "name": "x_out_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "ce0" }} , 
 	{ "name": "x_out_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_0", "role": "q0" }} , 
 	{ "name": "x_out_0_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_0", "role": "address1" }} , 
 	{ "name": "x_out_0_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "ce1" }} , 
 	{ "name": "x_out_0_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "we1" }} , 
 	{ "name": "x_out_0_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_0", "role": "d1" }} , 
 	{ "name": "x_out_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_1", "role": "address0" }} , 
 	{ "name": "x_out_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "ce0" }} , 
 	{ "name": "x_out_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_1", "role": "q0" }} , 
 	{ "name": "x_out_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_1", "role": "address1" }} , 
 	{ "name": "x_out_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "ce1" }} , 
 	{ "name": "x_out_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "we1" }} , 
 	{ "name": "x_out_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_1", "role": "d1" }} , 
 	{ "name": "x_out_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_2", "role": "address0" }} , 
 	{ "name": "x_out_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "ce0" }} , 
 	{ "name": "x_out_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_2", "role": "q0" }} , 
 	{ "name": "x_out_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_2", "role": "address1" }} , 
 	{ "name": "x_out_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "ce1" }} , 
 	{ "name": "x_out_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "we1" }} , 
 	{ "name": "x_out_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_2", "role": "d1" }} , 
 	{ "name": "x_out_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_3", "role": "address0" }} , 
 	{ "name": "x_out_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "ce0" }} , 
 	{ "name": "x_out_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_3", "role": "q0" }} , 
 	{ "name": "x_out_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_3", "role": "address1" }} , 
 	{ "name": "x_out_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "ce1" }} , 
 	{ "name": "x_out_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "we1" }} , 
 	{ "name": "x_out_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_3", "role": "d1" }} , 
 	{ "name": "x_out_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_4", "role": "address0" }} , 
 	{ "name": "x_out_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "ce0" }} , 
 	{ "name": "x_out_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_4", "role": "q0" }} , 
 	{ "name": "x_out_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_4", "role": "address1" }} , 
 	{ "name": "x_out_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "ce1" }} , 
 	{ "name": "x_out_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "we1" }} , 
 	{ "name": "x_out_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_4", "role": "d1" }} , 
 	{ "name": "x_out_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_5", "role": "address0" }} , 
 	{ "name": "x_out_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "ce0" }} , 
 	{ "name": "x_out_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_5", "role": "q0" }} , 
 	{ "name": "x_out_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_5", "role": "address1" }} , 
 	{ "name": "x_out_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "ce1" }} , 
 	{ "name": "x_out_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "we1" }} , 
 	{ "name": "x_out_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_5", "role": "d1" }} , 
 	{ "name": "x_out_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_6", "role": "address0" }} , 
 	{ "name": "x_out_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_6", "role": "ce0" }} , 
 	{ "name": "x_out_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_6", "role": "q0" }} , 
 	{ "name": "x_out_6_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_6", "role": "address1" }} , 
 	{ "name": "x_out_6_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_6", "role": "ce1" }} , 
 	{ "name": "x_out_6_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_6", "role": "we1" }} , 
 	{ "name": "x_out_6_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_6", "role": "d1" }} , 
 	{ "name": "x_out_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_7", "role": "address0" }} , 
 	{ "name": "x_out_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_7", "role": "ce0" }} , 
 	{ "name": "x_out_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_7", "role": "q0" }} , 
 	{ "name": "x_out_7_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_7", "role": "address1" }} , 
 	{ "name": "x_out_7_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_7", "role": "ce1" }} , 
 	{ "name": "x_out_7_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_7", "role": "we1" }} , 
 	{ "name": "x_out_7_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "x_out_7", "role": "d1" }} , 
 	{ "name": "u_out_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "u_out_0", "role": "address0" }} , 
 	{ "name": "u_out_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_0", "role": "ce0" }} , 
 	{ "name": "u_out_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_0", "role": "we0" }} , 
 	{ "name": "u_out_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "u_out_0", "role": "d0" }} , 
 	{ "name": "u_out_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "u_out_1", "role": "address0" }} , 
 	{ "name": "u_out_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_1", "role": "ce0" }} , 
 	{ "name": "u_out_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_1", "role": "we0" }} , 
 	{ "name": "u_out_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "u_out_1", "role": "d0" }}  ]}

set ArgLastReadFirstWriteLatency {
	riccati_forward_pass {
		step_data_0_0_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_0_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_1_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_2_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_3_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_4_5 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_0 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_1 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_2 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_3 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_4 {Type I LastRead 6 FirstWrite -1}
		step_data_0_5_5 {Type I LastRead 6 FirstWrite -1}
		step_data_1 {Type I LastRead 7 FirstWrite -1}
		B_sparse_0 {Type I LastRead 6 FirstWrite -1}
		B_sparse_1 {Type I LastRead 6 FirstWrite -1}
		B_sparse_2 {Type I LastRead 6 FirstWrite -1}
		B_sparse_3 {Type I LastRead 6 FirstWrite -1}
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 0 FirstWrite -1}
		p_read6 {Type I LastRead 0 FirstWrite -1}
		p_read7 {Type I LastRead 0 FirstWrite -1}
		K_0_0_0 {Type I LastRead 1 FirstWrite -1}
		K_0_0_1 {Type I LastRead 1 FirstWrite -1}
		K_0_0_2 {Type I LastRead 1 FirstWrite -1}
		K_0_0_3 {Type I LastRead 1 FirstWrite -1}
		K_0_0_4 {Type I LastRead 1 FirstWrite -1}
		K_0_0_5 {Type I LastRead 1 FirstWrite -1}
		K_0_0_6 {Type I LastRead 1 FirstWrite -1}
		K_0_0_7 {Type I LastRead 1 FirstWrite -1}
		K_0_1_0 {Type I LastRead 1 FirstWrite -1}
		K_0_1_1 {Type I LastRead 1 FirstWrite -1}
		K_0_1_2 {Type I LastRead 1 FirstWrite -1}
		K_0_1_3 {Type I LastRead 1 FirstWrite -1}
		K_0_1_4 {Type I LastRead 1 FirstWrite -1}
		K_0_1_5 {Type I LastRead 1 FirstWrite -1}
		K_0_1_6 {Type I LastRead 1 FirstWrite -1}
		K_0_1_7 {Type I LastRead 1 FirstWrite -1}
		K_1_0_0 {Type I LastRead 1 FirstWrite -1}
		K_1_0_1 {Type I LastRead 1 FirstWrite -1}
		K_1_0_2 {Type I LastRead 1 FirstWrite -1}
		K_1_0_3 {Type I LastRead 1 FirstWrite -1}
		K_1_0_4 {Type I LastRead 1 FirstWrite -1}
		K_1_0_5 {Type I LastRead 1 FirstWrite -1}
		K_1_0_6 {Type I LastRead 1 FirstWrite -1}
		K_1_0_7 {Type I LastRead 1 FirstWrite -1}
		K_1_1_0 {Type I LastRead 1 FirstWrite -1}
		K_1_1_1 {Type I LastRead 1 FirstWrite -1}
		K_1_1_2 {Type I LastRead 1 FirstWrite -1}
		K_1_1_3 {Type I LastRead 1 FirstWrite -1}
		K_1_1_4 {Type I LastRead 1 FirstWrite -1}
		K_1_1_5 {Type I LastRead 1 FirstWrite -1}
		K_1_1_6 {Type I LastRead 1 FirstWrite -1}
		K_1_1_7 {Type I LastRead 1 FirstWrite -1}
		K_2_0_0 {Type I LastRead 1 FirstWrite -1}
		K_2_0_1 {Type I LastRead 1 FirstWrite -1}
		K_2_0_2 {Type I LastRead 1 FirstWrite -1}
		K_2_0_3 {Type I LastRead 1 FirstWrite -1}
		K_2_0_4 {Type I LastRead 1 FirstWrite -1}
		K_2_0_5 {Type I LastRead 1 FirstWrite -1}
		K_2_0_6 {Type I LastRead 1 FirstWrite -1}
		K_2_0_7 {Type I LastRead 1 FirstWrite -1}
		K_2_1_0 {Type I LastRead 1 FirstWrite -1}
		K_2_1_1 {Type I LastRead 1 FirstWrite -1}
		K_2_1_2 {Type I LastRead 1 FirstWrite -1}
		K_2_1_3 {Type I LastRead 1 FirstWrite -1}
		K_2_1_4 {Type I LastRead 1 FirstWrite -1}
		K_2_1_5 {Type I LastRead 1 FirstWrite -1}
		K_2_1_6 {Type I LastRead 1 FirstWrite -1}
		K_2_1_7 {Type I LastRead 1 FirstWrite -1}
		K_3_0_0 {Type I LastRead 1 FirstWrite -1}
		K_3_0_1 {Type I LastRead 1 FirstWrite -1}
		K_3_0_2 {Type I LastRead 1 FirstWrite -1}
		K_3_0_3 {Type I LastRead 1 FirstWrite -1}
		K_3_0_4 {Type I LastRead 1 FirstWrite -1}
		K_3_0_5 {Type I LastRead 1 FirstWrite -1}
		K_3_0_6 {Type I LastRead 1 FirstWrite -1}
		K_3_0_7 {Type I LastRead 1 FirstWrite -1}
		K_3_1_0 {Type I LastRead 1 FirstWrite -1}
		K_3_1_1 {Type I LastRead 1 FirstWrite -1}
		K_3_1_2 {Type I LastRead 1 FirstWrite -1}
		K_3_1_3 {Type I LastRead 1 FirstWrite -1}
		K_3_1_4 {Type I LastRead 1 FirstWrite -1}
		K_3_1_5 {Type I LastRead 1 FirstWrite -1}
		K_3_1_6 {Type I LastRead 1 FirstWrite -1}
		K_3_1_7 {Type I LastRead 1 FirstWrite -1}
		kk_0 {Type I LastRead 5 FirstWrite -1}
		kk_1 {Type I LastRead 5 FirstWrite -1}
		x_out_0 {Type IO LastRead 1 FirstWrite 0}
		x_out_1 {Type IO LastRead 1 FirstWrite 0}
		x_out_2 {Type IO LastRead 1 FirstWrite 0}
		x_out_3 {Type IO LastRead 1 FirstWrite 0}
		x_out_4 {Type IO LastRead 1 FirstWrite 0}
		x_out_5 {Type IO LastRead 1 FirstWrite 0}
		x_out_6 {Type IO LastRead 1 FirstWrite 0}
		x_out_7 {Type IO LastRead 1 FirstWrite 0}
		u_out_0 {Type O LastRead -1 FirstWrite 6}
		u_out_1 {Type O LastRead -1 FirstWrite 6}}
	sum8_K_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_K_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
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
	{"Name" : "Latency", "Min" : "381", "Max" : "381"}
	, {"Name" : "Interval", "Min" : "381", "Max" : "381"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	step_data_0_0_0 { ap_memory {  { step_data_0_0_0_address0 mem_address 1 5 }  { step_data_0_0_0_ce0 mem_ce 1 1 }  { step_data_0_0_0_q0 mem_dout 0 32 } } }
	step_data_0_0_1 { ap_memory {  { step_data_0_0_1_address0 mem_address 1 5 }  { step_data_0_0_1_ce0 mem_ce 1 1 }  { step_data_0_0_1_q0 mem_dout 0 32 } } }
	step_data_0_0_2 { ap_memory {  { step_data_0_0_2_address0 mem_address 1 5 }  { step_data_0_0_2_ce0 mem_ce 1 1 }  { step_data_0_0_2_q0 mem_dout 0 32 } } }
	step_data_0_0_3 { ap_memory {  { step_data_0_0_3_address0 mem_address 1 5 }  { step_data_0_0_3_ce0 mem_ce 1 1 }  { step_data_0_0_3_q0 mem_dout 0 32 } } }
	step_data_0_0_4 { ap_memory {  { step_data_0_0_4_address0 mem_address 1 5 }  { step_data_0_0_4_ce0 mem_ce 1 1 }  { step_data_0_0_4_q0 mem_dout 0 32 } } }
	step_data_0_0_5 { ap_memory {  { step_data_0_0_5_address0 mem_address 1 5 }  { step_data_0_0_5_ce0 mem_ce 1 1 }  { step_data_0_0_5_q0 mem_dout 0 32 } } }
	step_data_0_1_0 { ap_memory {  { step_data_0_1_0_address0 mem_address 1 5 }  { step_data_0_1_0_ce0 mem_ce 1 1 }  { step_data_0_1_0_q0 mem_dout 0 32 } } }
	step_data_0_1_1 { ap_memory {  { step_data_0_1_1_address0 mem_address 1 5 }  { step_data_0_1_1_ce0 mem_ce 1 1 }  { step_data_0_1_1_q0 mem_dout 0 32 } } }
	step_data_0_1_2 { ap_memory {  { step_data_0_1_2_address0 mem_address 1 5 }  { step_data_0_1_2_ce0 mem_ce 1 1 }  { step_data_0_1_2_q0 mem_dout 0 32 } } }
	step_data_0_1_3 { ap_memory {  { step_data_0_1_3_address0 mem_address 1 5 }  { step_data_0_1_3_ce0 mem_ce 1 1 }  { step_data_0_1_3_q0 mem_dout 0 32 } } }
	step_data_0_1_4 { ap_memory {  { step_data_0_1_4_address0 mem_address 1 5 }  { step_data_0_1_4_ce0 mem_ce 1 1 }  { step_data_0_1_4_q0 mem_dout 0 32 } } }
	step_data_0_1_5 { ap_memory {  { step_data_0_1_5_address0 mem_address 1 5 }  { step_data_0_1_5_ce0 mem_ce 1 1 }  { step_data_0_1_5_q0 mem_dout 0 32 } } }
	step_data_0_2_0 { ap_memory {  { step_data_0_2_0_address0 mem_address 1 5 }  { step_data_0_2_0_ce0 mem_ce 1 1 }  { step_data_0_2_0_q0 mem_dout 0 32 } } }
	step_data_0_2_1 { ap_memory {  { step_data_0_2_1_address0 mem_address 1 5 }  { step_data_0_2_1_ce0 mem_ce 1 1 }  { step_data_0_2_1_q0 mem_dout 0 32 } } }
	step_data_0_2_2 { ap_memory {  { step_data_0_2_2_address0 mem_address 1 5 }  { step_data_0_2_2_ce0 mem_ce 1 1 }  { step_data_0_2_2_q0 mem_dout 0 32 } } }
	step_data_0_2_3 { ap_memory {  { step_data_0_2_3_address0 mem_address 1 5 }  { step_data_0_2_3_ce0 mem_ce 1 1 }  { step_data_0_2_3_q0 mem_dout 0 32 } } }
	step_data_0_2_4 { ap_memory {  { step_data_0_2_4_address0 mem_address 1 5 }  { step_data_0_2_4_ce0 mem_ce 1 1 }  { step_data_0_2_4_q0 mem_dout 0 32 } } }
	step_data_0_2_5 { ap_memory {  { step_data_0_2_5_address0 mem_address 1 5 }  { step_data_0_2_5_ce0 mem_ce 1 1 }  { step_data_0_2_5_q0 mem_dout 0 32 } } }
	step_data_0_3_0 { ap_memory {  { step_data_0_3_0_address0 mem_address 1 5 }  { step_data_0_3_0_ce0 mem_ce 1 1 }  { step_data_0_3_0_q0 mem_dout 0 32 } } }
	step_data_0_3_1 { ap_memory {  { step_data_0_3_1_address0 mem_address 1 5 }  { step_data_0_3_1_ce0 mem_ce 1 1 }  { step_data_0_3_1_q0 mem_dout 0 32 } } }
	step_data_0_3_2 { ap_memory {  { step_data_0_3_2_address0 mem_address 1 5 }  { step_data_0_3_2_ce0 mem_ce 1 1 }  { step_data_0_3_2_q0 mem_dout 0 32 } } }
	step_data_0_3_3 { ap_memory {  { step_data_0_3_3_address0 mem_address 1 5 }  { step_data_0_3_3_ce0 mem_ce 1 1 }  { step_data_0_3_3_q0 mem_dout 0 32 } } }
	step_data_0_3_4 { ap_memory {  { step_data_0_3_4_address0 mem_address 1 5 }  { step_data_0_3_4_ce0 mem_ce 1 1 }  { step_data_0_3_4_q0 mem_dout 0 32 } } }
	step_data_0_3_5 { ap_memory {  { step_data_0_3_5_address0 mem_address 1 5 }  { step_data_0_3_5_ce0 mem_ce 1 1 }  { step_data_0_3_5_q0 mem_dout 0 32 } } }
	step_data_0_4_0 { ap_memory {  { step_data_0_4_0_address0 mem_address 1 5 }  { step_data_0_4_0_ce0 mem_ce 1 1 }  { step_data_0_4_0_q0 mem_dout 0 32 } } }
	step_data_0_4_1 { ap_memory {  { step_data_0_4_1_address0 mem_address 1 5 }  { step_data_0_4_1_ce0 mem_ce 1 1 }  { step_data_0_4_1_q0 mem_dout 0 32 } } }
	step_data_0_4_2 { ap_memory {  { step_data_0_4_2_address0 mem_address 1 5 }  { step_data_0_4_2_ce0 mem_ce 1 1 }  { step_data_0_4_2_q0 mem_dout 0 32 } } }
	step_data_0_4_3 { ap_memory {  { step_data_0_4_3_address0 mem_address 1 5 }  { step_data_0_4_3_ce0 mem_ce 1 1 }  { step_data_0_4_3_q0 mem_dout 0 32 } } }
	step_data_0_4_4 { ap_memory {  { step_data_0_4_4_address0 mem_address 1 5 }  { step_data_0_4_4_ce0 mem_ce 1 1 }  { step_data_0_4_4_q0 mem_dout 0 32 } } }
	step_data_0_4_5 { ap_memory {  { step_data_0_4_5_address0 mem_address 1 5 }  { step_data_0_4_5_ce0 mem_ce 1 1 }  { step_data_0_4_5_q0 mem_dout 0 32 } } }
	step_data_0_5_0 { ap_memory {  { step_data_0_5_0_address0 mem_address 1 5 }  { step_data_0_5_0_ce0 mem_ce 1 1 }  { step_data_0_5_0_q0 mem_dout 0 32 } } }
	step_data_0_5_1 { ap_memory {  { step_data_0_5_1_address0 mem_address 1 5 }  { step_data_0_5_1_ce0 mem_ce 1 1 }  { step_data_0_5_1_q0 mem_dout 0 32 } } }
	step_data_0_5_2 { ap_memory {  { step_data_0_5_2_address0 mem_address 1 5 }  { step_data_0_5_2_ce0 mem_ce 1 1 }  { step_data_0_5_2_q0 mem_dout 0 32 } } }
	step_data_0_5_3 { ap_memory {  { step_data_0_5_3_address0 mem_address 1 5 }  { step_data_0_5_3_ce0 mem_ce 1 1 }  { step_data_0_5_3_q0 mem_dout 0 32 } } }
	step_data_0_5_4 { ap_memory {  { step_data_0_5_4_address0 mem_address 1 5 }  { step_data_0_5_4_ce0 mem_ce 1 1 }  { step_data_0_5_4_q0 mem_dout 0 32 } } }
	step_data_0_5_5 { ap_memory {  { step_data_0_5_5_address0 mem_address 1 5 }  { step_data_0_5_5_ce0 mem_ce 1 1 }  { step_data_0_5_5_q0 mem_dout 0 32 } } }
	step_data_1 { ap_memory {  { step_data_1_address0 mem_address 1 7 }  { step_data_1_ce0 mem_ce 1 1 }  { step_data_1_q0 mem_dout 0 32 }  { step_data_1_address1 MemPortADDR2 1 7 }  { step_data_1_ce1 MemPortCE2 1 1 }  { step_data_1_q1 MemPortDOUT2 0 32 } } }
	B_sparse_0 { ap_memory {  { B_sparse_0_address0 mem_address 1 5 }  { B_sparse_0_ce0 mem_ce 1 1 }  { B_sparse_0_q0 mem_dout 0 32 } } }
	B_sparse_1 { ap_memory {  { B_sparse_1_address0 mem_address 1 5 }  { B_sparse_1_ce0 mem_ce 1 1 }  { B_sparse_1_q0 mem_dout 0 32 } } }
	B_sparse_2 { ap_memory {  { B_sparse_2_address0 mem_address 1 5 }  { B_sparse_2_ce0 mem_ce 1 1 }  { B_sparse_2_q0 mem_dout 0 32 } } }
	B_sparse_3 { ap_memory {  { B_sparse_3_address0 mem_address 1 5 }  { B_sparse_3_ce0 mem_ce 1 1 }  { B_sparse_3_q0 mem_dout 0 32 } } }
	p_read { ap_none {  { p_read in_data 0 32 } } }
	p_read1 { ap_none {  { p_read1 in_data 0 32 } } }
	p_read2 { ap_none {  { p_read2 in_data 0 32 } } }
	p_read3 { ap_none {  { p_read3 in_data 0 32 } } }
	p_read4 { ap_none {  { p_read4 in_data 0 32 } } }
	p_read5 { ap_none {  { p_read5 in_data 0 32 } } }
	p_read6 { ap_none {  { p_read6 in_data 0 32 } } }
	p_read7 { ap_none {  { p_read7 in_data 0 32 } } }
	K_0_0_0 { ap_memory {  { K_0_0_0_address0 mem_address 1 3 }  { K_0_0_0_ce0 mem_ce 1 1 }  { K_0_0_0_q0 mem_dout 0 26 } } }
	K_0_0_1 { ap_memory {  { K_0_0_1_address0 mem_address 1 3 }  { K_0_0_1_ce0 mem_ce 1 1 }  { K_0_0_1_q0 mem_dout 0 26 } } }
	K_0_0_2 { ap_memory {  { K_0_0_2_address0 mem_address 1 3 }  { K_0_0_2_ce0 mem_ce 1 1 }  { K_0_0_2_q0 mem_dout 0 26 } } }
	K_0_0_3 { ap_memory {  { K_0_0_3_address0 mem_address 1 3 }  { K_0_0_3_ce0 mem_ce 1 1 }  { K_0_0_3_q0 mem_dout 0 26 } } }
	K_0_0_4 { ap_memory {  { K_0_0_4_address0 mem_address 1 3 }  { K_0_0_4_ce0 mem_ce 1 1 }  { K_0_0_4_q0 mem_dout 0 26 } } }
	K_0_0_5 { ap_memory {  { K_0_0_5_address0 mem_address 1 3 }  { K_0_0_5_ce0 mem_ce 1 1 }  { K_0_0_5_q0 mem_dout 0 26 } } }
	K_0_0_6 { ap_memory {  { K_0_0_6_address0 mem_address 1 3 }  { K_0_0_6_ce0 mem_ce 1 1 }  { K_0_0_6_q0 mem_dout 0 26 } } }
	K_0_0_7 { ap_memory {  { K_0_0_7_address0 mem_address 1 3 }  { K_0_0_7_ce0 mem_ce 1 1 }  { K_0_0_7_q0 mem_dout 0 26 } } }
	K_0_1_0 { ap_memory {  { K_0_1_0_address0 mem_address 1 3 }  { K_0_1_0_ce0 mem_ce 1 1 }  { K_0_1_0_q0 mem_dout 0 26 } } }
	K_0_1_1 { ap_memory {  { K_0_1_1_address0 mem_address 1 3 }  { K_0_1_1_ce0 mem_ce 1 1 }  { K_0_1_1_q0 mem_dout 0 26 } } }
	K_0_1_2 { ap_memory {  { K_0_1_2_address0 mem_address 1 3 }  { K_0_1_2_ce0 mem_ce 1 1 }  { K_0_1_2_q0 mem_dout 0 26 } } }
	K_0_1_3 { ap_memory {  { K_0_1_3_address0 mem_address 1 3 }  { K_0_1_3_ce0 mem_ce 1 1 }  { K_0_1_3_q0 mem_dout 0 26 } } }
	K_0_1_4 { ap_memory {  { K_0_1_4_address0 mem_address 1 3 }  { K_0_1_4_ce0 mem_ce 1 1 }  { K_0_1_4_q0 mem_dout 0 26 } } }
	K_0_1_5 { ap_memory {  { K_0_1_5_address0 mem_address 1 3 }  { K_0_1_5_ce0 mem_ce 1 1 }  { K_0_1_5_q0 mem_dout 0 26 } } }
	K_0_1_6 { ap_memory {  { K_0_1_6_address0 mem_address 1 3 }  { K_0_1_6_ce0 mem_ce 1 1 }  { K_0_1_6_q0 mem_dout 0 26 } } }
	K_0_1_7 { ap_memory {  { K_0_1_7_address0 mem_address 1 3 }  { K_0_1_7_ce0 mem_ce 1 1 }  { K_0_1_7_q0 mem_dout 0 26 } } }
	K_1_0_0 { ap_memory {  { K_1_0_0_address0 mem_address 1 3 }  { K_1_0_0_ce0 mem_ce 1 1 }  { K_1_0_0_q0 mem_dout 0 26 } } }
	K_1_0_1 { ap_memory {  { K_1_0_1_address0 mem_address 1 3 }  { K_1_0_1_ce0 mem_ce 1 1 }  { K_1_0_1_q0 mem_dout 0 26 } } }
	K_1_0_2 { ap_memory {  { K_1_0_2_address0 mem_address 1 3 }  { K_1_0_2_ce0 mem_ce 1 1 }  { K_1_0_2_q0 mem_dout 0 26 } } }
	K_1_0_3 { ap_memory {  { K_1_0_3_address0 mem_address 1 3 }  { K_1_0_3_ce0 mem_ce 1 1 }  { K_1_0_3_q0 mem_dout 0 26 } } }
	K_1_0_4 { ap_memory {  { K_1_0_4_address0 mem_address 1 3 }  { K_1_0_4_ce0 mem_ce 1 1 }  { K_1_0_4_q0 mem_dout 0 26 } } }
	K_1_0_5 { ap_memory {  { K_1_0_5_address0 mem_address 1 3 }  { K_1_0_5_ce0 mem_ce 1 1 }  { K_1_0_5_q0 mem_dout 0 26 } } }
	K_1_0_6 { ap_memory {  { K_1_0_6_address0 mem_address 1 3 }  { K_1_0_6_ce0 mem_ce 1 1 }  { K_1_0_6_q0 mem_dout 0 26 } } }
	K_1_0_7 { ap_memory {  { K_1_0_7_address0 mem_address 1 3 }  { K_1_0_7_ce0 mem_ce 1 1 }  { K_1_0_7_q0 mem_dout 0 26 } } }
	K_1_1_0 { ap_memory {  { K_1_1_0_address0 mem_address 1 3 }  { K_1_1_0_ce0 mem_ce 1 1 }  { K_1_1_0_q0 mem_dout 0 26 } } }
	K_1_1_1 { ap_memory {  { K_1_1_1_address0 mem_address 1 3 }  { K_1_1_1_ce0 mem_ce 1 1 }  { K_1_1_1_q0 mem_dout 0 26 } } }
	K_1_1_2 { ap_memory {  { K_1_1_2_address0 mem_address 1 3 }  { K_1_1_2_ce0 mem_ce 1 1 }  { K_1_1_2_q0 mem_dout 0 26 } } }
	K_1_1_3 { ap_memory {  { K_1_1_3_address0 mem_address 1 3 }  { K_1_1_3_ce0 mem_ce 1 1 }  { K_1_1_3_q0 mem_dout 0 26 } } }
	K_1_1_4 { ap_memory {  { K_1_1_4_address0 mem_address 1 3 }  { K_1_1_4_ce0 mem_ce 1 1 }  { K_1_1_4_q0 mem_dout 0 26 } } }
	K_1_1_5 { ap_memory {  { K_1_1_5_address0 mem_address 1 3 }  { K_1_1_5_ce0 mem_ce 1 1 }  { K_1_1_5_q0 mem_dout 0 26 } } }
	K_1_1_6 { ap_memory {  { K_1_1_6_address0 mem_address 1 3 }  { K_1_1_6_ce0 mem_ce 1 1 }  { K_1_1_6_q0 mem_dout 0 26 } } }
	K_1_1_7 { ap_memory {  { K_1_1_7_address0 mem_address 1 3 }  { K_1_1_7_ce0 mem_ce 1 1 }  { K_1_1_7_q0 mem_dout 0 26 } } }
	K_2_0_0 { ap_memory {  { K_2_0_0_address0 mem_address 1 3 }  { K_2_0_0_ce0 mem_ce 1 1 }  { K_2_0_0_q0 mem_dout 0 26 } } }
	K_2_0_1 { ap_memory {  { K_2_0_1_address0 mem_address 1 3 }  { K_2_0_1_ce0 mem_ce 1 1 }  { K_2_0_1_q0 mem_dout 0 26 } } }
	K_2_0_2 { ap_memory {  { K_2_0_2_address0 mem_address 1 3 }  { K_2_0_2_ce0 mem_ce 1 1 }  { K_2_0_2_q0 mem_dout 0 26 } } }
	K_2_0_3 { ap_memory {  { K_2_0_3_address0 mem_address 1 3 }  { K_2_0_3_ce0 mem_ce 1 1 }  { K_2_0_3_q0 mem_dout 0 26 } } }
	K_2_0_4 { ap_memory {  { K_2_0_4_address0 mem_address 1 3 }  { K_2_0_4_ce0 mem_ce 1 1 }  { K_2_0_4_q0 mem_dout 0 26 } } }
	K_2_0_5 { ap_memory {  { K_2_0_5_address0 mem_address 1 3 }  { K_2_0_5_ce0 mem_ce 1 1 }  { K_2_0_5_q0 mem_dout 0 26 } } }
	K_2_0_6 { ap_memory {  { K_2_0_6_address0 mem_address 1 3 }  { K_2_0_6_ce0 mem_ce 1 1 }  { K_2_0_6_q0 mem_dout 0 26 } } }
	K_2_0_7 { ap_memory {  { K_2_0_7_address0 mem_address 1 3 }  { K_2_0_7_ce0 mem_ce 1 1 }  { K_2_0_7_q0 mem_dout 0 26 } } }
	K_2_1_0 { ap_memory {  { K_2_1_0_address0 mem_address 1 3 }  { K_2_1_0_ce0 mem_ce 1 1 }  { K_2_1_0_q0 mem_dout 0 26 } } }
	K_2_1_1 { ap_memory {  { K_2_1_1_address0 mem_address 1 3 }  { K_2_1_1_ce0 mem_ce 1 1 }  { K_2_1_1_q0 mem_dout 0 26 } } }
	K_2_1_2 { ap_memory {  { K_2_1_2_address0 mem_address 1 3 }  { K_2_1_2_ce0 mem_ce 1 1 }  { K_2_1_2_q0 mem_dout 0 26 } } }
	K_2_1_3 { ap_memory {  { K_2_1_3_address0 mem_address 1 3 }  { K_2_1_3_ce0 mem_ce 1 1 }  { K_2_1_3_q0 mem_dout 0 26 } } }
	K_2_1_4 { ap_memory {  { K_2_1_4_address0 mem_address 1 3 }  { K_2_1_4_ce0 mem_ce 1 1 }  { K_2_1_4_q0 mem_dout 0 26 } } }
	K_2_1_5 { ap_memory {  { K_2_1_5_address0 mem_address 1 3 }  { K_2_1_5_ce0 mem_ce 1 1 }  { K_2_1_5_q0 mem_dout 0 26 } } }
	K_2_1_6 { ap_memory {  { K_2_1_6_address0 mem_address 1 3 }  { K_2_1_6_ce0 mem_ce 1 1 }  { K_2_1_6_q0 mem_dout 0 26 } } }
	K_2_1_7 { ap_memory {  { K_2_1_7_address0 mem_address 1 3 }  { K_2_1_7_ce0 mem_ce 1 1 }  { K_2_1_7_q0 mem_dout 0 26 } } }
	K_3_0_0 { ap_memory {  { K_3_0_0_address0 mem_address 1 3 }  { K_3_0_0_ce0 mem_ce 1 1 }  { K_3_0_0_q0 mem_dout 0 26 } } }
	K_3_0_1 { ap_memory {  { K_3_0_1_address0 mem_address 1 3 }  { K_3_0_1_ce0 mem_ce 1 1 }  { K_3_0_1_q0 mem_dout 0 26 } } }
	K_3_0_2 { ap_memory {  { K_3_0_2_address0 mem_address 1 3 }  { K_3_0_2_ce0 mem_ce 1 1 }  { K_3_0_2_q0 mem_dout 0 26 } } }
	K_3_0_3 { ap_memory {  { K_3_0_3_address0 mem_address 1 3 }  { K_3_0_3_ce0 mem_ce 1 1 }  { K_3_0_3_q0 mem_dout 0 26 } } }
	K_3_0_4 { ap_memory {  { K_3_0_4_address0 mem_address 1 3 }  { K_3_0_4_ce0 mem_ce 1 1 }  { K_3_0_4_q0 mem_dout 0 26 } } }
	K_3_0_5 { ap_memory {  { K_3_0_5_address0 mem_address 1 3 }  { K_3_0_5_ce0 mem_ce 1 1 }  { K_3_0_5_q0 mem_dout 0 26 } } }
	K_3_0_6 { ap_memory {  { K_3_0_6_address0 mem_address 1 3 }  { K_3_0_6_ce0 mem_ce 1 1 }  { K_3_0_6_q0 mem_dout 0 26 } } }
	K_3_0_7 { ap_memory {  { K_3_0_7_address0 mem_address 1 3 }  { K_3_0_7_ce0 mem_ce 1 1 }  { K_3_0_7_q0 mem_dout 0 26 } } }
	K_3_1_0 { ap_memory {  { K_3_1_0_address0 mem_address 1 3 }  { K_3_1_0_ce0 mem_ce 1 1 }  { K_3_1_0_q0 mem_dout 0 26 } } }
	K_3_1_1 { ap_memory {  { K_3_1_1_address0 mem_address 1 3 }  { K_3_1_1_ce0 mem_ce 1 1 }  { K_3_1_1_q0 mem_dout 0 26 } } }
	K_3_1_2 { ap_memory {  { K_3_1_2_address0 mem_address 1 3 }  { K_3_1_2_ce0 mem_ce 1 1 }  { K_3_1_2_q0 mem_dout 0 26 } } }
	K_3_1_3 { ap_memory {  { K_3_1_3_address0 mem_address 1 3 }  { K_3_1_3_ce0 mem_ce 1 1 }  { K_3_1_3_q0 mem_dout 0 26 } } }
	K_3_1_4 { ap_memory {  { K_3_1_4_address0 mem_address 1 3 }  { K_3_1_4_ce0 mem_ce 1 1 }  { K_3_1_4_q0 mem_dout 0 26 } } }
	K_3_1_5 { ap_memory {  { K_3_1_5_address0 mem_address 1 3 }  { K_3_1_5_ce0 mem_ce 1 1 }  { K_3_1_5_q0 mem_dout 0 26 } } }
	K_3_1_6 { ap_memory {  { K_3_1_6_address0 mem_address 1 3 }  { K_3_1_6_ce0 mem_ce 1 1 }  { K_3_1_6_q0 mem_dout 0 26 } } }
	K_3_1_7 { ap_memory {  { K_3_1_7_address0 mem_address 1 3 }  { K_3_1_7_ce0 mem_ce 1 1 }  { K_3_1_7_q0 mem_dout 0 26 } } }
	kk_0 { ap_memory {  { kk_0_address0 mem_address 1 5 }  { kk_0_ce0 mem_ce 1 1 }  { kk_0_q0 mem_dout 0 26 } } }
	kk_1 { ap_memory {  { kk_1_address0 mem_address 1 5 }  { kk_1_ce0 mem_ce 1 1 }  { kk_1_q0 mem_dout 0 26 } } }
	x_out_0 { ap_memory {  { x_out_0_address0 mem_address 1 5 }  { x_out_0_ce0 mem_ce 1 1 }  { x_out_0_q0 in_data 0 32 }  { x_out_0_address1 MemPortADDR2 1 5 }  { x_out_0_ce1 MemPortCE2 1 1 }  { x_out_0_we1 MemPortWE2 1 1 }  { x_out_0_d1 MemPortDIN2 1 32 } } }
	x_out_1 { ap_memory {  { x_out_1_address0 mem_address 1 5 }  { x_out_1_ce0 mem_ce 1 1 }  { x_out_1_q0 in_data 0 32 }  { x_out_1_address1 MemPortADDR2 1 5 }  { x_out_1_ce1 MemPortCE2 1 1 }  { x_out_1_we1 MemPortWE2 1 1 }  { x_out_1_d1 MemPortDIN2 1 32 } } }
	x_out_2 { ap_memory {  { x_out_2_address0 mem_address 1 5 }  { x_out_2_ce0 mem_ce 1 1 }  { x_out_2_q0 in_data 0 32 }  { x_out_2_address1 MemPortADDR2 1 5 }  { x_out_2_ce1 MemPortCE2 1 1 }  { x_out_2_we1 MemPortWE2 1 1 }  { x_out_2_d1 MemPortDIN2 1 32 } } }
	x_out_3 { ap_memory {  { x_out_3_address0 mem_address 1 5 }  { x_out_3_ce0 mem_ce 1 1 }  { x_out_3_q0 in_data 0 32 }  { x_out_3_address1 MemPortADDR2 1 5 }  { x_out_3_ce1 MemPortCE2 1 1 }  { x_out_3_we1 MemPortWE2 1 1 }  { x_out_3_d1 MemPortDIN2 1 32 } } }
	x_out_4 { ap_memory {  { x_out_4_address0 mem_address 1 5 }  { x_out_4_ce0 mem_ce 1 1 }  { x_out_4_q0 in_data 0 32 }  { x_out_4_address1 MemPortADDR2 1 5 }  { x_out_4_ce1 MemPortCE2 1 1 }  { x_out_4_we1 MemPortWE2 1 1 }  { x_out_4_d1 MemPortDIN2 1 32 } } }
	x_out_5 { ap_memory {  { x_out_5_address0 mem_address 1 5 }  { x_out_5_ce0 mem_ce 1 1 }  { x_out_5_q0 in_data 0 32 }  { x_out_5_address1 MemPortADDR2 1 5 }  { x_out_5_ce1 MemPortCE2 1 1 }  { x_out_5_we1 MemPortWE2 1 1 }  { x_out_5_d1 MemPortDIN2 1 32 } } }
	x_out_6 { ap_memory {  { x_out_6_address0 mem_address 1 5 }  { x_out_6_ce0 mem_ce 1 1 }  { x_out_6_q0 in_data 0 32 }  { x_out_6_address1 MemPortADDR2 1 5 }  { x_out_6_ce1 MemPortCE2 1 1 }  { x_out_6_we1 MemPortWE2 1 1 }  { x_out_6_d1 MemPortDIN2 1 32 } } }
	x_out_7 { ap_memory {  { x_out_7_address0 mem_address 1 5 }  { x_out_7_ce0 mem_ce 1 1 }  { x_out_7_q0 in_data 0 32 }  { x_out_7_address1 MemPortADDR2 1 5 }  { x_out_7_ce1 MemPortCE2 1 1 }  { x_out_7_we1 MemPortWE2 1 1 }  { x_out_7_d1 MemPortDIN2 1 32 } } }
	u_out_0 { ap_memory {  { u_out_0_address0 mem_address 1 5 }  { u_out_0_ce0 mem_ce 1 1 }  { u_out_0_we0 mem_we 1 1 }  { u_out_0_d0 mem_din 1 32 } } }
	u_out_1 { ap_memory {  { u_out_1_address0 mem_address 1 5 }  { u_out_1_ce0 mem_ce 1 1 }  { u_out_1_we0 mem_we 1 1 }  { u_out_1_d0 mem_din 1 32 } } }
}
