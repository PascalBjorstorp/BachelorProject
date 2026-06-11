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
set cdfgNum 74
set C_modelName {riccati_forward_pass}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict step_data_0_0 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_1 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_2 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_3 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_4 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_0_5 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_1 { MEM_WIDTH 26 MEM_SIZE 480 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_2 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict B_sparse_3 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_0_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_0_1_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_0_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_1_1_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_0_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_2_1_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_0_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_0 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_1 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_2 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_3 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_4 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_5 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_6 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict K_3_1_7 { MEM_WIDTH 17 MEM_SIZE 15 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict kk_0 { MEM_WIDTH 17 MEM_SIZE 60 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict kk_1 { MEM_WIDTH 17 MEM_SIZE 60 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_0 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict x_out_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict x_out_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict u_out_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict u_out_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ step_data_0_0 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_1 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_2 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_3 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_4 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_0_5 int 26 regular {array 120 { 1 3 } 1 1 bus  }  }
	{ step_data_1 int 26 regular {array 120 { 1 1 } 1 1 bus  }  }
	{ B_sparse_0 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ B_sparse_1 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ B_sparse_2 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ B_sparse_3 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ p_read int 26 regular  }
	{ p_read1 int 26 regular  }
	{ p_read2 int 26 regular  }
	{ p_read3 int 26 regular  }
	{ p_read4 int 26 regular  }
	{ p_read5 int 26 regular  }
	{ p_read6 int 26 regular  }
	{ p_read7 int 26 regular  }
	{ K_0_0_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_0_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_0_1_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_0_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_1_1_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_0_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_2_1_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_0_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_0 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_1 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_2 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_3 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_4 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_5 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_6 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ K_3_1_7 int 17 regular {array 5 { 1 3 } 1 1 bus  }  }
	{ kk_0 int 17 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ kk_1 int 17 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ x_out_0 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_1 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_2 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_3 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_4 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_5 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_6 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ x_out_7 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ u_out_0 int 26 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ u_out_1 int 26 regular {array 20 { 0 3 } 0 1 bus  }  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "step_data_0_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_0_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "B_sparse_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read1", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read2", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read3", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read4", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read5", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read6", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read7", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_0_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_0_1_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_0_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_1_1_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_0_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_2_1_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_0_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_2", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_3", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_4", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_5", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_6", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "K_3_1_7", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "kk_0", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "kk_1", "interface" : "memory", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "x_out_0", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "x_out_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "u_out_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "u_out_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 316
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ step_data_0_0_address0 sc_out sc_lv 7 signal 0 } 
	{ step_data_0_0_ce0 sc_out sc_logic 1 signal 0 } 
	{ step_data_0_0_q0 sc_in sc_lv 26 signal 0 } 
	{ step_data_0_1_address0 sc_out sc_lv 7 signal 1 } 
	{ step_data_0_1_ce0 sc_out sc_logic 1 signal 1 } 
	{ step_data_0_1_q0 sc_in sc_lv 26 signal 1 } 
	{ step_data_0_2_address0 sc_out sc_lv 7 signal 2 } 
	{ step_data_0_2_ce0 sc_out sc_logic 1 signal 2 } 
	{ step_data_0_2_q0 sc_in sc_lv 26 signal 2 } 
	{ step_data_0_3_address0 sc_out sc_lv 7 signal 3 } 
	{ step_data_0_3_ce0 sc_out sc_logic 1 signal 3 } 
	{ step_data_0_3_q0 sc_in sc_lv 26 signal 3 } 
	{ step_data_0_4_address0 sc_out sc_lv 7 signal 4 } 
	{ step_data_0_4_ce0 sc_out sc_logic 1 signal 4 } 
	{ step_data_0_4_q0 sc_in sc_lv 26 signal 4 } 
	{ step_data_0_5_address0 sc_out sc_lv 7 signal 5 } 
	{ step_data_0_5_ce0 sc_out sc_logic 1 signal 5 } 
	{ step_data_0_5_q0 sc_in sc_lv 26 signal 5 } 
	{ step_data_1_address0 sc_out sc_lv 7 signal 6 } 
	{ step_data_1_ce0 sc_out sc_logic 1 signal 6 } 
	{ step_data_1_q0 sc_in sc_lv 26 signal 6 } 
	{ step_data_1_address1 sc_out sc_lv 7 signal 6 } 
	{ step_data_1_ce1 sc_out sc_logic 1 signal 6 } 
	{ step_data_1_q1 sc_in sc_lv 26 signal 6 } 
	{ B_sparse_0_address0 sc_out sc_lv 5 signal 7 } 
	{ B_sparse_0_ce0 sc_out sc_logic 1 signal 7 } 
	{ B_sparse_0_q0 sc_in sc_lv 26 signal 7 } 
	{ B_sparse_1_address0 sc_out sc_lv 5 signal 8 } 
	{ B_sparse_1_ce0 sc_out sc_logic 1 signal 8 } 
	{ B_sparse_1_q0 sc_in sc_lv 26 signal 8 } 
	{ B_sparse_2_address0 sc_out sc_lv 5 signal 9 } 
	{ B_sparse_2_ce0 sc_out sc_logic 1 signal 9 } 
	{ B_sparse_2_q0 sc_in sc_lv 26 signal 9 } 
	{ B_sparse_3_address0 sc_out sc_lv 5 signal 10 } 
	{ B_sparse_3_ce0 sc_out sc_logic 1 signal 10 } 
	{ B_sparse_3_q0 sc_in sc_lv 26 signal 10 } 
	{ p_read sc_in sc_lv 26 signal 11 } 
	{ p_read1 sc_in sc_lv 26 signal 12 } 
	{ p_read2 sc_in sc_lv 26 signal 13 } 
	{ p_read3 sc_in sc_lv 26 signal 14 } 
	{ p_read4 sc_in sc_lv 26 signal 15 } 
	{ p_read5 sc_in sc_lv 26 signal 16 } 
	{ p_read6 sc_in sc_lv 26 signal 17 } 
	{ p_read7 sc_in sc_lv 26 signal 18 } 
	{ K_0_0_0_address0 sc_out sc_lv 3 signal 19 } 
	{ K_0_0_0_ce0 sc_out sc_logic 1 signal 19 } 
	{ K_0_0_0_q0 sc_in sc_lv 17 signal 19 } 
	{ K_0_0_1_address0 sc_out sc_lv 3 signal 20 } 
	{ K_0_0_1_ce0 sc_out sc_logic 1 signal 20 } 
	{ K_0_0_1_q0 sc_in sc_lv 17 signal 20 } 
	{ K_0_0_2_address0 sc_out sc_lv 3 signal 21 } 
	{ K_0_0_2_ce0 sc_out sc_logic 1 signal 21 } 
	{ K_0_0_2_q0 sc_in sc_lv 17 signal 21 } 
	{ K_0_0_3_address0 sc_out sc_lv 3 signal 22 } 
	{ K_0_0_3_ce0 sc_out sc_logic 1 signal 22 } 
	{ K_0_0_3_q0 sc_in sc_lv 17 signal 22 } 
	{ K_0_0_4_address0 sc_out sc_lv 3 signal 23 } 
	{ K_0_0_4_ce0 sc_out sc_logic 1 signal 23 } 
	{ K_0_0_4_q0 sc_in sc_lv 17 signal 23 } 
	{ K_0_0_5_address0 sc_out sc_lv 3 signal 24 } 
	{ K_0_0_5_ce0 sc_out sc_logic 1 signal 24 } 
	{ K_0_0_5_q0 sc_in sc_lv 17 signal 24 } 
	{ K_0_0_6_address0 sc_out sc_lv 3 signal 25 } 
	{ K_0_0_6_ce0 sc_out sc_logic 1 signal 25 } 
	{ K_0_0_6_q0 sc_in sc_lv 17 signal 25 } 
	{ K_0_0_7_address0 sc_out sc_lv 3 signal 26 } 
	{ K_0_0_7_ce0 sc_out sc_logic 1 signal 26 } 
	{ K_0_0_7_q0 sc_in sc_lv 17 signal 26 } 
	{ K_0_1_0_address0 sc_out sc_lv 3 signal 27 } 
	{ K_0_1_0_ce0 sc_out sc_logic 1 signal 27 } 
	{ K_0_1_0_q0 sc_in sc_lv 17 signal 27 } 
	{ K_0_1_1_address0 sc_out sc_lv 3 signal 28 } 
	{ K_0_1_1_ce0 sc_out sc_logic 1 signal 28 } 
	{ K_0_1_1_q0 sc_in sc_lv 17 signal 28 } 
	{ K_0_1_2_address0 sc_out sc_lv 3 signal 29 } 
	{ K_0_1_2_ce0 sc_out sc_logic 1 signal 29 } 
	{ K_0_1_2_q0 sc_in sc_lv 17 signal 29 } 
	{ K_0_1_3_address0 sc_out sc_lv 3 signal 30 } 
	{ K_0_1_3_ce0 sc_out sc_logic 1 signal 30 } 
	{ K_0_1_3_q0 sc_in sc_lv 17 signal 30 } 
	{ K_0_1_4_address0 sc_out sc_lv 3 signal 31 } 
	{ K_0_1_4_ce0 sc_out sc_logic 1 signal 31 } 
	{ K_0_1_4_q0 sc_in sc_lv 17 signal 31 } 
	{ K_0_1_5_address0 sc_out sc_lv 3 signal 32 } 
	{ K_0_1_5_ce0 sc_out sc_logic 1 signal 32 } 
	{ K_0_1_5_q0 sc_in sc_lv 17 signal 32 } 
	{ K_0_1_6_address0 sc_out sc_lv 3 signal 33 } 
	{ K_0_1_6_ce0 sc_out sc_logic 1 signal 33 } 
	{ K_0_1_6_q0 sc_in sc_lv 17 signal 33 } 
	{ K_0_1_7_address0 sc_out sc_lv 3 signal 34 } 
	{ K_0_1_7_ce0 sc_out sc_logic 1 signal 34 } 
	{ K_0_1_7_q0 sc_in sc_lv 17 signal 34 } 
	{ K_1_0_0_address0 sc_out sc_lv 3 signal 35 } 
	{ K_1_0_0_ce0 sc_out sc_logic 1 signal 35 } 
	{ K_1_0_0_q0 sc_in sc_lv 17 signal 35 } 
	{ K_1_0_1_address0 sc_out sc_lv 3 signal 36 } 
	{ K_1_0_1_ce0 sc_out sc_logic 1 signal 36 } 
	{ K_1_0_1_q0 sc_in sc_lv 17 signal 36 } 
	{ K_1_0_2_address0 sc_out sc_lv 3 signal 37 } 
	{ K_1_0_2_ce0 sc_out sc_logic 1 signal 37 } 
	{ K_1_0_2_q0 sc_in sc_lv 17 signal 37 } 
	{ K_1_0_3_address0 sc_out sc_lv 3 signal 38 } 
	{ K_1_0_3_ce0 sc_out sc_logic 1 signal 38 } 
	{ K_1_0_3_q0 sc_in sc_lv 17 signal 38 } 
	{ K_1_0_4_address0 sc_out sc_lv 3 signal 39 } 
	{ K_1_0_4_ce0 sc_out sc_logic 1 signal 39 } 
	{ K_1_0_4_q0 sc_in sc_lv 17 signal 39 } 
	{ K_1_0_5_address0 sc_out sc_lv 3 signal 40 } 
	{ K_1_0_5_ce0 sc_out sc_logic 1 signal 40 } 
	{ K_1_0_5_q0 sc_in sc_lv 17 signal 40 } 
	{ K_1_0_6_address0 sc_out sc_lv 3 signal 41 } 
	{ K_1_0_6_ce0 sc_out sc_logic 1 signal 41 } 
	{ K_1_0_6_q0 sc_in sc_lv 17 signal 41 } 
	{ K_1_0_7_address0 sc_out sc_lv 3 signal 42 } 
	{ K_1_0_7_ce0 sc_out sc_logic 1 signal 42 } 
	{ K_1_0_7_q0 sc_in sc_lv 17 signal 42 } 
	{ K_1_1_0_address0 sc_out sc_lv 3 signal 43 } 
	{ K_1_1_0_ce0 sc_out sc_logic 1 signal 43 } 
	{ K_1_1_0_q0 sc_in sc_lv 17 signal 43 } 
	{ K_1_1_1_address0 sc_out sc_lv 3 signal 44 } 
	{ K_1_1_1_ce0 sc_out sc_logic 1 signal 44 } 
	{ K_1_1_1_q0 sc_in sc_lv 17 signal 44 } 
	{ K_1_1_2_address0 sc_out sc_lv 3 signal 45 } 
	{ K_1_1_2_ce0 sc_out sc_logic 1 signal 45 } 
	{ K_1_1_2_q0 sc_in sc_lv 17 signal 45 } 
	{ K_1_1_3_address0 sc_out sc_lv 3 signal 46 } 
	{ K_1_1_3_ce0 sc_out sc_logic 1 signal 46 } 
	{ K_1_1_3_q0 sc_in sc_lv 17 signal 46 } 
	{ K_1_1_4_address0 sc_out sc_lv 3 signal 47 } 
	{ K_1_1_4_ce0 sc_out sc_logic 1 signal 47 } 
	{ K_1_1_4_q0 sc_in sc_lv 17 signal 47 } 
	{ K_1_1_5_address0 sc_out sc_lv 3 signal 48 } 
	{ K_1_1_5_ce0 sc_out sc_logic 1 signal 48 } 
	{ K_1_1_5_q0 sc_in sc_lv 17 signal 48 } 
	{ K_1_1_6_address0 sc_out sc_lv 3 signal 49 } 
	{ K_1_1_6_ce0 sc_out sc_logic 1 signal 49 } 
	{ K_1_1_6_q0 sc_in sc_lv 17 signal 49 } 
	{ K_1_1_7_address0 sc_out sc_lv 3 signal 50 } 
	{ K_1_1_7_ce0 sc_out sc_logic 1 signal 50 } 
	{ K_1_1_7_q0 sc_in sc_lv 17 signal 50 } 
	{ K_2_0_0_address0 sc_out sc_lv 3 signal 51 } 
	{ K_2_0_0_ce0 sc_out sc_logic 1 signal 51 } 
	{ K_2_0_0_q0 sc_in sc_lv 17 signal 51 } 
	{ K_2_0_1_address0 sc_out sc_lv 3 signal 52 } 
	{ K_2_0_1_ce0 sc_out sc_logic 1 signal 52 } 
	{ K_2_0_1_q0 sc_in sc_lv 17 signal 52 } 
	{ K_2_0_2_address0 sc_out sc_lv 3 signal 53 } 
	{ K_2_0_2_ce0 sc_out sc_logic 1 signal 53 } 
	{ K_2_0_2_q0 sc_in sc_lv 17 signal 53 } 
	{ K_2_0_3_address0 sc_out sc_lv 3 signal 54 } 
	{ K_2_0_3_ce0 sc_out sc_logic 1 signal 54 } 
	{ K_2_0_3_q0 sc_in sc_lv 17 signal 54 } 
	{ K_2_0_4_address0 sc_out sc_lv 3 signal 55 } 
	{ K_2_0_4_ce0 sc_out sc_logic 1 signal 55 } 
	{ K_2_0_4_q0 sc_in sc_lv 17 signal 55 } 
	{ K_2_0_5_address0 sc_out sc_lv 3 signal 56 } 
	{ K_2_0_5_ce0 sc_out sc_logic 1 signal 56 } 
	{ K_2_0_5_q0 sc_in sc_lv 17 signal 56 } 
	{ K_2_0_6_address0 sc_out sc_lv 3 signal 57 } 
	{ K_2_0_6_ce0 sc_out sc_logic 1 signal 57 } 
	{ K_2_0_6_q0 sc_in sc_lv 17 signal 57 } 
	{ K_2_0_7_address0 sc_out sc_lv 3 signal 58 } 
	{ K_2_0_7_ce0 sc_out sc_logic 1 signal 58 } 
	{ K_2_0_7_q0 sc_in sc_lv 17 signal 58 } 
	{ K_2_1_0_address0 sc_out sc_lv 3 signal 59 } 
	{ K_2_1_0_ce0 sc_out sc_logic 1 signal 59 } 
	{ K_2_1_0_q0 sc_in sc_lv 17 signal 59 } 
	{ K_2_1_1_address0 sc_out sc_lv 3 signal 60 } 
	{ K_2_1_1_ce0 sc_out sc_logic 1 signal 60 } 
	{ K_2_1_1_q0 sc_in sc_lv 17 signal 60 } 
	{ K_2_1_2_address0 sc_out sc_lv 3 signal 61 } 
	{ K_2_1_2_ce0 sc_out sc_logic 1 signal 61 } 
	{ K_2_1_2_q0 sc_in sc_lv 17 signal 61 } 
	{ K_2_1_3_address0 sc_out sc_lv 3 signal 62 } 
	{ K_2_1_3_ce0 sc_out sc_logic 1 signal 62 } 
	{ K_2_1_3_q0 sc_in sc_lv 17 signal 62 } 
	{ K_2_1_4_address0 sc_out sc_lv 3 signal 63 } 
	{ K_2_1_4_ce0 sc_out sc_logic 1 signal 63 } 
	{ K_2_1_4_q0 sc_in sc_lv 17 signal 63 } 
	{ K_2_1_5_address0 sc_out sc_lv 3 signal 64 } 
	{ K_2_1_5_ce0 sc_out sc_logic 1 signal 64 } 
	{ K_2_1_5_q0 sc_in sc_lv 17 signal 64 } 
	{ K_2_1_6_address0 sc_out sc_lv 3 signal 65 } 
	{ K_2_1_6_ce0 sc_out sc_logic 1 signal 65 } 
	{ K_2_1_6_q0 sc_in sc_lv 17 signal 65 } 
	{ K_2_1_7_address0 sc_out sc_lv 3 signal 66 } 
	{ K_2_1_7_ce0 sc_out sc_logic 1 signal 66 } 
	{ K_2_1_7_q0 sc_in sc_lv 17 signal 66 } 
	{ K_3_0_0_address0 sc_out sc_lv 3 signal 67 } 
	{ K_3_0_0_ce0 sc_out sc_logic 1 signal 67 } 
	{ K_3_0_0_q0 sc_in sc_lv 17 signal 67 } 
	{ K_3_0_1_address0 sc_out sc_lv 3 signal 68 } 
	{ K_3_0_1_ce0 sc_out sc_logic 1 signal 68 } 
	{ K_3_0_1_q0 sc_in sc_lv 17 signal 68 } 
	{ K_3_0_2_address0 sc_out sc_lv 3 signal 69 } 
	{ K_3_0_2_ce0 sc_out sc_logic 1 signal 69 } 
	{ K_3_0_2_q0 sc_in sc_lv 17 signal 69 } 
	{ K_3_0_3_address0 sc_out sc_lv 3 signal 70 } 
	{ K_3_0_3_ce0 sc_out sc_logic 1 signal 70 } 
	{ K_3_0_3_q0 sc_in sc_lv 17 signal 70 } 
	{ K_3_0_4_address0 sc_out sc_lv 3 signal 71 } 
	{ K_3_0_4_ce0 sc_out sc_logic 1 signal 71 } 
	{ K_3_0_4_q0 sc_in sc_lv 17 signal 71 } 
	{ K_3_0_5_address0 sc_out sc_lv 3 signal 72 } 
	{ K_3_0_5_ce0 sc_out sc_logic 1 signal 72 } 
	{ K_3_0_5_q0 sc_in sc_lv 17 signal 72 } 
	{ K_3_0_6_address0 sc_out sc_lv 3 signal 73 } 
	{ K_3_0_6_ce0 sc_out sc_logic 1 signal 73 } 
	{ K_3_0_6_q0 sc_in sc_lv 17 signal 73 } 
	{ K_3_0_7_address0 sc_out sc_lv 3 signal 74 } 
	{ K_3_0_7_ce0 sc_out sc_logic 1 signal 74 } 
	{ K_3_0_7_q0 sc_in sc_lv 17 signal 74 } 
	{ K_3_1_0_address0 sc_out sc_lv 3 signal 75 } 
	{ K_3_1_0_ce0 sc_out sc_logic 1 signal 75 } 
	{ K_3_1_0_q0 sc_in sc_lv 17 signal 75 } 
	{ K_3_1_1_address0 sc_out sc_lv 3 signal 76 } 
	{ K_3_1_1_ce0 sc_out sc_logic 1 signal 76 } 
	{ K_3_1_1_q0 sc_in sc_lv 17 signal 76 } 
	{ K_3_1_2_address0 sc_out sc_lv 3 signal 77 } 
	{ K_3_1_2_ce0 sc_out sc_logic 1 signal 77 } 
	{ K_3_1_2_q0 sc_in sc_lv 17 signal 77 } 
	{ K_3_1_3_address0 sc_out sc_lv 3 signal 78 } 
	{ K_3_1_3_ce0 sc_out sc_logic 1 signal 78 } 
	{ K_3_1_3_q0 sc_in sc_lv 17 signal 78 } 
	{ K_3_1_4_address0 sc_out sc_lv 3 signal 79 } 
	{ K_3_1_4_ce0 sc_out sc_logic 1 signal 79 } 
	{ K_3_1_4_q0 sc_in sc_lv 17 signal 79 } 
	{ K_3_1_5_address0 sc_out sc_lv 3 signal 80 } 
	{ K_3_1_5_ce0 sc_out sc_logic 1 signal 80 } 
	{ K_3_1_5_q0 sc_in sc_lv 17 signal 80 } 
	{ K_3_1_6_address0 sc_out sc_lv 3 signal 81 } 
	{ K_3_1_6_ce0 sc_out sc_logic 1 signal 81 } 
	{ K_3_1_6_q0 sc_in sc_lv 17 signal 81 } 
	{ K_3_1_7_address0 sc_out sc_lv 3 signal 82 } 
	{ K_3_1_7_ce0 sc_out sc_logic 1 signal 82 } 
	{ K_3_1_7_q0 sc_in sc_lv 17 signal 82 } 
	{ kk_0_address0 sc_out sc_lv 5 signal 83 } 
	{ kk_0_ce0 sc_out sc_logic 1 signal 83 } 
	{ kk_0_q0 sc_in sc_lv 17 signal 83 } 
	{ kk_1_address0 sc_out sc_lv 5 signal 84 } 
	{ kk_1_ce0 sc_out sc_logic 1 signal 84 } 
	{ kk_1_q0 sc_in sc_lv 17 signal 84 } 
	{ x_out_0_address0 sc_out sc_lv 5 signal 85 } 
	{ x_out_0_ce0 sc_out sc_logic 1 signal 85 } 
	{ x_out_0_q0 sc_in sc_lv 26 signal 85 } 
	{ x_out_0_address1 sc_out sc_lv 5 signal 85 } 
	{ x_out_0_ce1 sc_out sc_logic 1 signal 85 } 
	{ x_out_0_we1 sc_out sc_logic 1 signal 85 } 
	{ x_out_0_d1 sc_out sc_lv 26 signal 85 } 
	{ x_out_1_address0 sc_out sc_lv 5 signal 86 } 
	{ x_out_1_ce0 sc_out sc_logic 1 signal 86 } 
	{ x_out_1_q0 sc_in sc_lv 26 signal 86 } 
	{ x_out_1_address1 sc_out sc_lv 5 signal 86 } 
	{ x_out_1_ce1 sc_out sc_logic 1 signal 86 } 
	{ x_out_1_we1 sc_out sc_logic 1 signal 86 } 
	{ x_out_1_d1 sc_out sc_lv 26 signal 86 } 
	{ x_out_2_address0 sc_out sc_lv 5 signal 87 } 
	{ x_out_2_ce0 sc_out sc_logic 1 signal 87 } 
	{ x_out_2_q0 sc_in sc_lv 26 signal 87 } 
	{ x_out_2_address1 sc_out sc_lv 5 signal 87 } 
	{ x_out_2_ce1 sc_out sc_logic 1 signal 87 } 
	{ x_out_2_we1 sc_out sc_logic 1 signal 87 } 
	{ x_out_2_d1 sc_out sc_lv 26 signal 87 } 
	{ x_out_3_address0 sc_out sc_lv 5 signal 88 } 
	{ x_out_3_ce0 sc_out sc_logic 1 signal 88 } 
	{ x_out_3_q0 sc_in sc_lv 26 signal 88 } 
	{ x_out_3_address1 sc_out sc_lv 5 signal 88 } 
	{ x_out_3_ce1 sc_out sc_logic 1 signal 88 } 
	{ x_out_3_we1 sc_out sc_logic 1 signal 88 } 
	{ x_out_3_d1 sc_out sc_lv 26 signal 88 } 
	{ x_out_4_address0 sc_out sc_lv 5 signal 89 } 
	{ x_out_4_ce0 sc_out sc_logic 1 signal 89 } 
	{ x_out_4_q0 sc_in sc_lv 26 signal 89 } 
	{ x_out_4_address1 sc_out sc_lv 5 signal 89 } 
	{ x_out_4_ce1 sc_out sc_logic 1 signal 89 } 
	{ x_out_4_we1 sc_out sc_logic 1 signal 89 } 
	{ x_out_4_d1 sc_out sc_lv 26 signal 89 } 
	{ x_out_5_address0 sc_out sc_lv 5 signal 90 } 
	{ x_out_5_ce0 sc_out sc_logic 1 signal 90 } 
	{ x_out_5_q0 sc_in sc_lv 26 signal 90 } 
	{ x_out_5_address1 sc_out sc_lv 5 signal 90 } 
	{ x_out_5_ce1 sc_out sc_logic 1 signal 90 } 
	{ x_out_5_we1 sc_out sc_logic 1 signal 90 } 
	{ x_out_5_d1 sc_out sc_lv 26 signal 90 } 
	{ x_out_6_address0 sc_out sc_lv 5 signal 91 } 
	{ x_out_6_ce0 sc_out sc_logic 1 signal 91 } 
	{ x_out_6_q0 sc_in sc_lv 26 signal 91 } 
	{ x_out_6_address1 sc_out sc_lv 5 signal 91 } 
	{ x_out_6_ce1 sc_out sc_logic 1 signal 91 } 
	{ x_out_6_we1 sc_out sc_logic 1 signal 91 } 
	{ x_out_6_d1 sc_out sc_lv 26 signal 91 } 
	{ x_out_7_address0 sc_out sc_lv 5 signal 92 } 
	{ x_out_7_ce0 sc_out sc_logic 1 signal 92 } 
	{ x_out_7_q0 sc_in sc_lv 26 signal 92 } 
	{ x_out_7_address1 sc_out sc_lv 5 signal 92 } 
	{ x_out_7_ce1 sc_out sc_logic 1 signal 92 } 
	{ x_out_7_we1 sc_out sc_logic 1 signal 92 } 
	{ x_out_7_d1 sc_out sc_lv 26 signal 92 } 
	{ u_out_0_address0 sc_out sc_lv 5 signal 93 } 
	{ u_out_0_ce0 sc_out sc_logic 1 signal 93 } 
	{ u_out_0_we0 sc_out sc_logic 1 signal 93 } 
	{ u_out_0_d0 sc_out sc_lv 26 signal 93 } 
	{ u_out_1_address0 sc_out sc_lv 5 signal 94 } 
	{ u_out_1_ce0 sc_out sc_logic 1 signal 94 } 
	{ u_out_1_we0 sc_out sc_logic 1 signal 94 } 
	{ u_out_1_d0 sc_out sc_lv 26 signal 94 } 
	{ grp_fu_830_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_830_p_din1 sc_out sc_lv 25 signal -1 } 
	{ grp_fu_830_p_dout0 sc_in sc_lv 51 signal -1 } 
	{ grp_fu_830_p_ce sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
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
 	{ "name": "step_data_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_1", "role": "address0" }} , 
 	{ "name": "step_data_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_1", "role": "ce0" }} , 
 	{ "name": "step_data_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_1", "role": "q0" }} , 
 	{ "name": "step_data_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":7, "type": "signal", "bundle":{"name": "step_data_1", "role": "address1" }} , 
 	{ "name": "step_data_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_1", "role": "ce1" }} , 
 	{ "name": "step_data_1_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_1", "role": "q1" }} , 
 	{ "name": "B_sparse_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_0", "role": "address0" }} , 
 	{ "name": "B_sparse_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_0", "role": "ce0" }} , 
 	{ "name": "B_sparse_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse_0", "role": "q0" }} , 
 	{ "name": "B_sparse_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "address0" }} , 
 	{ "name": "B_sparse_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "ce0" }} , 
 	{ "name": "B_sparse_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse_1", "role": "q0" }} , 
 	{ "name": "B_sparse_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "address0" }} , 
 	{ "name": "B_sparse_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "ce0" }} , 
 	{ "name": "B_sparse_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse_2", "role": "q0" }} , 
 	{ "name": "B_sparse_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "address0" }} , 
 	{ "name": "B_sparse_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "ce0" }} , 
 	{ "name": "B_sparse_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "B_sparse_3", "role": "q0" }} , 
 	{ "name": "p_read", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read", "role": "default" }} , 
 	{ "name": "p_read1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read1", "role": "default" }} , 
 	{ "name": "p_read2", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read2", "role": "default" }} , 
 	{ "name": "p_read3", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read3", "role": "default" }} , 
 	{ "name": "p_read4", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read4", "role": "default" }} , 
 	{ "name": "p_read5", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read5", "role": "default" }} , 
 	{ "name": "p_read6", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read6", "role": "default" }} , 
 	{ "name": "p_read7", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read7", "role": "default" }} , 
 	{ "name": "K_0_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_0", "role": "address0" }} , 
 	{ "name": "K_0_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_0", "role": "ce0" }} , 
 	{ "name": "K_0_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_0", "role": "q0" }} , 
 	{ "name": "K_0_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_1", "role": "address0" }} , 
 	{ "name": "K_0_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_1", "role": "ce0" }} , 
 	{ "name": "K_0_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_1", "role": "q0" }} , 
 	{ "name": "K_0_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_2", "role": "address0" }} , 
 	{ "name": "K_0_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_2", "role": "ce0" }} , 
 	{ "name": "K_0_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_2", "role": "q0" }} , 
 	{ "name": "K_0_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_3", "role": "address0" }} , 
 	{ "name": "K_0_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_3", "role": "ce0" }} , 
 	{ "name": "K_0_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_3", "role": "q0" }} , 
 	{ "name": "K_0_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_4", "role": "address0" }} , 
 	{ "name": "K_0_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_4", "role": "ce0" }} , 
 	{ "name": "K_0_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_4", "role": "q0" }} , 
 	{ "name": "K_0_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_5", "role": "address0" }} , 
 	{ "name": "K_0_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_5", "role": "ce0" }} , 
 	{ "name": "K_0_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_5", "role": "q0" }} , 
 	{ "name": "K_0_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_6", "role": "address0" }} , 
 	{ "name": "K_0_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_6", "role": "ce0" }} , 
 	{ "name": "K_0_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_6", "role": "q0" }} , 
 	{ "name": "K_0_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_0_7", "role": "address0" }} , 
 	{ "name": "K_0_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_0_7", "role": "ce0" }} , 
 	{ "name": "K_0_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_0_7", "role": "q0" }} , 
 	{ "name": "K_0_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_0", "role": "address0" }} , 
 	{ "name": "K_0_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_0", "role": "ce0" }} , 
 	{ "name": "K_0_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_0", "role": "q0" }} , 
 	{ "name": "K_0_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_1", "role": "address0" }} , 
 	{ "name": "K_0_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_1", "role": "ce0" }} , 
 	{ "name": "K_0_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_1", "role": "q0" }} , 
 	{ "name": "K_0_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_2", "role": "address0" }} , 
 	{ "name": "K_0_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_2", "role": "ce0" }} , 
 	{ "name": "K_0_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_2", "role": "q0" }} , 
 	{ "name": "K_0_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_3", "role": "address0" }} , 
 	{ "name": "K_0_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_3", "role": "ce0" }} , 
 	{ "name": "K_0_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_3", "role": "q0" }} , 
 	{ "name": "K_0_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_4", "role": "address0" }} , 
 	{ "name": "K_0_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_4", "role": "ce0" }} , 
 	{ "name": "K_0_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_4", "role": "q0" }} , 
 	{ "name": "K_0_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_5", "role": "address0" }} , 
 	{ "name": "K_0_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_5", "role": "ce0" }} , 
 	{ "name": "K_0_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_5", "role": "q0" }} , 
 	{ "name": "K_0_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_6", "role": "address0" }} , 
 	{ "name": "K_0_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_6", "role": "ce0" }} , 
 	{ "name": "K_0_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_6", "role": "q0" }} , 
 	{ "name": "K_0_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_0_1_7", "role": "address0" }} , 
 	{ "name": "K_0_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_0_1_7", "role": "ce0" }} , 
 	{ "name": "K_0_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_0_1_7", "role": "q0" }} , 
 	{ "name": "K_1_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_0", "role": "address0" }} , 
 	{ "name": "K_1_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_0", "role": "ce0" }} , 
 	{ "name": "K_1_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_0", "role": "q0" }} , 
 	{ "name": "K_1_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_1", "role": "address0" }} , 
 	{ "name": "K_1_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_1", "role": "ce0" }} , 
 	{ "name": "K_1_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_1", "role": "q0" }} , 
 	{ "name": "K_1_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_2", "role": "address0" }} , 
 	{ "name": "K_1_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_2", "role": "ce0" }} , 
 	{ "name": "K_1_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_2", "role": "q0" }} , 
 	{ "name": "K_1_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_3", "role": "address0" }} , 
 	{ "name": "K_1_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_3", "role": "ce0" }} , 
 	{ "name": "K_1_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_3", "role": "q0" }} , 
 	{ "name": "K_1_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_4", "role": "address0" }} , 
 	{ "name": "K_1_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_4", "role": "ce0" }} , 
 	{ "name": "K_1_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_4", "role": "q0" }} , 
 	{ "name": "K_1_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_5", "role": "address0" }} , 
 	{ "name": "K_1_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_5", "role": "ce0" }} , 
 	{ "name": "K_1_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_5", "role": "q0" }} , 
 	{ "name": "K_1_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_6", "role": "address0" }} , 
 	{ "name": "K_1_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_6", "role": "ce0" }} , 
 	{ "name": "K_1_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_6", "role": "q0" }} , 
 	{ "name": "K_1_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_0_7", "role": "address0" }} , 
 	{ "name": "K_1_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_0_7", "role": "ce0" }} , 
 	{ "name": "K_1_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_0_7", "role": "q0" }} , 
 	{ "name": "K_1_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_0", "role": "address0" }} , 
 	{ "name": "K_1_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_0", "role": "ce0" }} , 
 	{ "name": "K_1_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_0", "role": "q0" }} , 
 	{ "name": "K_1_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_1", "role": "address0" }} , 
 	{ "name": "K_1_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_1", "role": "ce0" }} , 
 	{ "name": "K_1_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_1", "role": "q0" }} , 
 	{ "name": "K_1_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_2", "role": "address0" }} , 
 	{ "name": "K_1_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_2", "role": "ce0" }} , 
 	{ "name": "K_1_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_2", "role": "q0" }} , 
 	{ "name": "K_1_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_3", "role": "address0" }} , 
 	{ "name": "K_1_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_3", "role": "ce0" }} , 
 	{ "name": "K_1_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_3", "role": "q0" }} , 
 	{ "name": "K_1_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_4", "role": "address0" }} , 
 	{ "name": "K_1_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_4", "role": "ce0" }} , 
 	{ "name": "K_1_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_4", "role": "q0" }} , 
 	{ "name": "K_1_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_5", "role": "address0" }} , 
 	{ "name": "K_1_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_5", "role": "ce0" }} , 
 	{ "name": "K_1_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_5", "role": "q0" }} , 
 	{ "name": "K_1_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_6", "role": "address0" }} , 
 	{ "name": "K_1_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_6", "role": "ce0" }} , 
 	{ "name": "K_1_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_6", "role": "q0" }} , 
 	{ "name": "K_1_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_1_1_7", "role": "address0" }} , 
 	{ "name": "K_1_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_1_1_7", "role": "ce0" }} , 
 	{ "name": "K_1_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_1_1_7", "role": "q0" }} , 
 	{ "name": "K_2_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_0", "role": "address0" }} , 
 	{ "name": "K_2_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_0", "role": "ce0" }} , 
 	{ "name": "K_2_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_0", "role": "q0" }} , 
 	{ "name": "K_2_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_1", "role": "address0" }} , 
 	{ "name": "K_2_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_1", "role": "ce0" }} , 
 	{ "name": "K_2_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_1", "role": "q0" }} , 
 	{ "name": "K_2_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_2", "role": "address0" }} , 
 	{ "name": "K_2_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_2", "role": "ce0" }} , 
 	{ "name": "K_2_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_2", "role": "q0" }} , 
 	{ "name": "K_2_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_3", "role": "address0" }} , 
 	{ "name": "K_2_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_3", "role": "ce0" }} , 
 	{ "name": "K_2_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_3", "role": "q0" }} , 
 	{ "name": "K_2_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_4", "role": "address0" }} , 
 	{ "name": "K_2_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_4", "role": "ce0" }} , 
 	{ "name": "K_2_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_4", "role": "q0" }} , 
 	{ "name": "K_2_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_5", "role": "address0" }} , 
 	{ "name": "K_2_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_5", "role": "ce0" }} , 
 	{ "name": "K_2_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_5", "role": "q0" }} , 
 	{ "name": "K_2_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_6", "role": "address0" }} , 
 	{ "name": "K_2_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_6", "role": "ce0" }} , 
 	{ "name": "K_2_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_6", "role": "q0" }} , 
 	{ "name": "K_2_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_0_7", "role": "address0" }} , 
 	{ "name": "K_2_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_0_7", "role": "ce0" }} , 
 	{ "name": "K_2_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_0_7", "role": "q0" }} , 
 	{ "name": "K_2_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_0", "role": "address0" }} , 
 	{ "name": "K_2_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_0", "role": "ce0" }} , 
 	{ "name": "K_2_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_0", "role": "q0" }} , 
 	{ "name": "K_2_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_1", "role": "address0" }} , 
 	{ "name": "K_2_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_1", "role": "ce0" }} , 
 	{ "name": "K_2_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_1", "role": "q0" }} , 
 	{ "name": "K_2_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_2", "role": "address0" }} , 
 	{ "name": "K_2_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_2", "role": "ce0" }} , 
 	{ "name": "K_2_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_2", "role": "q0" }} , 
 	{ "name": "K_2_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_3", "role": "address0" }} , 
 	{ "name": "K_2_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_3", "role": "ce0" }} , 
 	{ "name": "K_2_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_3", "role": "q0" }} , 
 	{ "name": "K_2_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_4", "role": "address0" }} , 
 	{ "name": "K_2_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_4", "role": "ce0" }} , 
 	{ "name": "K_2_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_4", "role": "q0" }} , 
 	{ "name": "K_2_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_5", "role": "address0" }} , 
 	{ "name": "K_2_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_5", "role": "ce0" }} , 
 	{ "name": "K_2_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_5", "role": "q0" }} , 
 	{ "name": "K_2_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_6", "role": "address0" }} , 
 	{ "name": "K_2_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_6", "role": "ce0" }} , 
 	{ "name": "K_2_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_6", "role": "q0" }} , 
 	{ "name": "K_2_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_2_1_7", "role": "address0" }} , 
 	{ "name": "K_2_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_2_1_7", "role": "ce0" }} , 
 	{ "name": "K_2_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_2_1_7", "role": "q0" }} , 
 	{ "name": "K_3_0_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_0", "role": "address0" }} , 
 	{ "name": "K_3_0_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_0", "role": "ce0" }} , 
 	{ "name": "K_3_0_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_0", "role": "q0" }} , 
 	{ "name": "K_3_0_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_1", "role": "address0" }} , 
 	{ "name": "K_3_0_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_1", "role": "ce0" }} , 
 	{ "name": "K_3_0_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_1", "role": "q0" }} , 
 	{ "name": "K_3_0_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_2", "role": "address0" }} , 
 	{ "name": "K_3_0_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_2", "role": "ce0" }} , 
 	{ "name": "K_3_0_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_2", "role": "q0" }} , 
 	{ "name": "K_3_0_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_3", "role": "address0" }} , 
 	{ "name": "K_3_0_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_3", "role": "ce0" }} , 
 	{ "name": "K_3_0_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_3", "role": "q0" }} , 
 	{ "name": "K_3_0_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_4", "role": "address0" }} , 
 	{ "name": "K_3_0_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_4", "role": "ce0" }} , 
 	{ "name": "K_3_0_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_4", "role": "q0" }} , 
 	{ "name": "K_3_0_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_5", "role": "address0" }} , 
 	{ "name": "K_3_0_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_5", "role": "ce0" }} , 
 	{ "name": "K_3_0_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_5", "role": "q0" }} , 
 	{ "name": "K_3_0_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_6", "role": "address0" }} , 
 	{ "name": "K_3_0_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_6", "role": "ce0" }} , 
 	{ "name": "K_3_0_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_6", "role": "q0" }} , 
 	{ "name": "K_3_0_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_0_7", "role": "address0" }} , 
 	{ "name": "K_3_0_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_0_7", "role": "ce0" }} , 
 	{ "name": "K_3_0_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_0_7", "role": "q0" }} , 
 	{ "name": "K_3_1_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_0", "role": "address0" }} , 
 	{ "name": "K_3_1_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_0", "role": "ce0" }} , 
 	{ "name": "K_3_1_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_0", "role": "q0" }} , 
 	{ "name": "K_3_1_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_1", "role": "address0" }} , 
 	{ "name": "K_3_1_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_1", "role": "ce0" }} , 
 	{ "name": "K_3_1_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_1", "role": "q0" }} , 
 	{ "name": "K_3_1_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_2", "role": "address0" }} , 
 	{ "name": "K_3_1_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_2", "role": "ce0" }} , 
 	{ "name": "K_3_1_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_2", "role": "q0" }} , 
 	{ "name": "K_3_1_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_3", "role": "address0" }} , 
 	{ "name": "K_3_1_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_3", "role": "ce0" }} , 
 	{ "name": "K_3_1_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_3", "role": "q0" }} , 
 	{ "name": "K_3_1_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_4", "role": "address0" }} , 
 	{ "name": "K_3_1_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_4", "role": "ce0" }} , 
 	{ "name": "K_3_1_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_4", "role": "q0" }} , 
 	{ "name": "K_3_1_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_5", "role": "address0" }} , 
 	{ "name": "K_3_1_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_5", "role": "ce0" }} , 
 	{ "name": "K_3_1_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_5", "role": "q0" }} , 
 	{ "name": "K_3_1_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_6", "role": "address0" }} , 
 	{ "name": "K_3_1_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_6", "role": "ce0" }} , 
 	{ "name": "K_3_1_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_6", "role": "q0" }} , 
 	{ "name": "K_3_1_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "K_3_1_7", "role": "address0" }} , 
 	{ "name": "K_3_1_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "K_3_1_7", "role": "ce0" }} , 
 	{ "name": "K_3_1_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "K_3_1_7", "role": "q0" }} , 
 	{ "name": "kk_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "kk_0", "role": "address0" }} , 
 	{ "name": "kk_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "kk_0", "role": "ce0" }} , 
 	{ "name": "kk_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "kk_0", "role": "q0" }} , 
 	{ "name": "kk_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "kk_1", "role": "address0" }} , 
 	{ "name": "kk_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "kk_1", "role": "ce0" }} , 
 	{ "name": "kk_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "kk_1", "role": "q0" }} , 
 	{ "name": "x_out_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_0", "role": "address0" }} , 
 	{ "name": "x_out_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "ce0" }} , 
 	{ "name": "x_out_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_0", "role": "q0" }} , 
 	{ "name": "x_out_0_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_0", "role": "address1" }} , 
 	{ "name": "x_out_0_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "ce1" }} , 
 	{ "name": "x_out_0_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_0", "role": "we1" }} , 
 	{ "name": "x_out_0_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_0", "role": "d1" }} , 
 	{ "name": "x_out_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_1", "role": "address0" }} , 
 	{ "name": "x_out_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "ce0" }} , 
 	{ "name": "x_out_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_1", "role": "q0" }} , 
 	{ "name": "x_out_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_1", "role": "address1" }} , 
 	{ "name": "x_out_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "ce1" }} , 
 	{ "name": "x_out_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_1", "role": "we1" }} , 
 	{ "name": "x_out_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_1", "role": "d1" }} , 
 	{ "name": "x_out_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_2", "role": "address0" }} , 
 	{ "name": "x_out_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "ce0" }} , 
 	{ "name": "x_out_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_2", "role": "q0" }} , 
 	{ "name": "x_out_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_2", "role": "address1" }} , 
 	{ "name": "x_out_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "ce1" }} , 
 	{ "name": "x_out_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_2", "role": "we1" }} , 
 	{ "name": "x_out_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_2", "role": "d1" }} , 
 	{ "name": "x_out_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_3", "role": "address0" }} , 
 	{ "name": "x_out_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "ce0" }} , 
 	{ "name": "x_out_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_3", "role": "q0" }} , 
 	{ "name": "x_out_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_3", "role": "address1" }} , 
 	{ "name": "x_out_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "ce1" }} , 
 	{ "name": "x_out_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_3", "role": "we1" }} , 
 	{ "name": "x_out_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_3", "role": "d1" }} , 
 	{ "name": "x_out_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_4", "role": "address0" }} , 
 	{ "name": "x_out_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "ce0" }} , 
 	{ "name": "x_out_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_4", "role": "q0" }} , 
 	{ "name": "x_out_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_4", "role": "address1" }} , 
 	{ "name": "x_out_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "ce1" }} , 
 	{ "name": "x_out_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_4", "role": "we1" }} , 
 	{ "name": "x_out_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_4", "role": "d1" }} , 
 	{ "name": "x_out_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_5", "role": "address0" }} , 
 	{ "name": "x_out_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "ce0" }} , 
 	{ "name": "x_out_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_5", "role": "q0" }} , 
 	{ "name": "x_out_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_5", "role": "address1" }} , 
 	{ "name": "x_out_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "ce1" }} , 
 	{ "name": "x_out_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_5", "role": "we1" }} , 
 	{ "name": "x_out_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_5", "role": "d1" }} , 
 	{ "name": "x_out_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_6", "role": "address0" }} , 
 	{ "name": "x_out_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_6", "role": "ce0" }} , 
 	{ "name": "x_out_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_6", "role": "q0" }} , 
 	{ "name": "x_out_6_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_6", "role": "address1" }} , 
 	{ "name": "x_out_6_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_6", "role": "ce1" }} , 
 	{ "name": "x_out_6_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_6", "role": "we1" }} , 
 	{ "name": "x_out_6_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_6", "role": "d1" }} , 
 	{ "name": "x_out_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_7", "role": "address0" }} , 
 	{ "name": "x_out_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_7", "role": "ce0" }} , 
 	{ "name": "x_out_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_7", "role": "q0" }} , 
 	{ "name": "x_out_7_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "x_out_7", "role": "address1" }} , 
 	{ "name": "x_out_7_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_7", "role": "ce1" }} , 
 	{ "name": "x_out_7_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "x_out_7", "role": "we1" }} , 
 	{ "name": "x_out_7_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_out_7", "role": "d1" }} , 
 	{ "name": "u_out_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "u_out_0", "role": "address0" }} , 
 	{ "name": "u_out_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_0", "role": "ce0" }} , 
 	{ "name": "u_out_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_0", "role": "we0" }} , 
 	{ "name": "u_out_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "u_out_0", "role": "d0" }} , 
 	{ "name": "u_out_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "u_out_1", "role": "address0" }} , 
 	{ "name": "u_out_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_1", "role": "ce0" }} , 
 	{ "name": "u_out_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "u_out_1", "role": "we0" }} , 
 	{ "name": "u_out_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "u_out_1", "role": "d0" }} , 
 	{ "name": "grp_fu_830_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_830_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_830_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "grp_fu_830_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_830_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":51, "type": "signal", "bundle":{"name": "grp_fu_830_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_830_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_830_p_ce", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	riccati_forward_pass {
		step_data_0_0 {Type I LastRead 0 FirstWrite -1}
		step_data_0_1 {Type I LastRead 0 FirstWrite -1}
		step_data_0_2 {Type I LastRead 0 FirstWrite -1}
		step_data_0_3 {Type I LastRead 0 FirstWrite -1}
		step_data_0_4 {Type I LastRead 0 FirstWrite -1}
		step_data_0_5 {Type I LastRead 0 FirstWrite -1}
		step_data_1 {Type I LastRead 5 FirstWrite -1}
		B_sparse_0 {Type I LastRead 4 FirstWrite -1}
		B_sparse_1 {Type I LastRead 4 FirstWrite -1}
		B_sparse_2 {Type I LastRead 4 FirstWrite -1}
		B_sparse_3 {Type I LastRead 4 FirstWrite -1}
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
		kk_0 {Type I LastRead 3 FirstWrite -1}
		kk_1 {Type I LastRead 3 FirstWrite -1}
		x_out_0 {Type IO LastRead 1 FirstWrite 0}
		x_out_1 {Type IO LastRead 1 FirstWrite 0}
		x_out_2 {Type IO LastRead 1 FirstWrite 0}
		x_out_3 {Type IO LastRead 1 FirstWrite 0}
		x_out_4 {Type IO LastRead 1 FirstWrite 0}
		x_out_5 {Type IO LastRead 1 FirstWrite 0}
		x_out_6 {Type IO LastRead 1 FirstWrite 0}
		x_out_7 {Type IO LastRead 1 FirstWrite 0}
		u_out_0 {Type O LastRead -1 FirstWrite 4}
		u_out_1 {Type O LastRead -1 FirstWrite 4}}
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
	{"Name" : "Latency", "Min" : "341", "Max" : "341"}
	, {"Name" : "Interval", "Min" : "341", "Max" : "341"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	step_data_0_0 { ap_memory {  { step_data_0_0_address0 mem_address 1 7 }  { step_data_0_0_ce0 mem_ce 1 1 }  { step_data_0_0_q0 mem_dout 0 26 } } }
	step_data_0_1 { ap_memory {  { step_data_0_1_address0 mem_address 1 7 }  { step_data_0_1_ce0 mem_ce 1 1 }  { step_data_0_1_q0 mem_dout 0 26 } } }
	step_data_0_2 { ap_memory {  { step_data_0_2_address0 mem_address 1 7 }  { step_data_0_2_ce0 mem_ce 1 1 }  { step_data_0_2_q0 mem_dout 0 26 } } }
	step_data_0_3 { ap_memory {  { step_data_0_3_address0 mem_address 1 7 }  { step_data_0_3_ce0 mem_ce 1 1 }  { step_data_0_3_q0 mem_dout 0 26 } } }
	step_data_0_4 { ap_memory {  { step_data_0_4_address0 mem_address 1 7 }  { step_data_0_4_ce0 mem_ce 1 1 }  { step_data_0_4_q0 mem_dout 0 26 } } }
	step_data_0_5 { ap_memory {  { step_data_0_5_address0 mem_address 1 7 }  { step_data_0_5_ce0 mem_ce 1 1 }  { step_data_0_5_q0 mem_dout 0 26 } } }
	step_data_1 { ap_memory {  { step_data_1_address0 mem_address 1 7 }  { step_data_1_ce0 mem_ce 1 1 }  { step_data_1_q0 mem_dout 0 26 }  { step_data_1_address1 MemPortADDR2 1 7 }  { step_data_1_ce1 MemPortCE2 1 1 }  { step_data_1_q1 MemPortDOUT2 0 26 } } }
	B_sparse_0 { ap_memory {  { B_sparse_0_address0 mem_address 1 5 }  { B_sparse_0_ce0 mem_ce 1 1 }  { B_sparse_0_q0 mem_dout 0 26 } } }
	B_sparse_1 { ap_memory {  { B_sparse_1_address0 mem_address 1 5 }  { B_sparse_1_ce0 mem_ce 1 1 }  { B_sparse_1_q0 mem_dout 0 26 } } }
	B_sparse_2 { ap_memory {  { B_sparse_2_address0 mem_address 1 5 }  { B_sparse_2_ce0 mem_ce 1 1 }  { B_sparse_2_q0 mem_dout 0 26 } } }
	B_sparse_3 { ap_memory {  { B_sparse_3_address0 mem_address 1 5 }  { B_sparse_3_ce0 mem_ce 1 1 }  { B_sparse_3_q0 mem_dout 0 26 } } }
	p_read { ap_none {  { p_read in_data 0 26 } } }
	p_read1 { ap_none {  { p_read1 in_data 0 26 } } }
	p_read2 { ap_none {  { p_read2 in_data 0 26 } } }
	p_read3 { ap_none {  { p_read3 in_data 0 26 } } }
	p_read4 { ap_none {  { p_read4 in_data 0 26 } } }
	p_read5 { ap_none {  { p_read5 in_data 0 26 } } }
	p_read6 { ap_none {  { p_read6 in_data 0 26 } } }
	p_read7 { ap_none {  { p_read7 in_data 0 26 } } }
	K_0_0_0 { ap_memory {  { K_0_0_0_address0 mem_address 1 3 }  { K_0_0_0_ce0 mem_ce 1 1 }  { K_0_0_0_q0 mem_dout 0 17 } } }
	K_0_0_1 { ap_memory {  { K_0_0_1_address0 mem_address 1 3 }  { K_0_0_1_ce0 mem_ce 1 1 }  { K_0_0_1_q0 mem_dout 0 17 } } }
	K_0_0_2 { ap_memory {  { K_0_0_2_address0 mem_address 1 3 }  { K_0_0_2_ce0 mem_ce 1 1 }  { K_0_0_2_q0 mem_dout 0 17 } } }
	K_0_0_3 { ap_memory {  { K_0_0_3_address0 mem_address 1 3 }  { K_0_0_3_ce0 mem_ce 1 1 }  { K_0_0_3_q0 mem_dout 0 17 } } }
	K_0_0_4 { ap_memory {  { K_0_0_4_address0 mem_address 1 3 }  { K_0_0_4_ce0 mem_ce 1 1 }  { K_0_0_4_q0 mem_dout 0 17 } } }
	K_0_0_5 { ap_memory {  { K_0_0_5_address0 mem_address 1 3 }  { K_0_0_5_ce0 mem_ce 1 1 }  { K_0_0_5_q0 mem_dout 0 17 } } }
	K_0_0_6 { ap_memory {  { K_0_0_6_address0 mem_address 1 3 }  { K_0_0_6_ce0 mem_ce 1 1 }  { K_0_0_6_q0 mem_dout 0 17 } } }
	K_0_0_7 { ap_memory {  { K_0_0_7_address0 mem_address 1 3 }  { K_0_0_7_ce0 mem_ce 1 1 }  { K_0_0_7_q0 mem_dout 0 17 } } }
	K_0_1_0 { ap_memory {  { K_0_1_0_address0 mem_address 1 3 }  { K_0_1_0_ce0 mem_ce 1 1 }  { K_0_1_0_q0 mem_dout 0 17 } } }
	K_0_1_1 { ap_memory {  { K_0_1_1_address0 mem_address 1 3 }  { K_0_1_1_ce0 mem_ce 1 1 }  { K_0_1_1_q0 mem_dout 0 17 } } }
	K_0_1_2 { ap_memory {  { K_0_1_2_address0 mem_address 1 3 }  { K_0_1_2_ce0 mem_ce 1 1 }  { K_0_1_2_q0 mem_dout 0 17 } } }
	K_0_1_3 { ap_memory {  { K_0_1_3_address0 mem_address 1 3 }  { K_0_1_3_ce0 mem_ce 1 1 }  { K_0_1_3_q0 mem_dout 0 17 } } }
	K_0_1_4 { ap_memory {  { K_0_1_4_address0 mem_address 1 3 }  { K_0_1_4_ce0 mem_ce 1 1 }  { K_0_1_4_q0 mem_dout 0 17 } } }
	K_0_1_5 { ap_memory {  { K_0_1_5_address0 mem_address 1 3 }  { K_0_1_5_ce0 mem_ce 1 1 }  { K_0_1_5_q0 mem_dout 0 17 } } }
	K_0_1_6 { ap_memory {  { K_0_1_6_address0 mem_address 1 3 }  { K_0_1_6_ce0 mem_ce 1 1 }  { K_0_1_6_q0 mem_dout 0 17 } } }
	K_0_1_7 { ap_memory {  { K_0_1_7_address0 mem_address 1 3 }  { K_0_1_7_ce0 mem_ce 1 1 }  { K_0_1_7_q0 mem_dout 0 17 } } }
	K_1_0_0 { ap_memory {  { K_1_0_0_address0 mem_address 1 3 }  { K_1_0_0_ce0 mem_ce 1 1 }  { K_1_0_0_q0 mem_dout 0 17 } } }
	K_1_0_1 { ap_memory {  { K_1_0_1_address0 mem_address 1 3 }  { K_1_0_1_ce0 mem_ce 1 1 }  { K_1_0_1_q0 mem_dout 0 17 } } }
	K_1_0_2 { ap_memory {  { K_1_0_2_address0 mem_address 1 3 }  { K_1_0_2_ce0 mem_ce 1 1 }  { K_1_0_2_q0 mem_dout 0 17 } } }
	K_1_0_3 { ap_memory {  { K_1_0_3_address0 mem_address 1 3 }  { K_1_0_3_ce0 mem_ce 1 1 }  { K_1_0_3_q0 mem_dout 0 17 } } }
	K_1_0_4 { ap_memory {  { K_1_0_4_address0 mem_address 1 3 }  { K_1_0_4_ce0 mem_ce 1 1 }  { K_1_0_4_q0 mem_dout 0 17 } } }
	K_1_0_5 { ap_memory {  { K_1_0_5_address0 mem_address 1 3 }  { K_1_0_5_ce0 mem_ce 1 1 }  { K_1_0_5_q0 mem_dout 0 17 } } }
	K_1_0_6 { ap_memory {  { K_1_0_6_address0 mem_address 1 3 }  { K_1_0_6_ce0 mem_ce 1 1 }  { K_1_0_6_q0 mem_dout 0 17 } } }
	K_1_0_7 { ap_memory {  { K_1_0_7_address0 mem_address 1 3 }  { K_1_0_7_ce0 mem_ce 1 1 }  { K_1_0_7_q0 mem_dout 0 17 } } }
	K_1_1_0 { ap_memory {  { K_1_1_0_address0 mem_address 1 3 }  { K_1_1_0_ce0 mem_ce 1 1 }  { K_1_1_0_q0 mem_dout 0 17 } } }
	K_1_1_1 { ap_memory {  { K_1_1_1_address0 mem_address 1 3 }  { K_1_1_1_ce0 mem_ce 1 1 }  { K_1_1_1_q0 mem_dout 0 17 } } }
	K_1_1_2 { ap_memory {  { K_1_1_2_address0 mem_address 1 3 }  { K_1_1_2_ce0 mem_ce 1 1 }  { K_1_1_2_q0 mem_dout 0 17 } } }
	K_1_1_3 { ap_memory {  { K_1_1_3_address0 mem_address 1 3 }  { K_1_1_3_ce0 mem_ce 1 1 }  { K_1_1_3_q0 mem_dout 0 17 } } }
	K_1_1_4 { ap_memory {  { K_1_1_4_address0 mem_address 1 3 }  { K_1_1_4_ce0 mem_ce 1 1 }  { K_1_1_4_q0 mem_dout 0 17 } } }
	K_1_1_5 { ap_memory {  { K_1_1_5_address0 mem_address 1 3 }  { K_1_1_5_ce0 mem_ce 1 1 }  { K_1_1_5_q0 mem_dout 0 17 } } }
	K_1_1_6 { ap_memory {  { K_1_1_6_address0 mem_address 1 3 }  { K_1_1_6_ce0 mem_ce 1 1 }  { K_1_1_6_q0 mem_dout 0 17 } } }
	K_1_1_7 { ap_memory {  { K_1_1_7_address0 mem_address 1 3 }  { K_1_1_7_ce0 mem_ce 1 1 }  { K_1_1_7_q0 mem_dout 0 17 } } }
	K_2_0_0 { ap_memory {  { K_2_0_0_address0 mem_address 1 3 }  { K_2_0_0_ce0 mem_ce 1 1 }  { K_2_0_0_q0 mem_dout 0 17 } } }
	K_2_0_1 { ap_memory {  { K_2_0_1_address0 mem_address 1 3 }  { K_2_0_1_ce0 mem_ce 1 1 }  { K_2_0_1_q0 mem_dout 0 17 } } }
	K_2_0_2 { ap_memory {  { K_2_0_2_address0 mem_address 1 3 }  { K_2_0_2_ce0 mem_ce 1 1 }  { K_2_0_2_q0 mem_dout 0 17 } } }
	K_2_0_3 { ap_memory {  { K_2_0_3_address0 mem_address 1 3 }  { K_2_0_3_ce0 mem_ce 1 1 }  { K_2_0_3_q0 mem_dout 0 17 } } }
	K_2_0_4 { ap_memory {  { K_2_0_4_address0 mem_address 1 3 }  { K_2_0_4_ce0 mem_ce 1 1 }  { K_2_0_4_q0 mem_dout 0 17 } } }
	K_2_0_5 { ap_memory {  { K_2_0_5_address0 mem_address 1 3 }  { K_2_0_5_ce0 mem_ce 1 1 }  { K_2_0_5_q0 mem_dout 0 17 } } }
	K_2_0_6 { ap_memory {  { K_2_0_6_address0 mem_address 1 3 }  { K_2_0_6_ce0 mem_ce 1 1 }  { K_2_0_6_q0 mem_dout 0 17 } } }
	K_2_0_7 { ap_memory {  { K_2_0_7_address0 mem_address 1 3 }  { K_2_0_7_ce0 mem_ce 1 1 }  { K_2_0_7_q0 mem_dout 0 17 } } }
	K_2_1_0 { ap_memory {  { K_2_1_0_address0 mem_address 1 3 }  { K_2_1_0_ce0 mem_ce 1 1 }  { K_2_1_0_q0 mem_dout 0 17 } } }
	K_2_1_1 { ap_memory {  { K_2_1_1_address0 mem_address 1 3 }  { K_2_1_1_ce0 mem_ce 1 1 }  { K_2_1_1_q0 mem_dout 0 17 } } }
	K_2_1_2 { ap_memory {  { K_2_1_2_address0 mem_address 1 3 }  { K_2_1_2_ce0 mem_ce 1 1 }  { K_2_1_2_q0 mem_dout 0 17 } } }
	K_2_1_3 { ap_memory {  { K_2_1_3_address0 mem_address 1 3 }  { K_2_1_3_ce0 mem_ce 1 1 }  { K_2_1_3_q0 mem_dout 0 17 } } }
	K_2_1_4 { ap_memory {  { K_2_1_4_address0 mem_address 1 3 }  { K_2_1_4_ce0 mem_ce 1 1 }  { K_2_1_4_q0 mem_dout 0 17 } } }
	K_2_1_5 { ap_memory {  { K_2_1_5_address0 mem_address 1 3 }  { K_2_1_5_ce0 mem_ce 1 1 }  { K_2_1_5_q0 mem_dout 0 17 } } }
	K_2_1_6 { ap_memory {  { K_2_1_6_address0 mem_address 1 3 }  { K_2_1_6_ce0 mem_ce 1 1 }  { K_2_1_6_q0 mem_dout 0 17 } } }
	K_2_1_7 { ap_memory {  { K_2_1_7_address0 mem_address 1 3 }  { K_2_1_7_ce0 mem_ce 1 1 }  { K_2_1_7_q0 mem_dout 0 17 } } }
	K_3_0_0 { ap_memory {  { K_3_0_0_address0 mem_address 1 3 }  { K_3_0_0_ce0 mem_ce 1 1 }  { K_3_0_0_q0 mem_dout 0 17 } } }
	K_3_0_1 { ap_memory {  { K_3_0_1_address0 mem_address 1 3 }  { K_3_0_1_ce0 mem_ce 1 1 }  { K_3_0_1_q0 mem_dout 0 17 } } }
	K_3_0_2 { ap_memory {  { K_3_0_2_address0 mem_address 1 3 }  { K_3_0_2_ce0 mem_ce 1 1 }  { K_3_0_2_q0 mem_dout 0 17 } } }
	K_3_0_3 { ap_memory {  { K_3_0_3_address0 mem_address 1 3 }  { K_3_0_3_ce0 mem_ce 1 1 }  { K_3_0_3_q0 mem_dout 0 17 } } }
	K_3_0_4 { ap_memory {  { K_3_0_4_address0 mem_address 1 3 }  { K_3_0_4_ce0 mem_ce 1 1 }  { K_3_0_4_q0 mem_dout 0 17 } } }
	K_3_0_5 { ap_memory {  { K_3_0_5_address0 mem_address 1 3 }  { K_3_0_5_ce0 mem_ce 1 1 }  { K_3_0_5_q0 mem_dout 0 17 } } }
	K_3_0_6 { ap_memory {  { K_3_0_6_address0 mem_address 1 3 }  { K_3_0_6_ce0 mem_ce 1 1 }  { K_3_0_6_q0 mem_dout 0 17 } } }
	K_3_0_7 { ap_memory {  { K_3_0_7_address0 mem_address 1 3 }  { K_3_0_7_ce0 mem_ce 1 1 }  { K_3_0_7_q0 mem_dout 0 17 } } }
	K_3_1_0 { ap_memory {  { K_3_1_0_address0 mem_address 1 3 }  { K_3_1_0_ce0 mem_ce 1 1 }  { K_3_1_0_q0 mem_dout 0 17 } } }
	K_3_1_1 { ap_memory {  { K_3_1_1_address0 mem_address 1 3 }  { K_3_1_1_ce0 mem_ce 1 1 }  { K_3_1_1_q0 mem_dout 0 17 } } }
	K_3_1_2 { ap_memory {  { K_3_1_2_address0 mem_address 1 3 }  { K_3_1_2_ce0 mem_ce 1 1 }  { K_3_1_2_q0 mem_dout 0 17 } } }
	K_3_1_3 { ap_memory {  { K_3_1_3_address0 mem_address 1 3 }  { K_3_1_3_ce0 mem_ce 1 1 }  { K_3_1_3_q0 mem_dout 0 17 } } }
	K_3_1_4 { ap_memory {  { K_3_1_4_address0 mem_address 1 3 }  { K_3_1_4_ce0 mem_ce 1 1 }  { K_3_1_4_q0 mem_dout 0 17 } } }
	K_3_1_5 { ap_memory {  { K_3_1_5_address0 mem_address 1 3 }  { K_3_1_5_ce0 mem_ce 1 1 }  { K_3_1_5_q0 mem_dout 0 17 } } }
	K_3_1_6 { ap_memory {  { K_3_1_6_address0 mem_address 1 3 }  { K_3_1_6_ce0 mem_ce 1 1 }  { K_3_1_6_q0 mem_dout 0 17 } } }
	K_3_1_7 { ap_memory {  { K_3_1_7_address0 mem_address 1 3 }  { K_3_1_7_ce0 mem_ce 1 1 }  { K_3_1_7_q0 mem_dout 0 17 } } }
	kk_0 { ap_memory {  { kk_0_address0 mem_address 1 5 }  { kk_0_ce0 mem_ce 1 1 }  { kk_0_q0 mem_dout 0 17 } } }
	kk_1 { ap_memory {  { kk_1_address0 mem_address 1 5 }  { kk_1_ce0 mem_ce 1 1 }  { kk_1_q0 mem_dout 0 17 } } }
	x_out_0 { ap_memory {  { x_out_0_address0 mem_address 1 5 }  { x_out_0_ce0 mem_ce 1 1 }  { x_out_0_q0 in_data 0 26 }  { x_out_0_address1 MemPortADDR2 1 5 }  { x_out_0_ce1 MemPortCE2 1 1 }  { x_out_0_we1 MemPortWE2 1 1 }  { x_out_0_d1 MemPortDIN2 1 26 } } }
	x_out_1 { ap_memory {  { x_out_1_address0 mem_address 1 5 }  { x_out_1_ce0 mem_ce 1 1 }  { x_out_1_q0 in_data 0 26 }  { x_out_1_address1 MemPortADDR2 1 5 }  { x_out_1_ce1 MemPortCE2 1 1 }  { x_out_1_we1 MemPortWE2 1 1 }  { x_out_1_d1 MemPortDIN2 1 26 } } }
	x_out_2 { ap_memory {  { x_out_2_address0 mem_address 1 5 }  { x_out_2_ce0 mem_ce 1 1 }  { x_out_2_q0 in_data 0 26 }  { x_out_2_address1 MemPortADDR2 1 5 }  { x_out_2_ce1 MemPortCE2 1 1 }  { x_out_2_we1 MemPortWE2 1 1 }  { x_out_2_d1 MemPortDIN2 1 26 } } }
	x_out_3 { ap_memory {  { x_out_3_address0 mem_address 1 5 }  { x_out_3_ce0 mem_ce 1 1 }  { x_out_3_q0 in_data 0 26 }  { x_out_3_address1 MemPortADDR2 1 5 }  { x_out_3_ce1 MemPortCE2 1 1 }  { x_out_3_we1 MemPortWE2 1 1 }  { x_out_3_d1 MemPortDIN2 1 26 } } }
	x_out_4 { ap_memory {  { x_out_4_address0 mem_address 1 5 }  { x_out_4_ce0 mem_ce 1 1 }  { x_out_4_q0 in_data 0 26 }  { x_out_4_address1 MemPortADDR2 1 5 }  { x_out_4_ce1 MemPortCE2 1 1 }  { x_out_4_we1 MemPortWE2 1 1 }  { x_out_4_d1 MemPortDIN2 1 26 } } }
	x_out_5 { ap_memory {  { x_out_5_address0 mem_address 1 5 }  { x_out_5_ce0 mem_ce 1 1 }  { x_out_5_q0 in_data 0 26 }  { x_out_5_address1 MemPortADDR2 1 5 }  { x_out_5_ce1 MemPortCE2 1 1 }  { x_out_5_we1 MemPortWE2 1 1 }  { x_out_5_d1 MemPortDIN2 1 26 } } }
	x_out_6 { ap_memory {  { x_out_6_address0 mem_address 1 5 }  { x_out_6_ce0 mem_ce 1 1 }  { x_out_6_q0 in_data 0 26 }  { x_out_6_address1 MemPortADDR2 1 5 }  { x_out_6_ce1 MemPortCE2 1 1 }  { x_out_6_we1 MemPortWE2 1 1 }  { x_out_6_d1 MemPortDIN2 1 26 } } }
	x_out_7 { ap_memory {  { x_out_7_address0 mem_address 1 5 }  { x_out_7_ce0 mem_ce 1 1 }  { x_out_7_q0 in_data 0 26 }  { x_out_7_address1 MemPortADDR2 1 5 }  { x_out_7_ce1 MemPortCE2 1 1 }  { x_out_7_we1 MemPortWE2 1 1 }  { x_out_7_d1 MemPortDIN2 1 26 } } }
	u_out_0 { ap_memory {  { u_out_0_address0 mem_address 1 5 }  { u_out_0_ce0 mem_ce 1 1 }  { u_out_0_we0 mem_we 1 1 }  { u_out_0_d0 mem_din 1 26 } } }
	u_out_1 { ap_memory {  { u_out_1_address0 mem_address 1 5 }  { u_out_1_ce0 mem_ce 1 1 }  { u_out_1_we0 mem_we 1 1 }  { u_out_1_d0 mem_din 1 26 } } }
}
