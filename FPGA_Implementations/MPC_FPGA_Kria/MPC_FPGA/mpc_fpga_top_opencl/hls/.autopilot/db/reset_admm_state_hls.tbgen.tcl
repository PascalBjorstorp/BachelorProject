set moduleName reset_admm_state_hls
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
set C_modelName {reset_admm_state_hls}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_0 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_0 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ p_anonymous_namespace_g_core_state_admm_z_x_0 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_1 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_2 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_3 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_4 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_5 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_6 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_x_7 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_0 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_1 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_2 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_3 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_4 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_5 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_6 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_x_7 int 26 regular {array 21 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_u_0 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_u_1 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_0 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_1 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_rho int 26 regular {pointer 1} {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_rho_u int 26 regular {pointer 1} {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_initialized int 1 regular {pointer 1} {global 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_rho", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_rho_u", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_initialized", "interface" : "wire", "bitwidth" : 1, "direction" : "WRITEONLY", "extern" : 0} ]}
# RTL Port declarations: 
set portNum 92
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_address0 sc_out sc_lv 5 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_ce0 sc_out sc_logic 1 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_we0 sc_out sc_logic 1 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_0_d0 sc_out sc_lv 26 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_address0 sc_out sc_lv 5 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_ce0 sc_out sc_logic 1 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_we0 sc_out sc_logic 1 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_1_d0 sc_out sc_lv 26 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_address0 sc_out sc_lv 5 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_ce0 sc_out sc_logic 1 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_we0 sc_out sc_logic 1 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_2_d0 sc_out sc_lv 26 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_address0 sc_out sc_lv 5 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_ce0 sc_out sc_logic 1 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_we0 sc_out sc_logic 1 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_3_d0 sc_out sc_lv 26 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_address0 sc_out sc_lv 5 signal 4 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_ce0 sc_out sc_logic 1 signal 4 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_we0 sc_out sc_logic 1 signal 4 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_4_d0 sc_out sc_lv 26 signal 4 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_address0 sc_out sc_lv 5 signal 5 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_ce0 sc_out sc_logic 1 signal 5 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_we0 sc_out sc_logic 1 signal 5 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_5_d0 sc_out sc_lv 26 signal 5 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_address0 sc_out sc_lv 5 signal 6 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_ce0 sc_out sc_logic 1 signal 6 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_we0 sc_out sc_logic 1 signal 6 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_6_d0 sc_out sc_lv 26 signal 6 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_address0 sc_out sc_lv 5 signal 7 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_ce0 sc_out sc_logic 1 signal 7 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_we0 sc_out sc_logic 1 signal 7 } 
	{ p_anonymous_namespace_g_core_state_admm_z_x_7_d0 sc_out sc_lv 26 signal 7 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_address0 sc_out sc_lv 5 signal 8 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_ce0 sc_out sc_logic 1 signal 8 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_we0 sc_out sc_logic 1 signal 8 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_0_d0 sc_out sc_lv 26 signal 8 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_address0 sc_out sc_lv 5 signal 9 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_ce0 sc_out sc_logic 1 signal 9 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_we0 sc_out sc_logic 1 signal 9 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_1_d0 sc_out sc_lv 26 signal 9 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_address0 sc_out sc_lv 5 signal 10 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_ce0 sc_out sc_logic 1 signal 10 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_we0 sc_out sc_logic 1 signal 10 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_2_d0 sc_out sc_lv 26 signal 10 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_address0 sc_out sc_lv 5 signal 11 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_ce0 sc_out sc_logic 1 signal 11 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_we0 sc_out sc_logic 1 signal 11 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_3_d0 sc_out sc_lv 26 signal 11 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_address0 sc_out sc_lv 5 signal 12 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_ce0 sc_out sc_logic 1 signal 12 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_we0 sc_out sc_logic 1 signal 12 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_4_d0 sc_out sc_lv 26 signal 12 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_address0 sc_out sc_lv 5 signal 13 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_ce0 sc_out sc_logic 1 signal 13 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_we0 sc_out sc_logic 1 signal 13 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_5_d0 sc_out sc_lv 26 signal 13 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_address0 sc_out sc_lv 5 signal 14 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_ce0 sc_out sc_logic 1 signal 14 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_we0 sc_out sc_logic 1 signal 14 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_6_d0 sc_out sc_lv 26 signal 14 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_address0 sc_out sc_lv 5 signal 15 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_ce0 sc_out sc_logic 1 signal 15 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_we0 sc_out sc_logic 1 signal 15 } 
	{ p_anonymous_namespace_g_core_state_admm_y_x_7_d0 sc_out sc_lv 26 signal 15 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_address0 sc_out sc_lv 5 signal 16 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 sc_out sc_logic 1 signal 16 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_we0 sc_out sc_logic 1 signal 16 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_d0 sc_out sc_lv 26 signal 16 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_address0 sc_out sc_lv 5 signal 17 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 sc_out sc_logic 1 signal 17 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_we0 sc_out sc_logic 1 signal 17 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_d0 sc_out sc_lv 26 signal 17 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_address0 sc_out sc_lv 5 signal 18 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 sc_out sc_logic 1 signal 18 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_we0 sc_out sc_logic 1 signal 18 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_d0 sc_out sc_lv 26 signal 18 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_address0 sc_out sc_lv 5 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 sc_out sc_logic 1 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_we0 sc_out sc_logic 1 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_d0 sc_out sc_lv 26 signal 19 } 
	{ p_anonymous_namespace_g_core_state_admm_rho sc_out sc_lv 26 signal 20 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_ap_vld sc_out sc_logic 1 outvld 20 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_u sc_out sc_lv 26 signal 21 } 
	{ p_anonymous_namespace_g_core_state_admm_rho_u_ap_vld sc_out sc_logic 1 outvld 21 } 
	{ p_anonymous_namespace_g_core_state_admm_initialized sc_out sc_lv 1 signal 22 } 
	{ p_anonymous_namespace_g_core_state_admm_initialized_ap_vld sc_out sc_logic 1 outvld 22 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_2_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_2", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_3_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_3", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_4_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_4", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_5_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_5", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_6_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_6", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_x_7_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_x_7", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_2_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_2", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_3_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_3", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_4_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_4", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_5_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_5", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_6_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_6", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_x_7_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_x_7", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho", "role": "ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_u", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho_u", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_rho_u_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_rho_u", "role": "ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_initialized", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_initialized", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_initialized_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_initialized", "role": "ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
	reset_admm_state_hls {
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_rho {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_initialized {Type O LastRead -1 FirstWrite 1}}
	reset_admm_state_hls_Pipeline_VITIS_LOOP_322_1 {
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}}
	reset_admm_state_hls_Pipeline_VITIS_LOOP_330_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "24", "Max" : "24"}
	, {"Name" : "Interval", "Min" : "24", "Max" : "24"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	p_anonymous_namespace_g_core_state_admm_z_x_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_0_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_1_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_2 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_2_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_2_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_2_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_2_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_3 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_3_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_3_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_3_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_3_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_4 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_4_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_4_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_4_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_4_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_5 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_5_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_5_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_5_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_5_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_6 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_6_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_6_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_6_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_6_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_x_7 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_x_7_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_x_7_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_7_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_x_7_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_0_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_1_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_2 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_2_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_2_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_2_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_2_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_3 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_3_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_3_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_3_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_3_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_4 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_4_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_4_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_4_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_4_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_5 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_5_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_5_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_5_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_5_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_6 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_6_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_6_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_6_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_6_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_x_7 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_x_7_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_x_7_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_7_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_x_7_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_rho { ap_vld {  { p_anonymous_namespace_g_core_state_admm_rho out_data 1 26 }  { p_anonymous_namespace_g_core_state_admm_rho_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_admm_rho_u { ap_vld {  { p_anonymous_namespace_g_core_state_admm_rho_u out_data 1 26 }  { p_anonymous_namespace_g_core_state_admm_rho_u_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_admm_initialized { ap_vld {  { p_anonymous_namespace_g_core_state_admm_initialized out_data 1 1 }  { p_anonymous_namespace_g_core_state_admm_initialized_ap_vld out_vld 1 1 } } }
}
