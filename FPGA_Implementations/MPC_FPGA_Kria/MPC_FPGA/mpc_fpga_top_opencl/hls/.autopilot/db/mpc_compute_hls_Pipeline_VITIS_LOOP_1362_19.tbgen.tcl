set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19
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
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict z_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_u { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_u { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict sol_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_3 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_4 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_u { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict step_data_5 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ z_u_1 int 26 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ z_u int 26 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ y_u_1 int 26 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ y_u int 26 regular {array 20 { 0 1 } 1 1 bus  }  }
	{ y_x_5 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ y_x int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ z_x_7 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_6 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_5 int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ z_x_4 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_3 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_2 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_1 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x int 26 regular {array 21 { 1 0 } 1 1 bus  }  }
	{ sol_x_1 int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_2 int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_3 int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_4 int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_6 int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_7 int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_5 int 26 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ step_data_3 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ step_data_4 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ terminal_wall_x_lb_con_reload int 26 regular  }
	{ terminal_wall_x_ub_con_reload int 26 regular  }
	{ sext_ln156_35 int 26 regular  }
	{ sol_u int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ sol_u_1 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ sext_ln1362 int 26 regular  }
	{ step_data_5 int 26 regular {array 20 { 1 3 } 1 1 bus  }  }
	{ lnorm_u1_out int 26 regular {pointer 1}  }
	{ znorm_u1_out int 26 regular {pointer 1}  }
	{ dual_u1_out int 26 regular {pointer 1}  }
	{ primal_u1_out int 26 regular {pointer 1}  }
	{ lnorm_u0_out int 26 regular {pointer 1}  }
	{ znorm_u0_out int 26 regular {pointer 1}  }
	{ dual_u0_out int 26 regular {pointer 1}  }
	{ primal_u0_out int 26 regular {pointer 1}  }
	{ lnorm_da_out int 26 regular {pointer 1}  }
	{ znorm_da_out int 25 regular {pointer 1}  }
	{ dual_da_out int 26 regular {pointer 1}  }
	{ primal_da_out int 26 regular {pointer 1}  }
	{ lnorm_ey_out int 26 regular {pointer 1}  }
	{ znorm_ey_out int 26 regular {pointer 1}  }
	{ dual_ey_out int 26 regular {pointer 1}  }
	{ primal_ey_out int 26 regular {pointer 1}  }
	{ u_norm_out int 26 regular {pointer 1}  }
	{ x_norm_out int 26 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "z_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "z_u", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "y_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "y_u", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "y_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "y_x", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "z_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "z_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x", "interface" : "memory", "bitwidth" : 26, "direction" : "READWRITE"} , 
 	{ "Name" : "sol_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_3", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_4", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "terminal_wall_x_lb_con_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "terminal_wall_x_ub_con_reload", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln156_35", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_u", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sol_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln1362", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "step_data_5", "interface" : "memory", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "lnorm_u1_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_u1_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_u1_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_u1_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lnorm_u0_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_u0_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_u0_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_u0_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lnorm_da_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_da_out", "interface" : "wire", "bitwidth" : 25, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_da_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_da_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lnorm_ey_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "znorm_ey_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "dual_ey_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "primal_ey_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "u_norm_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "x_norm_out", "interface" : "wire", "bitwidth" : 26, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 173
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ z_u_1_address0 sc_out sc_lv 5 signal 0 } 
	{ z_u_1_ce0 sc_out sc_logic 1 signal 0 } 
	{ z_u_1_we0 sc_out sc_logic 1 signal 0 } 
	{ z_u_1_d0 sc_out sc_lv 26 signal 0 } 
	{ z_u_1_address1 sc_out sc_lv 5 signal 0 } 
	{ z_u_1_ce1 sc_out sc_logic 1 signal 0 } 
	{ z_u_1_q1 sc_in sc_lv 26 signal 0 } 
	{ z_u_address0 sc_out sc_lv 5 signal 1 } 
	{ z_u_ce0 sc_out sc_logic 1 signal 1 } 
	{ z_u_we0 sc_out sc_logic 1 signal 1 } 
	{ z_u_d0 sc_out sc_lv 26 signal 1 } 
	{ z_u_address1 sc_out sc_lv 5 signal 1 } 
	{ z_u_ce1 sc_out sc_logic 1 signal 1 } 
	{ z_u_q1 sc_in sc_lv 26 signal 1 } 
	{ y_u_1_address0 sc_out sc_lv 5 signal 2 } 
	{ y_u_1_ce0 sc_out sc_logic 1 signal 2 } 
	{ y_u_1_we0 sc_out sc_logic 1 signal 2 } 
	{ y_u_1_d0 sc_out sc_lv 26 signal 2 } 
	{ y_u_1_address1 sc_out sc_lv 5 signal 2 } 
	{ y_u_1_ce1 sc_out sc_logic 1 signal 2 } 
	{ y_u_1_q1 sc_in sc_lv 26 signal 2 } 
	{ y_u_address0 sc_out sc_lv 5 signal 3 } 
	{ y_u_ce0 sc_out sc_logic 1 signal 3 } 
	{ y_u_we0 sc_out sc_logic 1 signal 3 } 
	{ y_u_d0 sc_out sc_lv 26 signal 3 } 
	{ y_u_address1 sc_out sc_lv 5 signal 3 } 
	{ y_u_ce1 sc_out sc_logic 1 signal 3 } 
	{ y_u_q1 sc_in sc_lv 26 signal 3 } 
	{ y_x_5_address0 sc_out sc_lv 5 signal 4 } 
	{ y_x_5_ce0 sc_out sc_logic 1 signal 4 } 
	{ y_x_5_q0 sc_in sc_lv 26 signal 4 } 
	{ y_x_5_address1 sc_out sc_lv 5 signal 4 } 
	{ y_x_5_ce1 sc_out sc_logic 1 signal 4 } 
	{ y_x_5_we1 sc_out sc_logic 1 signal 4 } 
	{ y_x_5_d1 sc_out sc_lv 26 signal 4 } 
	{ y_x_address0 sc_out sc_lv 5 signal 5 } 
	{ y_x_ce0 sc_out sc_logic 1 signal 5 } 
	{ y_x_q0 sc_in sc_lv 26 signal 5 } 
	{ y_x_address1 sc_out sc_lv 5 signal 5 } 
	{ y_x_ce1 sc_out sc_logic 1 signal 5 } 
	{ y_x_we1 sc_out sc_logic 1 signal 5 } 
	{ y_x_d1 sc_out sc_lv 26 signal 5 } 
	{ z_x_7_address1 sc_out sc_lv 5 signal 6 } 
	{ z_x_7_ce1 sc_out sc_logic 1 signal 6 } 
	{ z_x_7_we1 sc_out sc_logic 1 signal 6 } 
	{ z_x_7_d1 sc_out sc_lv 26 signal 6 } 
	{ z_x_6_address1 sc_out sc_lv 5 signal 7 } 
	{ z_x_6_ce1 sc_out sc_logic 1 signal 7 } 
	{ z_x_6_we1 sc_out sc_logic 1 signal 7 } 
	{ z_x_6_d1 sc_out sc_lv 26 signal 7 } 
	{ z_x_5_address0 sc_out sc_lv 5 signal 8 } 
	{ z_x_5_ce0 sc_out sc_logic 1 signal 8 } 
	{ z_x_5_q0 sc_in sc_lv 26 signal 8 } 
	{ z_x_5_address1 sc_out sc_lv 5 signal 8 } 
	{ z_x_5_ce1 sc_out sc_logic 1 signal 8 } 
	{ z_x_5_we1 sc_out sc_logic 1 signal 8 } 
	{ z_x_5_d1 sc_out sc_lv 26 signal 8 } 
	{ z_x_4_address1 sc_out sc_lv 5 signal 9 } 
	{ z_x_4_ce1 sc_out sc_logic 1 signal 9 } 
	{ z_x_4_we1 sc_out sc_logic 1 signal 9 } 
	{ z_x_4_d1 sc_out sc_lv 26 signal 9 } 
	{ z_x_3_address1 sc_out sc_lv 5 signal 10 } 
	{ z_x_3_ce1 sc_out sc_logic 1 signal 10 } 
	{ z_x_3_we1 sc_out sc_logic 1 signal 10 } 
	{ z_x_3_d1 sc_out sc_lv 26 signal 10 } 
	{ z_x_2_address1 sc_out sc_lv 5 signal 11 } 
	{ z_x_2_ce1 sc_out sc_logic 1 signal 11 } 
	{ z_x_2_we1 sc_out sc_logic 1 signal 11 } 
	{ z_x_2_d1 sc_out sc_lv 26 signal 11 } 
	{ z_x_1_address1 sc_out sc_lv 5 signal 12 } 
	{ z_x_1_ce1 sc_out sc_logic 1 signal 12 } 
	{ z_x_1_we1 sc_out sc_logic 1 signal 12 } 
	{ z_x_1_d1 sc_out sc_lv 26 signal 12 } 
	{ z_x_address0 sc_out sc_lv 5 signal 13 } 
	{ z_x_ce0 sc_out sc_logic 1 signal 13 } 
	{ z_x_q0 sc_in sc_lv 26 signal 13 } 
	{ z_x_address1 sc_out sc_lv 5 signal 13 } 
	{ z_x_ce1 sc_out sc_logic 1 signal 13 } 
	{ z_x_we1 sc_out sc_logic 1 signal 13 } 
	{ z_x_d1 sc_out sc_lv 26 signal 13 } 
	{ sol_x_1_address0 sc_out sc_lv 5 signal 14 } 
	{ sol_x_1_ce0 sc_out sc_logic 1 signal 14 } 
	{ sol_x_1_q0 sc_in sc_lv 26 signal 14 } 
	{ sol_x_2_address0 sc_out sc_lv 5 signal 15 } 
	{ sol_x_2_ce0 sc_out sc_logic 1 signal 15 } 
	{ sol_x_2_q0 sc_in sc_lv 26 signal 15 } 
	{ sol_x_3_address0 sc_out sc_lv 5 signal 16 } 
	{ sol_x_3_ce0 sc_out sc_logic 1 signal 16 } 
	{ sol_x_3_q0 sc_in sc_lv 26 signal 16 } 
	{ sol_x_4_address0 sc_out sc_lv 5 signal 17 } 
	{ sol_x_4_ce0 sc_out sc_logic 1 signal 17 } 
	{ sol_x_4_q0 sc_in sc_lv 26 signal 17 } 
	{ sol_x_6_address0 sc_out sc_lv 5 signal 18 } 
	{ sol_x_6_ce0 sc_out sc_logic 1 signal 18 } 
	{ sol_x_6_q0 sc_in sc_lv 26 signal 18 } 
	{ sol_x_7_address0 sc_out sc_lv 5 signal 19 } 
	{ sol_x_7_ce0 sc_out sc_logic 1 signal 19 } 
	{ sol_x_7_q0 sc_in sc_lv 26 signal 19 } 
	{ sol_x_address0 sc_out sc_lv 5 signal 20 } 
	{ sol_x_ce0 sc_out sc_logic 1 signal 20 } 
	{ sol_x_q0 sc_in sc_lv 26 signal 20 } 
	{ sol_x_5_address0 sc_out sc_lv 5 signal 21 } 
	{ sol_x_5_ce0 sc_out sc_logic 1 signal 21 } 
	{ sol_x_5_q0 sc_in sc_lv 26 signal 21 } 
	{ step_data_3_address0 sc_out sc_lv 5 signal 22 } 
	{ step_data_3_ce0 sc_out sc_logic 1 signal 22 } 
	{ step_data_3_q0 sc_in sc_lv 26 signal 22 } 
	{ step_data_4_address0 sc_out sc_lv 5 signal 23 } 
	{ step_data_4_ce0 sc_out sc_logic 1 signal 23 } 
	{ step_data_4_q0 sc_in sc_lv 26 signal 23 } 
	{ terminal_wall_x_lb_con_reload sc_in sc_lv 26 signal 24 } 
	{ terminal_wall_x_ub_con_reload sc_in sc_lv 26 signal 25 } 
	{ sext_ln156_35 sc_in sc_lv 26 signal 26 } 
	{ sol_u_address0 sc_out sc_lv 5 signal 27 } 
	{ sol_u_ce0 sc_out sc_logic 1 signal 27 } 
	{ sol_u_q0 sc_in sc_lv 26 signal 27 } 
	{ sol_u_1_address0 sc_out sc_lv 5 signal 28 } 
	{ sol_u_1_ce0 sc_out sc_logic 1 signal 28 } 
	{ sol_u_1_q0 sc_in sc_lv 26 signal 28 } 
	{ sext_ln1362 sc_in sc_lv 26 signal 29 } 
	{ step_data_5_address0 sc_out sc_lv 5 signal 30 } 
	{ step_data_5_ce0 sc_out sc_logic 1 signal 30 } 
	{ step_data_5_q0 sc_in sc_lv 26 signal 30 } 
	{ lnorm_u1_out sc_out sc_lv 26 signal 31 } 
	{ lnorm_u1_out_ap_vld sc_out sc_logic 1 outvld 31 } 
	{ znorm_u1_out sc_out sc_lv 26 signal 32 } 
	{ znorm_u1_out_ap_vld sc_out sc_logic 1 outvld 32 } 
	{ dual_u1_out sc_out sc_lv 26 signal 33 } 
	{ dual_u1_out_ap_vld sc_out sc_logic 1 outvld 33 } 
	{ primal_u1_out sc_out sc_lv 26 signal 34 } 
	{ primal_u1_out_ap_vld sc_out sc_logic 1 outvld 34 } 
	{ lnorm_u0_out sc_out sc_lv 26 signal 35 } 
	{ lnorm_u0_out_ap_vld sc_out sc_logic 1 outvld 35 } 
	{ znorm_u0_out sc_out sc_lv 26 signal 36 } 
	{ znorm_u0_out_ap_vld sc_out sc_logic 1 outvld 36 } 
	{ dual_u0_out sc_out sc_lv 26 signal 37 } 
	{ dual_u0_out_ap_vld sc_out sc_logic 1 outvld 37 } 
	{ primal_u0_out sc_out sc_lv 26 signal 38 } 
	{ primal_u0_out_ap_vld sc_out sc_logic 1 outvld 38 } 
	{ lnorm_da_out sc_out sc_lv 26 signal 39 } 
	{ lnorm_da_out_ap_vld sc_out sc_logic 1 outvld 39 } 
	{ znorm_da_out sc_out sc_lv 25 signal 40 } 
	{ znorm_da_out_ap_vld sc_out sc_logic 1 outvld 40 } 
	{ dual_da_out sc_out sc_lv 26 signal 41 } 
	{ dual_da_out_ap_vld sc_out sc_logic 1 outvld 41 } 
	{ primal_da_out sc_out sc_lv 26 signal 42 } 
	{ primal_da_out_ap_vld sc_out sc_logic 1 outvld 42 } 
	{ lnorm_ey_out sc_out sc_lv 26 signal 43 } 
	{ lnorm_ey_out_ap_vld sc_out sc_logic 1 outvld 43 } 
	{ znorm_ey_out sc_out sc_lv 26 signal 44 } 
	{ znorm_ey_out_ap_vld sc_out sc_logic 1 outvld 44 } 
	{ dual_ey_out sc_out sc_lv 26 signal 45 } 
	{ dual_ey_out_ap_vld sc_out sc_logic 1 outvld 45 } 
	{ primal_ey_out sc_out sc_lv 26 signal 46 } 
	{ primal_ey_out_ap_vld sc_out sc_logic 1 outvld 46 } 
	{ u_norm_out sc_out sc_lv 26 signal 47 } 
	{ u_norm_out_ap_vld sc_out sc_logic 1 outvld 47 } 
	{ x_norm_out sc_out sc_lv 26 signal 48 } 
	{ x_norm_out_ap_vld sc_out sc_logic 1 outvld 48 } 
	{ grp_fu_5943_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_5943_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_5943_p_dout0 sc_in sc_lv 40 signal -1 } 
	{ grp_fu_5943_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_5947_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_5947_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_5947_p_dout0 sc_in sc_lv 40 signal -1 } 
	{ grp_fu_5947_p_ce sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "z_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u_1", "role": "address0" }} , 
 	{ "name": "z_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "ce0" }} , 
 	{ "name": "z_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "we0" }} , 
 	{ "name": "z_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_u_1", "role": "d0" }} , 
 	{ "name": "z_u_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u_1", "role": "address1" }} , 
 	{ "name": "z_u_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "ce1" }} , 
 	{ "name": "z_u_1_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_u_1", "role": "q1" }} , 
 	{ "name": "z_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u", "role": "address0" }} , 
 	{ "name": "z_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "ce0" }} , 
 	{ "name": "z_u_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "we0" }} , 
 	{ "name": "z_u_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_u", "role": "d0" }} , 
 	{ "name": "z_u_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u", "role": "address1" }} , 
 	{ "name": "z_u_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "ce1" }} , 
 	{ "name": "z_u_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_u", "role": "q1" }} , 
 	{ "name": "y_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u_1", "role": "address0" }} , 
 	{ "name": "y_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "ce0" }} , 
 	{ "name": "y_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "we0" }} , 
 	{ "name": "y_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_u_1", "role": "d0" }} , 
 	{ "name": "y_u_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u_1", "role": "address1" }} , 
 	{ "name": "y_u_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "ce1" }} , 
 	{ "name": "y_u_1_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_u_1", "role": "q1" }} , 
 	{ "name": "y_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u", "role": "address0" }} , 
 	{ "name": "y_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "ce0" }} , 
 	{ "name": "y_u_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "we0" }} , 
 	{ "name": "y_u_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_u", "role": "d0" }} , 
 	{ "name": "y_u_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u", "role": "address1" }} , 
 	{ "name": "y_u_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "ce1" }} , 
 	{ "name": "y_u_q1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_u", "role": "q1" }} , 
 	{ "name": "y_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_5", "role": "address0" }} , 
 	{ "name": "y_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "ce0" }} , 
 	{ "name": "y_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_5", "role": "q0" }} , 
 	{ "name": "y_x_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_5", "role": "address1" }} , 
 	{ "name": "y_x_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "ce1" }} , 
 	{ "name": "y_x_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "we1" }} , 
 	{ "name": "y_x_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_5", "role": "d1" }} , 
 	{ "name": "y_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x", "role": "address0" }} , 
 	{ "name": "y_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "ce0" }} , 
 	{ "name": "y_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x", "role": "q0" }} , 
 	{ "name": "y_x_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x", "role": "address1" }} , 
 	{ "name": "y_x_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "ce1" }} , 
 	{ "name": "y_x_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "we1" }} , 
 	{ "name": "y_x_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x", "role": "d1" }} , 
 	{ "name": "z_x_7_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_7", "role": "address1" }} , 
 	{ "name": "z_x_7_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_7", "role": "ce1" }} , 
 	{ "name": "z_x_7_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_7", "role": "we1" }} , 
 	{ "name": "z_x_7_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_7", "role": "d1" }} , 
 	{ "name": "z_x_6_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_6", "role": "address1" }} , 
 	{ "name": "z_x_6_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_6", "role": "ce1" }} , 
 	{ "name": "z_x_6_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_6", "role": "we1" }} , 
 	{ "name": "z_x_6_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_6", "role": "d1" }} , 
 	{ "name": "z_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_5", "role": "address0" }} , 
 	{ "name": "z_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_5", "role": "ce0" }} , 
 	{ "name": "z_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_5", "role": "q0" }} , 
 	{ "name": "z_x_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_5", "role": "address1" }} , 
 	{ "name": "z_x_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_5", "role": "ce1" }} , 
 	{ "name": "z_x_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_5", "role": "we1" }} , 
 	{ "name": "z_x_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_5", "role": "d1" }} , 
 	{ "name": "z_x_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_4", "role": "address1" }} , 
 	{ "name": "z_x_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_4", "role": "ce1" }} , 
 	{ "name": "z_x_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_4", "role": "we1" }} , 
 	{ "name": "z_x_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_4", "role": "d1" }} , 
 	{ "name": "z_x_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_3", "role": "address1" }} , 
 	{ "name": "z_x_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_3", "role": "ce1" }} , 
 	{ "name": "z_x_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_3", "role": "we1" }} , 
 	{ "name": "z_x_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_3", "role": "d1" }} , 
 	{ "name": "z_x_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_2", "role": "address1" }} , 
 	{ "name": "z_x_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_2", "role": "ce1" }} , 
 	{ "name": "z_x_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_2", "role": "we1" }} , 
 	{ "name": "z_x_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_2", "role": "d1" }} , 
 	{ "name": "z_x_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_1", "role": "address1" }} , 
 	{ "name": "z_x_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_1", "role": "ce1" }} , 
 	{ "name": "z_x_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_1", "role": "we1" }} , 
 	{ "name": "z_x_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x_1", "role": "d1" }} , 
 	{ "name": "z_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x", "role": "address0" }} , 
 	{ "name": "z_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "ce0" }} , 
 	{ "name": "z_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x", "role": "q0" }} , 
 	{ "name": "z_x_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x", "role": "address1" }} , 
 	{ "name": "z_x_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "ce1" }} , 
 	{ "name": "z_x_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "we1" }} , 
 	{ "name": "z_x_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x", "role": "d1" }} , 
 	{ "name": "sol_x_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_1", "role": "address0" }} , 
 	{ "name": "sol_x_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_1", "role": "ce0" }} , 
 	{ "name": "sol_x_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x_1", "role": "q0" }} , 
 	{ "name": "sol_x_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_2", "role": "address0" }} , 
 	{ "name": "sol_x_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_2", "role": "ce0" }} , 
 	{ "name": "sol_x_2_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x_2", "role": "q0" }} , 
 	{ "name": "sol_x_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_3", "role": "address0" }} , 
 	{ "name": "sol_x_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_3", "role": "ce0" }} , 
 	{ "name": "sol_x_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x_3", "role": "q0" }} , 
 	{ "name": "sol_x_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_4", "role": "address0" }} , 
 	{ "name": "sol_x_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_4", "role": "ce0" }} , 
 	{ "name": "sol_x_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x_4", "role": "q0" }} , 
 	{ "name": "sol_x_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_6", "role": "address0" }} , 
 	{ "name": "sol_x_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_6", "role": "ce0" }} , 
 	{ "name": "sol_x_6_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x_6", "role": "q0" }} , 
 	{ "name": "sol_x_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_7", "role": "address0" }} , 
 	{ "name": "sol_x_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_7", "role": "ce0" }} , 
 	{ "name": "sol_x_7_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x_7", "role": "q0" }} , 
 	{ "name": "sol_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x", "role": "address0" }} , 
 	{ "name": "sol_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x", "role": "ce0" }} , 
 	{ "name": "sol_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x", "role": "q0" }} , 
 	{ "name": "sol_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_5", "role": "address0" }} , 
 	{ "name": "sol_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_5", "role": "ce0" }} , 
 	{ "name": "sol_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_x_5", "role": "q0" }} , 
 	{ "name": "step_data_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_3", "role": "address0" }} , 
 	{ "name": "step_data_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_3", "role": "ce0" }} , 
 	{ "name": "step_data_3_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_3", "role": "q0" }} , 
 	{ "name": "step_data_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_4", "role": "address0" }} , 
 	{ "name": "step_data_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_4", "role": "ce0" }} , 
 	{ "name": "step_data_4_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_4", "role": "q0" }} , 
 	{ "name": "terminal_wall_x_lb_con_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "terminal_wall_x_lb_con_reload", "role": "default" }} , 
 	{ "name": "terminal_wall_x_ub_con_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "terminal_wall_x_ub_con_reload", "role": "default" }} , 
 	{ "name": "sext_ln156_35", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln156_35", "role": "default" }} , 
 	{ "name": "sol_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_u", "role": "address0" }} , 
 	{ "name": "sol_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_u", "role": "ce0" }} , 
 	{ "name": "sol_u_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_u", "role": "q0" }} , 
 	{ "name": "sol_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_u_1", "role": "address0" }} , 
 	{ "name": "sol_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_u_1", "role": "ce0" }} , 
 	{ "name": "sol_u_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sol_u_1", "role": "q0" }} , 
 	{ "name": "sext_ln1362", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln1362", "role": "default" }} , 
 	{ "name": "step_data_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "step_data_5", "role": "address0" }} , 
 	{ "name": "step_data_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "step_data_5", "role": "ce0" }} , 
 	{ "name": "step_data_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "step_data_5", "role": "q0" }} , 
 	{ "name": "lnorm_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lnorm_u1_out", "role": "default" }} , 
 	{ "name": "lnorm_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_u1_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "znorm_u1_out", "role": "default" }} , 
 	{ "name": "znorm_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_u1_out", "role": "ap_vld" }} , 
 	{ "name": "dual_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "dual_u1_out", "role": "default" }} , 
 	{ "name": "dual_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_u1_out", "role": "ap_vld" }} , 
 	{ "name": "primal_u1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "primal_u1_out", "role": "default" }} , 
 	{ "name": "primal_u1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_u1_out", "role": "ap_vld" }} , 
 	{ "name": "lnorm_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lnorm_u0_out", "role": "default" }} , 
 	{ "name": "lnorm_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_u0_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "znorm_u0_out", "role": "default" }} , 
 	{ "name": "znorm_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_u0_out", "role": "ap_vld" }} , 
 	{ "name": "dual_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "dual_u0_out", "role": "default" }} , 
 	{ "name": "dual_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_u0_out", "role": "ap_vld" }} , 
 	{ "name": "primal_u0_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "primal_u0_out", "role": "default" }} , 
 	{ "name": "primal_u0_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_u0_out", "role": "ap_vld" }} , 
 	{ "name": "lnorm_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lnorm_da_out", "role": "default" }} , 
 	{ "name": "lnorm_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_da_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "znorm_da_out", "role": "default" }} , 
 	{ "name": "znorm_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_da_out", "role": "ap_vld" }} , 
 	{ "name": "dual_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "dual_da_out", "role": "default" }} , 
 	{ "name": "dual_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_da_out", "role": "ap_vld" }} , 
 	{ "name": "primal_da_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "primal_da_out", "role": "default" }} , 
 	{ "name": "primal_da_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_da_out", "role": "ap_vld" }} , 
 	{ "name": "lnorm_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lnorm_ey_out", "role": "default" }} , 
 	{ "name": "lnorm_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "lnorm_ey_out", "role": "ap_vld" }} , 
 	{ "name": "znorm_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "znorm_ey_out", "role": "default" }} , 
 	{ "name": "znorm_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "znorm_ey_out", "role": "ap_vld" }} , 
 	{ "name": "dual_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "dual_ey_out", "role": "default" }} , 
 	{ "name": "dual_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "dual_ey_out", "role": "ap_vld" }} , 
 	{ "name": "primal_ey_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "primal_ey_out", "role": "default" }} , 
 	{ "name": "primal_ey_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "primal_ey_out", "role": "ap_vld" }} , 
 	{ "name": "u_norm_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "u_norm_out", "role": "default" }} , 
 	{ "name": "u_norm_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "u_norm_out", "role": "ap_vld" }} , 
 	{ "name": "x_norm_out", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "x_norm_out", "role": "default" }} , 
 	{ "name": "x_norm_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "x_norm_out", "role": "ap_vld" }} , 
 	{ "name": "grp_fu_5943_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_5943_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5943_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_5943_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5943_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5943_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5943_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5943_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_5947_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_5947_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_5947_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_5947_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_5947_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_5947_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_5947_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_5947_p_ce", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_compute_hls_Pipeline_VITIS_LOOP_1362_19 {
		z_u_1 {Type IO LastRead 1 FirstWrite 2}
		z_u {Type IO LastRead 1 FirstWrite 2}
		y_u_1 {Type IO LastRead 0 FirstWrite 2}
		y_u {Type IO LastRead 0 FirstWrite 2}
		y_x_5 {Type IO LastRead 0 FirstWrite 2}
		y_x {Type IO LastRead 0 FirstWrite 2}
		z_x_7 {Type O LastRead -1 FirstWrite 1}
		z_x_6 {Type O LastRead -1 FirstWrite 1}
		z_x_5 {Type IO LastRead 1 FirstWrite 2}
		z_x_4 {Type O LastRead -1 FirstWrite 1}
		z_x_3 {Type O LastRead -1 FirstWrite 1}
		z_x_2 {Type O LastRead -1 FirstWrite 1}
		z_x_1 {Type O LastRead -1 FirstWrite 1}
		z_x {Type IO LastRead 1 FirstWrite 2}
		sol_x_1 {Type I LastRead 0 FirstWrite -1}
		sol_x_2 {Type I LastRead 0 FirstWrite -1}
		sol_x_3 {Type I LastRead 0 FirstWrite -1}
		sol_x_4 {Type I LastRead 0 FirstWrite -1}
		sol_x_6 {Type I LastRead 0 FirstWrite -1}
		sol_x_7 {Type I LastRead 0 FirstWrite -1}
		sol_x {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		step_data_3 {Type I LastRead 0 FirstWrite -1}
		step_data_4 {Type I LastRead 0 FirstWrite -1}
		terminal_wall_x_lb_con_reload {Type I LastRead 0 FirstWrite -1}
		terminal_wall_x_ub_con_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln156_35 {Type I LastRead 0 FirstWrite -1}
		sol_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln1362 {Type I LastRead 0 FirstWrite -1}
		step_data_5 {Type I LastRead 0 FirstWrite -1}
		lnorm_u1_out {Type O LastRead -1 FirstWrite 4}
		znorm_u1_out {Type O LastRead -1 FirstWrite 4}
		dual_u1_out {Type O LastRead -1 FirstWrite 4}
		primal_u1_out {Type O LastRead -1 FirstWrite 4}
		lnorm_u0_out {Type O LastRead -1 FirstWrite 4}
		znorm_u0_out {Type O LastRead -1 FirstWrite 4}
		dual_u0_out {Type O LastRead -1 FirstWrite 4}
		primal_u0_out {Type O LastRead -1 FirstWrite 4}
		lnorm_da_out {Type O LastRead -1 FirstWrite 4}
		znorm_da_out {Type O LastRead -1 FirstWrite 4}
		dual_da_out {Type O LastRead -1 FirstWrite 4}
		primal_da_out {Type O LastRead -1 FirstWrite 4}
		lnorm_ey_out {Type O LastRead -1 FirstWrite 4}
		znorm_ey_out {Type O LastRead -1 FirstWrite 4}
		dual_ey_out {Type O LastRead -1 FirstWrite 4}
		primal_ey_out {Type O LastRead -1 FirstWrite 4}
		u_norm_out {Type O LastRead -1 FirstWrite 4}
		x_norm_out {Type O LastRead -1 FirstWrite 4}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "27", "Max" : "27"}
	, {"Name" : "Interval", "Min" : "22", "Max" : "22"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	z_u_1 { ap_memory {  { z_u_1_address0 mem_address 1 5 }  { z_u_1_ce0 mem_ce 1 1 }  { z_u_1_we0 mem_we 1 1 }  { z_u_1_d0 mem_din 1 26 }  { z_u_1_address1 MemPortADDR2 1 5 }  { z_u_1_ce1 MemPortCE2 1 1 }  { z_u_1_q1 MemPortDOUT2 0 26 } } }
	z_u { ap_memory {  { z_u_address0 mem_address 1 5 }  { z_u_ce0 mem_ce 1 1 }  { z_u_we0 mem_we 1 1 }  { z_u_d0 mem_din 1 26 }  { z_u_address1 MemPortADDR2 1 5 }  { z_u_ce1 MemPortCE2 1 1 }  { z_u_q1 MemPortDOUT2 0 26 } } }
	y_u_1 { ap_memory {  { y_u_1_address0 mem_address 1 5 }  { y_u_1_ce0 mem_ce 1 1 }  { y_u_1_we0 mem_we 1 1 }  { y_u_1_d0 mem_din 1 26 }  { y_u_1_address1 MemPortADDR2 1 5 }  { y_u_1_ce1 MemPortCE2 1 1 }  { y_u_1_q1 MemPortDOUT2 0 26 } } }
	y_u { ap_memory {  { y_u_address0 mem_address 1 5 }  { y_u_ce0 mem_ce 1 1 }  { y_u_we0 mem_we 1 1 }  { y_u_d0 mem_din 1 26 }  { y_u_address1 MemPortADDR2 1 5 }  { y_u_ce1 MemPortCE2 1 1 }  { y_u_q1 MemPortDOUT2 0 26 } } }
	y_x_5 { ap_memory {  { y_x_5_address0 mem_address 1 5 }  { y_x_5_ce0 mem_ce 1 1 }  { y_x_5_q0 mem_dout 0 26 }  { y_x_5_address1 MemPortADDR2 1 5 }  { y_x_5_ce1 MemPortCE2 1 1 }  { y_x_5_we1 MemPortWE2 1 1 }  { y_x_5_d1 MemPortDIN2 1 26 } } }
	y_x { ap_memory {  { y_x_address0 mem_address 1 5 }  { y_x_ce0 mem_ce 1 1 }  { y_x_q0 mem_dout 0 26 }  { y_x_address1 MemPortADDR2 1 5 }  { y_x_ce1 MemPortCE2 1 1 }  { y_x_we1 MemPortWE2 1 1 }  { y_x_d1 MemPortDIN2 1 26 } } }
	z_x_7 { ap_memory {  { z_x_7_address1 MemPortADDR2 1 5 }  { z_x_7_ce1 MemPortCE2 1 1 }  { z_x_7_we1 MemPortWE2 1 1 }  { z_x_7_d1 MemPortDIN2 1 26 } } }
	z_x_6 { ap_memory {  { z_x_6_address1 MemPortADDR2 1 5 }  { z_x_6_ce1 MemPortCE2 1 1 }  { z_x_6_we1 MemPortWE2 1 1 }  { z_x_6_d1 MemPortDIN2 1 26 } } }
	z_x_5 { ap_memory {  { z_x_5_address0 mem_address 1 5 }  { z_x_5_ce0 mem_ce 1 1 }  { z_x_5_q0 mem_dout 0 26 }  { z_x_5_address1 MemPortADDR2 1 5 }  { z_x_5_ce1 MemPortCE2 1 1 }  { z_x_5_we1 MemPortWE2 1 1 }  { z_x_5_d1 MemPortDIN2 1 26 } } }
	z_x_4 { ap_memory {  { z_x_4_address1 MemPortADDR2 1 5 }  { z_x_4_ce1 MemPortCE2 1 1 }  { z_x_4_we1 MemPortWE2 1 1 }  { z_x_4_d1 MemPortDIN2 1 26 } } }
	z_x_3 { ap_memory {  { z_x_3_address1 MemPortADDR2 1 5 }  { z_x_3_ce1 MemPortCE2 1 1 }  { z_x_3_we1 MemPortWE2 1 1 }  { z_x_3_d1 MemPortDIN2 1 26 } } }
	z_x_2 { ap_memory {  { z_x_2_address1 MemPortADDR2 1 5 }  { z_x_2_ce1 MemPortCE2 1 1 }  { z_x_2_we1 MemPortWE2 1 1 }  { z_x_2_d1 MemPortDIN2 1 26 } } }
	z_x_1 { ap_memory {  { z_x_1_address1 MemPortADDR2 1 5 }  { z_x_1_ce1 MemPortCE2 1 1 }  { z_x_1_we1 MemPortWE2 1 1 }  { z_x_1_d1 MemPortDIN2 1 26 } } }
	z_x { ap_memory {  { z_x_address0 mem_address 1 5 }  { z_x_ce0 mem_ce 1 1 }  { z_x_q0 mem_dout 0 26 }  { z_x_address1 MemPortADDR2 1 5 }  { z_x_ce1 MemPortCE2 1 1 }  { z_x_we1 MemPortWE2 1 1 }  { z_x_d1 MemPortDIN2 1 26 } } }
	sol_x_1 { ap_memory {  { sol_x_1_address0 mem_address 1 5 }  { sol_x_1_ce0 mem_ce 1 1 }  { sol_x_1_q0 mem_dout 0 26 } } }
	sol_x_2 { ap_memory {  { sol_x_2_address0 mem_address 1 5 }  { sol_x_2_ce0 mem_ce 1 1 }  { sol_x_2_q0 mem_dout 0 26 } } }
	sol_x_3 { ap_memory {  { sol_x_3_address0 mem_address 1 5 }  { sol_x_3_ce0 mem_ce 1 1 }  { sol_x_3_q0 mem_dout 0 26 } } }
	sol_x_4 { ap_memory {  { sol_x_4_address0 mem_address 1 5 }  { sol_x_4_ce0 mem_ce 1 1 }  { sol_x_4_q0 mem_dout 0 26 } } }
	sol_x_6 { ap_memory {  { sol_x_6_address0 mem_address 1 5 }  { sol_x_6_ce0 mem_ce 1 1 }  { sol_x_6_q0 mem_dout 0 26 } } }
	sol_x_7 { ap_memory {  { sol_x_7_address0 mem_address 1 5 }  { sol_x_7_ce0 mem_ce 1 1 }  { sol_x_7_q0 mem_dout 0 26 } } }
	sol_x { ap_memory {  { sol_x_address0 mem_address 1 5 }  { sol_x_ce0 mem_ce 1 1 }  { sol_x_q0 mem_dout 0 26 } } }
	sol_x_5 { ap_memory {  { sol_x_5_address0 mem_address 1 5 }  { sol_x_5_ce0 mem_ce 1 1 }  { sol_x_5_q0 mem_dout 0 26 } } }
	step_data_3 { ap_memory {  { step_data_3_address0 mem_address 1 5 }  { step_data_3_ce0 mem_ce 1 1 }  { step_data_3_q0 mem_dout 0 26 } } }
	step_data_4 { ap_memory {  { step_data_4_address0 mem_address 1 5 }  { step_data_4_ce0 mem_ce 1 1 }  { step_data_4_q0 mem_dout 0 26 } } }
	terminal_wall_x_lb_con_reload { ap_none {  { terminal_wall_x_lb_con_reload in_data 0 26 } } }
	terminal_wall_x_ub_con_reload { ap_none {  { terminal_wall_x_ub_con_reload in_data 0 26 } } }
	sext_ln156_35 { ap_none {  { sext_ln156_35 in_data 0 26 } } }
	sol_u { ap_memory {  { sol_u_address0 mem_address 1 5 }  { sol_u_ce0 mem_ce 1 1 }  { sol_u_q0 mem_dout 0 26 } } }
	sol_u_1 { ap_memory {  { sol_u_1_address0 mem_address 1 5 }  { sol_u_1_ce0 mem_ce 1 1 }  { sol_u_1_q0 mem_dout 0 26 } } }
	sext_ln1362 { ap_none {  { sext_ln1362 in_data 0 26 } } }
	step_data_5 { ap_memory {  { step_data_5_address0 mem_address 1 5 }  { step_data_5_ce0 mem_ce 1 1 }  { step_data_5_q0 mem_dout 0 26 } } }
	lnorm_u1_out { ap_vld {  { lnorm_u1_out out_data 1 26 }  { lnorm_u1_out_ap_vld out_vld 1 1 } } }
	znorm_u1_out { ap_vld {  { znorm_u1_out out_data 1 26 }  { znorm_u1_out_ap_vld out_vld 1 1 } } }
	dual_u1_out { ap_vld {  { dual_u1_out out_data 1 26 }  { dual_u1_out_ap_vld out_vld 1 1 } } }
	primal_u1_out { ap_vld {  { primal_u1_out out_data 1 26 }  { primal_u1_out_ap_vld out_vld 1 1 } } }
	lnorm_u0_out { ap_vld {  { lnorm_u0_out out_data 1 26 }  { lnorm_u0_out_ap_vld out_vld 1 1 } } }
	znorm_u0_out { ap_vld {  { znorm_u0_out out_data 1 26 }  { znorm_u0_out_ap_vld out_vld 1 1 } } }
	dual_u0_out { ap_vld {  { dual_u0_out out_data 1 26 }  { dual_u0_out_ap_vld out_vld 1 1 } } }
	primal_u0_out { ap_vld {  { primal_u0_out out_data 1 26 }  { primal_u0_out_ap_vld out_vld 1 1 } } }
	lnorm_da_out { ap_vld {  { lnorm_da_out out_data 1 26 }  { lnorm_da_out_ap_vld out_vld 1 1 } } }
	znorm_da_out { ap_vld {  { znorm_da_out out_data 1 25 }  { znorm_da_out_ap_vld out_vld 1 1 } } }
	dual_da_out { ap_vld {  { dual_da_out out_data 1 26 }  { dual_da_out_ap_vld out_vld 1 1 } } }
	primal_da_out { ap_vld {  { primal_da_out out_data 1 26 }  { primal_da_out_ap_vld out_vld 1 1 } } }
	lnorm_ey_out { ap_vld {  { lnorm_ey_out out_data 1 26 }  { lnorm_ey_out_ap_vld out_vld 1 1 } } }
	znorm_ey_out { ap_vld {  { znorm_ey_out out_data 1 26 }  { znorm_ey_out_ap_vld out_vld 1 1 } } }
	dual_ey_out { ap_vld {  { dual_ey_out out_data 1 26 }  { dual_ey_out_ap_vld out_vld 1 1 } } }
	primal_ey_out { ap_vld {  { primal_ey_out out_data 1 26 }  { primal_ey_out_ap_vld out_vld 1 1 } } }
	u_norm_out { ap_vld {  { u_norm_out out_data 1 26 }  { u_norm_out_ap_vld out_vld 1 1 } } }
	x_norm_out { ap_vld {  { x_norm_out out_data 1 26 }  { x_norm_out_ap_vld out_vld 1 1 } } }
}
