set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1
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
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict y_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_7 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_6 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_5 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_4 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_3 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_2 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x_1 { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_x { MEM_WIDTH 26 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ y_x_7 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x_6 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x_5 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x_4 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x_3 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x_2 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x_1 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_7 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_6 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_5 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_4 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_3 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_2 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x_1 int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ z_x int 26 regular {array 21 { 3 0 } 0 1 bus  }  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "y_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_7", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_6", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_2", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_x", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 70
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ y_x_7_address1 sc_out sc_lv 5 signal 0 } 
	{ y_x_7_ce1 sc_out sc_logic 1 signal 0 } 
	{ y_x_7_we1 sc_out sc_logic 1 signal 0 } 
	{ y_x_7_d1 sc_out sc_lv 26 signal 0 } 
	{ y_x_6_address1 sc_out sc_lv 5 signal 1 } 
	{ y_x_6_ce1 sc_out sc_logic 1 signal 1 } 
	{ y_x_6_we1 sc_out sc_logic 1 signal 1 } 
	{ y_x_6_d1 sc_out sc_lv 26 signal 1 } 
	{ y_x_5_address1 sc_out sc_lv 5 signal 2 } 
	{ y_x_5_ce1 sc_out sc_logic 1 signal 2 } 
	{ y_x_5_we1 sc_out sc_logic 1 signal 2 } 
	{ y_x_5_d1 sc_out sc_lv 26 signal 2 } 
	{ y_x_4_address1 sc_out sc_lv 5 signal 3 } 
	{ y_x_4_ce1 sc_out sc_logic 1 signal 3 } 
	{ y_x_4_we1 sc_out sc_logic 1 signal 3 } 
	{ y_x_4_d1 sc_out sc_lv 26 signal 3 } 
	{ y_x_3_address1 sc_out sc_lv 5 signal 4 } 
	{ y_x_3_ce1 sc_out sc_logic 1 signal 4 } 
	{ y_x_3_we1 sc_out sc_logic 1 signal 4 } 
	{ y_x_3_d1 sc_out sc_lv 26 signal 4 } 
	{ y_x_2_address1 sc_out sc_lv 5 signal 5 } 
	{ y_x_2_ce1 sc_out sc_logic 1 signal 5 } 
	{ y_x_2_we1 sc_out sc_logic 1 signal 5 } 
	{ y_x_2_d1 sc_out sc_lv 26 signal 5 } 
	{ y_x_1_address1 sc_out sc_lv 5 signal 6 } 
	{ y_x_1_ce1 sc_out sc_logic 1 signal 6 } 
	{ y_x_1_we1 sc_out sc_logic 1 signal 6 } 
	{ y_x_1_d1 sc_out sc_lv 26 signal 6 } 
	{ y_x_address1 sc_out sc_lv 5 signal 7 } 
	{ y_x_ce1 sc_out sc_logic 1 signal 7 } 
	{ y_x_we1 sc_out sc_logic 1 signal 7 } 
	{ y_x_d1 sc_out sc_lv 26 signal 7 } 
	{ z_x_7_address1 sc_out sc_lv 5 signal 8 } 
	{ z_x_7_ce1 sc_out sc_logic 1 signal 8 } 
	{ z_x_7_we1 sc_out sc_logic 1 signal 8 } 
	{ z_x_7_d1 sc_out sc_lv 26 signal 8 } 
	{ z_x_6_address1 sc_out sc_lv 5 signal 9 } 
	{ z_x_6_ce1 sc_out sc_logic 1 signal 9 } 
	{ z_x_6_we1 sc_out sc_logic 1 signal 9 } 
	{ z_x_6_d1 sc_out sc_lv 26 signal 9 } 
	{ z_x_5_address1 sc_out sc_lv 5 signal 10 } 
	{ z_x_5_ce1 sc_out sc_logic 1 signal 10 } 
	{ z_x_5_we1 sc_out sc_logic 1 signal 10 } 
	{ z_x_5_d1 sc_out sc_lv 26 signal 10 } 
	{ z_x_4_address1 sc_out sc_lv 5 signal 11 } 
	{ z_x_4_ce1 sc_out sc_logic 1 signal 11 } 
	{ z_x_4_we1 sc_out sc_logic 1 signal 11 } 
	{ z_x_4_d1 sc_out sc_lv 26 signal 11 } 
	{ z_x_3_address1 sc_out sc_lv 5 signal 12 } 
	{ z_x_3_ce1 sc_out sc_logic 1 signal 12 } 
	{ z_x_3_we1 sc_out sc_logic 1 signal 12 } 
	{ z_x_3_d1 sc_out sc_lv 26 signal 12 } 
	{ z_x_2_address1 sc_out sc_lv 5 signal 13 } 
	{ z_x_2_ce1 sc_out sc_logic 1 signal 13 } 
	{ z_x_2_we1 sc_out sc_logic 1 signal 13 } 
	{ z_x_2_d1 sc_out sc_lv 26 signal 13 } 
	{ z_x_1_address1 sc_out sc_lv 5 signal 14 } 
	{ z_x_1_ce1 sc_out sc_logic 1 signal 14 } 
	{ z_x_1_we1 sc_out sc_logic 1 signal 14 } 
	{ z_x_1_d1 sc_out sc_lv 26 signal 14 } 
	{ z_x_address1 sc_out sc_lv 5 signal 15 } 
	{ z_x_ce1 sc_out sc_logic 1 signal 15 } 
	{ z_x_we1 sc_out sc_logic 1 signal 15 } 
	{ z_x_d1 sc_out sc_lv 26 signal 15 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "y_x_7_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_7", "role": "address1" }} , 
 	{ "name": "y_x_7_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_7", "role": "ce1" }} , 
 	{ "name": "y_x_7_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_7", "role": "we1" }} , 
 	{ "name": "y_x_7_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_7", "role": "d1" }} , 
 	{ "name": "y_x_6_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_6", "role": "address1" }} , 
 	{ "name": "y_x_6_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_6", "role": "ce1" }} , 
 	{ "name": "y_x_6_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_6", "role": "we1" }} , 
 	{ "name": "y_x_6_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_6", "role": "d1" }} , 
 	{ "name": "y_x_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_5", "role": "address1" }} , 
 	{ "name": "y_x_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "ce1" }} , 
 	{ "name": "y_x_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "we1" }} , 
 	{ "name": "y_x_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_5", "role": "d1" }} , 
 	{ "name": "y_x_4_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_4", "role": "address1" }} , 
 	{ "name": "y_x_4_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_4", "role": "ce1" }} , 
 	{ "name": "y_x_4_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_4", "role": "we1" }} , 
 	{ "name": "y_x_4_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_4", "role": "d1" }} , 
 	{ "name": "y_x_3_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_3", "role": "address1" }} , 
 	{ "name": "y_x_3_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_3", "role": "ce1" }} , 
 	{ "name": "y_x_3_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_3", "role": "we1" }} , 
 	{ "name": "y_x_3_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_3", "role": "d1" }} , 
 	{ "name": "y_x_2_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_2", "role": "address1" }} , 
 	{ "name": "y_x_2_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_2", "role": "ce1" }} , 
 	{ "name": "y_x_2_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_2", "role": "we1" }} , 
 	{ "name": "y_x_2_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_2", "role": "d1" }} , 
 	{ "name": "y_x_1_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_1", "role": "address1" }} , 
 	{ "name": "y_x_1_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_1", "role": "ce1" }} , 
 	{ "name": "y_x_1_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_1", "role": "we1" }} , 
 	{ "name": "y_x_1_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "y_x_1", "role": "d1" }} , 
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
 	{ "name": "z_x_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x", "role": "address1" }} , 
 	{ "name": "z_x_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "ce1" }} , 
 	{ "name": "z_x_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "we1" }} , 
 	{ "name": "z_x_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "z_x", "role": "d1" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_compute_hls_Pipeline_VITIS_LOOP_1202_1 {
		y_x_7 {Type O LastRead -1 FirstWrite 0}
		y_x_6 {Type O LastRead -1 FirstWrite 0}
		y_x_5 {Type O LastRead -1 FirstWrite 0}
		y_x_4 {Type O LastRead -1 FirstWrite 0}
		y_x_3 {Type O LastRead -1 FirstWrite 0}
		y_x_2 {Type O LastRead -1 FirstWrite 0}
		y_x_1 {Type O LastRead -1 FirstWrite 0}
		y_x {Type O LastRead -1 FirstWrite 0}
		z_x_7 {Type O LastRead -1 FirstWrite 0}
		z_x_6 {Type O LastRead -1 FirstWrite 0}
		z_x_5 {Type O LastRead -1 FirstWrite 0}
		z_x_4 {Type O LastRead -1 FirstWrite 0}
		z_x_3 {Type O LastRead -1 FirstWrite 0}
		z_x_2 {Type O LastRead -1 FirstWrite 0}
		z_x_1 {Type O LastRead -1 FirstWrite 0}
		z_x {Type O LastRead -1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "22", "Max" : "22"}
	, {"Name" : "Interval", "Min" : "21", "Max" : "21"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	y_x_7 { ap_memory {  { y_x_7_address1 MemPortADDR2 1 5 }  { y_x_7_ce1 MemPortCE2 1 1 }  { y_x_7_we1 MemPortWE2 1 1 }  { y_x_7_d1 MemPortDIN2 1 26 } } }
	y_x_6 { ap_memory {  { y_x_6_address1 MemPortADDR2 1 5 }  { y_x_6_ce1 MemPortCE2 1 1 }  { y_x_6_we1 MemPortWE2 1 1 }  { y_x_6_d1 MemPortDIN2 1 26 } } }
	y_x_5 { ap_memory {  { y_x_5_address1 MemPortADDR2 1 5 }  { y_x_5_ce1 MemPortCE2 1 1 }  { y_x_5_we1 MemPortWE2 1 1 }  { y_x_5_d1 MemPortDIN2 1 26 } } }
	y_x_4 { ap_memory {  { y_x_4_address1 MemPortADDR2 1 5 }  { y_x_4_ce1 MemPortCE2 1 1 }  { y_x_4_we1 MemPortWE2 1 1 }  { y_x_4_d1 MemPortDIN2 1 26 } } }
	y_x_3 { ap_memory {  { y_x_3_address1 MemPortADDR2 1 5 }  { y_x_3_ce1 MemPortCE2 1 1 }  { y_x_3_we1 MemPortWE2 1 1 }  { y_x_3_d1 MemPortDIN2 1 26 } } }
	y_x_2 { ap_memory {  { y_x_2_address1 MemPortADDR2 1 5 }  { y_x_2_ce1 MemPortCE2 1 1 }  { y_x_2_we1 MemPortWE2 1 1 }  { y_x_2_d1 MemPortDIN2 1 26 } } }
	y_x_1 { ap_memory {  { y_x_1_address1 MemPortADDR2 1 5 }  { y_x_1_ce1 MemPortCE2 1 1 }  { y_x_1_we1 MemPortWE2 1 1 }  { y_x_1_d1 MemPortDIN2 1 26 } } }
	y_x { ap_memory {  { y_x_address1 MemPortADDR2 1 5 }  { y_x_ce1 MemPortCE2 1 1 }  { y_x_we1 MemPortWE2 1 1 }  { y_x_d1 MemPortDIN2 1 26 } } }
	z_x_7 { ap_memory {  { z_x_7_address1 MemPortADDR2 1 5 }  { z_x_7_ce1 MemPortCE2 1 1 }  { z_x_7_we1 MemPortWE2 1 1 }  { z_x_7_d1 MemPortDIN2 1 26 } } }
	z_x_6 { ap_memory {  { z_x_6_address1 MemPortADDR2 1 5 }  { z_x_6_ce1 MemPortCE2 1 1 }  { z_x_6_we1 MemPortWE2 1 1 }  { z_x_6_d1 MemPortDIN2 1 26 } } }
	z_x_5 { ap_memory {  { z_x_5_address1 MemPortADDR2 1 5 }  { z_x_5_ce1 MemPortCE2 1 1 }  { z_x_5_we1 MemPortWE2 1 1 }  { z_x_5_d1 MemPortDIN2 1 26 } } }
	z_x_4 { ap_memory {  { z_x_4_address1 MemPortADDR2 1 5 }  { z_x_4_ce1 MemPortCE2 1 1 }  { z_x_4_we1 MemPortWE2 1 1 }  { z_x_4_d1 MemPortDIN2 1 26 } } }
	z_x_3 { ap_memory {  { z_x_3_address1 MemPortADDR2 1 5 }  { z_x_3_ce1 MemPortCE2 1 1 }  { z_x_3_we1 MemPortWE2 1 1 }  { z_x_3_d1 MemPortDIN2 1 26 } } }
	z_x_2 { ap_memory {  { z_x_2_address1 MemPortADDR2 1 5 }  { z_x_2_ce1 MemPortCE2 1 1 }  { z_x_2_we1 MemPortWE2 1 1 }  { z_x_2_d1 MemPortDIN2 1 26 } } }
	z_x_1 { ap_memory {  { z_x_1_address1 MemPortADDR2 1 5 }  { z_x_1_ce1 MemPortCE2 1 1 }  { z_x_1_we1 MemPortWE2 1 1 }  { z_x_1_d1 MemPortDIN2 1 26 } } }
	z_x { ap_memory {  { z_x_address1 MemPortADDR2 1 5 }  { z_x_ce1 MemPortCE2 1 1 }  { z_x_we1 MemPortWE2 1 1 }  { z_x_d1 MemPortDIN2 1 26 } } }
}
