set moduleName p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1
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
set C_modelName {(anonymous namespace)unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict lane_words_15 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_14 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_13 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_12 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_11 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_10 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_9 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_8 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_7 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_6 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_5 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_4 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_3 { MEM_WIDTH 26 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_2 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_1 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict lane_words_0 { MEM_WIDTH 32 MEM_SIZE 44 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ gmem0 int 512 regular {axi_master 0}  }
	{ sext_ln213 int 58 regular  }
	{ lane_words_15 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_14 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_13 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_12 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_11 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_10 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_9 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_8 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_7 int 32 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_6 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_5 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_4 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_3 int 26 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_2 int 32 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_1 int 32 regular {array 11 { 0 3 } 0 1 bus  }  }
	{ lane_words_0 int 32 regular {array 11 { 0 3 } 0 1 bus  }  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "gmem0", "interface" : "axi_master", "bitwidth" : 512, "direction" : "READONLY", "id_num" : 0, "bitSlice":[ {"cElement": [{"cName": "input_words512","offset": { "type": "dynamic","port_name": "input_words512","bundle": "control"},"direction": "READONLY"}]}]} , 
 	{ "Name" : "sext_ln213", "interface" : "wire", "bitwidth" : 58, "direction" : "READONLY"} , 
 	{ "Name" : "lane_words_15", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_14", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_13", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_12", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_11", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_10", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_9", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_8", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_7", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_6", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_5", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_4", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_3", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_2", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_1", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "lane_words_0", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 117
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ m_axi_gmem0_0_AWVALID sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_AWREADY sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_AWADDR sc_out sc_lv 64 signal 0 } 
	{ m_axi_gmem0_0_AWID sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_AWLEN sc_out sc_lv 32 signal 0 } 
	{ m_axi_gmem0_0_AWSIZE sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_0_AWBURST sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_0_AWLOCK sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_0_AWCACHE sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_0_AWPROT sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_0_AWQOS sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_0_AWREGION sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_0_AWUSER sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_WVALID sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_WREADY sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_WDATA sc_out sc_lv 512 signal 0 } 
	{ m_axi_gmem0_0_WSTRB sc_out sc_lv 64 signal 0 } 
	{ m_axi_gmem0_0_WLAST sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_WID sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_WUSER sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_ARVALID sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_ARREADY sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_ARADDR sc_out sc_lv 64 signal 0 } 
	{ m_axi_gmem0_0_ARID sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_ARLEN sc_out sc_lv 32 signal 0 } 
	{ m_axi_gmem0_0_ARSIZE sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_0_ARBURST sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_0_ARLOCK sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_0_ARCACHE sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_0_ARPROT sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_0_ARQOS sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_0_ARREGION sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_0_ARUSER sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_RVALID sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_RREADY sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_RDATA sc_in sc_lv 512 signal 0 } 
	{ m_axi_gmem0_0_RLAST sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_RID sc_in sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_RFIFONUM sc_in sc_lv 9 signal 0 } 
	{ m_axi_gmem0_0_RUSER sc_in sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_RRESP sc_in sc_lv 2 signal 0 } 
	{ m_axi_gmem0_0_BVALID sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_BREADY sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_0_BRESP sc_in sc_lv 2 signal 0 } 
	{ m_axi_gmem0_0_BID sc_in sc_lv 1 signal 0 } 
	{ m_axi_gmem0_0_BUSER sc_in sc_lv 1 signal 0 } 
	{ sext_ln213 sc_in sc_lv 58 signal 1 } 
	{ lane_words_15_address0 sc_out sc_lv 4 signal 2 } 
	{ lane_words_15_ce0 sc_out sc_logic 1 signal 2 } 
	{ lane_words_15_we0 sc_out sc_logic 1 signal 2 } 
	{ lane_words_15_d0 sc_out sc_lv 26 signal 2 } 
	{ lane_words_14_address0 sc_out sc_lv 4 signal 3 } 
	{ lane_words_14_ce0 sc_out sc_logic 1 signal 3 } 
	{ lane_words_14_we0 sc_out sc_logic 1 signal 3 } 
	{ lane_words_14_d0 sc_out sc_lv 26 signal 3 } 
	{ lane_words_13_address0 sc_out sc_lv 4 signal 4 } 
	{ lane_words_13_ce0 sc_out sc_logic 1 signal 4 } 
	{ lane_words_13_we0 sc_out sc_logic 1 signal 4 } 
	{ lane_words_13_d0 sc_out sc_lv 26 signal 4 } 
	{ lane_words_12_address0 sc_out sc_lv 4 signal 5 } 
	{ lane_words_12_ce0 sc_out sc_logic 1 signal 5 } 
	{ lane_words_12_we0 sc_out sc_logic 1 signal 5 } 
	{ lane_words_12_d0 sc_out sc_lv 26 signal 5 } 
	{ lane_words_11_address0 sc_out sc_lv 4 signal 6 } 
	{ lane_words_11_ce0 sc_out sc_logic 1 signal 6 } 
	{ lane_words_11_we0 sc_out sc_logic 1 signal 6 } 
	{ lane_words_11_d0 sc_out sc_lv 26 signal 6 } 
	{ lane_words_10_address0 sc_out sc_lv 4 signal 7 } 
	{ lane_words_10_ce0 sc_out sc_logic 1 signal 7 } 
	{ lane_words_10_we0 sc_out sc_logic 1 signal 7 } 
	{ lane_words_10_d0 sc_out sc_lv 26 signal 7 } 
	{ lane_words_9_address0 sc_out sc_lv 4 signal 8 } 
	{ lane_words_9_ce0 sc_out sc_logic 1 signal 8 } 
	{ lane_words_9_we0 sc_out sc_logic 1 signal 8 } 
	{ lane_words_9_d0 sc_out sc_lv 26 signal 8 } 
	{ lane_words_8_address0 sc_out sc_lv 4 signal 9 } 
	{ lane_words_8_ce0 sc_out sc_logic 1 signal 9 } 
	{ lane_words_8_we0 sc_out sc_logic 1 signal 9 } 
	{ lane_words_8_d0 sc_out sc_lv 26 signal 9 } 
	{ lane_words_7_address0 sc_out sc_lv 4 signal 10 } 
	{ lane_words_7_ce0 sc_out sc_logic 1 signal 10 } 
	{ lane_words_7_we0 sc_out sc_logic 1 signal 10 } 
	{ lane_words_7_d0 sc_out sc_lv 32 signal 10 } 
	{ lane_words_6_address0 sc_out sc_lv 4 signal 11 } 
	{ lane_words_6_ce0 sc_out sc_logic 1 signal 11 } 
	{ lane_words_6_we0 sc_out sc_logic 1 signal 11 } 
	{ lane_words_6_d0 sc_out sc_lv 26 signal 11 } 
	{ lane_words_5_address0 sc_out sc_lv 4 signal 12 } 
	{ lane_words_5_ce0 sc_out sc_logic 1 signal 12 } 
	{ lane_words_5_we0 sc_out sc_logic 1 signal 12 } 
	{ lane_words_5_d0 sc_out sc_lv 26 signal 12 } 
	{ lane_words_4_address0 sc_out sc_lv 4 signal 13 } 
	{ lane_words_4_ce0 sc_out sc_logic 1 signal 13 } 
	{ lane_words_4_we0 sc_out sc_logic 1 signal 13 } 
	{ lane_words_4_d0 sc_out sc_lv 26 signal 13 } 
	{ lane_words_3_address0 sc_out sc_lv 4 signal 14 } 
	{ lane_words_3_ce0 sc_out sc_logic 1 signal 14 } 
	{ lane_words_3_we0 sc_out sc_logic 1 signal 14 } 
	{ lane_words_3_d0 sc_out sc_lv 26 signal 14 } 
	{ lane_words_2_address0 sc_out sc_lv 4 signal 15 } 
	{ lane_words_2_ce0 sc_out sc_logic 1 signal 15 } 
	{ lane_words_2_we0 sc_out sc_logic 1 signal 15 } 
	{ lane_words_2_d0 sc_out sc_lv 32 signal 15 } 
	{ lane_words_1_address0 sc_out sc_lv 4 signal 16 } 
	{ lane_words_1_ce0 sc_out sc_logic 1 signal 16 } 
	{ lane_words_1_we0 sc_out sc_logic 1 signal 16 } 
	{ lane_words_1_d0 sc_out sc_lv 32 signal 16 } 
	{ lane_words_0_address0 sc_out sc_lv 4 signal 17 } 
	{ lane_words_0_ce0 sc_out sc_logic 1 signal 17 } 
	{ lane_words_0_we0 sc_out sc_logic 1 signal 17 } 
	{ lane_words_0_d0 sc_out sc_lv 32 signal 17 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "m_axi_gmem0_0_AWVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWVALID" }} , 
 	{ "name": "m_axi_gmem0_0_AWREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWREADY" }} , 
 	{ "name": "m_axi_gmem0_0_AWADDR", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWADDR" }} , 
 	{ "name": "m_axi_gmem0_0_AWID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWID" }} , 
 	{ "name": "m_axi_gmem0_0_AWLEN", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWLEN" }} , 
 	{ "name": "m_axi_gmem0_0_AWSIZE", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWSIZE" }} , 
 	{ "name": "m_axi_gmem0_0_AWBURST", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWBURST" }} , 
 	{ "name": "m_axi_gmem0_0_AWLOCK", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWLOCK" }} , 
 	{ "name": "m_axi_gmem0_0_AWCACHE", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWCACHE" }} , 
 	{ "name": "m_axi_gmem0_0_AWPROT", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWPROT" }} , 
 	{ "name": "m_axi_gmem0_0_AWQOS", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWQOS" }} , 
 	{ "name": "m_axi_gmem0_0_AWREGION", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWREGION" }} , 
 	{ "name": "m_axi_gmem0_0_AWUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_AWUSER" }} , 
 	{ "name": "m_axi_gmem0_0_WVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_WVALID" }} , 
 	{ "name": "m_axi_gmem0_0_WREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_WREADY" }} , 
 	{ "name": "m_axi_gmem0_0_WDATA", "direction": "out", "datatype": "sc_lv", "bitwidth":512, "type": "signal", "bundle":{"name": "gmem0", "role": "0_WDATA" }} , 
 	{ "name": "m_axi_gmem0_0_WSTRB", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem0", "role": "0_WSTRB" }} , 
 	{ "name": "m_axi_gmem0_0_WLAST", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_WLAST" }} , 
 	{ "name": "m_axi_gmem0_0_WID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_WID" }} , 
 	{ "name": "m_axi_gmem0_0_WUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_WUSER" }} , 
 	{ "name": "m_axi_gmem0_0_ARVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARVALID" }} , 
 	{ "name": "m_axi_gmem0_0_ARREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARREADY" }} , 
 	{ "name": "m_axi_gmem0_0_ARADDR", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARADDR" }} , 
 	{ "name": "m_axi_gmem0_0_ARID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARID" }} , 
 	{ "name": "m_axi_gmem0_0_ARLEN", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARLEN" }} , 
 	{ "name": "m_axi_gmem0_0_ARSIZE", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARSIZE" }} , 
 	{ "name": "m_axi_gmem0_0_ARBURST", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARBURST" }} , 
 	{ "name": "m_axi_gmem0_0_ARLOCK", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARLOCK" }} , 
 	{ "name": "m_axi_gmem0_0_ARCACHE", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARCACHE" }} , 
 	{ "name": "m_axi_gmem0_0_ARPROT", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARPROT" }} , 
 	{ "name": "m_axi_gmem0_0_ARQOS", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARQOS" }} , 
 	{ "name": "m_axi_gmem0_0_ARREGION", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARREGION" }} , 
 	{ "name": "m_axi_gmem0_0_ARUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_ARUSER" }} , 
 	{ "name": "m_axi_gmem0_0_RVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RVALID" }} , 
 	{ "name": "m_axi_gmem0_0_RREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RREADY" }} , 
 	{ "name": "m_axi_gmem0_0_RDATA", "direction": "in", "datatype": "sc_lv", "bitwidth":512, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RDATA" }} , 
 	{ "name": "m_axi_gmem0_0_RLAST", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RLAST" }} , 
 	{ "name": "m_axi_gmem0_0_RID", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RID" }} , 
 	{ "name": "m_axi_gmem0_0_RFIFONUM", "direction": "in", "datatype": "sc_lv", "bitwidth":9, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RFIFONUM" }} , 
 	{ "name": "m_axi_gmem0_0_RUSER", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RUSER" }} , 
 	{ "name": "m_axi_gmem0_0_RRESP", "direction": "in", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "0_RRESP" }} , 
 	{ "name": "m_axi_gmem0_0_BVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_BVALID" }} , 
 	{ "name": "m_axi_gmem0_0_BREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_BREADY" }} , 
 	{ "name": "m_axi_gmem0_0_BRESP", "direction": "in", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "0_BRESP" }} , 
 	{ "name": "m_axi_gmem0_0_BID", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_BID" }} , 
 	{ "name": "m_axi_gmem0_0_BUSER", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "0_BUSER" }} , 
 	{ "name": "sext_ln213", "direction": "in", "datatype": "sc_lv", "bitwidth":58, "type": "signal", "bundle":{"name": "sext_ln213", "role": "default" }} , 
 	{ "name": "lane_words_15_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_15", "role": "address0" }} , 
 	{ "name": "lane_words_15_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_15", "role": "ce0" }} , 
 	{ "name": "lane_words_15_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_15", "role": "we0" }} , 
 	{ "name": "lane_words_15_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_15", "role": "d0" }} , 
 	{ "name": "lane_words_14_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_14", "role": "address0" }} , 
 	{ "name": "lane_words_14_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_14", "role": "ce0" }} , 
 	{ "name": "lane_words_14_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_14", "role": "we0" }} , 
 	{ "name": "lane_words_14_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_14", "role": "d0" }} , 
 	{ "name": "lane_words_13_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_13", "role": "address0" }} , 
 	{ "name": "lane_words_13_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_13", "role": "ce0" }} , 
 	{ "name": "lane_words_13_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_13", "role": "we0" }} , 
 	{ "name": "lane_words_13_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_13", "role": "d0" }} , 
 	{ "name": "lane_words_12_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_12", "role": "address0" }} , 
 	{ "name": "lane_words_12_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_12", "role": "ce0" }} , 
 	{ "name": "lane_words_12_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_12", "role": "we0" }} , 
 	{ "name": "lane_words_12_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_12", "role": "d0" }} , 
 	{ "name": "lane_words_11_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_11", "role": "address0" }} , 
 	{ "name": "lane_words_11_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_11", "role": "ce0" }} , 
 	{ "name": "lane_words_11_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_11", "role": "we0" }} , 
 	{ "name": "lane_words_11_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_11", "role": "d0" }} , 
 	{ "name": "lane_words_10_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_10", "role": "address0" }} , 
 	{ "name": "lane_words_10_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_10", "role": "ce0" }} , 
 	{ "name": "lane_words_10_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_10", "role": "we0" }} , 
 	{ "name": "lane_words_10_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_10", "role": "d0" }} , 
 	{ "name": "lane_words_9_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_9", "role": "address0" }} , 
 	{ "name": "lane_words_9_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_9", "role": "ce0" }} , 
 	{ "name": "lane_words_9_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_9", "role": "we0" }} , 
 	{ "name": "lane_words_9_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_9", "role": "d0" }} , 
 	{ "name": "lane_words_8_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_8", "role": "address0" }} , 
 	{ "name": "lane_words_8_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_8", "role": "ce0" }} , 
 	{ "name": "lane_words_8_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_8", "role": "we0" }} , 
 	{ "name": "lane_words_8_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_8", "role": "d0" }} , 
 	{ "name": "lane_words_7_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_7", "role": "address0" }} , 
 	{ "name": "lane_words_7_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_7", "role": "ce0" }} , 
 	{ "name": "lane_words_7_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_7", "role": "we0" }} , 
 	{ "name": "lane_words_7_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_7", "role": "d0" }} , 
 	{ "name": "lane_words_6_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_6", "role": "address0" }} , 
 	{ "name": "lane_words_6_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_6", "role": "ce0" }} , 
 	{ "name": "lane_words_6_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_6", "role": "we0" }} , 
 	{ "name": "lane_words_6_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_6", "role": "d0" }} , 
 	{ "name": "lane_words_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_5", "role": "address0" }} , 
 	{ "name": "lane_words_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_5", "role": "ce0" }} , 
 	{ "name": "lane_words_5_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_5", "role": "we0" }} , 
 	{ "name": "lane_words_5_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_5", "role": "d0" }} , 
 	{ "name": "lane_words_4_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_4", "role": "address0" }} , 
 	{ "name": "lane_words_4_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_4", "role": "ce0" }} , 
 	{ "name": "lane_words_4_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_4", "role": "we0" }} , 
 	{ "name": "lane_words_4_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_4", "role": "d0" }} , 
 	{ "name": "lane_words_3_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_3", "role": "address0" }} , 
 	{ "name": "lane_words_3_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_3", "role": "ce0" }} , 
 	{ "name": "lane_words_3_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_3", "role": "we0" }} , 
 	{ "name": "lane_words_3_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "lane_words_3", "role": "d0" }} , 
 	{ "name": "lane_words_2_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_2", "role": "address0" }} , 
 	{ "name": "lane_words_2_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_2", "role": "ce0" }} , 
 	{ "name": "lane_words_2_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_2", "role": "we0" }} , 
 	{ "name": "lane_words_2_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_2", "role": "d0" }} , 
 	{ "name": "lane_words_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_1", "role": "address0" }} , 
 	{ "name": "lane_words_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_1", "role": "ce0" }} , 
 	{ "name": "lane_words_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_1", "role": "we0" }} , 
 	{ "name": "lane_words_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_1", "role": "d0" }} , 
 	{ "name": "lane_words_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "lane_words_0", "role": "address0" }} , 
 	{ "name": "lane_words_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_0", "role": "ce0" }} , 
 	{ "name": "lane_words_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "lane_words_0", "role": "we0" }} , 
 	{ "name": "lane_words_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "lane_words_0", "role": "d0" }}  ]}

set ArgLastReadFirstWriteLatency {
	p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_213_1 {
		gmem0 {Type I LastRead 1 FirstWrite -1}
		sext_ln213 {Type I LastRead 0 FirstWrite -1}
		lane_words_15 {Type O LastRead -1 FirstWrite 2}
		lane_words_14 {Type O LastRead -1 FirstWrite 2}
		lane_words_13 {Type O LastRead -1 FirstWrite 2}
		lane_words_12 {Type O LastRead -1 FirstWrite 2}
		lane_words_11 {Type O LastRead -1 FirstWrite 2}
		lane_words_10 {Type O LastRead -1 FirstWrite 2}
		lane_words_9 {Type O LastRead -1 FirstWrite 2}
		lane_words_8 {Type O LastRead -1 FirstWrite 2}
		lane_words_7 {Type O LastRead -1 FirstWrite 2}
		lane_words_6 {Type O LastRead -1 FirstWrite 2}
		lane_words_5 {Type O LastRead -1 FirstWrite 2}
		lane_words_4 {Type O LastRead -1 FirstWrite 2}
		lane_words_3 {Type O LastRead -1 FirstWrite 2}
		lane_words_2 {Type O LastRead -1 FirstWrite 2}
		lane_words_1 {Type O LastRead -1 FirstWrite 2}
		lane_words_0 {Type O LastRead -1 FirstWrite 2}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "14", "Max" : "14"}
	, {"Name" : "Interval", "Min" : "12", "Max" : "12"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	 { m_axi {  { m_axi_gmem0_0_AWVALID VALID 1 1 }  { m_axi_gmem0_0_AWREADY READY 0 1 }  { m_axi_gmem0_0_AWADDR ADDR 1 64 }  { m_axi_gmem0_0_AWID ID 1 1 }  { m_axi_gmem0_0_AWLEN SIZE 1 32 }  { m_axi_gmem0_0_AWSIZE BURST 1 3 }  { m_axi_gmem0_0_AWBURST LOCK 1 2 }  { m_axi_gmem0_0_AWLOCK CACHE 1 2 }  { m_axi_gmem0_0_AWCACHE PROT 1 4 }  { m_axi_gmem0_0_AWPROT QOS 1 3 }  { m_axi_gmem0_0_AWQOS REGION 1 4 }  { m_axi_gmem0_0_AWREGION USER 1 4 }  { m_axi_gmem0_0_AWUSER DATA 1 1 }  { m_axi_gmem0_0_WVALID VALID 1 1 }  { m_axi_gmem0_0_WREADY READY 0 1 }  { m_axi_gmem0_0_WDATA FIFONUM 1 512 }  { m_axi_gmem0_0_WSTRB STRB 1 64 }  { m_axi_gmem0_0_WLAST LAST 1 1 }  { m_axi_gmem0_0_WID ID 1 1 }  { m_axi_gmem0_0_WUSER DATA 1 1 }  { m_axi_gmem0_0_ARVALID VALID 1 1 }  { m_axi_gmem0_0_ARREADY READY 0 1 }  { m_axi_gmem0_0_ARADDR ADDR 1 64 }  { m_axi_gmem0_0_ARID ID 1 1 }  { m_axi_gmem0_0_ARLEN SIZE 1 32 }  { m_axi_gmem0_0_ARSIZE BURST 1 3 }  { m_axi_gmem0_0_ARBURST LOCK 1 2 }  { m_axi_gmem0_0_ARLOCK CACHE 1 2 }  { m_axi_gmem0_0_ARCACHE PROT 1 4 }  { m_axi_gmem0_0_ARPROT QOS 1 3 }  { m_axi_gmem0_0_ARQOS REGION 1 4 }  { m_axi_gmem0_0_ARREGION USER 1 4 }  { m_axi_gmem0_0_ARUSER DATA 1 1 }  { m_axi_gmem0_0_RVALID VALID 0 1 }  { m_axi_gmem0_0_RREADY READY 1 1 }  { m_axi_gmem0_0_RDATA FIFONUM 0 512 }  { m_axi_gmem0_0_RLAST LAST 0 1 }  { m_axi_gmem0_0_RID ID 0 1 }  { m_axi_gmem0_0_RFIFONUM LEN 0 9 }  { m_axi_gmem0_0_RUSER DATA 0 1 }  { m_axi_gmem0_0_RRESP RESP 0 2 }  { m_axi_gmem0_0_BVALID VALID 0 1 }  { m_axi_gmem0_0_BREADY READY 1 1 }  { m_axi_gmem0_0_BRESP RESP 0 2 }  { m_axi_gmem0_0_BID ID 0 1 }  { m_axi_gmem0_0_BUSER DATA 0 1 } } }
	sext_ln213 { ap_none {  { sext_ln213 in_data 0 58 } } }
	lane_words_15 { ap_memory {  { lane_words_15_address0 mem_address 1 4 }  { lane_words_15_ce0 mem_ce 1 1 }  { lane_words_15_we0 mem_we 1 1 }  { lane_words_15_d0 mem_din 1 26 } } }
	lane_words_14 { ap_memory {  { lane_words_14_address0 mem_address 1 4 }  { lane_words_14_ce0 mem_ce 1 1 }  { lane_words_14_we0 mem_we 1 1 }  { lane_words_14_d0 mem_din 1 26 } } }
	lane_words_13 { ap_memory {  { lane_words_13_address0 mem_address 1 4 }  { lane_words_13_ce0 mem_ce 1 1 }  { lane_words_13_we0 mem_we 1 1 }  { lane_words_13_d0 mem_din 1 26 } } }
	lane_words_12 { ap_memory {  { lane_words_12_address0 mem_address 1 4 }  { lane_words_12_ce0 mem_ce 1 1 }  { lane_words_12_we0 mem_we 1 1 }  { lane_words_12_d0 mem_din 1 26 } } }
	lane_words_11 { ap_memory {  { lane_words_11_address0 mem_address 1 4 }  { lane_words_11_ce0 mem_ce 1 1 }  { lane_words_11_we0 mem_we 1 1 }  { lane_words_11_d0 mem_din 1 26 } } }
	lane_words_10 { ap_memory {  { lane_words_10_address0 mem_address 1 4 }  { lane_words_10_ce0 mem_ce 1 1 }  { lane_words_10_we0 mem_we 1 1 }  { lane_words_10_d0 mem_din 1 26 } } }
	lane_words_9 { ap_memory {  { lane_words_9_address0 mem_address 1 4 }  { lane_words_9_ce0 mem_ce 1 1 }  { lane_words_9_we0 mem_we 1 1 }  { lane_words_9_d0 mem_din 1 26 } } }
	lane_words_8 { ap_memory {  { lane_words_8_address0 mem_address 1 4 }  { lane_words_8_ce0 mem_ce 1 1 }  { lane_words_8_we0 mem_we 1 1 }  { lane_words_8_d0 mem_din 1 26 } } }
	lane_words_7 { ap_memory {  { lane_words_7_address0 mem_address 1 4 }  { lane_words_7_ce0 mem_ce 1 1 }  { lane_words_7_we0 mem_we 1 1 }  { lane_words_7_d0 mem_din 1 32 } } }
	lane_words_6 { ap_memory {  { lane_words_6_address0 mem_address 1 4 }  { lane_words_6_ce0 mem_ce 1 1 }  { lane_words_6_we0 mem_we 1 1 }  { lane_words_6_d0 mem_din 1 26 } } }
	lane_words_5 { ap_memory {  { lane_words_5_address0 mem_address 1 4 }  { lane_words_5_ce0 mem_ce 1 1 }  { lane_words_5_we0 mem_we 1 1 }  { lane_words_5_d0 mem_din 1 26 } } }
	lane_words_4 { ap_memory {  { lane_words_4_address0 mem_address 1 4 }  { lane_words_4_ce0 mem_ce 1 1 }  { lane_words_4_we0 mem_we 1 1 }  { lane_words_4_d0 mem_din 1 26 } } }
	lane_words_3 { ap_memory {  { lane_words_3_address0 mem_address 1 4 }  { lane_words_3_ce0 mem_ce 1 1 }  { lane_words_3_we0 mem_we 1 1 }  { lane_words_3_d0 mem_din 1 26 } } }
	lane_words_2 { ap_memory {  { lane_words_2_address0 mem_address 1 4 }  { lane_words_2_ce0 mem_ce 1 1 }  { lane_words_2_we0 mem_we 1 1 }  { lane_words_2_d0 mem_din 1 32 } } }
	lane_words_1 { ap_memory {  { lane_words_1_address0 mem_address 1 4 }  { lane_words_1_ce0 mem_ce 1 1 }  { lane_words_1_we0 mem_we 1 1 }  { lane_words_1_d0 mem_din 1 32 } } }
	lane_words_0 { ap_memory {  { lane_words_0_address0 mem_address 1 4 }  { lane_words_0_ce0 mem_ce 1 1 }  { lane_words_0_we0 mem_we 1 1 }  { lane_words_0_d0 mem_din 1 32 } } }
}
