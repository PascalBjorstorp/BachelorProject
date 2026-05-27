set moduleName mpc_fpga_top_opencl
set isTopModule 1
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set isPipelined_legacy 0
set pipeline_type none
set FunctionProtocol ap_ctrl_chain
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
set C_modelName {mpc_fpga_top_opencl}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ gmem0 int 512 regular {axi_master 0}  }
	{ gmem1 int 128 regular {axi_master 1}  }
	{ input_words512 int 64 regular {axi_slave 0}  }
	{ output_words128 int 64 regular {axi_slave 0}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "gmem0", "interface" : "axi_master", "bitwidth" : 512, "direction" : "READONLY", "id_num" : 1, "bitSlice":[ {"cElement": [{"cName": "input_words512","offset": { "type": "dynamic","port_name": "input_words512","bundle": "control"},"direction": "READONLY"}]}]} , 
 	{ "Name" : "gmem1", "interface" : "axi_master", "bitwidth" : 128, "direction" : "WRITEONLY", "id_num" : 1, "bitSlice":[ {"cElement": [{"cName": "output_words128","offset": { "type": "dynamic","port_name": "output_words128","bundle": "control"},"direction": "WRITEONLY"}]}]} , 
 	{ "Name" : "input_words512", "interface" : "axi_slave", "bundle":"control","type":"ap_none","bitwidth" : 64, "direction" : "READONLY", "offset" : {"in":16}, "offset_end" : {"in":27}} , 
 	{ "Name" : "output_words128", "interface" : "axi_slave", "bundle":"control","type":"ap_none","bitwidth" : 64, "direction" : "READONLY", "offset" : {"in":28}, "offset_end" : {"in":39}} ]}
# RTL Port declarations: 
set portNum 110
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst_n sc_in sc_logic 1 reset -1 active_low_sync } 
	{ m_axi_gmem0_AWVALID sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_AWREADY sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_AWADDR sc_out sc_lv 64 signal 0 } 
	{ m_axi_gmem0_AWID sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_AWLEN sc_out sc_lv 8 signal 0 } 
	{ m_axi_gmem0_AWSIZE sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_AWBURST sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_AWLOCK sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_AWCACHE sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_AWPROT sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_AWQOS sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_AWREGION sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_AWUSER sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_WVALID sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_WREADY sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_WDATA sc_out sc_lv 512 signal 0 } 
	{ m_axi_gmem0_WSTRB sc_out sc_lv 64 signal 0 } 
	{ m_axi_gmem0_WLAST sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_WID sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_WUSER sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_ARVALID sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_ARREADY sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_ARADDR sc_out sc_lv 64 signal 0 } 
	{ m_axi_gmem0_ARID sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_ARLEN sc_out sc_lv 8 signal 0 } 
	{ m_axi_gmem0_ARSIZE sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_ARBURST sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_ARLOCK sc_out sc_lv 2 signal 0 } 
	{ m_axi_gmem0_ARCACHE sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_ARPROT sc_out sc_lv 3 signal 0 } 
	{ m_axi_gmem0_ARQOS sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_ARREGION sc_out sc_lv 4 signal 0 } 
	{ m_axi_gmem0_ARUSER sc_out sc_lv 1 signal 0 } 
	{ m_axi_gmem0_RVALID sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_RREADY sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_RDATA sc_in sc_lv 512 signal 0 } 
	{ m_axi_gmem0_RLAST sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_RID sc_in sc_lv 1 signal 0 } 
	{ m_axi_gmem0_RUSER sc_in sc_lv 1 signal 0 } 
	{ m_axi_gmem0_RRESP sc_in sc_lv 2 signal 0 } 
	{ m_axi_gmem0_BVALID sc_in sc_logic 1 signal 0 } 
	{ m_axi_gmem0_BREADY sc_out sc_logic 1 signal 0 } 
	{ m_axi_gmem0_BRESP sc_in sc_lv 2 signal 0 } 
	{ m_axi_gmem0_BID sc_in sc_lv 1 signal 0 } 
	{ m_axi_gmem0_BUSER sc_in sc_lv 1 signal 0 } 
	{ m_axi_gmem1_AWVALID sc_out sc_logic 1 signal 1 } 
	{ m_axi_gmem1_AWREADY sc_in sc_logic 1 signal 1 } 
	{ m_axi_gmem1_AWADDR sc_out sc_lv 64 signal 1 } 
	{ m_axi_gmem1_AWID sc_out sc_lv 1 signal 1 } 
	{ m_axi_gmem1_AWLEN sc_out sc_lv 8 signal 1 } 
	{ m_axi_gmem1_AWSIZE sc_out sc_lv 3 signal 1 } 
	{ m_axi_gmem1_AWBURST sc_out sc_lv 2 signal 1 } 
	{ m_axi_gmem1_AWLOCK sc_out sc_lv 2 signal 1 } 
	{ m_axi_gmem1_AWCACHE sc_out sc_lv 4 signal 1 } 
	{ m_axi_gmem1_AWPROT sc_out sc_lv 3 signal 1 } 
	{ m_axi_gmem1_AWQOS sc_out sc_lv 4 signal 1 } 
	{ m_axi_gmem1_AWREGION sc_out sc_lv 4 signal 1 } 
	{ m_axi_gmem1_AWUSER sc_out sc_lv 1 signal 1 } 
	{ m_axi_gmem1_WVALID sc_out sc_logic 1 signal 1 } 
	{ m_axi_gmem1_WREADY sc_in sc_logic 1 signal 1 } 
	{ m_axi_gmem1_WDATA sc_out sc_lv 128 signal 1 } 
	{ m_axi_gmem1_WSTRB sc_out sc_lv 16 signal 1 } 
	{ m_axi_gmem1_WLAST sc_out sc_logic 1 signal 1 } 
	{ m_axi_gmem1_WID sc_out sc_lv 1 signal 1 } 
	{ m_axi_gmem1_WUSER sc_out sc_lv 1 signal 1 } 
	{ m_axi_gmem1_ARVALID sc_out sc_logic 1 signal 1 } 
	{ m_axi_gmem1_ARREADY sc_in sc_logic 1 signal 1 } 
	{ m_axi_gmem1_ARADDR sc_out sc_lv 64 signal 1 } 
	{ m_axi_gmem1_ARID sc_out sc_lv 1 signal 1 } 
	{ m_axi_gmem1_ARLEN sc_out sc_lv 8 signal 1 } 
	{ m_axi_gmem1_ARSIZE sc_out sc_lv 3 signal 1 } 
	{ m_axi_gmem1_ARBURST sc_out sc_lv 2 signal 1 } 
	{ m_axi_gmem1_ARLOCK sc_out sc_lv 2 signal 1 } 
	{ m_axi_gmem1_ARCACHE sc_out sc_lv 4 signal 1 } 
	{ m_axi_gmem1_ARPROT sc_out sc_lv 3 signal 1 } 
	{ m_axi_gmem1_ARQOS sc_out sc_lv 4 signal 1 } 
	{ m_axi_gmem1_ARREGION sc_out sc_lv 4 signal 1 } 
	{ m_axi_gmem1_ARUSER sc_out sc_lv 1 signal 1 } 
	{ m_axi_gmem1_RVALID sc_in sc_logic 1 signal 1 } 
	{ m_axi_gmem1_RREADY sc_out sc_logic 1 signal 1 } 
	{ m_axi_gmem1_RDATA sc_in sc_lv 128 signal 1 } 
	{ m_axi_gmem1_RLAST sc_in sc_logic 1 signal 1 } 
	{ m_axi_gmem1_RID sc_in sc_lv 1 signal 1 } 
	{ m_axi_gmem1_RUSER sc_in sc_lv 1 signal 1 } 
	{ m_axi_gmem1_RRESP sc_in sc_lv 2 signal 1 } 
	{ m_axi_gmem1_BVALID sc_in sc_logic 1 signal 1 } 
	{ m_axi_gmem1_BREADY sc_out sc_logic 1 signal 1 } 
	{ m_axi_gmem1_BRESP sc_in sc_lv 2 signal 1 } 
	{ m_axi_gmem1_BID sc_in sc_lv 1 signal 1 } 
	{ m_axi_gmem1_BUSER sc_in sc_lv 1 signal 1 } 
	{ s_axi_control_AWVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_control_AWREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_control_AWADDR sc_in sc_lv 6 signal -1 } 
	{ s_axi_control_WVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_control_WREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_control_WDATA sc_in sc_lv 32 signal -1 } 
	{ s_axi_control_WSTRB sc_in sc_lv 4 signal -1 } 
	{ s_axi_control_ARVALID sc_in sc_logic 1 signal -1 } 
	{ s_axi_control_ARREADY sc_out sc_logic 1 signal -1 } 
	{ s_axi_control_ARADDR sc_in sc_lv 6 signal -1 } 
	{ s_axi_control_RVALID sc_out sc_logic 1 signal -1 } 
	{ s_axi_control_RREADY sc_in sc_logic 1 signal -1 } 
	{ s_axi_control_RDATA sc_out sc_lv 32 signal -1 } 
	{ s_axi_control_RRESP sc_out sc_lv 2 signal -1 } 
	{ s_axi_control_BVALID sc_out sc_logic 1 signal -1 } 
	{ s_axi_control_BREADY sc_in sc_logic 1 signal -1 } 
	{ s_axi_control_BRESP sc_out sc_lv 2 signal -1 } 
	{ interrupt sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "s_axi_control_AWADDR", "direction": "in", "datatype": "sc_lv", "bitwidth":6, "type": "signal", "bundle":{"name": "control", "role": "AWADDR" },"address":[{"name":"mpc_fpga_top_opencl","role":"start","value":"0","valid_bit":"0"},{"name":"mpc_fpga_top_opencl","role":"continue","value":"0","valid_bit":"4"},{"name":"mpc_fpga_top_opencl","role":"auto_start","value":"0","valid_bit":"7"},{"name":"input_words512","role":"data","value":"16"},{"name":"output_words128","role":"data","value":"28"}] },
	{ "name": "s_axi_control_AWVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "AWVALID" } },
	{ "name": "s_axi_control_AWREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "AWREADY" } },
	{ "name": "s_axi_control_WVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "WVALID" } },
	{ "name": "s_axi_control_WREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "WREADY" } },
	{ "name": "s_axi_control_WDATA", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "control", "role": "WDATA" } },
	{ "name": "s_axi_control_WSTRB", "direction": "in", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "control", "role": "WSTRB" } },
	{ "name": "s_axi_control_ARADDR", "direction": "in", "datatype": "sc_lv", "bitwidth":6, "type": "signal", "bundle":{"name": "control", "role": "ARADDR" },"address":[{"name":"mpc_fpga_top_opencl","role":"start","value":"0","valid_bit":"0"},{"name":"mpc_fpga_top_opencl","role":"done","value":"0","valid_bit":"1"},{"name":"mpc_fpga_top_opencl","role":"idle","value":"0","valid_bit":"2"},{"name":"mpc_fpga_top_opencl","role":"ready","value":"0","valid_bit":"3"},{"name":"mpc_fpga_top_opencl","role":"auto_start","value":"0","valid_bit":"7"}] },
	{ "name": "s_axi_control_ARVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "ARVALID" } },
	{ "name": "s_axi_control_ARREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "ARREADY" } },
	{ "name": "s_axi_control_RVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "RVALID" } },
	{ "name": "s_axi_control_RREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "RREADY" } },
	{ "name": "s_axi_control_RDATA", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "control", "role": "RDATA" } },
	{ "name": "s_axi_control_RRESP", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "control", "role": "RRESP" } },
	{ "name": "s_axi_control_BVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "BVALID" } },
	{ "name": "s_axi_control_BREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "BREADY" } },
	{ "name": "s_axi_control_BRESP", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "control", "role": "BRESP" } },
	{ "name": "interrupt", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "control", "role": "interrupt" } }, 
 	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst_n", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst_n", "role": "default" }} , 
 	{ "name": "m_axi_gmem0_AWVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "AWVALID" }} , 
 	{ "name": "m_axi_gmem0_AWREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "AWREADY" }} , 
 	{ "name": "m_axi_gmem0_AWADDR", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem0", "role": "AWADDR" }} , 
 	{ "name": "m_axi_gmem0_AWID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "AWID" }} , 
 	{ "name": "m_axi_gmem0_AWLEN", "direction": "out", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "gmem0", "role": "AWLEN" }} , 
 	{ "name": "m_axi_gmem0_AWSIZE", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "AWSIZE" }} , 
 	{ "name": "m_axi_gmem0_AWBURST", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "AWBURST" }} , 
 	{ "name": "m_axi_gmem0_AWLOCK", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "AWLOCK" }} , 
 	{ "name": "m_axi_gmem0_AWCACHE", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "AWCACHE" }} , 
 	{ "name": "m_axi_gmem0_AWPROT", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "AWPROT" }} , 
 	{ "name": "m_axi_gmem0_AWQOS", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "AWQOS" }} , 
 	{ "name": "m_axi_gmem0_AWREGION", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "AWREGION" }} , 
 	{ "name": "m_axi_gmem0_AWUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "AWUSER" }} , 
 	{ "name": "m_axi_gmem0_WVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "WVALID" }} , 
 	{ "name": "m_axi_gmem0_WREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "WREADY" }} , 
 	{ "name": "m_axi_gmem0_WDATA", "direction": "out", "datatype": "sc_lv", "bitwidth":512, "type": "signal", "bundle":{"name": "gmem0", "role": "WDATA" }} , 
 	{ "name": "m_axi_gmem0_WSTRB", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem0", "role": "WSTRB" }} , 
 	{ "name": "m_axi_gmem0_WLAST", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "WLAST" }} , 
 	{ "name": "m_axi_gmem0_WID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "WID" }} , 
 	{ "name": "m_axi_gmem0_WUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "WUSER" }} , 
 	{ "name": "m_axi_gmem0_ARVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "ARVALID" }} , 
 	{ "name": "m_axi_gmem0_ARREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "ARREADY" }} , 
 	{ "name": "m_axi_gmem0_ARADDR", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem0", "role": "ARADDR" }} , 
 	{ "name": "m_axi_gmem0_ARID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "ARID" }} , 
 	{ "name": "m_axi_gmem0_ARLEN", "direction": "out", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "gmem0", "role": "ARLEN" }} , 
 	{ "name": "m_axi_gmem0_ARSIZE", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "ARSIZE" }} , 
 	{ "name": "m_axi_gmem0_ARBURST", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "ARBURST" }} , 
 	{ "name": "m_axi_gmem0_ARLOCK", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "ARLOCK" }} , 
 	{ "name": "m_axi_gmem0_ARCACHE", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "ARCACHE" }} , 
 	{ "name": "m_axi_gmem0_ARPROT", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem0", "role": "ARPROT" }} , 
 	{ "name": "m_axi_gmem0_ARQOS", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "ARQOS" }} , 
 	{ "name": "m_axi_gmem0_ARREGION", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem0", "role": "ARREGION" }} , 
 	{ "name": "m_axi_gmem0_ARUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "ARUSER" }} , 
 	{ "name": "m_axi_gmem0_RVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "RVALID" }} , 
 	{ "name": "m_axi_gmem0_RREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "RREADY" }} , 
 	{ "name": "m_axi_gmem0_RDATA", "direction": "in", "datatype": "sc_lv", "bitwidth":512, "type": "signal", "bundle":{"name": "gmem0", "role": "RDATA" }} , 
 	{ "name": "m_axi_gmem0_RLAST", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "RLAST" }} , 
 	{ "name": "m_axi_gmem0_RID", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "RID" }} , 
 	{ "name": "m_axi_gmem0_RUSER", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "RUSER" }} , 
 	{ "name": "m_axi_gmem0_RRESP", "direction": "in", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "RRESP" }} , 
 	{ "name": "m_axi_gmem0_BVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "BVALID" }} , 
 	{ "name": "m_axi_gmem0_BREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "BREADY" }} , 
 	{ "name": "m_axi_gmem0_BRESP", "direction": "in", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem0", "role": "BRESP" }} , 
 	{ "name": "m_axi_gmem0_BID", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "BID" }} , 
 	{ "name": "m_axi_gmem0_BUSER", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem0", "role": "BUSER" }} , 
 	{ "name": "m_axi_gmem1_AWVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "AWVALID" }} , 
 	{ "name": "m_axi_gmem1_AWREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "AWREADY" }} , 
 	{ "name": "m_axi_gmem1_AWADDR", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem1", "role": "AWADDR" }} , 
 	{ "name": "m_axi_gmem1_AWID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "AWID" }} , 
 	{ "name": "m_axi_gmem1_AWLEN", "direction": "out", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "gmem1", "role": "AWLEN" }} , 
 	{ "name": "m_axi_gmem1_AWSIZE", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem1", "role": "AWSIZE" }} , 
 	{ "name": "m_axi_gmem1_AWBURST", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem1", "role": "AWBURST" }} , 
 	{ "name": "m_axi_gmem1_AWLOCK", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem1", "role": "AWLOCK" }} , 
 	{ "name": "m_axi_gmem1_AWCACHE", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem1", "role": "AWCACHE" }} , 
 	{ "name": "m_axi_gmem1_AWPROT", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem1", "role": "AWPROT" }} , 
 	{ "name": "m_axi_gmem1_AWQOS", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem1", "role": "AWQOS" }} , 
 	{ "name": "m_axi_gmem1_AWREGION", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem1", "role": "AWREGION" }} , 
 	{ "name": "m_axi_gmem1_AWUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "AWUSER" }} , 
 	{ "name": "m_axi_gmem1_WVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "WVALID" }} , 
 	{ "name": "m_axi_gmem1_WREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "WREADY" }} , 
 	{ "name": "m_axi_gmem1_WDATA", "direction": "out", "datatype": "sc_lv", "bitwidth":128, "type": "signal", "bundle":{"name": "gmem1", "role": "WDATA" }} , 
 	{ "name": "m_axi_gmem1_WSTRB", "direction": "out", "datatype": "sc_lv", "bitwidth":16, "type": "signal", "bundle":{"name": "gmem1", "role": "WSTRB" }} , 
 	{ "name": "m_axi_gmem1_WLAST", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "WLAST" }} , 
 	{ "name": "m_axi_gmem1_WID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "WID" }} , 
 	{ "name": "m_axi_gmem1_WUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "WUSER" }} , 
 	{ "name": "m_axi_gmem1_ARVALID", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "ARVALID" }} , 
 	{ "name": "m_axi_gmem1_ARREADY", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "ARREADY" }} , 
 	{ "name": "m_axi_gmem1_ARADDR", "direction": "out", "datatype": "sc_lv", "bitwidth":64, "type": "signal", "bundle":{"name": "gmem1", "role": "ARADDR" }} , 
 	{ "name": "m_axi_gmem1_ARID", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "ARID" }} , 
 	{ "name": "m_axi_gmem1_ARLEN", "direction": "out", "datatype": "sc_lv", "bitwidth":8, "type": "signal", "bundle":{"name": "gmem1", "role": "ARLEN" }} , 
 	{ "name": "m_axi_gmem1_ARSIZE", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem1", "role": "ARSIZE" }} , 
 	{ "name": "m_axi_gmem1_ARBURST", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem1", "role": "ARBURST" }} , 
 	{ "name": "m_axi_gmem1_ARLOCK", "direction": "out", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem1", "role": "ARLOCK" }} , 
 	{ "name": "m_axi_gmem1_ARCACHE", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem1", "role": "ARCACHE" }} , 
 	{ "name": "m_axi_gmem1_ARPROT", "direction": "out", "datatype": "sc_lv", "bitwidth":3, "type": "signal", "bundle":{"name": "gmem1", "role": "ARPROT" }} , 
 	{ "name": "m_axi_gmem1_ARQOS", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem1", "role": "ARQOS" }} , 
 	{ "name": "m_axi_gmem1_ARREGION", "direction": "out", "datatype": "sc_lv", "bitwidth":4, "type": "signal", "bundle":{"name": "gmem1", "role": "ARREGION" }} , 
 	{ "name": "m_axi_gmem1_ARUSER", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "ARUSER" }} , 
 	{ "name": "m_axi_gmem1_RVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "RVALID" }} , 
 	{ "name": "m_axi_gmem1_RREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "RREADY" }} , 
 	{ "name": "m_axi_gmem1_RDATA", "direction": "in", "datatype": "sc_lv", "bitwidth":128, "type": "signal", "bundle":{"name": "gmem1", "role": "RDATA" }} , 
 	{ "name": "m_axi_gmem1_RLAST", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "RLAST" }} , 
 	{ "name": "m_axi_gmem1_RID", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "RID" }} , 
 	{ "name": "m_axi_gmem1_RUSER", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "RUSER" }} , 
 	{ "name": "m_axi_gmem1_RRESP", "direction": "in", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem1", "role": "RRESP" }} , 
 	{ "name": "m_axi_gmem1_BVALID", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "BVALID" }} , 
 	{ "name": "m_axi_gmem1_BREADY", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "BREADY" }} , 
 	{ "name": "m_axi_gmem1_BRESP", "direction": "in", "datatype": "sc_lv", "bitwidth":2, "type": "signal", "bundle":{"name": "gmem1", "role": "BRESP" }} , 
 	{ "name": "m_axi_gmem1_BID", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "BID" }} , 
 	{ "name": "m_axi_gmem1_BUSER", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "gmem1", "role": "BUSER" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_fpga_top_opencl {
		gmem0 {Type I LastRead 1 FirstWrite -1}
		gmem1 {Type O LastRead 11 FirstWrite 10}
		input_words512 {Type I LastRead 0 FirstWrite -1}
		output_words128 {Type I LastRead 0 FirstWrite -1}
		out_steering_arg_index {Type IO LastRead -1 FirstWrite -1}
		out_accel_arg_index {Type IO LastRead -1 FirstWrite -1}
		out_status_arg_index {Type IO LastRead -1 FirstWrite -1}
		out_iters_arg_index {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_initialized {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_max_iter_streak {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_initialized {Type IO LastRead -1 FirstWrite -1}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	p_anonymous_namespace_unpack_input_lane_words {
		gmem0 {Type I LastRead 1 FirstWrite -1}
		input_words512 {Type I LastRead 0 FirstWrite -1}
		lane_words_0 {Type O LastRead -1 FirstWrite 2}
		lane_words_1 {Type O LastRead -1 FirstWrite 2}
		lane_words_2 {Type O LastRead -1 FirstWrite 2}
		lane_words_3 {Type O LastRead -1 FirstWrite 2}
		lane_words_4 {Type O LastRead -1 FirstWrite 2}
		lane_words_5 {Type O LastRead -1 FirstWrite 2}
		lane_words_6 {Type O LastRead -1 FirstWrite 2}
		lane_words_7 {Type O LastRead -1 FirstWrite 2}
		lane_words_8 {Type O LastRead -1 FirstWrite 2}
		lane_words_9 {Type O LastRead -1 FirstWrite 2}
		lane_words_10 {Type O LastRead -1 FirstWrite 2}
		lane_words_11 {Type O LastRead -1 FirstWrite 2}
		lane_words_12 {Type O LastRead -1 FirstWrite 2}
		lane_words_13 {Type O LastRead -1 FirstWrite 2}
		lane_words_14 {Type O LastRead -1 FirstWrite 2}
		lane_words_15 {Type O LastRead -1 FirstWrite 2}}
	p_anonymous_namespace_unpack_input_lane_words_Pipeline_VITIS_LOOP_216_1 {
		gmem0 {Type I LastRead 1 FirstWrite -1}
		sext_ln216 {Type I LastRead 0 FirstWrite -1}
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
		lane_words_0 {Type O LastRead -1 FirstWrite 2}}
	p_anonymous_namespace_fill_mpc_reference_trajectory_from_lane_words {
		lane_words_0 {Type I LastRead 0 FirstWrite -1}
		lane_words_1 {Type I LastRead 0 FirstWrite -1}
		lane_words_2 {Type I LastRead 0 FirstWrite -1}
		lane_words_3 {Type I LastRead 0 FirstWrite -1}
		lane_words_4 {Type I LastRead 0 FirstWrite -1}
		lane_words_5 {Type I LastRead 0 FirstWrite -1}
		lane_words_6 {Type I LastRead 0 FirstWrite -1}
		lane_words_7 {Type I LastRead 0 FirstWrite -1}
		lane_words_8 {Type I LastRead 0 FirstWrite -1}
		lane_words_9 {Type I LastRead 0 FirstWrite -1}
		lane_words_10 {Type I LastRead 0 FirstWrite -1}
		lane_words_11 {Type I LastRead 0 FirstWrite -1}
		lane_words_12 {Type I LastRead 0 FirstWrite -1}
		lane_words_13 {Type I LastRead 0 FirstWrite -1}
		lane_words_14 {Type I LastRead 0 FirstWrite -1}
		lane_words_15 {Type I LastRead 0 FirstWrite -1}
		out_ref_reference_heading_error {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_lateral_error {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_velocity {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_lateral_velocity {Type O LastRead -1 FirstWrite 1}
		out_ref_reference_yaw_rate {Type O LastRead -1 FirstWrite 1}
		out_ref_path_curvature {Type O LastRead -1 FirstWrite 1}
		out_ref_left_wall_bound {Type O LastRead -1 FirstWrite 1}
		out_ref_right_wall_bound {Type O LastRead -1 FirstWrite 1}}
	p_anonymous_namespace_mpc_fpga_compute_core {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 0 FirstWrite -1}
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 0 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		steering {Type I LastRead 0 FirstWrite -1}
		prev_accel {Type I LastRead 0 FirstWrite -1}
		control_flags {Type I LastRead 0 FirstWrite -1}
		ref_reference_heading_error {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_yaw_rate {Type I LastRead 6 FirstWrite -1}
		ref_path_curvature {Type I LastRead 2 FirstWrite -1}
		ref_left_wall_bound {Type I LastRead 0 FirstWrite -1}
		ref_right_wall_bound {Type I LastRead 0 FirstWrite -1}
		out_steering {Type O LastRead -1 FirstWrite 10}
		out_accel {Type O LastRead -1 FirstWrite 10}
		out_status {Type O LastRead -1 FirstWrite 10}
		out_iters {Type O LastRead -1 FirstWrite 10}
		out_steering_arg_index {Type I LastRead 0 FirstWrite -1}
		out_accel_arg_index {Type I LastRead 0 FirstWrite -1}
		out_status_arg_index {Type I LastRead 0 FirstWrite -1}
		out_iters_arg_index {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_initialized {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_max_iter_streak {Type IO LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_initialized {Type IO LastRead -1 FirstWrite -1}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	p_anonymous_namespace_reset_core_state_hls {
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
		p_anonymous_namespace_g_core_state_admm_initialized {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_persist_max_iter_streak {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_initialized {Type O LastRead -1 FirstWrite 1}}
	p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_364_1 {
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
	p_anonymous_namespace_reset_core_state_hls_Pipeline_VITIS_LOOP_372_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_390_1 {
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_397_3 {
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_364_1 {
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
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_372_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls {
		state_ey {Type I LastRead 4 FirstWrite -1}
		state_epsi {Type I LastRead 4 FirstWrite -1}
		state_vx {Type I LastRead 3 FirstWrite -1}
		state_vy {Type I LastRead 4 FirstWrite -1}
		state_omega {Type I LastRead 4 FirstWrite -1}
		ref_reference_heading_error {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_lateral_velocity {Type I LastRead 6 FirstWrite -1}
		ref_reference_yaw_rate {Type I LastRead 6 FirstWrite -1}
		ref_path_curvature {Type I LastRead 2 FirstWrite -1}
		ref_left_wall_bound {Type I LastRead 0 FirstWrite -1}
		ref_right_wall_bound {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_actual_steering {Type I LastRead 3 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type IO LastRead 4 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type IO LastRead 4 FirstWrite 0}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type IO LastRead 10 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type IO LastRead 10 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_initialized {Type IO LastRead 12 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type IO LastRead 14 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type IO LastRead 27 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type IO LastRead 27 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type IO LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type IO LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_rho {Type IO LastRead 12 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_rho_u {Type IO LastRead 12 FirstWrite 1}}
	riccati_pass_hls {
		step_data_0_0_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_0_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_1_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_2_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_3_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_4_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_5_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_5 {Type I LastRead 32 FirstWrite -1}
		step_data_1 {Type I LastRead 21 FirstWrite -1}
		step_data_2 {Type I LastRead 21 FirstWrite -1}
		B_sparse_0 {Type I LastRead 9 FirstWrite -1}
		B_sparse_1 {Type I LastRead 9 FirstWrite -1}
		B_sparse_2 {Type I LastRead 9 FirstWrite -1}
		B_sparse_3 {Type I LastRead 9 FirstWrite -1}
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 0 FirstWrite -1}
		p_read6 {Type I LastRead 2 FirstWrite -1}
		p_read7 {Type I LastRead 2 FirstWrite -1}
		p_read8 {Type I LastRead 2 FirstWrite -1}
		p_read9 {Type I LastRead 2 FirstWrite -1}
		p_read10 {Type I LastRead 2 FirstWrite -1}
		p_read11 {Type I LastRead 2 FirstWrite -1}
		p_read12 {Type I LastRead 2 FirstWrite -1}
		p_read13 {Type I LastRead 2 FirstWrite -1}
		rho {Type I LastRead 0 FirstWrite -1}
		rho_u {Type I LastRead 0 FirstWrite -1}
		z_x_0 {Type I LastRead 15 FirstWrite -1}
		z_x_5 {Type I LastRead 15 FirstWrite -1}
		y_x_0 {Type I LastRead 15 FirstWrite -1}
		y_x_5 {Type I LastRead 15 FirstWrite -1}
		z_u_0 {Type I LastRead 17 FirstWrite -1}
		z_u_1 {Type I LastRead 17 FirstWrite -1}
		y_u_0 {Type I LastRead 17 FirstWrite -1}
		y_u_1 {Type I LastRead 17 FirstWrite -1}
		K_0_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_0_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_0_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_0_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_0_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_0_1_7 {Type IO LastRead 35 FirstWrite -1}
		K_1_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_1_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_1_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_1_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_1_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_1_1_7 {Type IO LastRead 35 FirstWrite -1}
		K_2_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_2_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_2_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_2_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_2_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_2_1_7 {Type IO LastRead 35 FirstWrite -1}
		K_3_0_0 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_1 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_2 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_3 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_4 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_5 {Type IO LastRead 37 FirstWrite -1}
		K_3_0_6 {Type IO LastRead 35 FirstWrite -1}
		K_3_0_7 {Type IO LastRead 35 FirstWrite -1}
		K_3_1_0 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_1 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_2 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_3 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_4 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_5 {Type IO LastRead 37 FirstWrite -1}
		K_3_1_6 {Type IO LastRead 35 FirstWrite -1}
		K_3_1_7 {Type IO LastRead 35 FirstWrite -1}
		kk_0 {Type IO LastRead 5 FirstWrite -1}
		kk_1 {Type IO LastRead 5 FirstWrite -1}
		x_out_0 {Type IO LastRead 1 FirstWrite 0}
		x_out_1 {Type IO LastRead 1 FirstWrite 0}
		x_out_2 {Type IO LastRead 1 FirstWrite 0}
		x_out_3 {Type IO LastRead 1 FirstWrite 0}
		x_out_4 {Type IO LastRead 1 FirstWrite 0}
		x_out_5 {Type IO LastRead 1 FirstWrite 0}
		x_out_6 {Type IO LastRead 1 FirstWrite 0}
		x_out_7 {Type IO LastRead 1 FirstWrite 0}
		u_out_0 {Type O LastRead -1 FirstWrite 6}
		u_out_1 {Type O LastRead -1 FirstWrite 6}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	riccati_backward_pass {
		step_data_0_0_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_0_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_0_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_1_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_1_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_2_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_2_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_3_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_3_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_4_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_4_5 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_0 {Type I LastRead 37 FirstWrite -1}
		step_data_0_5_1 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_2 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_3 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_4 {Type I LastRead 32 FirstWrite -1}
		step_data_0_5_5 {Type I LastRead 32 FirstWrite -1}
		step_data_1 {Type I LastRead 21 FirstWrite -1}
		step_data_2 {Type I LastRead 21 FirstWrite -1}
		B_sparse_0 {Type I LastRead 9 FirstWrite -1}
		B_sparse_1 {Type I LastRead 9 FirstWrite -1}
		B_sparse_2 {Type I LastRead 9 FirstWrite -1}
		B_sparse_3 {Type I LastRead 9 FirstWrite -1}
		p_read {Type I LastRead 7 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read2 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 0 FirstWrite -1}
		p_read5 {Type I LastRead 7 FirstWrite -1}
		rho {Type I LastRead 4 FirstWrite -1}
		rho_u {Type I LastRead 7 FirstWrite -1}
		z_x_0 {Type I LastRead 15 FirstWrite -1}
		z_x_5 {Type I LastRead 15 FirstWrite -1}
		y_x_0 {Type I LastRead 15 FirstWrite -1}
		y_x_5 {Type I LastRead 15 FirstWrite -1}
		z_u_0 {Type I LastRead 17 FirstWrite -1}
		z_u_1 {Type I LastRead 17 FirstWrite -1}
		y_u_0 {Type I LastRead 17 FirstWrite -1}
		y_u_1 {Type I LastRead 17 FirstWrite -1}
		K_0_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_0_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_0_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_0_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_0_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_0_1_7 {Type IO LastRead 35 FirstWrite 3}
		K_1_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_1_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_1_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_1_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_1_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_1_1_7 {Type IO LastRead 35 FirstWrite 3}
		K_2_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_2_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_2_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_2_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_2_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_2_1_7 {Type IO LastRead 35 FirstWrite 3}
		K_3_0_0 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_1 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_2 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_3 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_4 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_5 {Type IO LastRead 37 FirstWrite 3}
		K_3_0_6 {Type IO LastRead 35 FirstWrite 3}
		K_3_0_7 {Type IO LastRead 35 FirstWrite 3}
		K_3_1_0 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_1 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_2 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_3 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_4 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_5 {Type IO LastRead 37 FirstWrite 3}
		K_3_1_6 {Type IO LastRead 35 FirstWrite 3}
		K_3_1_7 {Type IO LastRead 35 FirstWrite 3}
		kk_0 {Type O LastRead -1 FirstWrite 38}
		kk_1 {Type O LastRead -1 FirstWrite 38}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	invert_2x2_qp_hls {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 1 FirstWrite -1}
		p_read4 {Type I LastRead 6 FirstWrite -1}
		p_read5 {Type I LastRead 6 FirstWrite -1}
		p_read6 {Type I LastRead 6 FirstWrite -1}
		p_read7 {Type I LastRead 6 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	fp_recip {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
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
		a5 {Type I LastRead 0 FirstWrite -1}}
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
		p_shift_out {Type O LastRead -1 FirstWrite 5}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_750_9 {
		K_0_0_0 {Type O LastRead -1 FirstWrite 3}
		zext_ln475 {Type I LastRead 0 FirstWrite -1}
		K_0_1_0 {Type O LastRead -1 FirstWrite 3}
		K_0_0_1 {Type O LastRead -1 FirstWrite 3}
		K_0_1_1 {Type O LastRead -1 FirstWrite 3}
		K_0_0_2 {Type O LastRead -1 FirstWrite 3}
		K_0_1_2 {Type O LastRead -1 FirstWrite 3}
		K_0_0_3 {Type O LastRead -1 FirstWrite 3}
		K_0_1_3 {Type O LastRead -1 FirstWrite 3}
		K_0_0_4 {Type O LastRead -1 FirstWrite 3}
		K_0_1_4 {Type O LastRead -1 FirstWrite 3}
		K_0_0_5 {Type O LastRead -1 FirstWrite 3}
		K_0_1_5 {Type O LastRead -1 FirstWrite 3}
		K_0_0_6 {Type O LastRead -1 FirstWrite 3}
		K_0_1_6 {Type O LastRead -1 FirstWrite 3}
		K_0_0_7 {Type O LastRead -1 FirstWrite 3}
		K_0_1_7 {Type O LastRead -1 FirstWrite 3}
		K_1_0_0 {Type O LastRead -1 FirstWrite 3}
		K_1_1_0 {Type O LastRead -1 FirstWrite 3}
		K_1_0_1 {Type O LastRead -1 FirstWrite 3}
		K_1_1_1 {Type O LastRead -1 FirstWrite 3}
		K_1_0_2 {Type O LastRead -1 FirstWrite 3}
		K_1_1_2 {Type O LastRead -1 FirstWrite 3}
		K_1_0_3 {Type O LastRead -1 FirstWrite 3}
		K_1_1_3 {Type O LastRead -1 FirstWrite 3}
		K_1_0_4 {Type O LastRead -1 FirstWrite 3}
		K_1_1_4 {Type O LastRead -1 FirstWrite 3}
		K_1_0_5 {Type O LastRead -1 FirstWrite 3}
		K_1_1_5 {Type O LastRead -1 FirstWrite 3}
		K_1_0_6 {Type O LastRead -1 FirstWrite 3}
		K_1_1_6 {Type O LastRead -1 FirstWrite 3}
		K_1_0_7 {Type O LastRead -1 FirstWrite 3}
		K_1_1_7 {Type O LastRead -1 FirstWrite 3}
		K_2_0_0 {Type O LastRead -1 FirstWrite 3}
		K_2_1_0 {Type O LastRead -1 FirstWrite 3}
		K_2_0_1 {Type O LastRead -1 FirstWrite 3}
		K_2_1_1 {Type O LastRead -1 FirstWrite 3}
		K_2_0_2 {Type O LastRead -1 FirstWrite 3}
		K_2_1_2 {Type O LastRead -1 FirstWrite 3}
		K_2_0_3 {Type O LastRead -1 FirstWrite 3}
		K_2_1_3 {Type O LastRead -1 FirstWrite 3}
		K_2_0_4 {Type O LastRead -1 FirstWrite 3}
		K_2_1_4 {Type O LastRead -1 FirstWrite 3}
		K_2_0_5 {Type O LastRead -1 FirstWrite 3}
		K_2_1_5 {Type O LastRead -1 FirstWrite 3}
		K_2_0_6 {Type O LastRead -1 FirstWrite 3}
		K_2_1_6 {Type O LastRead -1 FirstWrite 3}
		K_2_0_7 {Type O LastRead -1 FirstWrite 3}
		K_2_1_7 {Type O LastRead -1 FirstWrite 3}
		K_3_0_0 {Type O LastRead -1 FirstWrite 3}
		K_3_1_0 {Type O LastRead -1 FirstWrite 3}
		K_3_0_1 {Type O LastRead -1 FirstWrite 3}
		K_3_1_1 {Type O LastRead -1 FirstWrite 3}
		K_3_0_2 {Type O LastRead -1 FirstWrite 3}
		K_3_1_2 {Type O LastRead -1 FirstWrite 3}
		K_3_0_3 {Type O LastRead -1 FirstWrite 3}
		K_3_1_3 {Type O LastRead -1 FirstWrite 3}
		K_3_0_4 {Type O LastRead -1 FirstWrite 3}
		K_3_1_4 {Type O LastRead -1 FirstWrite 3}
		K_3_0_5 {Type O LastRead -1 FirstWrite 3}
		K_3_1_5 {Type O LastRead -1 FirstWrite 3}
		K_3_0_6 {Type O LastRead -1 FirstWrite 3}
		K_3_1_6 {Type O LastRead -1 FirstWrite 3}
		K_3_0_7 {Type O LastRead -1 FirstWrite 3}
		K_3_1_7 {Type O LastRead -1 FirstWrite 3}
		G_11_reload {Type I LastRead 0 FirstWrite -1}
		G_12_reload {Type I LastRead 0 FirstWrite -1}
		G_13_reload {Type I LastRead 0 FirstWrite -1}
		G_10_reload {Type I LastRead 0 FirstWrite -1}
		G_9_reload {Type I LastRead 0 FirstWrite -1}
		G_8_reload {Type I LastRead 0 FirstWrite -1}
		select_ln723 {Type I LastRead 0 FirstWrite -1}
		sext_ln283 {Type I LastRead 0 FirstWrite -1}
		G_7_reload {Type I LastRead 0 FirstWrite -1}
		G_6_reload {Type I LastRead 0 FirstWrite -1}
		G_5_reload {Type I LastRead 0 FirstWrite -1}
		G_4_reload {Type I LastRead 0 FirstWrite -1}
		G_3_reload {Type I LastRead 0 FirstWrite -1}
		G_2_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln283_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln283_2 {Type I LastRead 0 FirstWrite -1}
		sext_ln760 {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}}
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
		a_97 {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37 {
		a_52 {Type I LastRead 0 FirstWrite -1}
		a_60 {Type I LastRead 0 FirstWrite -1}
		a_68 {Type I LastRead 0 FirstWrite -1}
		a_76 {Type I LastRead 0 FirstWrite -1}
		a_84 {Type I LastRead 0 FirstWrite -1}
		a_92 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_121 {Type I LastRead 0 FirstWrite -1}
		a_53 {Type I LastRead 0 FirstWrite -1}
		a_61 {Type I LastRead 0 FirstWrite -1}
		a_69 {Type I LastRead 0 FirstWrite -1}
		a_77 {Type I LastRead 0 FirstWrite -1}
		a_85 {Type I LastRead 0 FirstWrite -1}
		a_93 {Type I LastRead 0 FirstWrite -1}
		sext_ln291_41 {Type I LastRead 0 FirstWrite -1}
		a_54 {Type I LastRead 0 FirstWrite -1}
		a_62 {Type I LastRead 0 FirstWrite -1}
		a_70 {Type I LastRead 0 FirstWrite -1}
		a_78 {Type I LastRead 0 FirstWrite -1}
		a_86 {Type I LastRead 0 FirstWrite -1}
		a_94 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_47 {Type I LastRead 0 FirstWrite -1}
		a_55 {Type I LastRead 0 FirstWrite -1}
		a_63 {Type I LastRead 0 FirstWrite -1}
		a_71 {Type I LastRead 0 FirstWrite -1}
		a_79 {Type I LastRead 0 FirstWrite -1}
		a_87 {Type I LastRead 0 FirstWrite -1}
		a_95 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_49 {Type I LastRead 0 FirstWrite -1}
		a_56 {Type I LastRead 0 FirstWrite -1}
		a_64 {Type I LastRead 0 FirstWrite -1}
		a_72 {Type I LastRead 0 FirstWrite -1}
		a_80 {Type I LastRead 0 FirstWrite -1}
		a_88 {Type I LastRead 0 FirstWrite -1}
		a_96 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_51 {Type I LastRead 0 FirstWrite -1}
		a_57 {Type I LastRead 0 FirstWrite -1}
		a_65 {Type I LastRead 0 FirstWrite -1}
		a_73 {Type I LastRead 0 FirstWrite -1}
		a_81 {Type I LastRead 0 FirstWrite -1}
		a_89 {Type I LastRead 0 FirstWrite -1}
		a_97 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_45 {Type I LastRead 0 FirstWrite -1}
		G_11_reload {Type I LastRead 0 FirstWrite -1}
		G_12_reload {Type I LastRead 0 FirstWrite -1}
		G_13_reload {Type I LastRead 0 FirstWrite -1}
		G_10_reload {Type I LastRead 0 FirstWrite -1}
		G_9_reload {Type I LastRead 0 FirstWrite -1}
		G_8_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln291_43 {Type I LastRead 0 FirstWrite -1}
		G_7_reload {Type I LastRead 0 FirstWrite -1}
		G_6_reload {Type I LastRead 0 FirstWrite -1}
		G_5_reload {Type I LastRead 0 FirstWrite -1}
		G_4_reload {Type I LastRead 0 FirstWrite -1}
		G_3_reload {Type I LastRead 0 FirstWrite -1}
		G_2_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln1013_1 {Type I LastRead 0 FirstWrite -1}
		q_aug_linear_4 {Type I LastRead 0 FirstWrite -1}
		sext_ln572 {Type I LastRead 0 FirstWrite -1}
		sext_ln573 {Type I LastRead 0 FirstWrite -1}
		sext_ln574 {Type I LastRead 0 FirstWrite -1}
		sext_ln575 {Type I LastRead 0 FirstWrite -1}
		q_aug_linear_5 {Type I LastRead 0 FirstWrite -1}
		p_new_5_out {Type O LastRead -1 FirstWrite 0}
		p_new_4_out {Type O LastRead -1 FirstWrite 0}
		p_new_3_out {Type O LastRead -1 FirstWrite 0}
		p_new_2_out {Type O LastRead -1 FirstWrite 0}
		p_new_1_out {Type O LastRead -1 FirstWrite 0}
		p_new_out {Type O LastRead -1 FirstWrite 0}}
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	riccati_backward_pass_Pipeline_VITIS_LOOP_1029_38 {
		G_37 {Type I LastRead 0 FirstWrite -1}
		sext_ln291_42 {Type I LastRead 0 FirstWrite -1}
		sext_ln1013 {Type I LastRead 0 FirstWrite -1}
		p_new_14_out {Type IO LastRead 3 FirstWrite 0}
		p_new_13_out {Type IO LastRead 3 FirstWrite 0}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}
	sum6_P_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
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
		a5 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_356_1 {
		ref_left_wall_bound {Type I LastRead 0 FirstWrite -1}
		ref_right_wall_bound {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144171_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145169_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144167_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145165_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144163_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145161_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144159_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145157_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144155_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145153_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144151_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145149_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144147_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145145_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144143_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145141_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144139_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145137_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144135_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145133_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144131_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145129_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144127_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145125_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144123_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145121_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144119_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145117_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144115_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145113_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144111_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145109_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144107_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145105_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010144103_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_010145101_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014499_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014597_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014495_out {Type O LastRead -1 FirstWrite 0}
		p_0_0_01014593_out {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_366_2 {
		p_0_0_010145165_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145161_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145157_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145153_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145149_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145145_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145141_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145137_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145133_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145129_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145125_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145121_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145117_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145113_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145109_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145105_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145101_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014597_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014593_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010145169_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144167_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144163_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144159_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144155_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144151_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144147_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144143_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144139_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144135_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144131_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144127_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144123_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144119_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144115_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144111_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144107_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144103_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014499_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_01014495_reload {Type I LastRead 0 FirstWrite -1}
		p_0_0_010144171_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_19_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_18_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_17_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_16_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_15_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_14_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_13_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_12_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_11_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_10_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_9_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_8_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_7_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_6_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_5_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_4_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_3_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_2_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_1_out {Type O LastRead -1 FirstWrite 1}
		wall_right_min_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_19_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_18_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_17_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_16_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_15_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_14_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_13_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_12_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_11_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_10_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_9_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_8_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_7_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_6_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_5_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_4_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_3_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_2_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_1_out {Type O LastRead -1 FirstWrite 1}
		wall_left_min_out {Type O LastRead -1 FirstWrite 1}}
	compute_window_min_hls {
		ref_wall_0_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_1_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_2_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_3_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_4_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_5_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_6_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_7_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_8_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_9_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_10_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_11_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_12_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_13_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_14_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_15_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_16_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_17_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_18_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_19_val {Type I LastRead 0 FirstWrite -1}
		center_idx {Type I LastRead 0 FirstWrite -1}}
	compute_window_min_hls {
		ref_wall_0_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_1_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_2_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_3_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_4_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_5_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_6_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_7_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_8_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_9_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_10_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_11_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_12_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_13_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_14_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_15_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_16_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_17_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_18_val {Type I LastRead 0 FirstWrite -1}
		ref_wall_19_val {Type I LastRead 0 FirstWrite -1}
		center_idx {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_371_3 {
		x0_1 {Type I LastRead 0 FirstWrite -1}
		state_omega {Type I LastRead 0 FirstWrite -1}
		state_vy {Type I LastRead 0 FirstWrite -1}
		zext_ln128 {Type I LastRead 0 FirstWrite -1}
		state_ey {Type I LastRead 0 FirstWrite -1}
		state_epsi {Type I LastRead 0 FirstWrite -1}
		sext_ln483 {Type I LastRead 0 FirstWrite -1}
		step_data_2 {Type O LastRead -1 FirstWrite 7}
		step_data_0_5_5 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_4 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_3 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_2 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_1 {Type O LastRead -1 FirstWrite 0}
		step_data_0_5_0 {Type O LastRead -1 FirstWrite 0}
		step_data_0_4_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_4 {Type O LastRead -1 FirstWrite 112}
		step_data_0_4_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_4_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_3_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_2_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_1_0 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_5 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_4 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_3 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_2 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_1 {Type O LastRead -1 FirstWrite 111}
		step_data_0_0_0 {Type O LastRead -1 FirstWrite 111}
		B_sparse_3 {Type O LastRead -1 FirstWrite 111}
		B_sparse_2 {Type O LastRead -1 FirstWrite 111}
		B_sparse_1 {Type O LastRead -1 FirstWrite 111}
		B_sparse {Type O LastRead -1 FirstWrite 0}
		step_data_3 {Type O LastRead -1 FirstWrite 1}
		step_data_4 {Type O LastRead -1 FirstWrite 1}
		step_data_5 {Type O LastRead -1 FirstWrite 30}
		ref_path_curvature {Type I LastRead 0 FirstWrite -1}
		wall_left_min_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_1_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_2_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_3_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_4_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_5_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_6_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_7_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_8_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_9_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_10_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_11_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_12_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_13_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_14_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_15_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_16_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_17_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_18_reload {Type I LastRead 0 FirstWrite -1}
		wall_left_min_19_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_1_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_2_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_3_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_4_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_5_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_6_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_7_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_8_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_9_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_10_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_11_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_12_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_13_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_14_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_15_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_16_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_17_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_18_reload {Type I LastRead 0 FirstWrite -1}
		wall_right_min_19_reload {Type I LastRead 0 FirstWrite -1}
		ref_reference_lateral_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_velocity {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		x0_2 {Type I LastRead 0 FirstWrite -1}
		step_data_1 {Type O LastRead -1 FirstWrite 28}
		ref_reference_heading_error {Type I LastRead 0 FirstWrite -1}
		ref_reference_lateral_velocity {Type I LastRead 0 FirstWrite -1}
		ref_reference_yaw_rate {Type I LastRead 0 FirstWrite -1}
		out_144_out {Type O LastRead -1 FirstWrite 112}
		terminal_dff_raw_0153_0_0_0_out {Type O LastRead -1 FirstWrite 112}
		terminal_wall_x_ub_con_out {Type O LastRead -1 FirstWrite 112}
		terminal_wall_x_lb_con_out {Type O LastRead -1 FirstWrite 112}
		atan_lut {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}
	fp_atan_lut {
		x {Type I LastRead 0 FirstWrite -1}
		atan_lut {Type I LastRead -1 FirstWrite -1}}
	compute_frenet_AB_and_next_hls {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 0 FirstWrite -1}
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 0 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		delta {Type I LastRead 0 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_frenet_AB_hls {
		ey {Type I LastRead 68 FirstWrite -1}
		epsi {Type I LastRead 65 FirstWrite -1}
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 0 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		delta {Type I LastRead 0 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		kappa {Type I LastRead 68 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_frenet_tire_hls {
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 4 FirstWrite -1}
		omega {Type I LastRead 9 FirstWrite -1}
		delta {Type I LastRead 24 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_slip_terms {
		vy {Type I LastRead 4 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		inv_vx {Type I LastRead 4 FirstWrite -1}}
	compute_front_tire_path_fn {
		conv_i_i_i_i1462_i {Type I LastRead 6 FirstWrite -1}
		p_0_0_02853_i {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		conv_i_i_i1476_i {Type I LastRead 31 FirstWrite -1}
		conv_i_i_i1991_i {Type I LastRead 38 FirstWrite -1}
		p_0_0_02849_i {Type I LastRead 38 FirstWrite -1}
		spec_select_i {Type I LastRead 38 FirstWrite -1}
		conv_i_i_i65_i {Type I LastRead 0 FirstWrite -1}
		cmp_i_i {Type I LastRead 47 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_atan_lut_fn {
		x {Type I LastRead 0 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_pacejka_ceff {
		cos_inner {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 0 FirstWrite -1}
		D_cb {Type I LastRead 3 FirstWrite -1}}
	fp_front_force_jacobians_fn {
		C_eff_f {Type I LastRead 4 FirstWrite -1}
		front_num {Type I LastRead 2 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_f {Type I LastRead 1 FirstWrite -1}}
	compute_rear_tire_path_fn {
		rear_ratio {Type I LastRead 0 FirstWrite -1}
		D_transfer {Type I LastRead 0 FirstWrite -1}
		D_pac_r {Type I LastRead 31 FirstWrite -1}
		C_min_r {Type I LastRead 38 FirstWrite -1}
		rear_num {Type I LastRead 38 FirstWrite -1}
		vx_safe {Type I LastRead 38 FirstWrite -1}
		D_r_fn {Type I LastRead 0 FirstWrite -1}
		low_speed {Type I LastRead 47 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_atan_lut_fn {
		x {Type I LastRead 0 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_pacejka_ceff {
		cos_inner {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 0 FirstWrite -1}
		D_cb {Type I LastRead 3 FirstWrite -1}}
	fp_rear_force_jacobians_fn {
		C_eff_r {Type I LastRead 4 FirstWrite -1}
		rear_num {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_r {Type I LastRead 0 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_frenet_recip {
		kappa {Type I LastRead 0 FirstWrite -1}
		ey {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_stage_shared_products {
		tr_dFyf_dvx_val {Type I LastRead 0 FirstWrite -1}
		tr_dFyf_dvy_val {Type I LastRead 1 FirstWrite -1}
		tr_dFyf_dom_val {Type I LastRead 2 FirstWrite -1}
		tr_dFyr_dvx_val {Type I LastRead 8 FirstWrite -1}
		tr_dFyr_dvy_val {Type I LastRead 9 FirstWrite -1}
		tr_dFyr_dom_val {Type I LastRead 10 FirstWrite -1}
		p_0_0_04262 {Type I LastRead 0 FirstWrite -1}
		p_0_0_04261 {Type I LastRead 3 FirstWrite -1}
		conv_i_i_i_i425 {Type I LastRead 7 FirstWrite -1}
		conv_i_i_i_i574 {Type I LastRead 6 FirstWrite -1}}
	compute_B_accel_load_transfer {
		tr_C_eff_f_raw_val {Type I LastRead 8 FirstWrite -1}
		tr_C_eff_r_val {Type I LastRead 10 FirstWrite -1}
		tr_C_min_f_val {Type I LastRead 9 FirstWrite -1}
		tr_cos_delta_val {Type I LastRead 17 FirstWrite -1}
		tr_alpha_f_op_val {Type I LastRead 11 FirstWrite -1}
		tr_alpha_r_op_val {Type I LastRead 13 FirstWrite -1}
		tr_Fz_transfer_val {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_rollout_from_forces_fn {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 23 FirstWrite -1}
		sin_epsi {Type I LastRead 10 FirstWrite -1}
		cos_epsi {Type I LastRead 2 FirstWrite -1}
		vx_safe {Type I LastRead 2 FirstWrite -1}
		vy {Type I LastRead 8 FirstWrite -1}
		omega {Type I LastRead 7 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		Fx {Type I LastRead 9 FirstWrite -1}
		F_yf {Type I LastRead 1 FirstWrite -1}
		F_yr {Type I LastRead 4 FirstWrite -1}
		sin_delta {Type I LastRead 3 FirstWrite -1}
		cos_delta {Type I LastRead 1 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	compute_B_steering {
		tr_C_eff_f_raw_val {Type I LastRead 0 FirstWrite -1}
		tr_C_min_f_val {Type I LastRead 0 FirstWrite -1}
		tr_F_yf_val {Type I LastRead 2 FirstWrite -1}
		tr_sin_delta_val {Type I LastRead 0 FirstWrite -1}
		tr_cos_delta_val {Type I LastRead 2 FirstWrite -1}}
	fp_frenet_rows01_fn {
		sin_epsi {Type I LastRead 6 FirstWrite -1}
		cos_epsi {Type I LastRead 2 FirstWrite -1}
		vx {Type I LastRead 3 FirstWrite -1}
		vy {Type I LastRead 6 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 9 FirstWrite -1}}
	compute_affine_bias_dense_hls {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 5 FirstWrite -1}
		p_read2 {Type I LastRead 10 FirstWrite -1}
		p_read3 {Type I LastRead 15 FirstWrite -1}
		p_read4 {Type I LastRead 20 FirstWrite -1}
		p_read5 {Type I LastRead 1 FirstWrite -1}
		p_read6 {Type I LastRead 6 FirstWrite -1}
		p_read7 {Type I LastRead 11 FirstWrite -1}
		p_read8 {Type I LastRead 16 FirstWrite -1}
		p_read9 {Type I LastRead 21 FirstWrite -1}
		p_read10 {Type I LastRead 2 FirstWrite -1}
		p_read11 {Type I LastRead 7 FirstWrite -1}
		p_read12 {Type I LastRead 12 FirstWrite -1}
		p_read13 {Type I LastRead 17 FirstWrite -1}
		p_read14 {Type I LastRead 22 FirstWrite -1}
		p_read15 {Type I LastRead 3 FirstWrite -1}
		p_read16 {Type I LastRead 8 FirstWrite -1}
		p_read17 {Type I LastRead 13 FirstWrite -1}
		p_read18 {Type I LastRead 18 FirstWrite -1}
		p_read19 {Type I LastRead 23 FirstWrite -1}
		p_read20 {Type I LastRead 4 FirstWrite -1}
		p_read21 {Type I LastRead 9 FirstWrite -1}
		p_read22 {Type I LastRead 14 FirstWrite -1}
		p_read23 {Type I LastRead 19 FirstWrite -1}
		p_read24 {Type I LastRead 24 FirstWrite -1}
		p_read25 {Type I LastRead 0 FirstWrite -1}
		p_read26 {Type I LastRead 4 FirstWrite -1}
		p_read27 {Type I LastRead 8 FirstWrite -1}
		p_read28 {Type I LastRead 12 FirstWrite -1}
		p_read29 {Type I LastRead 16 FirstWrite -1}
		p_read30 {Type I LastRead 25 FirstWrite -1}
		p_read31 {Type I LastRead 26 FirstWrite -1}
		p_read32 {Type I LastRead 27 FirstWrite -1}
		p_read33 {Type I LastRead 28 FirstWrite -1}
		p_read34 {Type I LastRead 29 FirstWrite -1}
		lin_ey {Type I LastRead 0 FirstWrite -1}
		lin_epsi {Type I LastRead 1 FirstWrite -1}
		lin_vx {Type I LastRead 2 FirstWrite -1}
		lin_vy {Type I LastRead 3 FirstWrite -1}
		lin_omega {Type I LastRead 4 FirstWrite -1}
		lin_delta {Type I LastRead 33 FirstWrite -1}
		uk0 {Type I LastRead 30 FirstWrite -1}
		uk1 {Type I LastRead 25 FirstWrite -1}
		lin_delta_k {Type I LastRead 0 FirstWrite -1}
		next_ey {Type I LastRead 5 FirstWrite -1}
		next_epsi {Type I LastRead 10 FirstWrite -1}
		next_vx {Type I LastRead 15 FirstWrite -1}
		next_vy {Type I LastRead 20 FirstWrite -1}
		next_omega {Type I LastRead 25 FirstWrite -1}
		next_delta {Type I LastRead 33 FirstWrite -1}
		sd_1 {Type O LastRead -1 FirstWrite 28}
		sd_1_offset {Type I LastRead 28 FirstWrite -1}}
	affine_b0_term_hls {
		b0_coeff_raw {Type I LastRead 0 FirstWrite -1}
		lin_delta_k_raw {Type I LastRead 0 FirstWrite -1}}
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
	reset_admm_state_hls_Pipeline_VITIS_LOOP_364_1 {
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
	reset_admm_state_hls_Pipeline_VITIS_LOOP_372_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1183_1 {
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
		z_x {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1195_4 {
		y_u_1 {Type O LastRead -1 FirstWrite 0}
		y_u {Type O LastRead -1 FirstWrite 0}
		z_u_1 {Type O LastRead -1 FirstWrite 0}
		z_u {Type O LastRead -1 FirstWrite 0}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1203_6 {
		y_x_7 {Type O LastRead -1 FirstWrite 1}
		y_x_6 {Type O LastRead -1 FirstWrite 1}
		y_x_5 {Type O LastRead -1 FirstWrite 1}
		y_x_4 {Type O LastRead -1 FirstWrite 1}
		y_x_3 {Type O LastRead -1 FirstWrite 1}
		y_x_2 {Type O LastRead -1 FirstWrite 1}
		y_x_1 {Type O LastRead -1 FirstWrite 1}
		y_x {Type O LastRead -1 FirstWrite 1}
		z_x_7 {Type O LastRead -1 FirstWrite 1}
		z_x_6 {Type O LastRead -1 FirstWrite 1}
		z_x_5 {Type O LastRead -1 FirstWrite 1}
		z_x_4 {Type O LastRead -1 FirstWrite 1}
		z_x_3 {Type O LastRead -1 FirstWrite 1}
		z_x_2 {Type O LastRead -1 FirstWrite 1}
		z_x_1 {Type O LastRead -1 FirstWrite 1}
		z_x {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9 {
		y_u_1 {Type O LastRead -1 FirstWrite 1}
		y_u {Type O LastRead -1 FirstWrite 1}
		z_u_1 {Type O LastRead -1 FirstWrite 1}
		z_u {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1337_19 {
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
		sext_ln1337 {Type I LastRead 0 FirstWrite -1}
		lnorm_da_out {Type O LastRead -1 FirstWrite 5}
		znorm_da_out {Type O LastRead -1 FirstWrite 5}
		dual_da_out {Type O LastRead -1 FirstWrite 5}
		primal_da_out {Type O LastRead -1 FirstWrite 5}
		lnorm_ey_out {Type O LastRead -1 FirstWrite 5}
		znorm_ey_out {Type O LastRead -1 FirstWrite 5}
		dual_ey_out {Type O LastRead -1 FirstWrite 5}
		primal_ey_out {Type O LastRead -1 FirstWrite 5}
		x_norm_out {Type O LastRead -1 FirstWrite 5}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1395_20 {
		z_u_1 {Type IO LastRead 1 FirstWrite 2}
		z_u {Type IO LastRead 1 FirstWrite 2}
		y_u_1 {Type IO LastRead 0 FirstWrite 2}
		y_u {Type IO LastRead 0 FirstWrite 2}
		sol_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}
		sext_ln1395 {Type I LastRead 0 FirstWrite -1}
		step_data_5 {Type I LastRead 0 FirstWrite -1}
		lnorm_u1_out {Type O LastRead -1 FirstWrite 5}
		znorm_u1_out {Type O LastRead -1 FirstWrite 5}
		dual_u1_out {Type O LastRead -1 FirstWrite 5}
		primal_u1_out {Type O LastRead -1 FirstWrite 5}
		lnorm_u0_out {Type O LastRead -1 FirstWrite 5}
		znorm_u0_out {Type O LastRead -1 FirstWrite 5}
		dual_u0_out {Type O LastRead -1 FirstWrite 5}
		primal_u0_out {Type O LastRead -1 FirstWrite 5}
		u_norm_out {Type O LastRead -1 FirstWrite 5}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1511_21 {
		y_x_5 {Type IO LastRead 1 FirstWrite 1}
		y_x {Type IO LastRead 0 FirstWrite 1}
		scale_rho_up {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1555_24 {
		z_x {Type I LastRead 0 FirstWrite -1}
		y_x {Type I LastRead 0 FirstWrite -1}
		z_x_1 {Type I LastRead 0 FirstWrite -1}
		y_x_1 {Type I LastRead 0 FirstWrite -1}
		z_x_2 {Type I LastRead 0 FirstWrite -1}
		y_x_2 {Type I LastRead 0 FirstWrite -1}
		z_x_3 {Type I LastRead 0 FirstWrite -1}
		y_x_3 {Type I LastRead 0 FirstWrite -1}
		z_x_4 {Type I LastRead 0 FirstWrite -1}
		y_x_4 {Type I LastRead 0 FirstWrite -1}
		z_x_5 {Type I LastRead 0 FirstWrite -1}
		y_x_5 {Type I LastRead 0 FirstWrite -1}
		z_x_6 {Type I LastRead 0 FirstWrite -1}
		y_x_6 {Type I LastRead 0 FirstWrite -1}
		z_x_7 {Type I LastRead 0 FirstWrite -1}
		y_x_7 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_x_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_1 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_2 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_3 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_4 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_5 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_6 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_x_7 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_1 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_2 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_3 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_4 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_5 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_6 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_x_7 {Type O LastRead -1 FirstWrite 1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1562_26 {
		z_u {Type I LastRead 0 FirstWrite -1}
		y_u {Type I LastRead 0 FirstWrite -1}
		z_u_1 {Type I LastRead 0 FirstWrite -1}
		y_u_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1525_22 {
		y_u_1 {Type IO LastRead 0 FirstWrite 1}
		y_u {Type IO LastRead 0 FirstWrite 1}
		scale_rho_u_up {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1246_12 {
		z_x_7 {Type O LastRead -1 FirstWrite 1}
		z_x_6 {Type O LastRead -1 FirstWrite 1}
		z_x_5 {Type O LastRead -1 FirstWrite 1}
		z_x_4 {Type O LastRead -1 FirstWrite 1}
		z_x_3 {Type O LastRead -1 FirstWrite 1}
		z_x_2 {Type O LastRead -1 FirstWrite 1}
		z_x_1 {Type O LastRead -1 FirstWrite 1}
		z_x {Type O LastRead -1 FirstWrite 2}
		sol_x {Type I LastRead 1 FirstWrite -1}
		sol_x_1 {Type I LastRead 0 FirstWrite -1}
		sol_x_2 {Type I LastRead 0 FirstWrite -1}
		sol_x_3 {Type I LastRead 0 FirstWrite -1}
		sol_x_4 {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		sol_x_6 {Type I LastRead 0 FirstWrite -1}
		sol_x_7 {Type I LastRead 0 FirstWrite -1}
		step_data_3 {Type I LastRead 0 FirstWrite -1}
		step_data_4 {Type I LastRead 1 FirstWrite -1}
		terminal_wall_x_lb_con_reload {Type I LastRead 0 FirstWrite -1}
		terminal_wall_x_ub_con_reload {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1282_14 {
		z_u_1 {Type O LastRead -1 FirstWrite 1}
		z_u {Type O LastRead -1 FirstWrite 1}
		step_data_5 {Type I LastRead 0 FirstWrite -1}
		sol_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16 {
		y_x_5 {Type O LastRead -1 FirstWrite 1}
		y_x {Type O LastRead -1 FirstWrite 1}
		sol_x {Type I LastRead 0 FirstWrite -1}
		z_x {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		z_x_5 {Type I LastRead 0 FirstWrite -1}}
	mpc_compute_hls_Pipeline_VITIS_LOOP_1305_17 {
		y_u_1 {Type O LastRead -1 FirstWrite 1}
		y_u {Type O LastRead -1 FirstWrite 1}
		sol_u {Type I LastRead 0 FirstWrite -1}
		z_u {Type I LastRead 0 FirstWrite -1}
		sol_u_1 {Type I LastRead 0 FirstWrite -1}
		z_u_1 {Type I LastRead 0 FirstWrite -1}}
	mpc_persist_writeback_hls {
		prev_steer_rate {Type I LastRead 0 FirstWrite -1}
		prev_accel {Type I LastRead 0 FirstWrite -1}
		prev_curvature {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type O LastRead -1 FirstWrite 0}}
	fp_recip {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "161", "Max" : "151272"}
	, {"Name" : "Interval", "Min" : "162", "Max" : "151273"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	gmem0 { m_axi {  { m_axi_gmem0_AWVALID VALID 1 1 }  { m_axi_gmem0_AWREADY READY 0 1 }  { m_axi_gmem0_AWADDR ADDR 1 64 }  { m_axi_gmem0_AWID ID 1 1 }  { m_axi_gmem0_AWLEN SIZE 1 8 }  { m_axi_gmem0_AWSIZE BURST 1 3 }  { m_axi_gmem0_AWBURST LOCK 1 2 }  { m_axi_gmem0_AWLOCK CACHE 1 2 }  { m_axi_gmem0_AWCACHE PROT 1 4 }  { m_axi_gmem0_AWPROT QOS 1 3 }  { m_axi_gmem0_AWQOS REGION 1 4 }  { m_axi_gmem0_AWREGION USER 1 4 }  { m_axi_gmem0_AWUSER DATA 1 1 }  { m_axi_gmem0_WVALID VALID 1 1 }  { m_axi_gmem0_WREADY READY 0 1 }  { m_axi_gmem0_WDATA FIFONUM 1 512 }  { m_axi_gmem0_WSTRB STRB 1 64 }  { m_axi_gmem0_WLAST LAST 1 1 }  { m_axi_gmem0_WID ID 1 1 }  { m_axi_gmem0_WUSER DATA 1 1 }  { m_axi_gmem0_ARVALID VALID 1 1 }  { m_axi_gmem0_ARREADY READY 0 1 }  { m_axi_gmem0_ARADDR ADDR 1 64 }  { m_axi_gmem0_ARID ID 1 1 }  { m_axi_gmem0_ARLEN SIZE 1 8 }  { m_axi_gmem0_ARSIZE BURST 1 3 }  { m_axi_gmem0_ARBURST LOCK 1 2 }  { m_axi_gmem0_ARLOCK CACHE 1 2 }  { m_axi_gmem0_ARCACHE PROT 1 4 }  { m_axi_gmem0_ARPROT QOS 1 3 }  { m_axi_gmem0_ARQOS REGION 1 4 }  { m_axi_gmem0_ARREGION USER 1 4 }  { m_axi_gmem0_ARUSER DATA 1 1 }  { m_axi_gmem0_RVALID VALID 0 1 }  { m_axi_gmem0_RREADY READY 1 1 }  { m_axi_gmem0_RDATA FIFONUM 0 512 }  { m_axi_gmem0_RLAST LAST 0 1 }  { m_axi_gmem0_RID ID 0 1 }  { m_axi_gmem0_RUSER DATA 0 1 }  { m_axi_gmem0_RRESP RESP 0 2 }  { m_axi_gmem0_BVALID VALID 0 1 }  { m_axi_gmem0_BREADY READY 1 1 }  { m_axi_gmem0_BRESP RESP 0 2 }  { m_axi_gmem0_BID ID 0 1 }  { m_axi_gmem0_BUSER DATA 0 1 } } }
	gmem1 { m_axi {  { m_axi_gmem1_AWVALID VALID 1 1 }  { m_axi_gmem1_AWREADY READY 0 1 }  { m_axi_gmem1_AWADDR ADDR 1 64 }  { m_axi_gmem1_AWID ID 1 1 }  { m_axi_gmem1_AWLEN SIZE 1 8 }  { m_axi_gmem1_AWSIZE BURST 1 3 }  { m_axi_gmem1_AWBURST LOCK 1 2 }  { m_axi_gmem1_AWLOCK CACHE 1 2 }  { m_axi_gmem1_AWCACHE PROT 1 4 }  { m_axi_gmem1_AWPROT QOS 1 3 }  { m_axi_gmem1_AWQOS REGION 1 4 }  { m_axi_gmem1_AWREGION USER 1 4 }  { m_axi_gmem1_AWUSER DATA 1 1 }  { m_axi_gmem1_WVALID VALID 1 1 }  { m_axi_gmem1_WREADY READY 0 1 }  { m_axi_gmem1_WDATA FIFONUM 1 128 }  { m_axi_gmem1_WSTRB STRB 1 16 }  { m_axi_gmem1_WLAST LAST 1 1 }  { m_axi_gmem1_WID ID 1 1 }  { m_axi_gmem1_WUSER DATA 1 1 }  { m_axi_gmem1_ARVALID VALID 1 1 }  { m_axi_gmem1_ARREADY READY 0 1 }  { m_axi_gmem1_ARADDR ADDR 1 64 }  { m_axi_gmem1_ARID ID 1 1 }  { m_axi_gmem1_ARLEN SIZE 1 8 }  { m_axi_gmem1_ARSIZE BURST 1 3 }  { m_axi_gmem1_ARBURST LOCK 1 2 }  { m_axi_gmem1_ARLOCK CACHE 1 2 }  { m_axi_gmem1_ARCACHE PROT 1 4 }  { m_axi_gmem1_ARPROT QOS 1 3 }  { m_axi_gmem1_ARQOS REGION 1 4 }  { m_axi_gmem1_ARREGION USER 1 4 }  { m_axi_gmem1_ARUSER DATA 1 1 }  { m_axi_gmem1_RVALID VALID 0 1 }  { m_axi_gmem1_RREADY READY 1 1 }  { m_axi_gmem1_RDATA FIFONUM 0 128 }  { m_axi_gmem1_RLAST LAST 0 1 }  { m_axi_gmem1_RID ID 0 1 }  { m_axi_gmem1_RUSER DATA 0 1 }  { m_axi_gmem1_RRESP RESP 0 2 }  { m_axi_gmem1_BVALID VALID 0 1 }  { m_axi_gmem1_BREADY READY 1 1 }  { m_axi_gmem1_BRESP RESP 0 2 }  { m_axi_gmem1_BID ID 0 1 }  { m_axi_gmem1_BUSER DATA 0 1 } } }
}

set maxi_interface_dict [dict create]
dict set maxi_interface_dict gmem0 { CHANNEL_NUM 0 BUNDLE gmem0 NUM_READ_OUTSTANDING 16 NUM_WRITE_OUTSTANDING 16 MAX_READ_BURST_LENGTH 16 MAX_WRITE_BURST_LENGTH 16 READ_WRITE_MODE READ_ONLY}
dict set maxi_interface_dict gmem1 { CHANNEL_NUM 0 BUNDLE gmem1 NUM_READ_OUTSTANDING 16 NUM_WRITE_OUTSTANDING 16 MAX_READ_BURST_LENGTH 16 MAX_WRITE_BURST_LENGTH 16 READ_WRITE_MODE WRITE_ONLY}

# RTL port scheduling information:
set fifoSchedulingInfoList { 
}

# RTL bus port read request latency information:
set busReadReqLatencyList { 
	{ gmem0 64 }
	{ gmem1 64 }
}

# RTL bus port write response latency information:
set busWriteResLatencyList { 
	{ gmem0 64 }
	{ gmem1 64 }
}

# RTL array port load latency information:
set memoryLoadLatencyList { 
}
