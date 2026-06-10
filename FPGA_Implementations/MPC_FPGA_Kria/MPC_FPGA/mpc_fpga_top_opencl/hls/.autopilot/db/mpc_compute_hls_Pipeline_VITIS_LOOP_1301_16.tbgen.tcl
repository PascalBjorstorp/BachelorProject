set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16
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
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict y_x_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_x { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict sol_x { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict z_x { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict sol_x_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict z_x_5 { MEM_WIDTH 32 MEM_SIZE 84 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ y_x_5 int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ y_x int 32 regular {array 21 { 3 0 } 0 1 bus  }  }
	{ sol_x int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ z_x int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ sol_x_5 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
	{ z_x_5 int 32 regular {array 21 { 1 3 } 1 1 bus  }  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "y_x_5", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_x", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "sol_x", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "z_x", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sol_x_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "z_x_5", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY"} ]}
# RTL Port declarations: 
set portNum 26
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ y_x_5_address1 sc_out sc_lv 5 signal 0 } 
	{ y_x_5_ce1 sc_out sc_logic 1 signal 0 } 
	{ y_x_5_we1 sc_out sc_logic 1 signal 0 } 
	{ y_x_5_d1 sc_out sc_lv 32 signal 0 } 
	{ y_x_address1 sc_out sc_lv 5 signal 1 } 
	{ y_x_ce1 sc_out sc_logic 1 signal 1 } 
	{ y_x_we1 sc_out sc_logic 1 signal 1 } 
	{ y_x_d1 sc_out sc_lv 32 signal 1 } 
	{ sol_x_address0 sc_out sc_lv 5 signal 2 } 
	{ sol_x_ce0 sc_out sc_logic 1 signal 2 } 
	{ sol_x_q0 sc_in sc_lv 32 signal 2 } 
	{ z_x_address0 sc_out sc_lv 5 signal 3 } 
	{ z_x_ce0 sc_out sc_logic 1 signal 3 } 
	{ z_x_q0 sc_in sc_lv 32 signal 3 } 
	{ sol_x_5_address0 sc_out sc_lv 5 signal 4 } 
	{ sol_x_5_ce0 sc_out sc_logic 1 signal 4 } 
	{ sol_x_5_q0 sc_in sc_lv 32 signal 4 } 
	{ z_x_5_address0 sc_out sc_lv 5 signal 5 } 
	{ z_x_5_ce0 sc_out sc_logic 1 signal 5 } 
	{ z_x_5_q0 sc_in sc_lv 32 signal 5 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "y_x_5_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x_5", "role": "address1" }} , 
 	{ "name": "y_x_5_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "ce1" }} , 
 	{ "name": "y_x_5_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x_5", "role": "we1" }} , 
 	{ "name": "y_x_5_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_x_5", "role": "d1" }} , 
 	{ "name": "y_x_address1", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_x", "role": "address1" }} , 
 	{ "name": "y_x_ce1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "ce1" }} , 
 	{ "name": "y_x_we1", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_x", "role": "we1" }} , 
 	{ "name": "y_x_d1", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_x", "role": "d1" }} , 
 	{ "name": "sol_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x", "role": "address0" }} , 
 	{ "name": "sol_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x", "role": "ce0" }} , 
 	{ "name": "sol_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x", "role": "q0" }} , 
 	{ "name": "z_x_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x", "role": "address0" }} , 
 	{ "name": "z_x_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x", "role": "ce0" }} , 
 	{ "name": "z_x_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x", "role": "q0" }} , 
 	{ "name": "sol_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "sol_x_5", "role": "address0" }} , 
 	{ "name": "sol_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "sol_x_5", "role": "ce0" }} , 
 	{ "name": "sol_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sol_x_5", "role": "q0" }} , 
 	{ "name": "z_x_5_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_x_5", "role": "address0" }} , 
 	{ "name": "z_x_5_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_x_5", "role": "ce0" }} , 
 	{ "name": "z_x_5_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_x_5", "role": "q0" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_compute_hls_Pipeline_VITIS_LOOP_1301_16 {
		y_x_5 {Type O LastRead -1 FirstWrite 1}
		y_x {Type O LastRead -1 FirstWrite 1}
		sol_x {Type I LastRead 0 FirstWrite -1}
		z_x {Type I LastRead 0 FirstWrite -1}
		sol_x_5 {Type I LastRead 0 FirstWrite -1}
		z_x_5 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "23", "Max" : "23"}
	, {"Name" : "Interval", "Min" : "22", "Max" : "22"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	y_x_5 { ap_memory {  { y_x_5_address1 MemPortADDR2 1 5 }  { y_x_5_ce1 MemPortCE2 1 1 }  { y_x_5_we1 MemPortWE2 1 1 }  { y_x_5_d1 MemPortDIN2 1 32 } } }
	y_x { ap_memory {  { y_x_address1 MemPortADDR2 1 5 }  { y_x_ce1 MemPortCE2 1 1 }  { y_x_we1 MemPortWE2 1 1 }  { y_x_d1 MemPortDIN2 1 32 } } }
	sol_x { ap_memory {  { sol_x_address0 mem_address 1 5 }  { sol_x_ce0 mem_ce 1 1 }  { sol_x_q0 mem_dout 0 32 } } }
	z_x { ap_memory {  { z_x_address0 mem_address 1 5 }  { z_x_ce0 mem_ce 1 1 }  { z_x_q0 mem_dout 0 32 } } }
	sol_x_5 { ap_memory {  { sol_x_5_address0 mem_address 1 5 }  { sol_x_5_ce0 mem_ce 1 1 }  { sol_x_5_q0 mem_dout 0 32 } } }
	z_x_5 { ap_memory {  { z_x_5_address0 mem_address 1 5 }  { z_x_5_ce0 mem_ce 1 1 }  { z_x_5_q0 mem_dout 0 32 } } }
}
