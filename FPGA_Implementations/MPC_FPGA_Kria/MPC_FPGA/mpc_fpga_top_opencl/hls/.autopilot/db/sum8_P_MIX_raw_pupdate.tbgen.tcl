set moduleName sum8_P_MIX_raw_pupdate
set isTopModule 0
set isCombinational 0
set isDatapathOnly 0
set isPipelined 1
set isPipelined_legacy 1
set pipeline_type function
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
set C_modelName {sum8_P_MIX_raw_pupdate}
set C_modelType { int 41 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ a0 int 41 regular  }
	{ a1 int 41 regular  }
	{ a2 int 41 regular  }
	{ a3 int 41 regular  }
	{ a4 int 41 regular  }
	{ a5 int 41 regular  }
	{ a6 int 44 regular  }
	{ a7 int 44 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "a0", "interface" : "wire", "bitwidth" : 41, "direction" : "READONLY"} , 
 	{ "Name" : "a1", "interface" : "wire", "bitwidth" : 41, "direction" : "READONLY"} , 
 	{ "Name" : "a2", "interface" : "wire", "bitwidth" : 41, "direction" : "READONLY"} , 
 	{ "Name" : "a3", "interface" : "wire", "bitwidth" : 41, "direction" : "READONLY"} , 
 	{ "Name" : "a4", "interface" : "wire", "bitwidth" : 41, "direction" : "READONLY"} , 
 	{ "Name" : "a5", "interface" : "wire", "bitwidth" : 41, "direction" : "READONLY"} , 
 	{ "Name" : "a6", "interface" : "wire", "bitwidth" : 44, "direction" : "READONLY"} , 
 	{ "Name" : "a7", "interface" : "wire", "bitwidth" : 44, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 41} ]}
# RTL Port declarations: 
set portNum 16
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ ap_ce sc_in sc_logic 1 ce -1 } 
	{ a0 sc_in sc_lv 41 signal 0 } 
	{ a1 sc_in sc_lv 41 signal 1 } 
	{ a2 sc_in sc_lv 41 signal 2 } 
	{ a3 sc_in sc_lv 41 signal 3 } 
	{ a4 sc_in sc_lv 41 signal 4 } 
	{ a5 sc_in sc_lv 41 signal 5 } 
	{ a6 sc_in sc_lv 44 signal 6 } 
	{ a7 sc_in sc_lv 44 signal 7 } 
	{ ap_return sc_out sc_lv 41 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "ap_ce", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "ce", "bundle":{"name": "ap_ce", "role": "default" }} , 
 	{ "name": "a0", "direction": "in", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "a0", "role": "default" }} , 
 	{ "name": "a1", "direction": "in", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "a1", "role": "default" }} , 
 	{ "name": "a2", "direction": "in", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "a2", "role": "default" }} , 
 	{ "name": "a3", "direction": "in", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "a3", "role": "default" }} , 
 	{ "name": "a4", "direction": "in", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "a4", "role": "default" }} , 
 	{ "name": "a5", "direction": "in", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "a5", "role": "default" }} , 
 	{ "name": "a6", "direction": "in", "datatype": "sc_lv", "bitwidth":44, "type": "signal", "bundle":{"name": "a6", "role": "default" }} , 
 	{ "name": "a7", "direction": "in", "datatype": "sc_lv", "bitwidth":44, "type": "signal", "bundle":{"name": "a7", "role": "default" }} , 
 	{ "name": "ap_return", "direction": "out", "datatype": "sc_lv", "bitwidth":41, "type": "signal", "bundle":{"name": "ap_return", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	sum8_P_MIX_raw_pupdate {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "1", "Max" : "1"}
	, {"Name" : "Interval", "Min" : "1", "Max" : "1"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	a0 { ap_none {  { a0 in_data 0 41 } } }
	a1 { ap_none {  { a1 in_data 0 41 } } }
	a2 { ap_none {  { a2 in_data 0 41 } } }
	a3 { ap_none {  { a3 in_data 0 41 } } }
	a4 { ap_none {  { a4 in_data 0 41 } } }
	a5 { ap_none {  { a5 in_data 0 41 } } }
	a6 { ap_none {  { a6 in_data 0 44 } } }
	a7 { ap_none {  { a7 in_data 0 44 } } }
}
