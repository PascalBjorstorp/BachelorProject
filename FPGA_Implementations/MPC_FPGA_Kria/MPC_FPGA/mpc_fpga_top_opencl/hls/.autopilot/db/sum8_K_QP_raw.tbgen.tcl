set moduleName sum8_K_QP_raw
set isTopModule 0
set isCombinational 1
set isDatapathOnly 0
set isPipelined 0
set isPipelined_legacy 0
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
set C_modelName {sum8_K_QP_raw}
set C_modelType { int 32 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ a0 int 32 regular  }
	{ a1 int 32 regular  }
	{ a2 int 32 regular  }
	{ a3 int 32 regular  }
	{ a4 int 32 regular  }
	{ a5 int 32 regular  }
	{ a6 int 32 regular  }
	{ a7 int 32 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "a0", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a1", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a2", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a3", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a4", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a5", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a6", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a7", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 32} ]}
# RTL Port declarations: 
set portNum 11
set portList { 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ a0 sc_in sc_lv 32 signal 0 } 
	{ a1 sc_in sc_lv 32 signal 1 } 
	{ a2 sc_in sc_lv 32 signal 2 } 
	{ a3 sc_in sc_lv 32 signal 3 } 
	{ a4 sc_in sc_lv 32 signal 4 } 
	{ a5 sc_in sc_lv 32 signal 5 } 
	{ a6 sc_in sc_lv 32 signal 6 } 
	{ a7 sc_in sc_lv 32 signal 7 } 
	{ ap_return sc_out sc_lv 32 signal -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
}
set NewPortList {[ 
	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "a0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a0", "role": "default" }} , 
 	{ "name": "a1", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a1", "role": "default" }} , 
 	{ "name": "a2", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a2", "role": "default" }} , 
 	{ "name": "a3", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a3", "role": "default" }} , 
 	{ "name": "a4", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a4", "role": "default" }} , 
 	{ "name": "a5", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a5", "role": "default" }} , 
 	{ "name": "a6", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a6", "role": "default" }} , 
 	{ "name": "a7", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a7", "role": "default" }} , 
 	{ "name": "ap_return", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ap_return", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	sum8_K_QP_raw {
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
	{"Name" : "Latency", "Min" : "0", "Max" : "0"}
	, {"Name" : "Interval", "Min" : "1", "Max" : "1"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	a0 { ap_none {  { a0 in_data 0 32 } } }
	a1 { ap_none {  { a1 in_data 0 32 } } }
	a2 { ap_none {  { a2 in_data 0 32 } } }
	a3 { ap_none {  { a3 in_data 0 32 } } }
	a4 { ap_none {  { a4 in_data 0 32 } } }
	a5 { ap_none {  { a5 in_data 0 32 } } }
	a6 { ap_none {  { a6 in_data 0 32 } } }
	a7 { ap_none {  { a7 in_data 0 32 } } }
}
