set moduleName fp_frenet_rows01_fn
set isTopModule 0
set isCombinational 0
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
set cdfgNum 76
set C_modelName {fp_frenet_rows01_fn}
set C_modelType { int 156 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ sin_epsi int 26 regular  }
	{ cos_epsi int 26 regular  }
	{ vx int 26 regular  }
	{ vy int 26 regular  }
	{ kappa int 26 regular  }
	{ inv_denom int 23 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "sin_epsi", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "cos_epsi", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "vx", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "vy", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "kappa", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "inv_denom", "interface" : "wire", "bitwidth" : 23, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 156} ]}
# RTL Port declarations: 
set portNum 18
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ sin_epsi sc_in sc_lv 26 signal 0 } 
	{ cos_epsi sc_in sc_lv 26 signal 1 } 
	{ vx sc_in sc_lv 26 signal 2 } 
	{ vy sc_in sc_lv 26 signal 3 } 
	{ kappa sc_in sc_lv 26 signal 4 } 
	{ inv_denom sc_in sc_lv 23 signal 5 } 
	{ ap_return_0 sc_out sc_lv 26 signal -1 } 
	{ ap_return_1 sc_out sc_lv 26 signal -1 } 
	{ ap_return_2 sc_out sc_lv 26 signal -1 } 
	{ ap_return_3 sc_out sc_lv 26 signal -1 } 
	{ ap_return_4 sc_out sc_lv 26 signal -1 } 
	{ ap_return_5 sc_out sc_lv 26 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "sin_epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sin_epsi", "role": "default" }} , 
 	{ "name": "cos_epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "cos_epsi", "role": "default" }} , 
 	{ "name": "vx", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "vx", "role": "default" }} , 
 	{ "name": "vy", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "vy", "role": "default" }} , 
 	{ "name": "kappa", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "kappa", "role": "default" }} , 
 	{ "name": "inv_denom", "direction": "in", "datatype": "sc_lv", "bitwidth":23, "type": "signal", "bundle":{"name": "inv_denom", "role": "default" }} , 
 	{ "name": "ap_return_0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_0", "role": "default" }} , 
 	{ "name": "ap_return_1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_1", "role": "default" }} , 
 	{ "name": "ap_return_2", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_2", "role": "default" }} , 
 	{ "name": "ap_return_3", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_3", "role": "default" }} , 
 	{ "name": "ap_return_4", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_4", "role": "default" }} , 
 	{ "name": "ap_return_5", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_5", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	fp_frenet_rows01_fn {
		sin_epsi {Type I LastRead 6 FirstWrite -1}
		cos_epsi {Type I LastRead 2 FirstWrite -1}
		vx {Type I LastRead 3 FirstWrite -1}
		vy {Type I LastRead 6 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		inv_denom {Type I LastRead 9 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "18", "Max" : "18"}
	, {"Name" : "Interval", "Min" : "19", "Max" : "19"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	sin_epsi { ap_none {  { sin_epsi in_data 0 26 } } }
	cos_epsi { ap_none {  { cos_epsi in_data 0 26 } } }
	vx { ap_none {  { vx in_data 0 26 } } }
	vy { ap_none {  { vy in_data 0 26 } } }
	kappa { ap_none {  { kappa in_data 0 26 } } }
	inv_denom { ap_none {  { inv_denom in_data 0 23 } } }
}
