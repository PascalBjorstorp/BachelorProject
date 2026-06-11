set moduleName compute_window_min_hls
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
set cdfgNum 74
set C_modelName {compute_window_min_hls}
set C_modelType { int 26 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ ref_wall_0_val int 26 regular  }
	{ ref_wall_1_val int 26 regular  }
	{ ref_wall_2_val int 26 regular  }
	{ ref_wall_3_val int 26 regular  }
	{ ref_wall_4_val int 26 regular  }
	{ ref_wall_5_val int 26 regular  }
	{ ref_wall_6_val int 26 regular  }
	{ ref_wall_7_val int 26 regular  }
	{ ref_wall_8_val int 26 regular  }
	{ ref_wall_9_val int 26 regular  }
	{ ref_wall_10_val int 26 regular  }
	{ ref_wall_11_val int 26 regular  }
	{ ref_wall_12_val int 26 regular  }
	{ ref_wall_13_val int 26 regular  }
	{ ref_wall_14_val int 26 regular  }
	{ ref_wall_15_val int 26 regular  }
	{ ref_wall_16_val int 26 regular  }
	{ ref_wall_17_val int 26 regular  }
	{ ref_wall_18_val int 26 regular  }
	{ ref_wall_19_val int 26 regular  }
	{ center_idx int 5 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "ref_wall_0_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_1_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_2_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_3_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_4_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_5_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_6_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_7_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_8_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_9_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_10_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_11_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_12_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_13_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_14_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_15_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_16_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_17_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_18_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ref_wall_19_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "center_idx", "interface" : "wire", "bitwidth" : 5, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 26} ]}
# RTL Port declarations: 
set portNum 28
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ ref_wall_0_val sc_in sc_lv 26 signal 0 } 
	{ ref_wall_1_val sc_in sc_lv 26 signal 1 } 
	{ ref_wall_2_val sc_in sc_lv 26 signal 2 } 
	{ ref_wall_3_val sc_in sc_lv 26 signal 3 } 
	{ ref_wall_4_val sc_in sc_lv 26 signal 4 } 
	{ ref_wall_5_val sc_in sc_lv 26 signal 5 } 
	{ ref_wall_6_val sc_in sc_lv 26 signal 6 } 
	{ ref_wall_7_val sc_in sc_lv 26 signal 7 } 
	{ ref_wall_8_val sc_in sc_lv 26 signal 8 } 
	{ ref_wall_9_val sc_in sc_lv 26 signal 9 } 
	{ ref_wall_10_val sc_in sc_lv 26 signal 10 } 
	{ ref_wall_11_val sc_in sc_lv 26 signal 11 } 
	{ ref_wall_12_val sc_in sc_lv 26 signal 12 } 
	{ ref_wall_13_val sc_in sc_lv 26 signal 13 } 
	{ ref_wall_14_val sc_in sc_lv 26 signal 14 } 
	{ ref_wall_15_val sc_in sc_lv 26 signal 15 } 
	{ ref_wall_16_val sc_in sc_lv 26 signal 16 } 
	{ ref_wall_17_val sc_in sc_lv 26 signal 17 } 
	{ ref_wall_18_val sc_in sc_lv 26 signal 18 } 
	{ ref_wall_19_val sc_in sc_lv 26 signal 19 } 
	{ center_idx sc_in sc_lv 5 signal 20 } 
	{ ap_return sc_out sc_lv 26 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "ref_wall_0_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_0_val", "role": "default" }} , 
 	{ "name": "ref_wall_1_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_1_val", "role": "default" }} , 
 	{ "name": "ref_wall_2_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_2_val", "role": "default" }} , 
 	{ "name": "ref_wall_3_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_3_val", "role": "default" }} , 
 	{ "name": "ref_wall_4_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_4_val", "role": "default" }} , 
 	{ "name": "ref_wall_5_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_5_val", "role": "default" }} , 
 	{ "name": "ref_wall_6_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_6_val", "role": "default" }} , 
 	{ "name": "ref_wall_7_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_7_val", "role": "default" }} , 
 	{ "name": "ref_wall_8_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_8_val", "role": "default" }} , 
 	{ "name": "ref_wall_9_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_9_val", "role": "default" }} , 
 	{ "name": "ref_wall_10_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_10_val", "role": "default" }} , 
 	{ "name": "ref_wall_11_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_11_val", "role": "default" }} , 
 	{ "name": "ref_wall_12_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_12_val", "role": "default" }} , 
 	{ "name": "ref_wall_13_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_13_val", "role": "default" }} , 
 	{ "name": "ref_wall_14_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_14_val", "role": "default" }} , 
 	{ "name": "ref_wall_15_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_15_val", "role": "default" }} , 
 	{ "name": "ref_wall_16_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_16_val", "role": "default" }} , 
 	{ "name": "ref_wall_17_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_17_val", "role": "default" }} , 
 	{ "name": "ref_wall_18_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_18_val", "role": "default" }} , 
 	{ "name": "ref_wall_19_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ref_wall_19_val", "role": "default" }} , 
 	{ "name": "center_idx", "direction": "in", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "center_idx", "role": "default" }} , 
 	{ "name": "ap_return", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		center_idx {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "1", "Max" : "1"}
	, {"Name" : "Interval", "Min" : "1", "Max" : "1"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	ref_wall_0_val { ap_none {  { ref_wall_0_val in_data 0 26 } } }
	ref_wall_1_val { ap_none {  { ref_wall_1_val in_data 0 26 } } }
	ref_wall_2_val { ap_none {  { ref_wall_2_val in_data 0 26 } } }
	ref_wall_3_val { ap_none {  { ref_wall_3_val in_data 0 26 } } }
	ref_wall_4_val { ap_none {  { ref_wall_4_val in_data 0 26 } } }
	ref_wall_5_val { ap_none {  { ref_wall_5_val in_data 0 26 } } }
	ref_wall_6_val { ap_none {  { ref_wall_6_val in_data 0 26 } } }
	ref_wall_7_val { ap_none {  { ref_wall_7_val in_data 0 26 } } }
	ref_wall_8_val { ap_none {  { ref_wall_8_val in_data 0 26 } } }
	ref_wall_9_val { ap_none {  { ref_wall_9_val in_data 0 26 } } }
	ref_wall_10_val { ap_none {  { ref_wall_10_val in_data 0 26 } } }
	ref_wall_11_val { ap_none {  { ref_wall_11_val in_data 0 26 } } }
	ref_wall_12_val { ap_none {  { ref_wall_12_val in_data 0 26 } } }
	ref_wall_13_val { ap_none {  { ref_wall_13_val in_data 0 26 } } }
	ref_wall_14_val { ap_none {  { ref_wall_14_val in_data 0 26 } } }
	ref_wall_15_val { ap_none {  { ref_wall_15_val in_data 0 26 } } }
	ref_wall_16_val { ap_none {  { ref_wall_16_val in_data 0 26 } } }
	ref_wall_17_val { ap_none {  { ref_wall_17_val in_data 0 26 } } }
	ref_wall_18_val { ap_none {  { ref_wall_18_val in_data 0 26 } } }
	ref_wall_19_val { ap_none {  { ref_wall_19_val in_data 0 26 } } }
	center_idx { ap_none {  { center_idx in_data 0 5 } } }
}
