set moduleName compute_stage_shared_products
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
set C_modelName {compute_stage_shared_products}
set C_modelType { int 294 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ tr_dFyf_dvx_val int 21 regular  }
	{ tr_dFyf_dvy_val int 21 regular  }
	{ tr_dFyf_dom_val int 21 regular  }
	{ tr_dFyr_dvx_val int 21 regular  }
	{ tr_dFyr_dvy_val int 21 regular  }
	{ tr_dFyr_dom_val int 21 regular  }
	{ p_0_0_05039 int 21 regular  }
	{ p_0_0_05038 int 21 regular  }
	{ conv5_i_i_i_i465 int 21 regular  }
	{ conv5_i_i_i_i629 int 21 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "tr_dFyf_dvx_val", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "tr_dFyf_dvy_val", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "tr_dFyf_dom_val", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "tr_dFyr_dvx_val", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "tr_dFyr_dvy_val", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "tr_dFyr_dom_val", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "p_0_0_05039", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "p_0_0_05038", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "conv5_i_i_i_i465", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "conv5_i_i_i_i629", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 294} ]}
# RTL Port declarations: 
set portNum 30
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ tr_dFyf_dvx_val sc_in sc_lv 21 signal 0 } 
	{ tr_dFyf_dvy_val sc_in sc_lv 21 signal 1 } 
	{ tr_dFyf_dom_val sc_in sc_lv 21 signal 2 } 
	{ tr_dFyr_dvx_val sc_in sc_lv 21 signal 3 } 
	{ tr_dFyr_dvy_val sc_in sc_lv 21 signal 4 } 
	{ tr_dFyr_dom_val sc_in sc_lv 21 signal 5 } 
	{ p_0_0_05039 sc_in sc_lv 21 signal 6 } 
	{ p_0_0_05038 sc_in sc_lv 21 signal 7 } 
	{ conv5_i_i_i_i465 sc_in sc_lv 21 signal 8 } 
	{ conv5_i_i_i_i629 sc_in sc_lv 21 signal 9 } 
	{ ap_return_0 sc_out sc_lv 21 signal -1 } 
	{ ap_return_1 sc_out sc_lv 21 signal -1 } 
	{ ap_return_2 sc_out sc_lv 21 signal -1 } 
	{ ap_return_3 sc_out sc_lv 21 signal -1 } 
	{ ap_return_4 sc_out sc_lv 21 signal -1 } 
	{ ap_return_5 sc_out sc_lv 21 signal -1 } 
	{ ap_return_6 sc_out sc_lv 21 signal -1 } 
	{ ap_return_7 sc_out sc_lv 21 signal -1 } 
	{ ap_return_8 sc_out sc_lv 21 signal -1 } 
	{ ap_return_9 sc_out sc_lv 21 signal -1 } 
	{ ap_return_10 sc_out sc_lv 21 signal -1 } 
	{ ap_return_11 sc_out sc_lv 21 signal -1 } 
	{ ap_return_12 sc_out sc_lv 21 signal -1 } 
	{ ap_return_13 sc_out sc_lv 21 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "tr_dFyf_dvx_val", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "tr_dFyf_dvx_val", "role": "default" }} , 
 	{ "name": "tr_dFyf_dvy_val", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "tr_dFyf_dvy_val", "role": "default" }} , 
 	{ "name": "tr_dFyf_dom_val", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "tr_dFyf_dom_val", "role": "default" }} , 
 	{ "name": "tr_dFyr_dvx_val", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "tr_dFyr_dvx_val", "role": "default" }} , 
 	{ "name": "tr_dFyr_dvy_val", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "tr_dFyr_dvy_val", "role": "default" }} , 
 	{ "name": "tr_dFyr_dom_val", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "tr_dFyr_dom_val", "role": "default" }} , 
 	{ "name": "p_0_0_05039", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "p_0_0_05039", "role": "default" }} , 
 	{ "name": "p_0_0_05038", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "p_0_0_05038", "role": "default" }} , 
 	{ "name": "conv5_i_i_i_i465", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "conv5_i_i_i_i465", "role": "default" }} , 
 	{ "name": "conv5_i_i_i_i629", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "conv5_i_i_i_i629", "role": "default" }} , 
 	{ "name": "ap_return_0", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_0", "role": "default" }} , 
 	{ "name": "ap_return_1", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_1", "role": "default" }} , 
 	{ "name": "ap_return_2", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_2", "role": "default" }} , 
 	{ "name": "ap_return_3", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_3", "role": "default" }} , 
 	{ "name": "ap_return_4", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_4", "role": "default" }} , 
 	{ "name": "ap_return_5", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_5", "role": "default" }} , 
 	{ "name": "ap_return_6", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_6", "role": "default" }} , 
 	{ "name": "ap_return_7", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_7", "role": "default" }} , 
 	{ "name": "ap_return_8", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_8", "role": "default" }} , 
 	{ "name": "ap_return_9", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_9", "role": "default" }} , 
 	{ "name": "ap_return_10", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_10", "role": "default" }} , 
 	{ "name": "ap_return_11", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_11", "role": "default" }} , 
 	{ "name": "ap_return_12", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_12", "role": "default" }} , 
 	{ "name": "ap_return_13", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_13", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	compute_stage_shared_products {
		tr_dFyf_dvx_val {Type I LastRead 0 FirstWrite -1}
		tr_dFyf_dvy_val {Type I LastRead 1 FirstWrite -1}
		tr_dFyf_dom_val {Type I LastRead 2 FirstWrite -1}
		tr_dFyr_dvx_val {Type I LastRead 2 FirstWrite -1}
		tr_dFyr_dvy_val {Type I LastRead 3 FirstWrite -1}
		tr_dFyr_dom_val {Type I LastRead 4 FirstWrite -1}
		p_0_0_05039 {Type I LastRead 0 FirstWrite -1}
		p_0_0_05038 {Type I LastRead 3 FirstWrite -1}
		conv5_i_i_i_i465 {Type I LastRead 1 FirstWrite -1}
		conv5_i_i_i_i629 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "10", "Max" : "10"}
	, {"Name" : "Interval", "Min" : "11", "Max" : "11"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	tr_dFyf_dvx_val { ap_none {  { tr_dFyf_dvx_val in_data 0 21 } } }
	tr_dFyf_dvy_val { ap_none {  { tr_dFyf_dvy_val in_data 0 21 } } }
	tr_dFyf_dom_val { ap_none {  { tr_dFyf_dom_val in_data 0 21 } } }
	tr_dFyr_dvx_val { ap_none {  { tr_dFyr_dvx_val in_data 0 21 } } }
	tr_dFyr_dvy_val { ap_none {  { tr_dFyr_dvy_val in_data 0 21 } } }
	tr_dFyr_dom_val { ap_none {  { tr_dFyr_dom_val in_data 0 21 } } }
	p_0_0_05039 { ap_none {  { p_0_0_05039 in_data 0 21 } } }
	p_0_0_05038 { ap_none {  { p_0_0_05038 in_data 0 21 } } }
	conv5_i_i_i_i465 { ap_none {  { conv5_i_i_i_i465 in_data 0 21 } } }
	conv5_i_i_i_i629 { ap_none {  { conv5_i_i_i_i629 in_data 0 21 } } }
}
