set moduleName riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40
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
set C_modelName {riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ G_36 int 6 regular  }
	{ sext_ln288_30 int 17 regular  }
	{ sext_ln261_2 int 17 regular  }
	{ p_new_6_out int 27 regular {pointer 2}  }
	{ p_new_out int 27 regular {pointer 2}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "G_36", "interface" : "wire", "bitwidth" : 6, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln288_30", "interface" : "wire", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln261_2", "interface" : "wire", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "p_new_6_out", "interface" : "wire", "bitwidth" : 27, "direction" : "READWRITE"} , 
 	{ "Name" : "p_new_out", "interface" : "wire", "bitwidth" : 27, "direction" : "READWRITE"} ]}
# RTL Port declarations: 
set portNum 15
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ G_36 sc_in sc_lv 6 signal 0 } 
	{ sext_ln288_30 sc_in sc_lv 17 signal 1 } 
	{ sext_ln261_2 sc_in sc_lv 17 signal 2 } 
	{ p_new_6_out_i sc_in sc_lv 27 signal 3 } 
	{ p_new_6_out_o sc_out sc_lv 27 signal 3 } 
	{ p_new_6_out_o_ap_vld sc_out sc_logic 1 outvld 3 } 
	{ p_new_out_i sc_in sc_lv 27 signal 4 } 
	{ p_new_out_o sc_out sc_lv 27 signal 4 } 
	{ p_new_out_o_ap_vld sc_out sc_logic 1 outvld 4 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "G_36", "direction": "in", "datatype": "sc_lv", "bitwidth":6, "type": "signal", "bundle":{"name": "G_36", "role": "default" }} , 
 	{ "name": "sext_ln288_30", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "sext_ln288_30", "role": "default" }} , 
 	{ "name": "sext_ln261_2", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "sext_ln261_2", "role": "default" }} , 
 	{ "name": "p_new_6_out_i", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "p_new_6_out", "role": "i" }} , 
 	{ "name": "p_new_6_out_o", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "p_new_6_out", "role": "o" }} , 
 	{ "name": "p_new_6_out_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_6_out", "role": "o_ap_vld" }} , 
 	{ "name": "p_new_out_i", "direction": "in", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "p_new_out", "role": "i" }} , 
 	{ "name": "p_new_out_o", "direction": "out", "datatype": "sc_lv", "bitwidth":27, "type": "signal", "bundle":{"name": "p_new_out", "role": "o" }} , 
 	{ "name": "p_new_out_o_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_out", "role": "o_ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
	riccati_backward_pass_Pipeline_VITIS_LOOP_1048_40 {
		G_36 {Type I LastRead 0 FirstWrite -1}
		sext_ln288_30 {Type I LastRead 0 FirstWrite -1}
		sext_ln261_2 {Type I LastRead 0 FirstWrite -1}
		p_new_6_out {Type IO LastRead 1 FirstWrite 0}
		p_new_out {Type IO LastRead 1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "4", "Max" : "4"}
	, {"Name" : "Interval", "Min" : "3", "Max" : "3"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	G_36 { ap_none {  { G_36 in_data 0 6 } } }
	sext_ln288_30 { ap_none {  { sext_ln288_30 in_data 0 17 } } }
	sext_ln261_2 { ap_none {  { sext_ln261_2 in_data 0 17 } } }
	p_new_6_out { ap_ovld {  { p_new_6_out_i in_data 0 27 }  { p_new_6_out_o out_data 1 27 }  { p_new_6_out_o_ap_vld out_vld 1 1 } } }
	p_new_out { ap_ovld {  { p_new_out_i in_data 0 27 }  { p_new_out_o out_data 1 27 }  { p_new_out_o_ap_vld out_vld 1 1 } } }
}
