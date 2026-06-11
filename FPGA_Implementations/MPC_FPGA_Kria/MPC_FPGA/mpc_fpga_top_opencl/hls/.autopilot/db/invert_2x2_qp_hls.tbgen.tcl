set moduleName invert_2x2_qp_hls
set isTopModule 0
set isCombinational 0
set isDatapathOnly 0
set isPipelined 0
set isPipelined_legacy 0
set pipeline_type none
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
set C_modelName {invert_2x2_qp_hls}
set C_modelType { int 136 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ p_read int 26 regular  }
	{ p_read1 int 26 regular  }
	{ p_read3 int 26 regular  }
	{ p_read4 int 26 regular  }
	{ p_read5 int 26 regular  }
	{ p_read6 int 26 regular  }
	{ p_read7 int 26 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "p_read", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read1", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read3", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read4", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read5", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read6", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_read7", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 136} ]}
# RTL Port declarations: 
set portNum 28
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ p_read sc_in sc_lv 26 signal 0 } 
	{ p_read1 sc_in sc_lv 26 signal 1 } 
	{ p_read3 sc_in sc_lv 26 signal 2 } 
	{ p_read4 sc_in sc_lv 26 signal 3 } 
	{ p_read5 sc_in sc_lv 26 signal 4 } 
	{ p_read6 sc_in sc_lv 26 signal 5 } 
	{ p_read7 sc_in sc_lv 26 signal 6 } 
	{ ap_return_0 sc_out sc_lv 32 signal -1 } 
	{ ap_return_1 sc_out sc_lv 26 signal -1 } 
	{ ap_return_2 sc_out sc_lv 26 signal -1 } 
	{ ap_return_3 sc_out sc_lv 26 signal -1 } 
	{ ap_return_4 sc_out sc_lv 26 signal -1 } 
	{ grp_fp_recip_fu_3707_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fp_recip_fu_3707_p_dout0 sc_in sc_lv 17 signal -1 } 
	{ grp_fu_15620_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15620_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15620_p_dout0 sc_in sc_lv 40 signal -1 } 
	{ grp_fu_15620_p_ce sc_out sc_logic 1 signal -1 } 
	{ grp_fu_15624_p_din0 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15624_p_din1 sc_out sc_lv 26 signal -1 } 
	{ grp_fu_15624_p_dout0 sc_in sc_lv 40 signal -1 } 
	{ grp_fu_15624_p_ce sc_out sc_logic 1 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "p_read", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read", "role": "default" }} , 
 	{ "name": "p_read1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read1", "role": "default" }} , 
 	{ "name": "p_read3", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read3", "role": "default" }} , 
 	{ "name": "p_read4", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read4", "role": "default" }} , 
 	{ "name": "p_read5", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read5", "role": "default" }} , 
 	{ "name": "p_read6", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read6", "role": "default" }} , 
 	{ "name": "p_read7", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_read7", "role": "default" }} , 
 	{ "name": "ap_return_0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "ap_return_0", "role": "default" }} , 
 	{ "name": "ap_return_1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_1", "role": "default" }} , 
 	{ "name": "ap_return_2", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_2", "role": "default" }} , 
 	{ "name": "ap_return_3", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_3", "role": "default" }} , 
 	{ "name": "ap_return_4", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_4", "role": "default" }} , 
 	{ "name": "grp_fp_recip_fu_3707_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fp_recip_fu_3707_p_din1", "role": "default" }} , 
 	{ "name": "grp_fp_recip_fu_3707_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "grp_fp_recip_fu_3707_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15620_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15620_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15620_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15620_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15620_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_15620_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15620_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15620_p_ce", "role": "default" }} , 
 	{ "name": "grp_fu_15624_p_din0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15624_p_din0", "role": "default" }} , 
 	{ "name": "grp_fu_15624_p_din1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "grp_fu_15624_p_din1", "role": "default" }} , 
 	{ "name": "grp_fu_15624_p_dout0", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "grp_fu_15624_p_dout0", "role": "default" }} , 
 	{ "name": "grp_fu_15624_p_ce", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "grp_fu_15624_p_ce", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	invert_2x2_qp_hls {
		p_read {Type I LastRead 0 FirstWrite -1}
		p_read1 {Type I LastRead 0 FirstWrite -1}
		p_read3 {Type I LastRead 0 FirstWrite -1}
		p_read4 {Type I LastRead 4 FirstWrite -1}
		p_read5 {Type I LastRead 4 FirstWrite -1}
		p_read6 {Type I LastRead 4 FirstWrite -1}
		p_read7 {Type I LastRead 4 FirstWrite -1}
		recip_lut {Type I LastRead -1 FirstWrite -1}
		slope_lut {Type I LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "5", "Max" : "13"}
	, {"Name" : "Interval", "Min" : "5", "Max" : "13"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	p_read { ap_none {  { p_read in_data 0 26 } } }
	p_read1 { ap_none {  { p_read1 in_data 0 26 } } }
	p_read3 { ap_none {  { p_read3 in_data 0 26 } } }
	p_read4 { ap_none {  { p_read4 in_data 0 26 } } }
	p_read5 { ap_none {  { p_read5 in_data 0 26 } } }
	p_read6 { ap_none {  { p_read6 in_data 0 26 } } }
	p_read7 { ap_none {  { p_read7 in_data 0 26 } } }
}
