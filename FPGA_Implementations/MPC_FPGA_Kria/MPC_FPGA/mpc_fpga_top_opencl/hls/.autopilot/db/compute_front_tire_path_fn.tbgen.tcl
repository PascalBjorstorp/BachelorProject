set moduleName compute_front_tire_path_fn
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
set C_modelName {compute_front_tire_path_fn}
set C_modelType { int 156 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ conv_i_i_i_i1462_i int 17 regular  }
	{ p_0_0_02853_i int 26 regular  }
	{ empty int 26 regular  }
	{ conv_i_i_i1476_i int 26 regular  }
	{ conv_i_i_i1991_i int 26 regular  }
	{ p_0_0_02849_i int 26 regular  }
	{ spec_select_i int 25 regular  }
	{ conv_i_i_i65_i int 26 regular  }
	{ cmp_i_i int 1 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "conv_i_i_i_i1462_i", "interface" : "wire", "bitwidth" : 17, "direction" : "READONLY"} , 
 	{ "Name" : "p_0_0_02853_i", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "empty", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "conv_i_i_i1476_i", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "conv_i_i_i1991_i", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "p_0_0_02849_i", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "spec_select_i", "interface" : "wire", "bitwidth" : 25, "direction" : "READONLY"} , 
 	{ "Name" : "conv_i_i_i65_i", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "cmp_i_i", "interface" : "wire", "bitwidth" : 1, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 156} ]}
# RTL Port declarations: 
set portNum 21
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ conv_i_i_i_i1462_i sc_in sc_lv 17 signal 0 } 
	{ p_0_0_02853_i sc_in sc_lv 26 signal 1 } 
	{ empty sc_in sc_lv 26 signal 2 } 
	{ conv_i_i_i1476_i sc_in sc_lv 26 signal 3 } 
	{ conv_i_i_i1991_i sc_in sc_lv 26 signal 4 } 
	{ p_0_0_02849_i sc_in sc_lv 26 signal 5 } 
	{ spec_select_i sc_in sc_lv 25 signal 6 } 
	{ conv_i_i_i65_i sc_in sc_lv 26 signal 7 } 
	{ cmp_i_i sc_in sc_lv 1 signal 8 } 
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
 	{ "name": "conv_i_i_i_i1462_i", "direction": "in", "datatype": "sc_lv", "bitwidth":17, "type": "signal", "bundle":{"name": "conv_i_i_i_i1462_i", "role": "default" }} , 
 	{ "name": "p_0_0_02853_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_02853_i", "role": "default" }} , 
 	{ "name": "empty", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "empty", "role": "default" }} , 
 	{ "name": "conv_i_i_i1476_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "conv_i_i_i1476_i", "role": "default" }} , 
 	{ "name": "conv_i_i_i1991_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "conv_i_i_i1991_i", "role": "default" }} , 
 	{ "name": "p_0_0_02849_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_0_0_02849_i", "role": "default" }} , 
 	{ "name": "spec_select_i", "direction": "in", "datatype": "sc_lv", "bitwidth":25, "type": "signal", "bundle":{"name": "spec_select_i", "role": "default" }} , 
 	{ "name": "conv_i_i_i65_i", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "conv_i_i_i65_i", "role": "default" }} , 
 	{ "name": "cmp_i_i", "direction": "in", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "cmp_i_i", "role": "default" }} , 
 	{ "name": "ap_return_0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_0", "role": "default" }} , 
 	{ "name": "ap_return_1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_1", "role": "default" }} , 
 	{ "name": "ap_return_2", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_2", "role": "default" }} , 
 	{ "name": "ap_return_3", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_3", "role": "default" }} , 
 	{ "name": "ap_return_4", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_4", "role": "default" }} , 
 	{ "name": "ap_return_5", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_5", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
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
		inv_D_f {Type I LastRead 1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "47", "Max" : "47"}
	, {"Name" : "Interval", "Min" : "48", "Max" : "48"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	conv_i_i_i_i1462_i { ap_none {  { conv_i_i_i_i1462_i in_data 0 17 } } }
	p_0_0_02853_i { ap_none {  { p_0_0_02853_i in_data 0 26 } } }
	empty { ap_none {  { empty in_data 0 26 } } }
	conv_i_i_i1476_i { ap_none {  { conv_i_i_i1476_i in_data 0 26 } } }
	conv_i_i_i1991_i { ap_none {  { conv_i_i_i1991_i in_data 0 26 } } }
	p_0_0_02849_i { ap_none {  { p_0_0_02849_i in_data 0 26 } } }
	spec_select_i { ap_none {  { spec_select_i in_data 0 25 } } }
	conv_i_i_i65_i { ap_none {  { conv_i_i_i65_i in_data 0 26 } } }
	cmp_i_i { ap_none {  { cmp_i_i in_data 0 1 } } }
}
