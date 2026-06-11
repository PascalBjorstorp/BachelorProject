set moduleName compute_frenet_tire_hls
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
set C_modelName {compute_frenet_tire_hls}
set C_modelType { int 357 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ vx int 23 regular  }
	{ vy int 23 regular  }
	{ omega int 23 regular  }
	{ delta int 14 regular  }
	{ a_cmd int 23 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "vx", "interface" : "wire", "bitwidth" : 23, "direction" : "READONLY"} , 
 	{ "Name" : "vy", "interface" : "wire", "bitwidth" : 23, "direction" : "READONLY"} , 
 	{ "Name" : "omega", "interface" : "wire", "bitwidth" : 23, "direction" : "READONLY"} , 
 	{ "Name" : "delta", "interface" : "wire", "bitwidth" : 14, "direction" : "READONLY"} , 
 	{ "Name" : "a_cmd", "interface" : "wire", "bitwidth" : 23, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 357} ]}
# RTL Port declarations: 
set portNum 28
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ vx sc_in sc_lv 23 signal 0 } 
	{ vy sc_in sc_lv 23 signal 1 } 
	{ omega sc_in sc_lv 23 signal 2 } 
	{ delta sc_in sc_lv 14 signal 3 } 
	{ a_cmd sc_in sc_lv 23 signal 4 } 
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
	{ ap_return_14 sc_out sc_lv 21 signal -1 } 
	{ ap_return_15 sc_out sc_lv 21 signal -1 } 
	{ ap_return_16 sc_out sc_lv 21 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "vx", "direction": "in", "datatype": "sc_lv", "bitwidth":23, "type": "signal", "bundle":{"name": "vx", "role": "default" }} , 
 	{ "name": "vy", "direction": "in", "datatype": "sc_lv", "bitwidth":23, "type": "signal", "bundle":{"name": "vy", "role": "default" }} , 
 	{ "name": "omega", "direction": "in", "datatype": "sc_lv", "bitwidth":23, "type": "signal", "bundle":{"name": "omega", "role": "default" }} , 
 	{ "name": "delta", "direction": "in", "datatype": "sc_lv", "bitwidth":14, "type": "signal", "bundle":{"name": "delta", "role": "default" }} , 
 	{ "name": "a_cmd", "direction": "in", "datatype": "sc_lv", "bitwidth":23, "type": "signal", "bundle":{"name": "a_cmd", "role": "default" }} , 
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
 	{ "name": "ap_return_13", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_13", "role": "default" }} , 
 	{ "name": "ap_return_14", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_14", "role": "default" }} , 
 	{ "name": "ap_return_15", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_15", "role": "default" }} , 
 	{ "name": "ap_return_16", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_16", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	compute_frenet_tire_hls {
		vx {Type I LastRead 0 FirstWrite -1}
		vy {Type I LastRead 1 FirstWrite -1}
		omega {Type I LastRead 8 FirstWrite -1}
		delta {Type I LastRead 17 FirstWrite -1}
		a_cmd {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}
		atan_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_slip_terms {
		vy {Type I LastRead 1 FirstWrite -1}
		omega {Type I LastRead 0 FirstWrite -1}
		inv_vx {Type I LastRead 1 FirstWrite -1}}
	compute_front_tire_path_fn {
		conv5_i_i_i_i1781_i {Type I LastRead 5 FirstWrite -1}
		p_0_0_03467_i {Type I LastRead 0 FirstWrite -1}
		empty {Type I LastRead 0 FirstWrite -1}
		conv_i_i_i1795_i {Type I LastRead 20 FirstWrite -1}
		conv_i_i_i2437_i {Type I LastRead 25 FirstWrite -1}
		p_0_0_03463_i {Type I LastRead 25 FirstWrite -1}
		spec_select_i {Type I LastRead 25 FirstWrite -1}
		conv_i_i_i65_i {Type I LastRead 0 FirstWrite -1}
		cmp_i_i {Type I LastRead 32 FirstWrite -1}
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
		D_cb {Type I LastRead 2 FirstWrite -1}}
	fp_front_force_jacobians_fn {
		C_eff_f {Type I LastRead 3 FirstWrite -1}
		front_num {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_f {Type I LastRead 0 FirstWrite -1}}
	compute_rear_tire_path_fn {
		rear_ratio {Type I LastRead 0 FirstWrite -1}
		D_transfer {Type I LastRead 0 FirstWrite -1}
		D_pac_r {Type I LastRead 20 FirstWrite -1}
		C_min_r {Type I LastRead 25 FirstWrite -1}
		rear_num {Type I LastRead 25 FirstWrite -1}
		vx_safe {Type I LastRead 25 FirstWrite -1}
		D_r_fn {Type I LastRead 0 FirstWrite -1}
		low_speed {Type I LastRead 31 FirstWrite -1}
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
		D_cb {Type I LastRead 2 FirstWrite -1}}
	fp_rear_force_jacobians_fn {
		C_eff_r {Type I LastRead 2 FirstWrite -1}
		rear_num {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 0 FirstWrite -1}
		inv_D_r {Type I LastRead 0 FirstWrite -1}}
	fp_trig_pair_fused_fn {
		angle {Type I LastRead 0 FirstWrite -1}
		sin_lut_fn {Type I LastRead -1 FirstWrite -1}
		cos_lut_fn {Type I LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "49", "Max" : "49"}
	, {"Name" : "Interval", "Min" : "50", "Max" : "50"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	vx { ap_none {  { vx in_data 0 23 } } }
	vy { ap_none {  { vy in_data 0 23 } } }
	omega { ap_none {  { omega in_data 0 23 } } }
	delta { ap_none {  { delta in_data 0 14 } } }
	a_cmd { ap_none {  { a_cmd in_data 0 23 } } }
}
