set moduleName fp_rollout_from_forces_fn
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
set C_modelName {fp_rollout_from_forces_fn}
set C_modelType { int 105 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ ey int 21 regular  }
	{ epsi int 21 regular  }
	{ sin_epsi int 21 regular  }
	{ cos_epsi int 21 regular  }
	{ vx_safe int 21 regular  }
	{ vy int 21 regular  }
	{ omega int 21 regular  }
	{ kappa int 21 regular  }
	{ Fx int 21 regular  }
	{ F_yf int 21 regular  }
	{ F_yr int 21 regular  }
	{ sin_delta int 21 regular  }
	{ cos_delta int 21 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "ey", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "epsi", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "sin_epsi", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "cos_epsi", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "vx_safe", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "vy", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "omega", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "kappa", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "Fx", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "F_yf", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "F_yr", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "sin_delta", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "cos_delta", "interface" : "wire", "bitwidth" : 21, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 105} ]}
# RTL Port declarations: 
set portNum 24
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ ey sc_in sc_lv 21 signal 0 } 
	{ epsi sc_in sc_lv 21 signal 1 } 
	{ sin_epsi sc_in sc_lv 21 signal 2 } 
	{ cos_epsi sc_in sc_lv 21 signal 3 } 
	{ vx_safe sc_in sc_lv 21 signal 4 } 
	{ vy sc_in sc_lv 21 signal 5 } 
	{ omega sc_in sc_lv 21 signal 6 } 
	{ kappa sc_in sc_lv 21 signal 7 } 
	{ Fx sc_in sc_lv 21 signal 8 } 
	{ F_yf sc_in sc_lv 21 signal 9 } 
	{ F_yr sc_in sc_lv 21 signal 10 } 
	{ sin_delta sc_in sc_lv 21 signal 11 } 
	{ cos_delta sc_in sc_lv 21 signal 12 } 
	{ ap_return_0 sc_out sc_lv 21 signal -1 } 
	{ ap_return_1 sc_out sc_lv 21 signal -1 } 
	{ ap_return_2 sc_out sc_lv 21 signal -1 } 
	{ ap_return_3 sc_out sc_lv 21 signal -1 } 
	{ ap_return_4 sc_out sc_lv 21 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "ey", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ey", "role": "default" }} , 
 	{ "name": "epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "epsi", "role": "default" }} , 
 	{ "name": "sin_epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "sin_epsi", "role": "default" }} , 
 	{ "name": "cos_epsi", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "cos_epsi", "role": "default" }} , 
 	{ "name": "vx_safe", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "vx_safe", "role": "default" }} , 
 	{ "name": "vy", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "vy", "role": "default" }} , 
 	{ "name": "omega", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "omega", "role": "default" }} , 
 	{ "name": "kappa", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "kappa", "role": "default" }} , 
 	{ "name": "Fx", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "Fx", "role": "default" }} , 
 	{ "name": "F_yf", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "F_yf", "role": "default" }} , 
 	{ "name": "F_yr", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "F_yr", "role": "default" }} , 
 	{ "name": "sin_delta", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "sin_delta", "role": "default" }} , 
 	{ "name": "cos_delta", "direction": "in", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "cos_delta", "role": "default" }} , 
 	{ "name": "ap_return_0", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_0", "role": "default" }} , 
 	{ "name": "ap_return_1", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_1", "role": "default" }} , 
 	{ "name": "ap_return_2", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_2", "role": "default" }} , 
 	{ "name": "ap_return_3", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_3", "role": "default" }} , 
 	{ "name": "ap_return_4", "direction": "out", "datatype": "sc_lv", "bitwidth":21, "type": "signal", "bundle":{"name": "ap_return_4", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	fp_rollout_from_forces_fn {
		ey {Type I LastRead 0 FirstWrite -1}
		epsi {Type I LastRead 18 FirstWrite -1}
		sin_epsi {Type I LastRead 5 FirstWrite -1}
		cos_epsi {Type I LastRead 1 FirstWrite -1}
		vx_safe {Type I LastRead 1 FirstWrite -1}
		vy {Type I LastRead 6 FirstWrite -1}
		omega {Type I LastRead 7 FirstWrite -1}
		kappa {Type I LastRead 0 FirstWrite -1}
		Fx {Type I LastRead 6 FirstWrite -1}
		F_yf {Type I LastRead 2 FirstWrite -1}
		F_yr {Type I LastRead 0 FirstWrite -1}
		sin_delta {Type I LastRead 3 FirstWrite -1}
		cos_delta {Type I LastRead 2 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "18", "Max" : "18"}
	, {"Name" : "Interval", "Min" : "19", "Max" : "19"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	ey { ap_none {  { ey in_data 0 21 } } }
	epsi { ap_none {  { epsi in_data 0 21 } } }
	sin_epsi { ap_none {  { sin_epsi in_data 0 21 } } }
	cos_epsi { ap_none {  { cos_epsi in_data 0 21 } } }
	vx_safe { ap_none {  { vx_safe in_data 0 21 } } }
	vy { ap_none {  { vy in_data 0 21 } } }
	omega { ap_none {  { omega in_data 0 21 } } }
	kappa { ap_none {  { kappa in_data 0 21 } } }
	Fx { ap_none {  { Fx in_data 0 21 } } }
	F_yf { ap_none {  { F_yf in_data 0 21 } } }
	F_yr { ap_none {  { F_yr in_data 0 21 } } }
	sin_delta { ap_none {  { sin_delta in_data 0 21 } } }
	cos_delta { ap_none {  { cos_delta in_data 0 21 } } }
}
