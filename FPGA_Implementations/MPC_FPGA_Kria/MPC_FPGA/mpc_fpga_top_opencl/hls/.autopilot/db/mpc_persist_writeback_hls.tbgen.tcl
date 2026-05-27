set moduleName mpc_persist_writeback_hls
set isTopModule 0
set isCombinational 1
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
set cdfgNum 76
set C_modelName {mpc_persist_writeback_hls}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ prev_steer_rate int 32 regular  }
	{ prev_accel int 22 regular  }
	{ prev_curvature int 32 regular  }
	{ p_anonymous_namespace_g_core_state_persist_prev_steer_rate int 32 regular {pointer 1} {global 1}  }
	{ p_anonymous_namespace_g_core_state_persist_prev_accel int 32 regular {pointer 1} {global 1}  }
	{ p_anonymous_namespace_g_core_state_persist_prev_curvature int 32 regular {pointer 1} {global 1}  }
	{ p_anonymous_namespace_g_core_state_persist_prev_model_signature int 1 regular {pointer 1} {global 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "prev_steer_rate", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "prev_accel", "interface" : "wire", "bitwidth" : 22, "direction" : "READONLY"} , 
 	{ "Name" : "prev_curvature", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_accel", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_curvature", "interface" : "wire", "bitwidth" : 32, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "interface" : "wire", "bitwidth" : 1, "direction" : "WRITEONLY", "extern" : 0} ]}
# RTL Port declarations: 
set portNum 13
set portList { 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ prev_steer_rate sc_in sc_lv 32 signal 0 } 
	{ prev_accel sc_in sc_lv 22 signal 1 } 
	{ prev_curvature sc_in sc_lv 32 signal 2 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_steer_rate sc_out sc_lv 32 signal 3 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_steer_rate_ap_vld sc_out sc_logic 1 outvld 3 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_accel sc_out sc_lv 32 signal 4 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_accel_ap_vld sc_out sc_logic 1 outvld 4 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_curvature sc_out sc_lv 32 signal 5 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_curvature_ap_vld sc_out sc_logic 1 outvld 5 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_model_signature sc_out sc_lv 1 signal 6 } 
	{ p_anonymous_namespace_g_core_state_persist_prev_model_signature_ap_vld sc_out sc_logic 1 outvld 6 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
}
set NewPortList {[ 
	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "prev_steer_rate", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "prev_steer_rate", "role": "default" }} , 
 	{ "name": "prev_accel", "direction": "in", "datatype": "sc_lv", "bitwidth":22, "type": "signal", "bundle":{"name": "prev_accel", "role": "default" }} , 
 	{ "name": "prev_curvature", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "prev_curvature", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_steer_rate", "role": "ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_accel", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_accel", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_accel_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_accel", "role": "ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_curvature", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_curvature", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_curvature_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_curvature", "role": "ap_vld" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "direction": "out", "datatype": "sc_lv", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_anonymous_namespace_g_core_state_persist_prev_model_signature", "role": "ap_vld" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_persist_writeback_hls {
		prev_steer_rate {Type I LastRead 0 FirstWrite -1}
		prev_accel {Type I LastRead 0 FirstWrite -1}
		prev_curvature {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_persist_prev_steer_rate {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_accel {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_curvature {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_persist_prev_model_signature {Type O LastRead -1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "0", "Max" : "0"}
	, {"Name" : "Interval", "Min" : "0", "Max" : "0"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	prev_steer_rate { ap_none {  { prev_steer_rate in_data 0 32 } } }
	prev_accel { ap_none {  { prev_accel in_data 0 22 } } }
	prev_curvature { ap_none {  { prev_curvature in_data 0 32 } } }
	p_anonymous_namespace_g_core_state_persist_prev_steer_rate { ap_vld {  { p_anonymous_namespace_g_core_state_persist_prev_steer_rate out_data 1 32 }  { p_anonymous_namespace_g_core_state_persist_prev_steer_rate_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_persist_prev_accel { ap_vld {  { p_anonymous_namespace_g_core_state_persist_prev_accel out_data 1 32 }  { p_anonymous_namespace_g_core_state_persist_prev_accel_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_persist_prev_curvature { ap_vld {  { p_anonymous_namespace_g_core_state_persist_prev_curvature out_data 1 32 }  { p_anonymous_namespace_g_core_state_persist_prev_curvature_ap_vld out_vld 1 1 } } }
	p_anonymous_namespace_g_core_state_persist_prev_model_signature { ap_vld {  { p_anonymous_namespace_g_core_state_persist_prev_model_signature out_data 1 1 }  { p_anonymous_namespace_g_core_state_persist_prev_model_signature_ap_vld out_vld 1 1 } } }
}
