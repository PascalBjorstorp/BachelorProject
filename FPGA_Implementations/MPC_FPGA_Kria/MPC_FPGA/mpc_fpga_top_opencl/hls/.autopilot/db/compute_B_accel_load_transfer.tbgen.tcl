set moduleName compute_B_accel_load_transfer
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
set C_modelName {compute_B_accel_load_transfer}
set C_modelType { int 52 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ tr_C_eff_f_raw_val int 26 regular  }
	{ tr_C_eff_r_val int 26 regular  }
	{ tr_C_min_f_val int 26 regular  }
	{ tr_cos_delta_val int 26 regular  }
	{ tr_alpha_f_op_val int 26 regular  }
	{ tr_alpha_r_op_val int 26 regular  }
	{ tr_Fz_transfer_val int 26 regular  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "tr_C_eff_f_raw_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "tr_C_eff_r_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "tr_C_min_f_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "tr_cos_delta_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "tr_alpha_f_op_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "tr_alpha_r_op_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "tr_Fz_transfer_val", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "ap_return", "interface" : "wire", "bitwidth" : 52} ]}
# RTL Port declarations: 
set portNum 15
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ tr_C_eff_f_raw_val sc_in sc_lv 26 signal 0 } 
	{ tr_C_eff_r_val sc_in sc_lv 26 signal 1 } 
	{ tr_C_min_f_val sc_in sc_lv 26 signal 2 } 
	{ tr_cos_delta_val sc_in sc_lv 26 signal 3 } 
	{ tr_alpha_f_op_val sc_in sc_lv 26 signal 4 } 
	{ tr_alpha_r_op_val sc_in sc_lv 26 signal 5 } 
	{ tr_Fz_transfer_val sc_in sc_lv 26 signal 6 } 
	{ ap_return_0 sc_out sc_lv 26 signal -1 } 
	{ ap_return_1 sc_out sc_lv 26 signal -1 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "tr_C_eff_f_raw_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "tr_C_eff_f_raw_val", "role": "default" }} , 
 	{ "name": "tr_C_eff_r_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "tr_C_eff_r_val", "role": "default" }} , 
 	{ "name": "tr_C_min_f_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "tr_C_min_f_val", "role": "default" }} , 
 	{ "name": "tr_cos_delta_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "tr_cos_delta_val", "role": "default" }} , 
 	{ "name": "tr_alpha_f_op_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "tr_alpha_f_op_val", "role": "default" }} , 
 	{ "name": "tr_alpha_r_op_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "tr_alpha_r_op_val", "role": "default" }} , 
 	{ "name": "tr_Fz_transfer_val", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "tr_Fz_transfer_val", "role": "default" }} , 
 	{ "name": "ap_return_0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_0", "role": "default" }} , 
 	{ "name": "ap_return_1", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "ap_return_1", "role": "default" }}  ]}

set ArgLastReadFirstWriteLatency {
	compute_B_accel_load_transfer {
		tr_C_eff_f_raw_val {Type I LastRead 8 FirstWrite -1}
		tr_C_eff_r_val {Type I LastRead 10 FirstWrite -1}
		tr_C_min_f_val {Type I LastRead 9 FirstWrite -1}
		tr_cos_delta_val {Type I LastRead 17 FirstWrite -1}
		tr_alpha_f_op_val {Type I LastRead 11 FirstWrite -1}
		tr_alpha_r_op_val {Type I LastRead 13 FirstWrite -1}
		tr_Fz_transfer_val {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}
	fp_recip_fn {
		x {Type I LastRead 0 FirstWrite -1}
		recip_lut_fn {Type I LastRead -1 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "25", "Max" : "25"}
	, {"Name" : "Interval", "Min" : "26", "Max" : "26"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	tr_C_eff_f_raw_val { ap_none {  { tr_C_eff_f_raw_val in_data 0 26 } } }
	tr_C_eff_r_val { ap_none {  { tr_C_eff_r_val in_data 0 26 } } }
	tr_C_min_f_val { ap_none {  { tr_C_min_f_val in_data 0 26 } } }
	tr_cos_delta_val { ap_none {  { tr_cos_delta_val in_data 0 26 } } }
	tr_alpha_f_op_val { ap_none {  { tr_alpha_f_op_val in_data 0 26 } } }
	tr_alpha_r_op_val { ap_none {  { tr_alpha_r_op_val in_data 0 26 } } }
	tr_Fz_transfer_val { ap_none {  { tr_Fz_transfer_val in_data 0 26 } } }
}
