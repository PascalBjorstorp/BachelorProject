set moduleName mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9
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
set cdfgNum 76
set C_modelName {mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict y_u_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict y_u { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_u_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict z_u { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_0 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_1 { MEM_WIDTH 32 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 1 }
set C_modelArgList {
	{ y_u_1 int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ y_u int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ z_u_1 int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ z_u int 32 regular {array 20 { 0 3 } 0 1 bus  }  }
	{ p_anonymous_namespace_g_core_state_admm_z_u_0 int 32 regular {array 20 { 1 3 } 1 1 bus  } {global 0}  }
	{ p_anonymous_namespace_g_core_state_admm_z_u_1 int 32 regular {array 20 { 1 3 } 1 1 bus  } {global 0}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_0 int 32 regular {array 20 { 1 3 } 1 1 bus  } {global 0}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_1 int 32 regular {array 20 { 1 3 } 1 1 bus  } {global 0}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "y_u_1", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "y_u", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_u_1", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "z_u", "interface" : "memory", "bitwidth" : 32, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_0", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_1", "interface" : "memory", "bitwidth" : 32, "direction" : "READONLY", "extern" : 0} ]}
# RTL Port declarations: 
set portNum 34
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ y_u_1_address0 sc_out sc_lv 5 signal 0 } 
	{ y_u_1_ce0 sc_out sc_logic 1 signal 0 } 
	{ y_u_1_we0 sc_out sc_logic 1 signal 0 } 
	{ y_u_1_d0 sc_out sc_lv 32 signal 0 } 
	{ y_u_address0 sc_out sc_lv 5 signal 1 } 
	{ y_u_ce0 sc_out sc_logic 1 signal 1 } 
	{ y_u_we0 sc_out sc_logic 1 signal 1 } 
	{ y_u_d0 sc_out sc_lv 32 signal 1 } 
	{ z_u_1_address0 sc_out sc_lv 5 signal 2 } 
	{ z_u_1_ce0 sc_out sc_logic 1 signal 2 } 
	{ z_u_1_we0 sc_out sc_logic 1 signal 2 } 
	{ z_u_1_d0 sc_out sc_lv 32 signal 2 } 
	{ z_u_address0 sc_out sc_lv 5 signal 3 } 
	{ z_u_ce0 sc_out sc_logic 1 signal 3 } 
	{ z_u_we0 sc_out sc_logic 1 signal 3 } 
	{ z_u_d0 sc_out sc_lv 32 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_address0 sc_out sc_lv 5 signal 4 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 sc_out sc_logic 1 signal 4 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_q0 sc_in sc_lv 32 signal 4 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_address0 sc_out sc_lv 5 signal 5 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 sc_out sc_logic 1 signal 5 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_q0 sc_in sc_lv 32 signal 5 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_address0 sc_out sc_lv 5 signal 6 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 sc_out sc_logic 1 signal 6 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_q0 sc_in sc_lv 32 signal 6 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_address0 sc_out sc_lv 5 signal 7 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 sc_out sc_logic 1 signal 7 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_q0 sc_in sc_lv 32 signal 7 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "y_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u_1", "role": "address0" }} , 
 	{ "name": "y_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "ce0" }} , 
 	{ "name": "y_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u_1", "role": "we0" }} , 
 	{ "name": "y_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_u_1", "role": "d0" }} , 
 	{ "name": "y_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "y_u", "role": "address0" }} , 
 	{ "name": "y_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "ce0" }} , 
 	{ "name": "y_u_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "y_u", "role": "we0" }} , 
 	{ "name": "y_u_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "y_u", "role": "d0" }} , 
 	{ "name": "z_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u_1", "role": "address0" }} , 
 	{ "name": "z_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "ce0" }} , 
 	{ "name": "z_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u_1", "role": "we0" }} , 
 	{ "name": "z_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_u_1", "role": "d0" }} , 
 	{ "name": "z_u_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "z_u", "role": "address0" }} , 
 	{ "name": "z_u_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "ce0" }} , 
 	{ "name": "z_u_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "z_u", "role": "we0" }} , 
 	{ "name": "z_u_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "z_u", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "q0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_q0", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "q0" }}  ]}

set ArgLastReadFirstWriteLatency {
	mpc_compute_hls_Pipeline_VITIS_LOOP_1215_9 {
		y_u_1 {Type O LastRead -1 FirstWrite 1}
		y_u {Type O LastRead -1 FirstWrite 1}
		z_u_1 {Type O LastRead -1 FirstWrite 1}
		z_u {Type O LastRead -1 FirstWrite 1}
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type I LastRead 0 FirstWrite -1}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "22", "Max" : "22"}
	, {"Name" : "Interval", "Min" : "21", "Max" : "21"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	y_u_1 { ap_memory {  { y_u_1_address0 mem_address 1 5 }  { y_u_1_ce0 mem_ce 1 1 }  { y_u_1_we0 mem_we 1 1 }  { y_u_1_d0 mem_din 1 32 } } }
	y_u { ap_memory {  { y_u_address0 mem_address 1 5 }  { y_u_ce0 mem_ce 1 1 }  { y_u_we0 mem_we 1 1 }  { y_u_d0 mem_din 1 32 } } }
	z_u_1 { ap_memory {  { z_u_1_address0 mem_address 1 5 }  { z_u_1_ce0 mem_ce 1 1 }  { z_u_1_we0 mem_we 1 1 }  { z_u_1_d0 mem_din 1 32 } } }
	z_u { ap_memory {  { z_u_address0 mem_address 1 5 }  { z_u_ce0 mem_ce 1 1 }  { z_u_we0 mem_we 1 1 }  { z_u_d0 mem_din 1 32 } } }
	p_anonymous_namespace_g_core_state_admm_z_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_q0 mem_dout 0 32 } } }
	p_anonymous_namespace_g_core_state_admm_z_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_q0 mem_dout 0 32 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_q0 mem_dout 0 32 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_q0 mem_dout 0 32 } } }
}
