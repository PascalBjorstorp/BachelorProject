set moduleName p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3
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
set C_modelName {(anonymous namespace)mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_z_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_0 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
dict set ap_memory_interface_dict p_anonymous_namespace_g_core_state_admm_y_u_1 { MEM_WIDTH 26 MEM_SIZE 80 MASTER_TYPE BRAM_CTRL MEM_ADDRESS_MODE WORD_ADDRESS PACKAGE_IO bus READ_LATENCY 0 }
set C_modelArgList {
	{ p_anonymous_namespace_g_core_state_admm_z_u_0 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_z_u_1 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_0 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
	{ p_anonymous_namespace_g_core_state_admm_y_u_1 int 26 regular {array 20 { 0 3 } 0 1 bus  } {global 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_z_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_0", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} , 
 	{ "Name" : "p_anonymous_namespace_g_core_state_admm_y_u_1", "interface" : "memory", "bitwidth" : 26, "direction" : "WRITEONLY", "extern" : 0} ]}
# RTL Port declarations: 
set portNum 22
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_address0 sc_out sc_lv 5 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 sc_out sc_logic 1 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_we0 sc_out sc_logic 1 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_0_d0 sc_out sc_lv 26 signal 0 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_address0 sc_out sc_lv 5 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 sc_out sc_logic 1 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_we0 sc_out sc_logic 1 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_z_u_1_d0 sc_out sc_lv 26 signal 1 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_address0 sc_out sc_lv 5 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 sc_out sc_logic 1 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_we0 sc_out sc_logic 1 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_0_d0 sc_out sc_lv 26 signal 2 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_address0 sc_out sc_lv 5 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 sc_out sc_logic 1 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_we0 sc_out sc_logic 1 signal 3 } 
	{ p_anonymous_namespace_g_core_state_admm_y_u_1_d0 sc_out sc_lv 26 signal 3 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_z_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_z_u_1", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_0_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_0", "role": "d0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_address0", "direction": "out", "datatype": "sc_lv", "bitwidth":5, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "address0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_ce0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "ce0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_we0", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "we0" }} , 
 	{ "name": "p_anonymous_namespace_g_core_state_admm_y_u_1_d0", "direction": "out", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "p_anonymous_namespace_g_core_state_admm_y_u_1", "role": "d0" }}  ]}

set ArgLastReadFirstWriteLatency {
	p_anonymous_namespace_mpc_fpga_compute_core_Pipeline_VITIS_LOOP_330_3 {
		p_anonymous_namespace_g_core_state_admm_z_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_z_u_1 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_0 {Type O LastRead -1 FirstWrite 0}
		p_anonymous_namespace_g_core_state_admm_y_u_1 {Type O LastRead -1 FirstWrite 0}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "22", "Max" : "22"}
	, {"Name" : "Interval", "Min" : "21", "Max" : "21"}
]}

set PipelineEnableSignalInfo {[
]}

set Spec2ImplPortList { 
	p_anonymous_namespace_g_core_state_admm_z_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_0_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_z_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_z_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_z_u_1_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_0 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_0_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_0_d0 mem_din 1 26 } } }
	p_anonymous_namespace_g_core_state_admm_y_u_1 { ap_memory {  { p_anonymous_namespace_g_core_state_admm_y_u_1_address0 mem_address 1 5 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_ce0 mem_ce 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_we0 mem_we 1 1 }  { p_anonymous_namespace_g_core_state_admm_y_u_1_d0 mem_din 1 26 } } }
}
