set moduleName riccati_backward_pass_Pipeline_VITIS_LOOP_742_9
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
set C_modelName {riccati_backward_pass_Pipeline_VITIS_LOOP_742_9}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ s1 int 35 regular  }
	{ s0 int 35 regular  }
	{ s1_1 int 35 regular  }
	{ s0_1 int 35 regular  }
	{ M_8 int 18 regular  }
	{ M int 18 regular  }
	{ M_10 int 18 regular  }
	{ M_9 int 18 regular  }
	{ M_12 int 18 regular  }
	{ M_11 int 18 regular  }
	{ M_14 int 18 regular  }
	{ M_13 int 18 regular  }
	{ b_215_cast int 26 regular  }
	{ b_216_cast int 26 regular  }
	{ b_217_cast int 26 regular  }
	{ b_218_cast int 26 regular  }
	{ b_219_cast int 26 regular  }
	{ b_220_cast int 26 regular  }
	{ b_158_cast int 26 regular  }
	{ b_159_cast int 26 regular  }
	{ b_160_cast int 26 regular  }
	{ b_161_cast int 26 regular  }
	{ b_162_cast int 26 regular  }
	{ b_163_cast int 26 regular  }
	{ b_164_cast int 26 regular  }
	{ b_165_cast int 26 regular  }
	{ b_166_cast int 26 regular  }
	{ b_167_cast int 26 regular  }
	{ b_168_cast int 26 regular  }
	{ b_169_cast int 26 regular  }
	{ b_170_cast int 26 regular  }
	{ b_171_cast int 26 regular  }
	{ b_172_cast int 26 regular  }
	{ b_173_cast int 26 regular  }
	{ b_174_cast int 26 regular  }
	{ b_175_cast int 26 regular  }
	{ b_176_cast int 26 regular  }
	{ b_177_cast int 26 regular  }
	{ b_178_cast int 26 regular  }
	{ b_179_cast int 26 regular  }
	{ b_180_cast int 26 regular  }
	{ b_181_cast int 26 regular  }
	{ b_182_cast int 26 regular  }
	{ b_183_cast int 26 regular  }
	{ b_184_cast int 26 regular  }
	{ b_185_cast int 26 regular  }
	{ b_186_cast int 26 regular  }
	{ sext_ln742 int 26 regular  }
	{ G_12_out int 18 regular {pointer 1}  }
	{ G_11_out int 18 regular {pointer 1}  }
	{ G_10_out int 18 regular {pointer 1}  }
	{ G_9_out int 18 regular {pointer 1}  }
	{ G_8_out int 18 regular {pointer 1}  }
	{ G_7_out int 18 regular {pointer 1}  }
	{ G_6_out int 18 regular {pointer 1}  }
	{ G_5_out int 18 regular {pointer 1}  }
	{ G_4_out int 18 regular {pointer 1}  }
	{ G_3_out int 18 regular {pointer 1}  }
	{ G_2_out int 18 regular {pointer 1}  }
	{ G_1_out int 18 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "s1", "interface" : "wire", "bitwidth" : 35, "direction" : "READONLY"} , 
 	{ "Name" : "s0", "interface" : "wire", "bitwidth" : 35, "direction" : "READONLY"} , 
 	{ "Name" : "s1_1", "interface" : "wire", "bitwidth" : 35, "direction" : "READONLY"} , 
 	{ "Name" : "s0_1", "interface" : "wire", "bitwidth" : 35, "direction" : "READONLY"} , 
 	{ "Name" : "M_8", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "M", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "M_10", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "M_9", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "M_12", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "M_11", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "M_14", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "M_13", "interface" : "wire", "bitwidth" : 18, "direction" : "READONLY"} , 
 	{ "Name" : "b_215_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_216_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_217_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_218_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_219_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_220_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_158_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_159_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_160_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_161_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_162_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_163_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_164_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_165_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_166_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_167_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_168_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_169_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_170_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_171_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_172_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_173_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_174_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_175_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_176_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_177_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_178_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_179_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_180_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_181_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_182_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_183_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_184_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_185_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "b_186_cast", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln742", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "G_12_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_11_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_10_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_9_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_8_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_7_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_6_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_5_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_4_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_3_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_2_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} , 
 	{ "Name" : "G_1_out", "interface" : "wire", "bitwidth" : 18, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 78
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ s1 sc_in sc_lv 35 signal 0 } 
	{ s0 sc_in sc_lv 35 signal 1 } 
	{ s1_1 sc_in sc_lv 35 signal 2 } 
	{ s0_1 sc_in sc_lv 35 signal 3 } 
	{ M_8 sc_in sc_lv 18 signal 4 } 
	{ M sc_in sc_lv 18 signal 5 } 
	{ M_10 sc_in sc_lv 18 signal 6 } 
	{ M_9 sc_in sc_lv 18 signal 7 } 
	{ M_12 sc_in sc_lv 18 signal 8 } 
	{ M_11 sc_in sc_lv 18 signal 9 } 
	{ M_14 sc_in sc_lv 18 signal 10 } 
	{ M_13 sc_in sc_lv 18 signal 11 } 
	{ b_215_cast sc_in sc_lv 26 signal 12 } 
	{ b_216_cast sc_in sc_lv 26 signal 13 } 
	{ b_217_cast sc_in sc_lv 26 signal 14 } 
	{ b_218_cast sc_in sc_lv 26 signal 15 } 
	{ b_219_cast sc_in sc_lv 26 signal 16 } 
	{ b_220_cast sc_in sc_lv 26 signal 17 } 
	{ b_158_cast sc_in sc_lv 26 signal 18 } 
	{ b_159_cast sc_in sc_lv 26 signal 19 } 
	{ b_160_cast sc_in sc_lv 26 signal 20 } 
	{ b_161_cast sc_in sc_lv 26 signal 21 } 
	{ b_162_cast sc_in sc_lv 26 signal 22 } 
	{ b_163_cast sc_in sc_lv 26 signal 23 } 
	{ b_164_cast sc_in sc_lv 26 signal 24 } 
	{ b_165_cast sc_in sc_lv 26 signal 25 } 
	{ b_166_cast sc_in sc_lv 26 signal 26 } 
	{ b_167_cast sc_in sc_lv 26 signal 27 } 
	{ b_168_cast sc_in sc_lv 26 signal 28 } 
	{ b_169_cast sc_in sc_lv 26 signal 29 } 
	{ b_170_cast sc_in sc_lv 26 signal 30 } 
	{ b_171_cast sc_in sc_lv 26 signal 31 } 
	{ b_172_cast sc_in sc_lv 26 signal 32 } 
	{ b_173_cast sc_in sc_lv 26 signal 33 } 
	{ b_174_cast sc_in sc_lv 26 signal 34 } 
	{ b_175_cast sc_in sc_lv 26 signal 35 } 
	{ b_176_cast sc_in sc_lv 26 signal 36 } 
	{ b_177_cast sc_in sc_lv 26 signal 37 } 
	{ b_178_cast sc_in sc_lv 26 signal 38 } 
	{ b_179_cast sc_in sc_lv 26 signal 39 } 
	{ b_180_cast sc_in sc_lv 26 signal 40 } 
	{ b_181_cast sc_in sc_lv 26 signal 41 } 
	{ b_182_cast sc_in sc_lv 26 signal 42 } 
	{ b_183_cast sc_in sc_lv 26 signal 43 } 
	{ b_184_cast sc_in sc_lv 26 signal 44 } 
	{ b_185_cast sc_in sc_lv 26 signal 45 } 
	{ b_186_cast sc_in sc_lv 26 signal 46 } 
	{ sext_ln742 sc_in sc_lv 26 signal 47 } 
	{ G_12_out sc_out sc_lv 18 signal 48 } 
	{ G_12_out_ap_vld sc_out sc_logic 1 outvld 48 } 
	{ G_11_out sc_out sc_lv 18 signal 49 } 
	{ G_11_out_ap_vld sc_out sc_logic 1 outvld 49 } 
	{ G_10_out sc_out sc_lv 18 signal 50 } 
	{ G_10_out_ap_vld sc_out sc_logic 1 outvld 50 } 
	{ G_9_out sc_out sc_lv 18 signal 51 } 
	{ G_9_out_ap_vld sc_out sc_logic 1 outvld 51 } 
	{ G_8_out sc_out sc_lv 18 signal 52 } 
	{ G_8_out_ap_vld sc_out sc_logic 1 outvld 52 } 
	{ G_7_out sc_out sc_lv 18 signal 53 } 
	{ G_7_out_ap_vld sc_out sc_logic 1 outvld 53 } 
	{ G_6_out sc_out sc_lv 18 signal 54 } 
	{ G_6_out_ap_vld sc_out sc_logic 1 outvld 54 } 
	{ G_5_out sc_out sc_lv 18 signal 55 } 
	{ G_5_out_ap_vld sc_out sc_logic 1 outvld 55 } 
	{ G_4_out sc_out sc_lv 18 signal 56 } 
	{ G_4_out_ap_vld sc_out sc_logic 1 outvld 56 } 
	{ G_3_out sc_out sc_lv 18 signal 57 } 
	{ G_3_out_ap_vld sc_out sc_logic 1 outvld 57 } 
	{ G_2_out sc_out sc_lv 18 signal 58 } 
	{ G_2_out_ap_vld sc_out sc_logic 1 outvld 58 } 
	{ G_1_out sc_out sc_lv 18 signal 59 } 
	{ G_1_out_ap_vld sc_out sc_logic 1 outvld 59 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "s1", "direction": "in", "datatype": "sc_lv", "bitwidth":35, "type": "signal", "bundle":{"name": "s1", "role": "default" }} , 
 	{ "name": "s0", "direction": "in", "datatype": "sc_lv", "bitwidth":35, "type": "signal", "bundle":{"name": "s0", "role": "default" }} , 
 	{ "name": "s1_1", "direction": "in", "datatype": "sc_lv", "bitwidth":35, "type": "signal", "bundle":{"name": "s1_1", "role": "default" }} , 
 	{ "name": "s0_1", "direction": "in", "datatype": "sc_lv", "bitwidth":35, "type": "signal", "bundle":{"name": "s0_1", "role": "default" }} , 
 	{ "name": "M_8", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M_8", "role": "default" }} , 
 	{ "name": "M", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M", "role": "default" }} , 
 	{ "name": "M_10", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M_10", "role": "default" }} , 
 	{ "name": "M_9", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M_9", "role": "default" }} , 
 	{ "name": "M_12", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M_12", "role": "default" }} , 
 	{ "name": "M_11", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M_11", "role": "default" }} , 
 	{ "name": "M_14", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M_14", "role": "default" }} , 
 	{ "name": "M_13", "direction": "in", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "M_13", "role": "default" }} , 
 	{ "name": "b_215_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_215_cast", "role": "default" }} , 
 	{ "name": "b_216_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_216_cast", "role": "default" }} , 
 	{ "name": "b_217_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_217_cast", "role": "default" }} , 
 	{ "name": "b_218_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_218_cast", "role": "default" }} , 
 	{ "name": "b_219_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_219_cast", "role": "default" }} , 
 	{ "name": "b_220_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_220_cast", "role": "default" }} , 
 	{ "name": "b_158_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_158_cast", "role": "default" }} , 
 	{ "name": "b_159_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_159_cast", "role": "default" }} , 
 	{ "name": "b_160_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_160_cast", "role": "default" }} , 
 	{ "name": "b_161_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_161_cast", "role": "default" }} , 
 	{ "name": "b_162_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_162_cast", "role": "default" }} , 
 	{ "name": "b_163_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_163_cast", "role": "default" }} , 
 	{ "name": "b_164_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_164_cast", "role": "default" }} , 
 	{ "name": "b_165_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_165_cast", "role": "default" }} , 
 	{ "name": "b_166_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_166_cast", "role": "default" }} , 
 	{ "name": "b_167_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_167_cast", "role": "default" }} , 
 	{ "name": "b_168_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_168_cast", "role": "default" }} , 
 	{ "name": "b_169_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_169_cast", "role": "default" }} , 
 	{ "name": "b_170_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_170_cast", "role": "default" }} , 
 	{ "name": "b_171_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_171_cast", "role": "default" }} , 
 	{ "name": "b_172_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_172_cast", "role": "default" }} , 
 	{ "name": "b_173_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_173_cast", "role": "default" }} , 
 	{ "name": "b_174_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_174_cast", "role": "default" }} , 
 	{ "name": "b_175_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_175_cast", "role": "default" }} , 
 	{ "name": "b_176_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_176_cast", "role": "default" }} , 
 	{ "name": "b_177_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_177_cast", "role": "default" }} , 
 	{ "name": "b_178_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_178_cast", "role": "default" }} , 
 	{ "name": "b_179_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_179_cast", "role": "default" }} , 
 	{ "name": "b_180_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_180_cast", "role": "default" }} , 
 	{ "name": "b_181_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_181_cast", "role": "default" }} , 
 	{ "name": "b_182_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_182_cast", "role": "default" }} , 
 	{ "name": "b_183_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_183_cast", "role": "default" }} , 
 	{ "name": "b_184_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_184_cast", "role": "default" }} , 
 	{ "name": "b_185_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_185_cast", "role": "default" }} , 
 	{ "name": "b_186_cast", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "b_186_cast", "role": "default" }} , 
 	{ "name": "sext_ln742", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln742", "role": "default" }} , 
 	{ "name": "G_12_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_12_out", "role": "default" }} , 
 	{ "name": "G_12_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_12_out", "role": "ap_vld" }} , 
 	{ "name": "G_11_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_11_out", "role": "default" }} , 
 	{ "name": "G_11_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_11_out", "role": "ap_vld" }} , 
 	{ "name": "G_10_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_10_out", "role": "default" }} , 
 	{ "name": "G_10_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_10_out", "role": "ap_vld" }} , 
 	{ "name": "G_9_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_9_out", "role": "default" }} , 
 	{ "name": "G_9_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_9_out", "role": "ap_vld" }} , 
 	{ "name": "G_8_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_8_out", "role": "default" }} , 
 	{ "name": "G_8_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_8_out", "role": "ap_vld" }} , 
 	{ "name": "G_7_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_7_out", "role": "default" }} , 
 	{ "name": "G_7_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_7_out", "role": "ap_vld" }} , 
 	{ "name": "G_6_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_6_out", "role": "default" }} , 
 	{ "name": "G_6_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_6_out", "role": "ap_vld" }} , 
 	{ "name": "G_5_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_5_out", "role": "default" }} , 
 	{ "name": "G_5_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_5_out", "role": "ap_vld" }} , 
 	{ "name": "G_4_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_4_out", "role": "default" }} , 
 	{ "name": "G_4_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_4_out", "role": "ap_vld" }} , 
 	{ "name": "G_3_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_3_out", "role": "default" }} , 
 	{ "name": "G_3_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_3_out", "role": "ap_vld" }} , 
 	{ "name": "G_2_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_2_out", "role": "default" }} , 
 	{ "name": "G_2_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_2_out", "role": "ap_vld" }} , 
 	{ "name": "G_1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":18, "type": "signal", "bundle":{"name": "G_1_out", "role": "default" }} , 
 	{ "name": "G_1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "G_1_out", "role": "ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
	riccati_backward_pass_Pipeline_VITIS_LOOP_742_9 {
		s1 {Type I LastRead 0 FirstWrite -1}
		s0 {Type I LastRead 0 FirstWrite -1}
		s1_1 {Type I LastRead 0 FirstWrite -1}
		s0_1 {Type I LastRead 0 FirstWrite -1}
		M_8 {Type I LastRead 0 FirstWrite -1}
		M {Type I LastRead 0 FirstWrite -1}
		M_10 {Type I LastRead 0 FirstWrite -1}
		M_9 {Type I LastRead 0 FirstWrite -1}
		M_12 {Type I LastRead 0 FirstWrite -1}
		M_11 {Type I LastRead 0 FirstWrite -1}
		M_14 {Type I LastRead 0 FirstWrite -1}
		M_13 {Type I LastRead 0 FirstWrite -1}
		b_215_cast {Type I LastRead 0 FirstWrite -1}
		b_216_cast {Type I LastRead 0 FirstWrite -1}
		b_217_cast {Type I LastRead 0 FirstWrite -1}
		b_218_cast {Type I LastRead 0 FirstWrite -1}
		b_219_cast {Type I LastRead 0 FirstWrite -1}
		b_220_cast {Type I LastRead 0 FirstWrite -1}
		b_158_cast {Type I LastRead 0 FirstWrite -1}
		b_159_cast {Type I LastRead 0 FirstWrite -1}
		b_160_cast {Type I LastRead 0 FirstWrite -1}
		b_161_cast {Type I LastRead 0 FirstWrite -1}
		b_162_cast {Type I LastRead 0 FirstWrite -1}
		b_163_cast {Type I LastRead 0 FirstWrite -1}
		b_164_cast {Type I LastRead 0 FirstWrite -1}
		b_165_cast {Type I LastRead 0 FirstWrite -1}
		b_166_cast {Type I LastRead 0 FirstWrite -1}
		b_167_cast {Type I LastRead 0 FirstWrite -1}
		b_168_cast {Type I LastRead 0 FirstWrite -1}
		b_169_cast {Type I LastRead 0 FirstWrite -1}
		b_170_cast {Type I LastRead 0 FirstWrite -1}
		b_171_cast {Type I LastRead 0 FirstWrite -1}
		b_172_cast {Type I LastRead 0 FirstWrite -1}
		b_173_cast {Type I LastRead 0 FirstWrite -1}
		b_174_cast {Type I LastRead 0 FirstWrite -1}
		b_175_cast {Type I LastRead 0 FirstWrite -1}
		b_176_cast {Type I LastRead 0 FirstWrite -1}
		b_177_cast {Type I LastRead 0 FirstWrite -1}
		b_178_cast {Type I LastRead 0 FirstWrite -1}
		b_179_cast {Type I LastRead 0 FirstWrite -1}
		b_180_cast {Type I LastRead 0 FirstWrite -1}
		b_181_cast {Type I LastRead 0 FirstWrite -1}
		b_182_cast {Type I LastRead 0 FirstWrite -1}
		b_183_cast {Type I LastRead 0 FirstWrite -1}
		b_184_cast {Type I LastRead 0 FirstWrite -1}
		b_185_cast {Type I LastRead 0 FirstWrite -1}
		b_186_cast {Type I LastRead 0 FirstWrite -1}
		sext_ln742 {Type I LastRead 0 FirstWrite -1}
		G_12_out {Type O LastRead -1 FirstWrite 0}
		G_11_out {Type O LastRead -1 FirstWrite 0}
		G_10_out {Type O LastRead -1 FirstWrite 0}
		G_9_out {Type O LastRead -1 FirstWrite 0}
		G_8_out {Type O LastRead -1 FirstWrite 0}
		G_7_out {Type O LastRead -1 FirstWrite 0}
		G_6_out {Type O LastRead -1 FirstWrite 0}
		G_5_out {Type O LastRead -1 FirstWrite 0}
		G_4_out {Type O LastRead -1 FirstWrite 0}
		G_3_out {Type O LastRead -1 FirstWrite 0}
		G_2_out {Type O LastRead -1 FirstWrite 0}
		G_1_out {Type O LastRead -1 FirstWrite 0}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}
	sum6_MG_QP_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "4", "Max" : "4"}
	, {"Name" : "Interval", "Min" : "3", "Max" : "3"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	s1 { ap_none {  { s1 in_data 0 35 } } }
	s0 { ap_none {  { s0 in_data 0 35 } } }
	s1_1 { ap_none {  { s1_1 in_data 0 35 } } }
	s0_1 { ap_none {  { s0_1 in_data 0 35 } } }
	M_8 { ap_none {  { M_8 in_data 0 18 } } }
	M { ap_none {  { M in_data 0 18 } } }
	M_10 { ap_none {  { M_10 in_data 0 18 } } }
	M_9 { ap_none {  { M_9 in_data 0 18 } } }
	M_12 { ap_none {  { M_12 in_data 0 18 } } }
	M_11 { ap_none {  { M_11 in_data 0 18 } } }
	M_14 { ap_none {  { M_14 in_data 0 18 } } }
	M_13 { ap_none {  { M_13 in_data 0 18 } } }
	b_215_cast { ap_none {  { b_215_cast in_data 0 26 } } }
	b_216_cast { ap_none {  { b_216_cast in_data 0 26 } } }
	b_217_cast { ap_none {  { b_217_cast in_data 0 26 } } }
	b_218_cast { ap_none {  { b_218_cast in_data 0 26 } } }
	b_219_cast { ap_none {  { b_219_cast in_data 0 26 } } }
	b_220_cast { ap_none {  { b_220_cast in_data 0 26 } } }
	b_158_cast { ap_none {  { b_158_cast in_data 0 26 } } }
	b_159_cast { ap_none {  { b_159_cast in_data 0 26 } } }
	b_160_cast { ap_none {  { b_160_cast in_data 0 26 } } }
	b_161_cast { ap_none {  { b_161_cast in_data 0 26 } } }
	b_162_cast { ap_none {  { b_162_cast in_data 0 26 } } }
	b_163_cast { ap_none {  { b_163_cast in_data 0 26 } } }
	b_164_cast { ap_none {  { b_164_cast in_data 0 26 } } }
	b_165_cast { ap_none {  { b_165_cast in_data 0 26 } } }
	b_166_cast { ap_none {  { b_166_cast in_data 0 26 } } }
	b_167_cast { ap_none {  { b_167_cast in_data 0 26 } } }
	b_168_cast { ap_none {  { b_168_cast in_data 0 26 } } }
	b_169_cast { ap_none {  { b_169_cast in_data 0 26 } } }
	b_170_cast { ap_none {  { b_170_cast in_data 0 26 } } }
	b_171_cast { ap_none {  { b_171_cast in_data 0 26 } } }
	b_172_cast { ap_none {  { b_172_cast in_data 0 26 } } }
	b_173_cast { ap_none {  { b_173_cast in_data 0 26 } } }
	b_174_cast { ap_none {  { b_174_cast in_data 0 26 } } }
	b_175_cast { ap_none {  { b_175_cast in_data 0 26 } } }
	b_176_cast { ap_none {  { b_176_cast in_data 0 26 } } }
	b_177_cast { ap_none {  { b_177_cast in_data 0 26 } } }
	b_178_cast { ap_none {  { b_178_cast in_data 0 26 } } }
	b_179_cast { ap_none {  { b_179_cast in_data 0 26 } } }
	b_180_cast { ap_none {  { b_180_cast in_data 0 26 } } }
	b_181_cast { ap_none {  { b_181_cast in_data 0 26 } } }
	b_182_cast { ap_none {  { b_182_cast in_data 0 26 } } }
	b_183_cast { ap_none {  { b_183_cast in_data 0 26 } } }
	b_184_cast { ap_none {  { b_184_cast in_data 0 26 } } }
	b_185_cast { ap_none {  { b_185_cast in_data 0 26 } } }
	b_186_cast { ap_none {  { b_186_cast in_data 0 26 } } }
	sext_ln742 { ap_none {  { sext_ln742 in_data 0 26 } } }
	G_12_out { ap_vld {  { G_12_out out_data 1 18 }  { G_12_out_ap_vld out_vld 1 1 } } }
	G_11_out { ap_vld {  { G_11_out out_data 1 18 }  { G_11_out_ap_vld out_vld 1 1 } } }
	G_10_out { ap_vld {  { G_10_out out_data 1 18 }  { G_10_out_ap_vld out_vld 1 1 } } }
	G_9_out { ap_vld {  { G_9_out out_data 1 18 }  { G_9_out_ap_vld out_vld 1 1 } } }
	G_8_out { ap_vld {  { G_8_out out_data 1 18 }  { G_8_out_ap_vld out_vld 1 1 } } }
	G_7_out { ap_vld {  { G_7_out out_data 1 18 }  { G_7_out_ap_vld out_vld 1 1 } } }
	G_6_out { ap_vld {  { G_6_out out_data 1 18 }  { G_6_out_ap_vld out_vld 1 1 } } }
	G_5_out { ap_vld {  { G_5_out out_data 1 18 }  { G_5_out_ap_vld out_vld 1 1 } } }
	G_4_out { ap_vld {  { G_4_out out_data 1 18 }  { G_4_out_ap_vld out_vld 1 1 } } }
	G_3_out { ap_vld {  { G_3_out out_data 1 18 }  { G_3_out_ap_vld out_vld 1 1 } } }
	G_2_out { ap_vld {  { G_2_out out_data 1 18 }  { G_2_out_ap_vld out_vld 1 1 } } }
	G_1_out { ap_vld {  { G_1_out out_data 1 18 }  { G_1_out_ap_vld out_vld 1 1 } } }
}
