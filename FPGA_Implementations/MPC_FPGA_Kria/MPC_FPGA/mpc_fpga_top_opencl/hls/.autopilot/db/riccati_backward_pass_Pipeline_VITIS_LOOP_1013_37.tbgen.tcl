set moduleName riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37
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
set C_modelName {riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37}
set C_modelType { void 0 }
set ap_memory_interface_dict [dict create]
set C_modelArgList {
	{ a_52 int 32 regular  }
	{ a_60 int 32 regular  }
	{ a_68 int 32 regular  }
	{ a_76 int 32 regular  }
	{ a_84 int 32 regular  }
	{ a_92 int 32 regular  }
	{ sext_ln267_121 int 40 regular  }
	{ a_53 int 32 regular  }
	{ a_61 int 32 regular  }
	{ a_69 int 32 regular  }
	{ a_77 int 32 regular  }
	{ a_85 int 32 regular  }
	{ a_93 int 32 regular  }
	{ sext_ln291_41 int 40 regular  }
	{ a_54 int 32 regular  }
	{ a_62 int 32 regular  }
	{ a_70 int 32 regular  }
	{ a_78 int 32 regular  }
	{ a_86 int 32 regular  }
	{ a_94 int 32 regular  }
	{ sext_ln267_47 int 40 regular  }
	{ a_55 int 32 regular  }
	{ a_63 int 32 regular  }
	{ a_71 int 32 regular  }
	{ a_79 int 32 regular  }
	{ a_87 int 32 regular  }
	{ a_95 int 32 regular  }
	{ sext_ln267_49 int 40 regular  }
	{ a_56 int 32 regular  }
	{ a_64 int 32 regular  }
	{ a_72 int 32 regular  }
	{ a_80 int 32 regular  }
	{ a_88 int 32 regular  }
	{ a_96 int 32 regular  }
	{ sext_ln267_51 int 40 regular  }
	{ a_57 int 32 regular  }
	{ a_65 int 32 regular  }
	{ a_73 int 32 regular  }
	{ a_81 int 32 regular  }
	{ a_89 int 32 regular  }
	{ a_97 int 32 regular  }
	{ sext_ln267_45 int 40 regular  }
	{ G_11_reload int 34 regular  }
	{ G_12_reload int 34 regular  }
	{ G_13_reload int 34 regular  }
	{ G_10_reload int 34 regular  }
	{ G_9_reload int 34 regular  }
	{ G_8_reload int 34 regular  }
	{ sext_ln291_43 int 26 regular  }
	{ G_7_reload int 34 regular  }
	{ G_6_reload int 34 regular  }
	{ G_5_reload int 34 regular  }
	{ G_4_reload int 34 regular  }
	{ G_3_reload int 34 regular  }
	{ G_2_reload int 34 regular  }
	{ sext_ln1013_1 int 26 regular  }
	{ q_aug_linear_4 int 35 regular  }
	{ sext_ln572 int 32 regular  }
	{ sext_ln573 int 32 regular  }
	{ sext_ln574 int 32 regular  }
	{ sext_ln575 int 32 regular  }
	{ q_aug_linear_5 int 35 regular  }
	{ p_new_5_out int 40 regular {pointer 1}  }
	{ p_new_4_out int 40 regular {pointer 1}  }
	{ p_new_3_out int 40 regular {pointer 1}  }
	{ p_new_2_out int 40 regular {pointer 1}  }
	{ p_new_1_out int 40 regular {pointer 1}  }
	{ p_new_out int 40 regular {pointer 1}  }
}
set hasAXIMCache 0
set l_AXIML2Cache [list]
set AXIMCacheInstDict [dict create]
set C_modelArgMapList {[ 
	{ "Name" : "a_52", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_60", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_68", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_76", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_84", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_92", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln267_121", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "a_53", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_61", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_69", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_77", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_85", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_93", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln291_41", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "a_54", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_62", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_70", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_78", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_86", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_94", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln267_47", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "a_55", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_63", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_71", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_79", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_87", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_95", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln267_49", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "a_56", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_64", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_72", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_80", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_88", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_96", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln267_51", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "a_57", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_65", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_73", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_81", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_89", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "a_97", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln267_45", "interface" : "wire", "bitwidth" : 40, "direction" : "READONLY"} , 
 	{ "Name" : "G_11_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_12_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_13_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_10_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_9_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_8_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln291_43", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "G_7_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_6_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_5_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_4_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_3_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "G_2_reload", "interface" : "wire", "bitwidth" : 34, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln1013_1", "interface" : "wire", "bitwidth" : 26, "direction" : "READONLY"} , 
 	{ "Name" : "q_aug_linear_4", "interface" : "wire", "bitwidth" : 35, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln572", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln573", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln574", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "sext_ln575", "interface" : "wire", "bitwidth" : 32, "direction" : "READONLY"} , 
 	{ "Name" : "q_aug_linear_5", "interface" : "wire", "bitwidth" : 35, "direction" : "READONLY"} , 
 	{ "Name" : "p_new_5_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_new_4_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_new_3_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_new_2_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_new_1_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} , 
 	{ "Name" : "p_new_out", "interface" : "wire", "bitwidth" : 40, "direction" : "WRITEONLY"} ]}
# RTL Port declarations: 
set portNum 80
set portList { 
	{ ap_clk sc_in sc_logic 1 clock -1 } 
	{ ap_rst sc_in sc_logic 1 reset -1 active_high_sync } 
	{ ap_start sc_in sc_logic 1 start -1 } 
	{ ap_done sc_out sc_logic 1 predone -1 } 
	{ ap_idle sc_out sc_logic 1 done -1 } 
	{ ap_ready sc_out sc_logic 1 ready -1 } 
	{ a_52 sc_in sc_lv 32 signal 0 } 
	{ a_60 sc_in sc_lv 32 signal 1 } 
	{ a_68 sc_in sc_lv 32 signal 2 } 
	{ a_76 sc_in sc_lv 32 signal 3 } 
	{ a_84 sc_in sc_lv 32 signal 4 } 
	{ a_92 sc_in sc_lv 32 signal 5 } 
	{ sext_ln267_121 sc_in sc_lv 40 signal 6 } 
	{ a_53 sc_in sc_lv 32 signal 7 } 
	{ a_61 sc_in sc_lv 32 signal 8 } 
	{ a_69 sc_in sc_lv 32 signal 9 } 
	{ a_77 sc_in sc_lv 32 signal 10 } 
	{ a_85 sc_in sc_lv 32 signal 11 } 
	{ a_93 sc_in sc_lv 32 signal 12 } 
	{ sext_ln291_41 sc_in sc_lv 40 signal 13 } 
	{ a_54 sc_in sc_lv 32 signal 14 } 
	{ a_62 sc_in sc_lv 32 signal 15 } 
	{ a_70 sc_in sc_lv 32 signal 16 } 
	{ a_78 sc_in sc_lv 32 signal 17 } 
	{ a_86 sc_in sc_lv 32 signal 18 } 
	{ a_94 sc_in sc_lv 32 signal 19 } 
	{ sext_ln267_47 sc_in sc_lv 40 signal 20 } 
	{ a_55 sc_in sc_lv 32 signal 21 } 
	{ a_63 sc_in sc_lv 32 signal 22 } 
	{ a_71 sc_in sc_lv 32 signal 23 } 
	{ a_79 sc_in sc_lv 32 signal 24 } 
	{ a_87 sc_in sc_lv 32 signal 25 } 
	{ a_95 sc_in sc_lv 32 signal 26 } 
	{ sext_ln267_49 sc_in sc_lv 40 signal 27 } 
	{ a_56 sc_in sc_lv 32 signal 28 } 
	{ a_64 sc_in sc_lv 32 signal 29 } 
	{ a_72 sc_in sc_lv 32 signal 30 } 
	{ a_80 sc_in sc_lv 32 signal 31 } 
	{ a_88 sc_in sc_lv 32 signal 32 } 
	{ a_96 sc_in sc_lv 32 signal 33 } 
	{ sext_ln267_51 sc_in sc_lv 40 signal 34 } 
	{ a_57 sc_in sc_lv 32 signal 35 } 
	{ a_65 sc_in sc_lv 32 signal 36 } 
	{ a_73 sc_in sc_lv 32 signal 37 } 
	{ a_81 sc_in sc_lv 32 signal 38 } 
	{ a_89 sc_in sc_lv 32 signal 39 } 
	{ a_97 sc_in sc_lv 32 signal 40 } 
	{ sext_ln267_45 sc_in sc_lv 40 signal 41 } 
	{ G_11_reload sc_in sc_lv 34 signal 42 } 
	{ G_12_reload sc_in sc_lv 34 signal 43 } 
	{ G_13_reload sc_in sc_lv 34 signal 44 } 
	{ G_10_reload sc_in sc_lv 34 signal 45 } 
	{ G_9_reload sc_in sc_lv 34 signal 46 } 
	{ G_8_reload sc_in sc_lv 34 signal 47 } 
	{ sext_ln291_43 sc_in sc_lv 26 signal 48 } 
	{ G_7_reload sc_in sc_lv 34 signal 49 } 
	{ G_6_reload sc_in sc_lv 34 signal 50 } 
	{ G_5_reload sc_in sc_lv 34 signal 51 } 
	{ G_4_reload sc_in sc_lv 34 signal 52 } 
	{ G_3_reload sc_in sc_lv 34 signal 53 } 
	{ G_2_reload sc_in sc_lv 34 signal 54 } 
	{ sext_ln1013_1 sc_in sc_lv 26 signal 55 } 
	{ q_aug_linear_4 sc_in sc_lv 35 signal 56 } 
	{ sext_ln572 sc_in sc_lv 32 signal 57 } 
	{ sext_ln573 sc_in sc_lv 32 signal 58 } 
	{ sext_ln574 sc_in sc_lv 32 signal 59 } 
	{ sext_ln575 sc_in sc_lv 32 signal 60 } 
	{ q_aug_linear_5 sc_in sc_lv 35 signal 61 } 
	{ p_new_5_out sc_out sc_lv 40 signal 62 } 
	{ p_new_5_out_ap_vld sc_out sc_logic 1 outvld 62 } 
	{ p_new_4_out sc_out sc_lv 40 signal 63 } 
	{ p_new_4_out_ap_vld sc_out sc_logic 1 outvld 63 } 
	{ p_new_3_out sc_out sc_lv 40 signal 64 } 
	{ p_new_3_out_ap_vld sc_out sc_logic 1 outvld 64 } 
	{ p_new_2_out sc_out sc_lv 40 signal 65 } 
	{ p_new_2_out_ap_vld sc_out sc_logic 1 outvld 65 } 
	{ p_new_1_out sc_out sc_lv 40 signal 66 } 
	{ p_new_1_out_ap_vld sc_out sc_logic 1 outvld 66 } 
	{ p_new_out sc_out sc_lv 40 signal 67 } 
	{ p_new_out_ap_vld sc_out sc_logic 1 outvld 67 } 
}
set NewPortList {[ 
	{ "name": "ap_clk", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "clock", "bundle":{"name": "ap_clk", "role": "default" }} , 
 	{ "name": "ap_rst", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "reset", "bundle":{"name": "ap_rst", "role": "default" }} , 
 	{ "name": "ap_start", "direction": "in", "datatype": "sc_logic", "bitwidth":1, "type": "start", "bundle":{"name": "ap_start", "role": "default" }} , 
 	{ "name": "ap_done", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "predone", "bundle":{"name": "ap_done", "role": "default" }} , 
 	{ "name": "ap_idle", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "done", "bundle":{"name": "ap_idle", "role": "default" }} , 
 	{ "name": "ap_ready", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "ready", "bundle":{"name": "ap_ready", "role": "default" }} , 
 	{ "name": "a_52", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_52", "role": "default" }} , 
 	{ "name": "a_60", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_60", "role": "default" }} , 
 	{ "name": "a_68", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_68", "role": "default" }} , 
 	{ "name": "a_76", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_76", "role": "default" }} , 
 	{ "name": "a_84", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_84", "role": "default" }} , 
 	{ "name": "a_92", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_92", "role": "default" }} , 
 	{ "name": "sext_ln267_121", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "sext_ln267_121", "role": "default" }} , 
 	{ "name": "a_53", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_53", "role": "default" }} , 
 	{ "name": "a_61", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_61", "role": "default" }} , 
 	{ "name": "a_69", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_69", "role": "default" }} , 
 	{ "name": "a_77", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_77", "role": "default" }} , 
 	{ "name": "a_85", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_85", "role": "default" }} , 
 	{ "name": "a_93", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_93", "role": "default" }} , 
 	{ "name": "sext_ln291_41", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "sext_ln291_41", "role": "default" }} , 
 	{ "name": "a_54", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_54", "role": "default" }} , 
 	{ "name": "a_62", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_62", "role": "default" }} , 
 	{ "name": "a_70", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_70", "role": "default" }} , 
 	{ "name": "a_78", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_78", "role": "default" }} , 
 	{ "name": "a_86", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_86", "role": "default" }} , 
 	{ "name": "a_94", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_94", "role": "default" }} , 
 	{ "name": "sext_ln267_47", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "sext_ln267_47", "role": "default" }} , 
 	{ "name": "a_55", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_55", "role": "default" }} , 
 	{ "name": "a_63", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_63", "role": "default" }} , 
 	{ "name": "a_71", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_71", "role": "default" }} , 
 	{ "name": "a_79", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_79", "role": "default" }} , 
 	{ "name": "a_87", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_87", "role": "default" }} , 
 	{ "name": "a_95", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_95", "role": "default" }} , 
 	{ "name": "sext_ln267_49", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "sext_ln267_49", "role": "default" }} , 
 	{ "name": "a_56", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_56", "role": "default" }} , 
 	{ "name": "a_64", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_64", "role": "default" }} , 
 	{ "name": "a_72", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_72", "role": "default" }} , 
 	{ "name": "a_80", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_80", "role": "default" }} , 
 	{ "name": "a_88", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_88", "role": "default" }} , 
 	{ "name": "a_96", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_96", "role": "default" }} , 
 	{ "name": "sext_ln267_51", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "sext_ln267_51", "role": "default" }} , 
 	{ "name": "a_57", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_57", "role": "default" }} , 
 	{ "name": "a_65", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_65", "role": "default" }} , 
 	{ "name": "a_73", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_73", "role": "default" }} , 
 	{ "name": "a_81", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_81", "role": "default" }} , 
 	{ "name": "a_89", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_89", "role": "default" }} , 
 	{ "name": "a_97", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "a_97", "role": "default" }} , 
 	{ "name": "sext_ln267_45", "direction": "in", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "sext_ln267_45", "role": "default" }} , 
 	{ "name": "G_11_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_11_reload", "role": "default" }} , 
 	{ "name": "G_12_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_12_reload", "role": "default" }} , 
 	{ "name": "G_13_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_13_reload", "role": "default" }} , 
 	{ "name": "G_10_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_10_reload", "role": "default" }} , 
 	{ "name": "G_9_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_9_reload", "role": "default" }} , 
 	{ "name": "G_8_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_8_reload", "role": "default" }} , 
 	{ "name": "sext_ln291_43", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln291_43", "role": "default" }} , 
 	{ "name": "G_7_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_7_reload", "role": "default" }} , 
 	{ "name": "G_6_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_6_reload", "role": "default" }} , 
 	{ "name": "G_5_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_5_reload", "role": "default" }} , 
 	{ "name": "G_4_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_4_reload", "role": "default" }} , 
 	{ "name": "G_3_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_3_reload", "role": "default" }} , 
 	{ "name": "G_2_reload", "direction": "in", "datatype": "sc_lv", "bitwidth":34, "type": "signal", "bundle":{"name": "G_2_reload", "role": "default" }} , 
 	{ "name": "sext_ln1013_1", "direction": "in", "datatype": "sc_lv", "bitwidth":26, "type": "signal", "bundle":{"name": "sext_ln1013_1", "role": "default" }} , 
 	{ "name": "q_aug_linear_4", "direction": "in", "datatype": "sc_lv", "bitwidth":35, "type": "signal", "bundle":{"name": "q_aug_linear_4", "role": "default" }} , 
 	{ "name": "sext_ln572", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln572", "role": "default" }} , 
 	{ "name": "sext_ln573", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln573", "role": "default" }} , 
 	{ "name": "sext_ln574", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln574", "role": "default" }} , 
 	{ "name": "sext_ln575", "direction": "in", "datatype": "sc_lv", "bitwidth":32, "type": "signal", "bundle":{"name": "sext_ln575", "role": "default" }} , 
 	{ "name": "q_aug_linear_5", "direction": "in", "datatype": "sc_lv", "bitwidth":35, "type": "signal", "bundle":{"name": "q_aug_linear_5", "role": "default" }} , 
 	{ "name": "p_new_5_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_new_5_out", "role": "default" }} , 
 	{ "name": "p_new_5_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_5_out", "role": "ap_vld" }} , 
 	{ "name": "p_new_4_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_new_4_out", "role": "default" }} , 
 	{ "name": "p_new_4_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_4_out", "role": "ap_vld" }} , 
 	{ "name": "p_new_3_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_new_3_out", "role": "default" }} , 
 	{ "name": "p_new_3_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_3_out", "role": "ap_vld" }} , 
 	{ "name": "p_new_2_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_new_2_out", "role": "default" }} , 
 	{ "name": "p_new_2_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_2_out", "role": "ap_vld" }} , 
 	{ "name": "p_new_1_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_new_1_out", "role": "default" }} , 
 	{ "name": "p_new_1_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_1_out", "role": "ap_vld" }} , 
 	{ "name": "p_new_out", "direction": "out", "datatype": "sc_lv", "bitwidth":40, "type": "signal", "bundle":{"name": "p_new_out", "role": "default" }} , 
 	{ "name": "p_new_out_ap_vld", "direction": "out", "datatype": "sc_logic", "bitwidth":1, "type": "outvld", "bundle":{"name": "p_new_out", "role": "ap_vld" }}  ]}

set ArgLastReadFirstWriteLatency {
	riccati_backward_pass_Pipeline_VITIS_LOOP_1013_37 {
		a_52 {Type I LastRead 0 FirstWrite -1}
		a_60 {Type I LastRead 0 FirstWrite -1}
		a_68 {Type I LastRead 0 FirstWrite -1}
		a_76 {Type I LastRead 0 FirstWrite -1}
		a_84 {Type I LastRead 0 FirstWrite -1}
		a_92 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_121 {Type I LastRead 0 FirstWrite -1}
		a_53 {Type I LastRead 0 FirstWrite -1}
		a_61 {Type I LastRead 0 FirstWrite -1}
		a_69 {Type I LastRead 0 FirstWrite -1}
		a_77 {Type I LastRead 0 FirstWrite -1}
		a_85 {Type I LastRead 0 FirstWrite -1}
		a_93 {Type I LastRead 0 FirstWrite -1}
		sext_ln291_41 {Type I LastRead 0 FirstWrite -1}
		a_54 {Type I LastRead 0 FirstWrite -1}
		a_62 {Type I LastRead 0 FirstWrite -1}
		a_70 {Type I LastRead 0 FirstWrite -1}
		a_78 {Type I LastRead 0 FirstWrite -1}
		a_86 {Type I LastRead 0 FirstWrite -1}
		a_94 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_47 {Type I LastRead 0 FirstWrite -1}
		a_55 {Type I LastRead 0 FirstWrite -1}
		a_63 {Type I LastRead 0 FirstWrite -1}
		a_71 {Type I LastRead 0 FirstWrite -1}
		a_79 {Type I LastRead 0 FirstWrite -1}
		a_87 {Type I LastRead 0 FirstWrite -1}
		a_95 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_49 {Type I LastRead 0 FirstWrite -1}
		a_56 {Type I LastRead 0 FirstWrite -1}
		a_64 {Type I LastRead 0 FirstWrite -1}
		a_72 {Type I LastRead 0 FirstWrite -1}
		a_80 {Type I LastRead 0 FirstWrite -1}
		a_88 {Type I LastRead 0 FirstWrite -1}
		a_96 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_51 {Type I LastRead 0 FirstWrite -1}
		a_57 {Type I LastRead 0 FirstWrite -1}
		a_65 {Type I LastRead 0 FirstWrite -1}
		a_73 {Type I LastRead 0 FirstWrite -1}
		a_81 {Type I LastRead 0 FirstWrite -1}
		a_89 {Type I LastRead 0 FirstWrite -1}
		a_97 {Type I LastRead 0 FirstWrite -1}
		sext_ln267_45 {Type I LastRead 0 FirstWrite -1}
		G_11_reload {Type I LastRead 0 FirstWrite -1}
		G_12_reload {Type I LastRead 0 FirstWrite -1}
		G_13_reload {Type I LastRead 0 FirstWrite -1}
		G_10_reload {Type I LastRead 0 FirstWrite -1}
		G_9_reload {Type I LastRead 0 FirstWrite -1}
		G_8_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln291_43 {Type I LastRead 0 FirstWrite -1}
		G_7_reload {Type I LastRead 0 FirstWrite -1}
		G_6_reload {Type I LastRead 0 FirstWrite -1}
		G_5_reload {Type I LastRead 0 FirstWrite -1}
		G_4_reload {Type I LastRead 0 FirstWrite -1}
		G_3_reload {Type I LastRead 0 FirstWrite -1}
		G_2_reload {Type I LastRead 0 FirstWrite -1}
		sext_ln1013_1 {Type I LastRead 0 FirstWrite -1}
		q_aug_linear_4 {Type I LastRead 0 FirstWrite -1}
		sext_ln572 {Type I LastRead 0 FirstWrite -1}
		sext_ln573 {Type I LastRead 0 FirstWrite -1}
		sext_ln574 {Type I LastRead 0 FirstWrite -1}
		sext_ln575 {Type I LastRead 0 FirstWrite -1}
		q_aug_linear_5 {Type I LastRead 0 FirstWrite -1}
		p_new_5_out {Type O LastRead -1 FirstWrite 0}
		p_new_4_out {Type O LastRead -1 FirstWrite 0}
		p_new_3_out {Type O LastRead -1 FirstWrite 0}
		p_new_2_out {Type O LastRead -1 FirstWrite 0}
		p_new_1_out {Type O LastRead -1 FirstWrite 0}
		p_new_out {Type O LastRead -1 FirstWrite 0}}
	sum8_P_MIX_raw {
		a0 {Type I LastRead 0 FirstWrite -1}
		a1 {Type I LastRead 0 FirstWrite -1}
		a2 {Type I LastRead 0 FirstWrite -1}
		a3 {Type I LastRead 0 FirstWrite -1}
		a4 {Type I LastRead 0 FirstWrite -1}
		a5 {Type I LastRead 0 FirstWrite -1}
		a6 {Type I LastRead 0 FirstWrite -1}
		a7 {Type I LastRead 0 FirstWrite -1}}}

set hasDtUnsupportedChannel 0

set PerformanceInfo {[
	{"Name" : "Latency", "Min" : "11", "Max" : "11"}
	, {"Name" : "Interval", "Min" : "7", "Max" : "7"}
]}

set PipelineEnableSignalInfo {[
	{"Pipeline" : "0", "EnableSignal" : "ap_enable_pp0"}
]}

set Spec2ImplPortList { 
	a_52 { ap_none {  { a_52 in_data 0 32 } } }
	a_60 { ap_none {  { a_60 in_data 0 32 } } }
	a_68 { ap_none {  { a_68 in_data 0 32 } } }
	a_76 { ap_none {  { a_76 in_data 0 32 } } }
	a_84 { ap_none {  { a_84 in_data 0 32 } } }
	a_92 { ap_none {  { a_92 in_data 0 32 } } }
	sext_ln267_121 { ap_none {  { sext_ln267_121 in_data 0 40 } } }
	a_53 { ap_none {  { a_53 in_data 0 32 } } }
	a_61 { ap_none {  { a_61 in_data 0 32 } } }
	a_69 { ap_none {  { a_69 in_data 0 32 } } }
	a_77 { ap_none {  { a_77 in_data 0 32 } } }
	a_85 { ap_none {  { a_85 in_data 0 32 } } }
	a_93 { ap_none {  { a_93 in_data 0 32 } } }
	sext_ln291_41 { ap_none {  { sext_ln291_41 in_data 0 40 } } }
	a_54 { ap_none {  { a_54 in_data 0 32 } } }
	a_62 { ap_none {  { a_62 in_data 0 32 } } }
	a_70 { ap_none {  { a_70 in_data 0 32 } } }
	a_78 { ap_none {  { a_78 in_data 0 32 } } }
	a_86 { ap_none {  { a_86 in_data 0 32 } } }
	a_94 { ap_none {  { a_94 in_data 0 32 } } }
	sext_ln267_47 { ap_none {  { sext_ln267_47 in_data 0 40 } } }
	a_55 { ap_none {  { a_55 in_data 0 32 } } }
	a_63 { ap_none {  { a_63 in_data 0 32 } } }
	a_71 { ap_none {  { a_71 in_data 0 32 } } }
	a_79 { ap_none {  { a_79 in_data 0 32 } } }
	a_87 { ap_none {  { a_87 in_data 0 32 } } }
	a_95 { ap_none {  { a_95 in_data 0 32 } } }
	sext_ln267_49 { ap_none {  { sext_ln267_49 in_data 0 40 } } }
	a_56 { ap_none {  { a_56 in_data 0 32 } } }
	a_64 { ap_none {  { a_64 in_data 0 32 } } }
	a_72 { ap_none {  { a_72 in_data 0 32 } } }
	a_80 { ap_none {  { a_80 in_data 0 32 } } }
	a_88 { ap_none {  { a_88 in_data 0 32 } } }
	a_96 { ap_none {  { a_96 in_data 0 32 } } }
	sext_ln267_51 { ap_none {  { sext_ln267_51 in_data 0 40 } } }
	a_57 { ap_none {  { a_57 in_data 0 32 } } }
	a_65 { ap_none {  { a_65 in_data 0 32 } } }
	a_73 { ap_none {  { a_73 in_data 0 32 } } }
	a_81 { ap_none {  { a_81 in_data 0 32 } } }
	a_89 { ap_none {  { a_89 in_data 0 32 } } }
	a_97 { ap_none {  { a_97 in_data 0 32 } } }
	sext_ln267_45 { ap_none {  { sext_ln267_45 in_data 0 40 } } }
	G_11_reload { ap_none {  { G_11_reload in_data 0 34 } } }
	G_12_reload { ap_none {  { G_12_reload in_data 0 34 } } }
	G_13_reload { ap_none {  { G_13_reload in_data 0 34 } } }
	G_10_reload { ap_none {  { G_10_reload in_data 0 34 } } }
	G_9_reload { ap_none {  { G_9_reload in_data 0 34 } } }
	G_8_reload { ap_none {  { G_8_reload in_data 0 34 } } }
	sext_ln291_43 { ap_none {  { sext_ln291_43 in_data 0 26 } } }
	G_7_reload { ap_none {  { G_7_reload in_data 0 34 } } }
	G_6_reload { ap_none {  { G_6_reload in_data 0 34 } } }
	G_5_reload { ap_none {  { G_5_reload in_data 0 34 } } }
	G_4_reload { ap_none {  { G_4_reload in_data 0 34 } } }
	G_3_reload { ap_none {  { G_3_reload in_data 0 34 } } }
	G_2_reload { ap_none {  { G_2_reload in_data 0 34 } } }
	sext_ln1013_1 { ap_none {  { sext_ln1013_1 in_data 0 26 } } }
	q_aug_linear_4 { ap_none {  { q_aug_linear_4 in_data 0 35 } } }
	sext_ln572 { ap_none {  { sext_ln572 in_data 0 32 } } }
	sext_ln573 { ap_none {  { sext_ln573 in_data 0 32 } } }
	sext_ln574 { ap_none {  { sext_ln574 in_data 0 32 } } }
	sext_ln575 { ap_none {  { sext_ln575 in_data 0 32 } } }
	q_aug_linear_5 { ap_none {  { q_aug_linear_5 in_data 0 35 } } }
	p_new_5_out { ap_vld {  { p_new_5_out out_data 1 40 }  { p_new_5_out_ap_vld out_vld 1 1 } } }
	p_new_4_out { ap_vld {  { p_new_4_out out_data 1 40 }  { p_new_4_out_ap_vld out_vld 1 1 } } }
	p_new_3_out { ap_vld {  { p_new_3_out out_data 1 40 }  { p_new_3_out_ap_vld out_vld 1 1 } } }
	p_new_2_out { ap_vld {  { p_new_2_out out_data 1 40 }  { p_new_2_out_ap_vld out_vld 1 1 } } }
	p_new_1_out { ap_vld {  { p_new_1_out out_data 1 40 }  { p_new_1_out_ap_vld out_vld 1 1 } } }
	p_new_out { ap_vld {  { p_new_out out_data 1 40 }  { p_new_out_ap_vld out_vld 1 1 } } }
}
