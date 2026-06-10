//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
`ifndef MISC_INTERFACE__SV
`define MISC_INTERFACE__SV
`timescale 1ns/1ps
interface misc_interface (input clock, input reset);


    logic    dut2tb_ap_done   ;
    logic    dut2tb_ap_done_kernel   ;
    logic    dut2tb_ap_idle   ;
    logic    dut2tb_ap_ready  ;
    logic    tb2dut_ap_start  = 0;
    logic    tb2dut_ap_continue = 0;
    logic    ap_ready_for_nexttrans = 0;
    logic    ap_done_for_nexttrans = 0;
    logic    initialed = 0;
    logic    finished = 0;
    event    dut2tb_ap_ready_evt;
    event    dut2tb_ap_done_evt;
    event    dut2tb_ap_done_kernel_evt;
    event    initialed_evt;
    event    finished_evt;


    clocking pclk @(posedge clock);
        default input #0.1ns output #0.1ns; 
    endclocking


    clocking nclk @(negedge clock);
        default input #0.1ns output #0.1ns; 
    endclocking


endinterface
`endif
