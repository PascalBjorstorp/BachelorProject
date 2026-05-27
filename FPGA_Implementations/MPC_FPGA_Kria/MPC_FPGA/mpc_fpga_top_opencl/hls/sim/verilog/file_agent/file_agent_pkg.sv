//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.

`timescale 1ns/1ps 

`ifndef FILE_AGENT_PKG__SV
    `define FILE_AGENT_PKG__SV

    package file_agent_pkg;
        
        import uvm_pkg::*;
        `include "uvm_macros.svh"
        `include "file_read_agent.sv"
        `include "file_write_agent.sv"
        `include "mem_model.sv"
    endpackage

`endif
