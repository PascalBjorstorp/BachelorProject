//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
`timescale 1ns/1ps 

`ifndef MPC_FPGA_TOP_OPENCL_SUBSYSTEM_PKG__SV          
    `define MPC_FPGA_TOP_OPENCL_SUBSYSTEM_PKG__SV      
                                                     
    package mpc_fpga_top_opencl_subsystem_pkg;               
                                                     
        import uvm_pkg::*;                           
        import file_agent_pkg::*;                    
        import axi_pkg::*;
                                                     
        `include "uvm_macros.svh"                  
                                                     
        `include "mpc_fpga_top_opencl_config.sv"           
        `include "mpc_fpga_top_opencl_reference_model.sv"  
        `include "mpc_fpga_top_opencl_scoreboard.sv"       
        `include "mpc_fpga_top_opencl_subsystem_monitor.sv"
        `include "mpc_fpga_top_opencl_virtual_sequencer.sv"
        `include "mpc_fpga_top_opencl_pkg_sequence_lib.sv" 
        `include "mpc_fpga_top_opencl_env.sv"              
                                                     
    endpackage                                       
                                                     
`endif                                               
