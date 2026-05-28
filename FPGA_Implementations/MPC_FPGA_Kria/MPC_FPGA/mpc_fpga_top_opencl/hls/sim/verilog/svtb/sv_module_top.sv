//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================

`ifndef SV_MODULE_TOP_SV
`define SV_MODULE_TOP_SV


`timescale 1ns/1ps


`include "uvm_macros.svh"
import uvm_pkg::*;
import file_agent_pkg::*;
import mpc_fpga_top_opencl_subsystem_pkg::*;
`include "mpc_fpga_top_opencl_subsys_test_sequence_lib.sv"
`include "mpc_fpga_top_opencl_test_lib.sv"


module sv_module_top;


    misc_interface              misc_if ( .clock(apatb_mpc_fpga_top_opencl_top.AESL_clock), .reset(apatb_mpc_fpga_top_opencl_top.AESL_reset) );
    assign misc_if.dut2tb_ap_ready = apatb_mpc_fpga_top_opencl_top.AESL_inst_mpc_fpga_top_opencl.ap_ready;
    assign misc_if.dut2tb_ap_done_kernel = apatb_mpc_fpga_top_opencl_top.AESL_inst_mpc_fpga_top_opencl.ap_done;
    initial begin
        uvm_config_db #(virtual misc_interface)::set(null, "uvm_test_top.top_env.*", "misc_if", misc_if);
    end


    axi_if #(64,64,8,3,1)  axi_gmem0_if (.clk  (apatb_mpc_fpga_top_opencl_top.AESL_clock), .rst(apatb_mpc_fpga_top_opencl_top.AESL_reset));
    assign axi_gmem0_if.AWVALID = apatb_mpc_fpga_top_opencl_top.gmem0_AWVALID;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_AWREADY = axi_gmem0_if.AWREADY;
    assign axi_gmem0_if.AWADDR = apatb_mpc_fpga_top_opencl_top.gmem0_AWADDR;
    assign axi_gmem0_if.AWID = apatb_mpc_fpga_top_opencl_top.gmem0_AWID;
    assign axi_gmem0_if.AWLEN = apatb_mpc_fpga_top_opencl_top.gmem0_AWLEN;
    assign axi_gmem0_if.AWSIZE = apatb_mpc_fpga_top_opencl_top.gmem0_AWSIZE;
    assign axi_gmem0_if.AWBURST = apatb_mpc_fpga_top_opencl_top.gmem0_AWBURST;
    assign axi_gmem0_if.AWLOCK = apatb_mpc_fpga_top_opencl_top.gmem0_AWLOCK;
    assign axi_gmem0_if.AWCACHE = apatb_mpc_fpga_top_opencl_top.gmem0_AWCACHE;
    assign axi_gmem0_if.AWPROT = apatb_mpc_fpga_top_opencl_top.gmem0_AWPROT;
    assign axi_gmem0_if.AWQOS = apatb_mpc_fpga_top_opencl_top.gmem0_AWQOS;
    assign axi_gmem0_if.AWREGION = apatb_mpc_fpga_top_opencl_top.gmem0_AWREGION;
    assign axi_gmem0_if.AWUSER = apatb_mpc_fpga_top_opencl_top.gmem0_AWUSER;
    assign axi_gmem0_if.WVALID = apatb_mpc_fpga_top_opencl_top.gmem0_WVALID;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_WREADY = axi_gmem0_if.WREADY;
    assign axi_gmem0_if.WDATA = apatb_mpc_fpga_top_opencl_top.gmem0_WDATA;
    assign axi_gmem0_if.WSTRB = apatb_mpc_fpga_top_opencl_top.gmem0_WSTRB;
    assign axi_gmem0_if.WLAST = apatb_mpc_fpga_top_opencl_top.gmem0_WLAST;
    assign axi_gmem0_if.WID = apatb_mpc_fpga_top_opencl_top.gmem0_WID;
    assign axi_gmem0_if.WUSER = apatb_mpc_fpga_top_opencl_top.gmem0_WUSER;
    assign axi_gmem0_if.ARVALID = apatb_mpc_fpga_top_opencl_top.gmem0_ARVALID;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_ARREADY = axi_gmem0_if.ARREADY;
    assign axi_gmem0_if.ARADDR = apatb_mpc_fpga_top_opencl_top.gmem0_ARADDR;
    assign axi_gmem0_if.ARID = apatb_mpc_fpga_top_opencl_top.gmem0_ARID;
    assign axi_gmem0_if.ARLEN = apatb_mpc_fpga_top_opencl_top.gmem0_ARLEN;
    assign axi_gmem0_if.ARSIZE = apatb_mpc_fpga_top_opencl_top.gmem0_ARSIZE;
    assign axi_gmem0_if.ARBURST = apatb_mpc_fpga_top_opencl_top.gmem0_ARBURST;
    assign axi_gmem0_if.ARLOCK = apatb_mpc_fpga_top_opencl_top.gmem0_ARLOCK;
    assign axi_gmem0_if.ARCACHE = apatb_mpc_fpga_top_opencl_top.gmem0_ARCACHE;
    assign axi_gmem0_if.ARPROT = apatb_mpc_fpga_top_opencl_top.gmem0_ARPROT;
    assign axi_gmem0_if.ARQOS = apatb_mpc_fpga_top_opencl_top.gmem0_ARQOS;
    assign axi_gmem0_if.ARREGION = apatb_mpc_fpga_top_opencl_top.gmem0_ARREGION;
    assign axi_gmem0_if.ARUSER = apatb_mpc_fpga_top_opencl_top.gmem0_ARUSER;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_RVALID = axi_gmem0_if.RVALID;
    assign axi_gmem0_if.RREADY = apatb_mpc_fpga_top_opencl_top.gmem0_RREADY;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_RDATA = axi_gmem0_if.RDATA;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_RLAST = axi_gmem0_if.RLAST;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_RID = axi_gmem0_if.RID;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_RUSER = axi_gmem0_if.RUSER;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_RRESP = axi_gmem0_if.RRESP;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_BVALID = axi_gmem0_if.BVALID;
    assign axi_gmem0_if.BREADY = apatb_mpc_fpga_top_opencl_top.gmem0_BREADY;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_BRESP = axi_gmem0_if.BRESP;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_BID = axi_gmem0_if.BID;
    assign apatb_mpc_fpga_top_opencl_top.gmem0_BUSER = axi_gmem0_if.BUSER;
    initial begin
        uvm_config_db #( virtual axi_if#(64,64,8,3,1) )::set(null, "uvm_test_top.top_env.axi_master_gmem0.*", "vif", axi_gmem0_if);
    end


    axi_if #(64,16,8,3,1)  axi_gmem1_if (.clk  (apatb_mpc_fpga_top_opencl_top.AESL_clock), .rst(apatb_mpc_fpga_top_opencl_top.AESL_reset));
    assign axi_gmem1_if.AWVALID = apatb_mpc_fpga_top_opencl_top.gmem1_AWVALID;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_AWREADY = axi_gmem1_if.AWREADY;
    assign axi_gmem1_if.AWADDR = apatb_mpc_fpga_top_opencl_top.gmem1_AWADDR;
    assign axi_gmem1_if.AWID = apatb_mpc_fpga_top_opencl_top.gmem1_AWID;
    assign axi_gmem1_if.AWLEN = apatb_mpc_fpga_top_opencl_top.gmem1_AWLEN;
    assign axi_gmem1_if.AWSIZE = apatb_mpc_fpga_top_opencl_top.gmem1_AWSIZE;
    assign axi_gmem1_if.AWBURST = apatb_mpc_fpga_top_opencl_top.gmem1_AWBURST;
    assign axi_gmem1_if.AWLOCK = apatb_mpc_fpga_top_opencl_top.gmem1_AWLOCK;
    assign axi_gmem1_if.AWCACHE = apatb_mpc_fpga_top_opencl_top.gmem1_AWCACHE;
    assign axi_gmem1_if.AWPROT = apatb_mpc_fpga_top_opencl_top.gmem1_AWPROT;
    assign axi_gmem1_if.AWQOS = apatb_mpc_fpga_top_opencl_top.gmem1_AWQOS;
    assign axi_gmem1_if.AWREGION = apatb_mpc_fpga_top_opencl_top.gmem1_AWREGION;
    assign axi_gmem1_if.AWUSER = apatb_mpc_fpga_top_opencl_top.gmem1_AWUSER;
    assign axi_gmem1_if.WVALID = apatb_mpc_fpga_top_opencl_top.gmem1_WVALID;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_WREADY = axi_gmem1_if.WREADY;
    assign axi_gmem1_if.WDATA = apatb_mpc_fpga_top_opencl_top.gmem1_WDATA;
    assign axi_gmem1_if.WSTRB = apatb_mpc_fpga_top_opencl_top.gmem1_WSTRB;
    assign axi_gmem1_if.WLAST = apatb_mpc_fpga_top_opencl_top.gmem1_WLAST;
    assign axi_gmem1_if.WID = apatb_mpc_fpga_top_opencl_top.gmem1_WID;
    assign axi_gmem1_if.WUSER = apatb_mpc_fpga_top_opencl_top.gmem1_WUSER;
    assign axi_gmem1_if.ARVALID = apatb_mpc_fpga_top_opencl_top.gmem1_ARVALID;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_ARREADY = axi_gmem1_if.ARREADY;
    assign axi_gmem1_if.ARADDR = apatb_mpc_fpga_top_opencl_top.gmem1_ARADDR;
    assign axi_gmem1_if.ARID = apatb_mpc_fpga_top_opencl_top.gmem1_ARID;
    assign axi_gmem1_if.ARLEN = apatb_mpc_fpga_top_opencl_top.gmem1_ARLEN;
    assign axi_gmem1_if.ARSIZE = apatb_mpc_fpga_top_opencl_top.gmem1_ARSIZE;
    assign axi_gmem1_if.ARBURST = apatb_mpc_fpga_top_opencl_top.gmem1_ARBURST;
    assign axi_gmem1_if.ARLOCK = apatb_mpc_fpga_top_opencl_top.gmem1_ARLOCK;
    assign axi_gmem1_if.ARCACHE = apatb_mpc_fpga_top_opencl_top.gmem1_ARCACHE;
    assign axi_gmem1_if.ARPROT = apatb_mpc_fpga_top_opencl_top.gmem1_ARPROT;
    assign axi_gmem1_if.ARQOS = apatb_mpc_fpga_top_opencl_top.gmem1_ARQOS;
    assign axi_gmem1_if.ARREGION = apatb_mpc_fpga_top_opencl_top.gmem1_ARREGION;
    assign axi_gmem1_if.ARUSER = apatb_mpc_fpga_top_opencl_top.gmem1_ARUSER;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_RVALID = axi_gmem1_if.RVALID;
    assign axi_gmem1_if.RREADY = apatb_mpc_fpga_top_opencl_top.gmem1_RREADY;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_RDATA = axi_gmem1_if.RDATA;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_RLAST = axi_gmem1_if.RLAST;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_RID = axi_gmem1_if.RID;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_RUSER = axi_gmem1_if.RUSER;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_RRESP = axi_gmem1_if.RRESP;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_BVALID = axi_gmem1_if.BVALID;
    assign axi_gmem1_if.BREADY = apatb_mpc_fpga_top_opencl_top.gmem1_BREADY;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_BRESP = axi_gmem1_if.BRESP;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_BID = axi_gmem1_if.BID;
    assign apatb_mpc_fpga_top_opencl_top.gmem1_BUSER = axi_gmem1_if.BUSER;
    initial begin
        uvm_config_db #( virtual axi_if#(64,16,8,3,1) )::set(null, "uvm_test_top.top_env.axi_master_gmem1.*", "vif", axi_gmem1_if);
    end


    axi_if #(6,4,4,3,1)  axi_control_if (.clk  (apatb_mpc_fpga_top_opencl_top.AESL_clock), .rst(apatb_mpc_fpga_top_opencl_top.AESL_reset));
    assign apatb_mpc_fpga_top_opencl_top.control_AWADDR = axi_control_if.AWADDR;
    assign apatb_mpc_fpga_top_opencl_top.control_AWVALID = axi_control_if.AWVALID;
    assign axi_control_if.AWREADY = apatb_mpc_fpga_top_opencl_top.control_AWREADY;
    assign apatb_mpc_fpga_top_opencl_top.control_WVALID = axi_control_if.WVALID;
    assign axi_control_if.WREADY = apatb_mpc_fpga_top_opencl_top.control_WREADY;
    assign apatb_mpc_fpga_top_opencl_top.control_WDATA = axi_control_if.WDATA;
    assign apatb_mpc_fpga_top_opencl_top.control_WSTRB = axi_control_if.WSTRB;
    assign apatb_mpc_fpga_top_opencl_top.control_ARADDR = axi_control_if.ARADDR;
    assign apatb_mpc_fpga_top_opencl_top.control_ARVALID = axi_control_if.ARVALID;
    assign axi_control_if.ARREADY = apatb_mpc_fpga_top_opencl_top.control_ARREADY;
    assign axi_control_if.RVALID = apatb_mpc_fpga_top_opencl_top.control_RVALID;
    assign apatb_mpc_fpga_top_opencl_top.control_RREADY = axi_control_if.RREADY;
    assign axi_control_if.RDATA = apatb_mpc_fpga_top_opencl_top.control_RDATA;
    assign axi_control_if.RRESP = apatb_mpc_fpga_top_opencl_top.control_RRESP;
    assign axi_control_if.BVALID = apatb_mpc_fpga_top_opencl_top.control_BVALID;
    assign apatb_mpc_fpga_top_opencl_top.control_BREADY = axi_control_if.BREADY;
    assign axi_control_if.BRESP = apatb_mpc_fpga_top_opencl_top.control_BRESP;
    assign axi_control_if.BID = 0;
    assign axi_control_if.RID = 0;
    assign axi_control_if.RLAST = 1;
    initial begin
        uvm_config_db #( virtual axi_if#(6,4,4,3,1) )::set(null, "uvm_test_top.top_env.axi_lite_control.*", "vif", axi_control_if);
    end


    initial begin
        run_test();
    end
endmodule
`endif
