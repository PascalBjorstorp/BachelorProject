//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.

`ifndef AXI_VIRTUAL_SEQUENCER__SV
`define AXI_VIRTUAL_SEQUENCER__SV

class axi_virtual_sequencer extends uvm_sequencer;
    axi_cfg             cfg ;
    axi_state          state;
    axi_sequencer      slave_awsqr;
    axi_sequencer      slave_wsqr ;
    axi_sequencer      slave_bsqr ;
    axi_sequencer      slave_arsqr;
    axi_sequencer      slave_rsqr ;
    axi_sequencer      master_awsqr;
    axi_sequencer      master_wsqr ;
    axi_sequencer      master_bsqr ;
    axi_sequencer      master_arsqr;
    axi_sequencer      master_rsqr ;
    `uvm_component_utils_begin(axi_virtual_sequencer)
    `uvm_component_utils_end

    function new (string name, uvm_component parent);
        super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
        super.build();
    endfunction
endclass
`endif
