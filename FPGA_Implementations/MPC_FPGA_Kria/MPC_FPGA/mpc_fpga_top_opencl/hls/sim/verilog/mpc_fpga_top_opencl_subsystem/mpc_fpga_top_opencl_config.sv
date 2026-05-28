//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
`ifndef MPC_FPGA_TOP_OPENCL_CONFIG__SV                        
    `define MPC_FPGA_TOP_OPENCL_CONFIG__SV                    
                                                            
    class mpc_fpga_top_opencl_config extends uvm_object;            
                                                            
        int check_ena;                                      
        int cover_ena;                                      
        axi_pkg::axi_cfg gmem0_cfg;
        axi_pkg::axi_cfg gmem1_cfg;
        axi_pkg::axi_cfg control_cfg;

        `uvm_object_utils_begin(mpc_fpga_top_opencl_config)         
        `uvm_field_object(gmem0_cfg, UVM_DEFAULT);
        `uvm_field_object(gmem1_cfg, UVM_DEFAULT);
        `uvm_field_object(control_cfg, UVM_DEFAULT);
        `uvm_field_int   (check_ena , UVM_DEFAULT)          
        `uvm_field_int   (cover_ena , UVM_DEFAULT)          
        `uvm_object_utils_end                               

        function new (string name = "mpc_fpga_top_opencl_config");
            super.new(name);                                
            gmem0_cfg = new("gmem0_cfg", 1);
            gmem1_cfg = new("gmem1_cfg", 1);
            control_cfg = axi_pkg::axi_cfg::type_id::create("control_cfg");
        endfunction                                         
                                                            
    endclass                                                
                                                            
`endif                                                      
