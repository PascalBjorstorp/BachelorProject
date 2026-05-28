//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
`ifndef MPC_FPGA_TOP_OPENCL_TEST_LIB__SV                                    
    `define MPC_FPGA_TOP_OPENCL_TEST_LIB__SV                                
                                                                          
    `include "uvm_macros.svh"                                           
                                                                          
    class mpc_fpga_top_opencl_test_lib extends uvm_test;                          
                                                                          
        mpc_fpga_top_opencl_env top_env;                                          
                                                                          
        `uvm_component_utils(mpc_fpga_top_opencl_test_lib)                        
                                                                          
        function new (string name, uvm_component parent = null);          
            super.new(name, parent);                                      
        endfunction                                                       
                                                                          
        virtual function void build_phase(uvm_phase phase);               
            super.build_phase(phase);                                     
            top_env = mpc_fpga_top_opencl_subsystem_pkg::mpc_fpga_top_opencl_env::type_id::create("top_env", this);
            uvm_config_db#(uvm_object_wrapper)::set(                      
                    this,                                                 
                    "top_env.mpc_fpga_top_opencl_virtual_sqr.run_phase",        
                    "default_sequence",                                 
                    mpc_fpga_top_opencl_subsys_test_sequence_lib::type_id::get()  
                );                                                        
            `uvm_info(this.get_full_name(), "build_phase done", UVM_LOW)
        endfunction                                                       
        virtual function void end_of_elaboration_phase(uvm_phase phase);  
            uvm_root top = uvm_root::get();                               
            top.print_topology();                                         
        endfunction                                                       
    endclass                                                              
                                                                          
`endif                                                                    
