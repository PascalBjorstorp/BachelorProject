//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
`ifndef MPC_FPGA_TOP_OPENCL_SCOREBOARD__SV                                                       
    `define MPC_FPGA_TOP_OPENCL_SCOREBOARD__SV                                                   
                                                                                               
                                                                                               
    class mpc_fpga_top_opencl_scoreboard extends uvm_component;                                        
                                                                                               
        mpc_fpga_top_opencl_reference_model refm;                                                      
                                                                                               
                                                                                               
        `uvm_component_utils_begin(mpc_fpga_top_opencl_scoreboard)                                     
        `uvm_field_object(refm  , UVM_DEFAULT)                                                 
        `uvm_component_utils_end                                                               
                                                                                               
        virtual function void build_phase(uvm_phase phase);                                    
            if (!uvm_config_db #(mpc_fpga_top_opencl_reference_model)::get(this, "", "refm", refm))
                `uvm_fatal(this.get_full_name(), "No refm from high level")                  
            `uvm_info(this.get_full_name(), "get reference model by uvm_config_db", UVM_MEDIUM) 
                                                                                               
        endfunction                                                                            
                                                                                               
        function new (string name = "", uvm_component parent = null);                        
            super.new(name, parent);                                                           
        endfunction                                                                            
                                                                                               
        virtual task run_phase(uvm_phase phase);                                               

            fork                                                                               
                forever begin
                    @refm.allaxilite_write_data_finish;
                    `uvm_info(this.get_full_name(), "receive allaxilite_write_finish axilite write_mem_page_process", UVM_LOW)
                end
                                                                                               
                forever begin
                    @refm.dut2tb_ap_done;
                    `uvm_info(this.get_full_name(), "receive dut2tb_ap_done and do axim dump", UVM_LOW)
                            refm.mem_blk_pages_gmem0.tvout_dump_frontpage(0);
                            refm.mem_blk_pages_gmem1.tvout_dump_frontpage(1);
                end                                                                            
                begin                                                                          
                    @refm.finish;                                                              
                    `uvm_info(this.get_full_name(), "receive FINISH", UVM_LOW)               
                end                                                                            
            join                                                                               
        endtask                                                                                
                                                                                               
        virtual function void write_axi_wtr_gmem0(axi_pkg::axi_transfer tr);
        endfunction

        virtual function void write_axi_rtr_gmem0(axi_pkg::axi_transfer tr);
        endfunction

        virtual function void write_axi_wtr_gmem1(axi_pkg::axi_transfer tr);
        endfunction

        virtual function void write_axi_rtr_gmem1(axi_pkg::axi_transfer tr);
        endfunction

        virtual function void write_axi_wtr_control(axi_pkg::axi_transfer tr);
        endfunction

        virtual function void write_axi_rtr_control(axi_pkg::axi_transfer tr);
        endfunction

    endclass                                                                                   
                                                                                               
`endif                                                                                         
