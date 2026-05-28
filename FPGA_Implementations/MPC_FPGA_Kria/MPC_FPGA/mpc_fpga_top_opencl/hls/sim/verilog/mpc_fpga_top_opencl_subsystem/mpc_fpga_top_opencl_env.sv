//==============================================================
//Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2025.2 (64-bit)
//Tool Version Limit: 2025.11
//Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
//Copyright 2022-2025 Advanced Micro Devices, Inc. All Rights Reserved.
//
//==============================================================
`ifndef MPC_FPGA_TOP_OPENCL_ENV__SV                                                                                   
    `define MPC_FPGA_TOP_OPENCL_ENV__SV                                                                               
                                                                                                                    
    class axi_latency_gmem0 extends axi_latency;
        rand int    wr_latency;
        rand int    rd_latency;
        `uvm_object_utils_begin(axi_latency_gmem0)
        `uvm_object_utils_end
        function new ( string name = "axi_latency_gmem0" );
            super.new(name);
        endfunction
        virtual function int get_wr_lat();
            int delay;
            void'(std::randomize(delay) with { delay == 64;});
            wr_latency = delay;
            return wr_latency;
        endfunction
        virtual function int get_rd_lat();
            int delay;
            void'(std::randomize(delay) with { delay == 64;});
            rd_latency = delay;
            return rd_latency;
        endfunction
    endclass

    class axi_latency_gmem1 extends axi_latency;
        rand int    wr_latency;
        rand int    rd_latency;
        `uvm_object_utils_begin(axi_latency_gmem1)
        `uvm_object_utils_end
        function new ( string name = "axi_latency_gmem1" );
            super.new(name);
        endfunction
        virtual function int get_wr_lat();
            int delay;
            void'(std::randomize(delay) with { delay == 64;});
            wr_latency = delay;
            return wr_latency;
        endfunction
        virtual function int get_rd_lat();
            int delay;
            void'(std::randomize(delay) with { delay == 64;});
            rd_latency = delay;
            return rd_latency;
        endfunction
    endclass

                                                                                                                    
    class mpc_fpga_top_opencl_env extends uvm_env;                                                                          
                                                                                                                    
        axi_latency_gmem0    lat_gmem0;
        axi_latency_gmem1    lat_gmem1;
        mpc_fpga_top_opencl_virtual_sequencer mpc_fpga_top_opencl_virtual_sqr;                                                      
        mpc_fpga_top_opencl_config mpc_fpga_top_opencl_cfg;                                                                         
                                                                                                                    
        axi_pkg::axi_env#(64,64,8,3,1) axi_master_gmem0;
        axi_pkg::axi_env#(64,16,8,3,1) axi_master_gmem1;
        axi_pkg::axi_env#(6,4,4,3,1) axi_lite_control;
                                                                                                                    
        mpc_fpga_top_opencl_reference_model   refm;                                                                         
                                                                                                                    
        mpc_fpga_top_opencl_subsystem_monitor subsys_mon;                                                                   
                                                                                                                    
        `uvm_component_utils_begin(mpc_fpga_top_opencl_env)                                                                 
        `uvm_field_object (refm, UVM_DEFAULT | UVM_REFERENCE)                                                       
        `uvm_field_object (mpc_fpga_top_opencl_virtual_sqr, UVM_DEFAULT | UVM_REFERENCE)                                    
        `uvm_field_object (mpc_fpga_top_opencl_cfg        , UVM_DEFAULT)                                                    
        `uvm_component_utils_end                                                                                    
                                                                                                                    
        function new (string name = "mpc_fpga_top_opencl_env", uvm_component parent = null);                              
            super.new(name, parent);                                                                                
        endfunction                                                                                                 
                                                                                                                    
        extern virtual function void build_phase(uvm_phase phase);                                                  
        extern virtual function void connect_phase(uvm_phase phase);                                                
        extern virtual task          run_phase(uvm_phase phase);                                                    
                                                                                                                    
    endclass                                                                                                        
                                                                                                                    
    function void mpc_fpga_top_opencl_env::build_phase(uvm_phase phase);                                                    
        super.build_phase(phase);                                                                                   
        mpc_fpga_top_opencl_cfg = mpc_fpga_top_opencl_config::type_id::create("mpc_fpga_top_opencl_cfg", this);                           
                                                                                                                    

        mpc_fpga_top_opencl_cfg.gmem0_cfg.set_default();
        mpc_fpga_top_opencl_cfg.gmem0_cfg.drv_type = axi_pkg::SLAVE;
        mpc_fpga_top_opencl_cfg.gmem0_cfg.reset_level = axi_pkg::RESET_LEVEL_LOW;
        lat_gmem0 = axi_latency_gmem0::type_id::create("lat_gmem0", this);
        mpc_fpga_top_opencl_cfg.gmem0_cfg.clatency = lat_gmem0;
        mpc_fpga_top_opencl_cfg.gmem0_cfg.write_latency_mode = TRANSACTION_FIRST;
        mpc_fpga_top_opencl_cfg.gmem0_cfg.read_latency_mode = TRANSACTION_FIRST;
        uvm_config_db#(axi_pkg::axi_cfg)::set(this, "axi_master_gmem0*", "cfg", mpc_fpga_top_opencl_cfg.gmem0_cfg);
        axi_master_gmem0 = axi_pkg::axi_env#(64,64,8,3,1)::type_id::create("axi_master_gmem0", this);

        mpc_fpga_top_opencl_cfg.gmem1_cfg.set_default();
        mpc_fpga_top_opencl_cfg.gmem1_cfg.drv_type = axi_pkg::SLAVE;
        mpc_fpga_top_opencl_cfg.gmem1_cfg.reset_level = axi_pkg::RESET_LEVEL_LOW;
        lat_gmem1 = axi_latency_gmem1::type_id::create("lat_gmem1", this);
        mpc_fpga_top_opencl_cfg.gmem1_cfg.clatency = lat_gmem1;
        mpc_fpga_top_opencl_cfg.gmem1_cfg.write_latency_mode = TRANSACTION_FIRST;
        mpc_fpga_top_opencl_cfg.gmem1_cfg.read_latency_mode = TRANSACTION_FIRST;
        uvm_config_db#(axi_pkg::axi_cfg)::set(this, "axi_master_gmem1*", "cfg", mpc_fpga_top_opencl_cfg.gmem1_cfg);
        axi_master_gmem1 = axi_pkg::axi_env#(64,16,8,3,1)::type_id::create("axi_master_gmem1", this);

        mpc_fpga_top_opencl_cfg.control_cfg.set_default();
        mpc_fpga_top_opencl_cfg.control_cfg.drv_type = axi_pkg::MASTER;
        mpc_fpga_top_opencl_cfg.control_cfg.reset_level = axi_pkg::RESET_LEVEL_LOW;
        uvm_config_db#(axi_pkg::axi_cfg)::set(this, "axi_lite_control*", "cfg", mpc_fpga_top_opencl_cfg.control_cfg);
        axi_lite_control = axi_pkg::axi_env#(6,4,4,3,1)::type_id::create("axi_lite_control", this);



        refm = mpc_fpga_top_opencl_reference_model::type_id::create("refm", this);


        uvm_config_db#(mpc_fpga_top_opencl_reference_model)::set(this, "*", "refm", refm);


        `uvm_info(this.get_full_name(), "set reference model by uvm_config_db", UVM_LOW)


        subsys_mon = mpc_fpga_top_opencl_subsystem_monitor::type_id::create("subsys_mon", this);


        mpc_fpga_top_opencl_virtual_sqr = mpc_fpga_top_opencl_virtual_sequencer::type_id::create("mpc_fpga_top_opencl_virtual_sqr", this);
        `uvm_info(this.get_full_name(), "build_phase done", UVM_LOW)
    endfunction


    function void mpc_fpga_top_opencl_env::connect_phase(uvm_phase phase);
        super.connect_phase(phase);


        if(mpc_fpga_top_opencl_cfg.gmem0_cfg.drv_type==axi_pkg::MASTER ||mpc_fpga_top_opencl_cfg.gmem0_cfg.drv_type==axi_pkg::SLAVE)
            mpc_fpga_top_opencl_virtual_sqr.gmem0_sqr = axi_master_gmem0.vsqr;
        axi_master_gmem0.item_wtr_port.connect(subsys_mon.gmem0_wtr_imp);
        axi_master_gmem0.item_rtr_port.connect(subsys_mon.gmem0_rtr_imp);
        uvm_callbacks#(axi_pkg::axi_state, axi_pkg::axi_state_cbs)::add(axi_master_gmem0.state, refm.axi_memaccess_cb_gmem0);
        if(mpc_fpga_top_opencl_cfg.gmem1_cfg.drv_type==axi_pkg::MASTER ||mpc_fpga_top_opencl_cfg.gmem1_cfg.drv_type==axi_pkg::SLAVE)
            mpc_fpga_top_opencl_virtual_sqr.gmem1_sqr = axi_master_gmem1.vsqr;
        axi_master_gmem1.item_wtr_port.connect(subsys_mon.gmem1_wtr_imp);
        axi_master_gmem1.item_rtr_port.connect(subsys_mon.gmem1_rtr_imp);
        uvm_callbacks#(axi_pkg::axi_state, axi_pkg::axi_state_cbs)::add(axi_master_gmem1.state, refm.axi_memaccess_cb_gmem1);
        if(mpc_fpga_top_opencl_cfg.control_cfg.drv_type==axi_pkg::MASTER ||mpc_fpga_top_opencl_cfg.control_cfg.drv_type==axi_pkg::SLAVE)
            mpc_fpga_top_opencl_virtual_sqr.control_sqr = axi_lite_control.vsqr;
        axi_lite_control.item_wtr_port.connect(subsys_mon.control_wtr_imp);
        axi_lite_control.item_rtr_port.connect(subsys_mon.control_rtr_imp);
        refm.mpc_fpga_top_opencl_cfg = mpc_fpga_top_opencl_cfg;
        `uvm_info(this.get_full_name(), "connect phase done", UVM_LOW)
    endfunction


    task mpc_fpga_top_opencl_env::run_phase(uvm_phase phase);
        `uvm_info(this.get_full_name(), "mpc_fpga_top_opencl_env is running", UVM_LOW)
    endtask


`endif
