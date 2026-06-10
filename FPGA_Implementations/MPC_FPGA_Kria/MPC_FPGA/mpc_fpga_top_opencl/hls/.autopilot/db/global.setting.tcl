
set TopModule "mpc_fpga_top_opencl"
set ClockPeriod 5
set ClockList ap_clk
set AxiliteClockList {}
set HasVivadoClockPeriod 1
set CombLogicFlag 0
set PipelineFlag 0
set DataflowTaskPipelineFlag 1
set TrivialPipelineFlag 0
set noPortSwitchingFlag 0
set FloatingPointFlag 0
set FftOrFirFlag 0
set NbRWValue 0
set intNbAccess 0
set NewDSPMapping 1
set HasDSPModule 1
set ResetLevelFlag 0
set ResetStyle control
set ResetSyncFlag 1
set ResetRegisterFlag 1
set ResetVariableFlag 0
set ResetRegisterNum 1
set FsmEncStyle onehot
set MaxFanout 0
set RtlPrefix {}
set RtlSubPrefix mpc_fpga_top_opencl_
set ExtraCCFlags {}
set ExtraCLdFlags {}
set SynCheckOptions {}
set PresynOptions {}
set PreprocOptions {}
set RtlWriterOptions {}
set CbcGenFlag 0
set CasGenFlag 0
set CasMonitorFlag 0
set AutoSimOptions {}
set ExportMCPathFlag 0
set SCTraceFileName mytrace
set SCTraceFileFormat vcd
set SCTraceOption all
set TargetInfo xck26:-sfvc784:-2LV-c
set SourceFiles {sc {} c {../../../src/mpc_runtime_tune.cpp ../../../src/mpc_fpga_top.cpp ../../../src/mpc_riccati_hls.cpp ../../../src/riccati_solver_hls.cpp ../../../src/vehicle_model_hls.cpp ../../../src/fp_math_hls.cpp}}
set SourceFlags {sc {} c {{-I/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/include -DMPC_HLS_TARGET -DMPC_HLS_BUILD} {-I/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/include -DMPC_HLS_TARGET -DMPC_HLS_BUILD} {-I/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/include -DMPC_HLS_TARGET -DMPC_HLS_BUILD} {-I/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/include -DMPC_HLS_TARGET -DMPC_HLS_BUILD} {-I/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/include -DMPC_HLS_TARGET -DMPC_HLS_BUILD} {-I/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/include -DMPC_HLS_TARGET -DMPC_HLS_BUILD}}}
set DirectiveFile {}
set TBFiles {verilog /home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/testbench/tb_top_min.cpp bc /home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/testbench/tb_top_min.cpp vhdl /home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/testbench/tb_top_min.cpp sc /home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/testbench/tb_top_min.cpp cas /home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/testbench/tb_top_min.cpp c {}}
set SpecLanguage C
set TVInFiles {bc {} c {} sc {} cas {} vhdl {} verilog {}}
set TVOutFiles {bc {} c {} sc {} cas {} vhdl {} verilog {}}
set TBTops {verilog {} bc {} vhdl {} sc {} cas {} c {}}
set TBInstNames {verilog {} bc {} vhdl {} sc {} cas {} c {}}
set XDCFiles {}
set ExtraGlobalOptions {"area_timing" 1 "clock_gate" 1 "impl_flow" map "power_gate" 0}
set TBTVFileNotFound {}
set AppFile {}
set ApsFile hls.aps
set AvePath ../../.
set DefaultPlatform DefaultPlatform
set multiClockList {}
set SCPortClockMap {}
set intNbAccess 0
set PlatformFiles {{DefaultPlatform {xilinx/zynquplus/zynquplus}}}
set HPFPO 0
