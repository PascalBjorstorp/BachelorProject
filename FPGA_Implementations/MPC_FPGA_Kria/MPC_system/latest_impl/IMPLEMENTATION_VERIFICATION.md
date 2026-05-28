# Implementation Verification

Generated: 2026-05-11T13:51:27Z

## Result
Implementation and link completed successfully for latest build under build/hw.

## Evidence
- build/hw/.buildstatus contains: hw$mpc_fpga_top_opencl=SUCCESS
- v++ log contains: Run Status: impl Complete!
- v++ log contains: Created mpc_fpga_top_opencl.xclbin
- vivado log contains: Bitgen Completed Successfully.
- vivado log contains: write_bitstream completed successfully
- vivado log contains: impl_1 finished
- vivado post-route timing: WNS=0.038, TNS=0.000, WHS=0.010

## Canonical artifacts
- latest_impl/mpc_fpga_top_opencl.xclbin
- latest_impl/mpc_fpga_top_opencl.xsa
- latest_impl/system.bit
- latest_impl/mpc_fpga_top_opencl.xclbin.link_summary
- latest_impl/timing_summary_routed.rpt

Checksums and sizes: latest_impl/artifact_manifest.txt
