#!/usr/bin/env python3
"""
run_hls.py — Vitis 2025.1 HLS Script for MPC FPGA IP

Target: Xilinx Zynq UltraScale+ ZU3EG (Ultra96-V2)
Part:   xczu3eg-sbva484-1-e
Clock:  100 MHz (10 ns period)

Usage:
    cd FPGA_Implementations/MPC_FPGA
    vitis -s scripts/run_hls.py [csim|synth|cosim|package|all]
"""

import vitis
import os
import sys
import shutil

# ====== Configuration ======
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORKSPACE   = os.path.join(PROJECT_DIR, "hls_workspace")
COMP_NAME   = "mpc_fpga_hls"
PART        = "xczu3eg-sbva484-1-e"
CLOCK_NS    = "10"
TOP_FUNC    = "mpc_fpga_top"

SRC_DIR = os.path.join(PROJECT_DIR, "src")
INC_DIR = os.path.join(PROJECT_DIR, "include")
TB_DIR  = os.path.join(PROJECT_DIR, "testbench")

SRC_FILES = [
    "fp_math_hls.c",
    "vehicle_model_hls.c",
    "riccati_solver_hls.c",
    "mpc_riccati_hls.c",
    "mpc_fpga_top.c",
]

TB_FILES = [
    "tb_mpc_fpga.c",
]

# ====== Parse run mode ======
run_mode = "all"
if len(sys.argv) > 1:
    run_mode = sys.argv[1].lower()

print("=" * 60)
print(f"  MPC FPGA HLS - Vitis 2025.1")
print(f"  Part: {PART}")
print(f"  Clock: {CLOCK_NS} ns (100 MHz)")
print(f"  Mode: {run_mode}")
print("=" * 60)

# ====== Create fresh workspace ======
if os.path.exists(WORKSPACE):
    print(f"Removing existing workspace: {WORKSPACE}")
    shutil.rmtree(WORKSPACE, ignore_errors=True)

# ====== Write config file to temp location ======
cfg_path = os.path.join("/tmp", "mpc_hls_config.cfg")
cfg_lines = [f"part={PART}", "", "[hls]"]

# Source files with individual cflags
cflags = f"-I{INC_DIR} -DMPC_HLS_TARGET"
for src in SRC_FILES:
    src_path = os.path.join(SRC_DIR, src)
    cfg_lines.append(f"syn.file={src_path}")
    cfg_lines.append(f"syn.file_cflags={src_path},{cflags}")

# Testbench files with cflags
tb_cflags = f"-I{INC_DIR}"
for tb in TB_FILES:
    tb_path = os.path.join(TB_DIR, tb)
    cfg_lines.append(f"tb.file={tb_path}")
    cfg_lines.append(f"tb.file_cflags={tb_path},{tb_cflags}")

# Top function, clock, flow
cfg_lines.append(f"syn.top={TOP_FUNC}")
cfg_lines.append(f"clock={CLOCK_NS}")
cfg_lines.append("clock_uncertainty=12.5%")
cfg_lines.append("flow_target=vivado")
cfg_lines.append("package.output.format=ip_catalog")

# Optimization (pipeline_loops is supported)
cfg_lines.append("syn.compile.pipeline_loops=6")

with open(cfg_path, "w") as f:
    f.write("\n".join(cfg_lines) + "\n")

print(f"Config written to: {cfg_path}")
print("--- Config contents ---")
with open(cfg_path) as f:
    print(f.read())
print("-" * 40)

# ====== Create client and component ======
client = vitis.create_client(workspace=WORKSPACE)

comp = client.create_hls_component(
    name=COMP_NAME,
    cfg_file=cfg_path,
)
print(f"Created component: {COMP_NAME}")

# ====== Run operations ======
if run_mode in ("all", "csim"):
    print("\n" + "=" * 60)
    print("  Running C Simulation")
    print("=" * 60)
    comp.run(operation="C_SIMULATION")
    print("  C Simulation PASSED")

if run_mode in ("all", "synth"):
    print("\n" + "=" * 60)
    print("  Running C Synthesis")
    print("=" * 60)
    comp.run(operation="SYNTHESIS")
    print("  C Synthesis COMPLETE")

if run_mode in ("all", "cosim"):
    print("\n" + "=" * 60)
    print("  Running C/RTL Co-Simulation")
    print("=" * 60)
    comp.run(operation="CO_SIMULATION")
    print("  Co-Simulation COMPLETE")

if run_mode in ("all", "package"):
    print("\n" + "=" * 60)
    print("  Packaging IP")
    print("=" * 60)
    comp.run(operation="PACKAGE")
    print("  IP Package COMPLETE")

if run_mode == "impl":
    print("\n" + "=" * 60)
    print("  Running Implementation")
    print("=" * 60)
    comp.run(operation="IMPLEMENTATION")

# ====== Print report location ======
report_dir = os.path.join(WORKSPACE, COMP_NAME, "hls", "syn", "report")
if os.path.exists(report_dir):
    print(f"\nSynthesis reports in: {report_dir}")
    for f_name in os.listdir(report_dir):
        print(f"  {f_name}")

print("\n" + "=" * 60)
print(f"  Done: {run_mode}")
print("=" * 60)
