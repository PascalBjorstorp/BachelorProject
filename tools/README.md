# tools/ — Replay, Analysis, and Plotting for the Thesis

Off-line tooling used to (a) replay logged car runs through both the CPU and FPGA MPC code paths, (b) measure FPGA fixed-point widths for the word-sizing appendix, and (c) generate the figures in the thesis. Every retained script corresponds to a test in the thesis (Chapter *Test*) or to the Appendix *Empirical fixed-point width measurements*.

Bag recordings (`input/`) and per-run outputs (`output/`, `mpc_replay/width_report_out/`) are gitignored — they're regenerable from the source scripts and the recorded `.mcap` files. The compiled helper binaries are gitignored too; build them on demand from the matching `.c` / `.cpp` (build flags are documented at the top of each source file).

## Layout

```
tools/
├── plot_mpc_receding_horizon.py        # Receding-horizon visualization of the CPU MPC
├── reconstruct_raceline_from_bag.py    # Rebuild the MPC-internal raceline from a state_replay.csv
│
├── input/                              # (gitignored) recorded bags / state_replay.csv inputs
├── output/                             # (gitignored) per-run outputs (plots, CSVs)
│
└── mpc_replay/                         # Replay + test infrastructure
    ├── run_test1.sh                    # Test 1 — CPU baseline vs FPGA timing per iteration
    ├── run_test2.sh                    # Test 2 — cost-map visualization
    ├── run_test2_landscape.sh          # Test 2 — cost landscape at a single pose
    ├── run_test2_low_velocity_weight.sh# Test 2 — low-velocity weight ablation
    ├── run_test3.sh                    # Test 3 — ablation (steering-rate / acceleration-rate off, etc.)
    ├── run_test4.sh                    # Test 4 — iteration-count warm vs cold start
    ├── run_width_probe.sh              # Word-sizing probe (instrumented replay)
    ├── run_width_report.sh             # Word-sizing report aggregation
    ├── run_width_probe_profile.sh      # Width probe with a specific narrow profile
    ├── run_width_report_profile.sh     # Width report with a specific narrow profile
    ├── set_width_profile.sh            # Activate a fixed-point width profile
    ├── set_store_width_profile.sh      # Activate a stored-width profile
    │
    ├── width_report_out/               # (gitignored) word-sizing run outputs
    │
    └── helper/                         # Replay binaries, exporters, plotters
        ├── replay_cpu_mpc.c            # CPU MPC replay over a state_replay.csv
        ├── replay_cpu_mpc_cost.c       # CPU MPC replay that also records cost components
        ├── replay_fpga_scalar.cpp      # Scalar FPGA-path replay (instrumentable)
        ├── replay_fpga_opencl.cpp      # OpenCL-path replay (talks to the actual kernel)
        ├── dump_widths.cpp             # Probe-side width recorder used by run_width_*.sh
        ├── cost_landscape_at_pose.c    # Cost landscape evaluator at a fixed pose
        ├── export_mpc_plan_snapshot.c  # Dump a single MPC plan snapshot
        │
        ├── export_mpc_state_csv.py             # bag → state_replay.csv (FPGA-side)
        ├── export_mpc_state_csv_from_cpu_bag.py# bag → state_replay.csv (CPU-side)
        ├── export_ablation_csv.py              # Test 3 — per-ablation CSV export
        ├── export_baseline_timing_csv.py       # Test 1 — baseline timing export
        ├── export_iteration_count_csv.py       # Test 4 — iteration-count export
        ├── aggregate_width_report.py           # Width-sizing run aggregation → word-sizing tables
        │
        ├── make_comparison_csv.py              # Side-by-side replay comparison CSV
        ├── make_test3_baseline_steeroff_csv.py # Test 3 — baseline-vs-steer-off CSV
        │
        ├── plot_test1_baseline_vs_fpga.py      # → test1_per_iter_affine_fit.png (FPGA Timings)
        ├── plot_test2_maps.py                  # → test 2 map figures
        ├── plot_test2_landscape.py             # → test 2 cost-landscape figure
        ├── plot_test3_ablation.py              # → test3_*.png (MPC Test)
        ├── plot_test4_iterations.py            # → test4_iterations_boxplot.png (MPC Test)
        ├── plot_test4_iter_diff_heatmap.py     # → test4_iter_diff_heatmap.png (MPC Test)
        ├── plot_affine_comparison.py           # Affine-fit comparison overlay
        │
        └── compare_cpu_vs_jetson.py            # Controller_comparison.tex tables (CPU vs FPGA, 10 laps)
```

## Mapping from scripts to thesis figures

| Thesis figure / table                                                | Produced by |
|----------------------------------------------------------------------|-------------|
| `iter-timing.png` (FPGA `Timings.tex`, 105.8 µs setup, 13.82 µs/iter, R² = 0.9941) | `run_test1.sh` → `plot_test1_baseline_vs_fpga.py` (renamed from `test1_per_iter_affine_fit.png`) |
| `test3_tracking_error_track.png` (MPC `Test.tex`)                    | `run_test3.sh` → `plot_test3_ablation.py` |
| `test3_iter_track.png` (MPC `Test.tex`)                              | same |
| `test3_steering_smoothness.png` (MPC `Test.tex`)                     | same |
| `test4_iterations_boxplot.png` (MPC `Test.tex`)                      | `run_test4.sh` → `plot_test4_iterations.py` |
| `test4_iter_diff_heatmap.png` (MPC `Test.tex`)                       | `run_test4.sh` → `plot_test4_iter_diff_heatmap.py` |
| Controller comparison tables (CPU vs FPGA, 18 726 / 19 860 samples)  | `compare_cpu_vs_jetson.py` (consumes the CPU and FPGA 10-lap bags directly) |
| Word-sizing appendix tables (four-run replay)                        | `run_width_report.sh` → `replay_fpga_scalar_wreport` + `aggregate_width_report.py` |

## Building the helper binaries

Each `.c` / `.cpp` source has its build line (typical `gcc -O3 -I../../../FPGA_Implementations/MPC_FPGA_Kria/include …`) in the header comment. The four most commonly used ones:

```bash
cd tools/mpc_replay/helper

# CPU MPC replay
gcc -O3 -I../../../MPC/include \
    -I../../../FPGA_Implementations/MPC_FPGA_Kria/include replay_cpu_mpc.c \
    ../../../MPC/src/mpc.c ../../../MPC/src/riccati_solver.c \
    ../../../MPC/src/vehicle_model.c ../../../MPC/src/util_math.c \
    -o replay_cpu_mpc -lm

# FPGA scalar (host) replay
g++ -O3 -I../../../FPGA_Implementations/MPC_FPGA_Kria/include \
       -I/home/akselmo/Vivado_program/2025.2/Vitis/include \
    -Wno-unknown-pragmas \
    replay_fpga_scalar.cpp \
    ../../../FPGA_Implementations/MPC_FPGA_Kria/src/fp_math_hls.cpp \
    ../../../FPGA_Implementations/MPC_FPGA_Kria/src/vehicle_model_hls.cpp \
    ../../../FPGA_Implementations/MPC_FPGA_Kria/src/riccati_solver_hls.cpp \
    ../../../FPGA_Implementations/MPC_FPGA_Kria/src/mpc_riccati_hls.cpp \
    ../../../FPGA_Implementations/MPC_FPGA_Kria/src/mpc_fpga_top.cpp \
    -o replay_fpga_scalar -lm
```

Add `-DPROBE_WIDTHS` for `replay_fpga_scalar_wprobe`, `-DREPORT_WIDTHS` for `replay_fpga_scalar_wreport`.

## Running the tests

The `run_*.sh` scripts expect to be executed from inside `tools/mpc_replay/`. Each one prints its expected input bag(s) and writes a timestamped subdirectory under `tools/output/`. A typical end-to-end test 3 run:

```bash
cd tools/mpc_replay
./run_test3.sh           # builds CSVs, runs the ablation replays, makes the test3_*.png plots
```

The Test 1 (`iter-timing`) and Controller_comparison results consume the actual recorded 10-lap bags in `tools/input/` — those bags are not in the repo, copy them in from the lab share before running.

## Inputs and outputs (gitignored)

`tools/input/` (~11 GB) holds the recorded `.mcap` bags from the car and the simulator. `tools/output/` and `tools/mpc_replay/width_report_out/` hold the regenerable per-run analysis outputs (CSVs, PNGs, logs). Neither directory is committed.
