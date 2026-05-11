# ARM Benchmark

Standalone CPU benchmark for the vehicle-model Frenet construction path.

## What It Measures

This project times the CPU implementation of:

- `vehicle_model_compute_frenet_linearization`
- `vehicle_model_predict_next_state`

The benchmark uses the CPU math path from the MPC vehicle model, not the HLS-optimized FPGA code.

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/arm_benchmark 20000 500
```

Arguments:

- First argument: timed iterations
- Second argument: warmup iterations

## FPGA Offload Analysis

If the Frenet construction were moved to FPGA, the minimum per-step interface would be based on the current HLS scalar type, `ap_fixed<32, 17>` (`fp_QP_t`), so each value is 32 bits = 4 bytes.

With that representation, the minimum per-step interface would be:

- Inputs:
  - `FrenetState_t` = 5 values = 20 bytes
  - `ControlInput_t` = 2 values = 8 bytes
  - `time_step`, `path_curvature`, `reference_velocity` = 3 values = 12 bytes
  - Total input = 40 bytes per step

- Outputs for linearization only:
  - `A` = 5 x 5 values = 100 bytes
  - `B` = 5 x 2 values = 40 bytes
  - Total output = 140 bytes per step

- Outputs if you also offload next-state prediction:
  - `next_state` = 5 values = 20 bytes
  - Total output = 160 bytes per step

Per MPC horizon step, that is about 180 bytes of total traffic for linearization only, or 200 bytes if next-state prediction is included.

For a 20-step horizon, the transfer is about:

- 800 bytes in
- 2800 bytes out for linearization only
- 3200 bytes out if next-state prediction is also returned
- 3600 bytes total per solve for linearization only
- 4000 bytes total per solve if next-state prediction is also returned

That is small in bandwidth terms, but the real cost is latency and buffering overhead. If the FPGA is only doing the Frenet math, the CPU benchmark here is useful because it gives you the host-side compute cost to compare against the DMA + kernel launch + return-path overhead.

## How The FPGA Version Would Look

The clean FPGA split is a batched kernel that accepts one horizon's worth of state/control/reference tuples, computes the Frenet model slice for each stage, and writes back packed `A`/`B` matrices.

That means the software side would typically:

1. Build a contiguous input buffer for the horizon.
2. Copy the buffer to the FPGA.
3. Run one kernel invocation for the whole horizon, not one invocation per stage.
4. Read back the packed `A`/`B` results and hand them to the solver.

If you launch the kernel once per stage, the fixed transfer overhead and launch latency can dominate the actual math. The batch-per-horizon approach is the one that makes sense for an FPGA implementation.

## Implementation Implication

If you replace the current FPGA Frenet construction with this CPU model, the FPGA design would likely no longer need:

- the HLS vehicle-model trigonometric pipeline
- the tire linearization helpers
- the separate predicted next-state path for the FPGA model slice

Instead, the FPGA would become a consumer of precomputed `A`/`B` data, or it would need a narrower interface that only passes the raw state/control/reference data needed to build those matrices. In the current layout, the host would still need to ship the same 40 bytes of inputs per stage and retrieve 140 bytes of linearization output per stage, so the main FPGA question is not raw payload size but whether the fixed-point compute savings outweigh DMA, buffering, and kernel-launch overhead.

In practice, the breakeven question is not the raw bytes. It is whether the FPGA-side fixed-point pipeline saves more time than the cost of moving the inputs and outputs across the software/hardware boundary.
