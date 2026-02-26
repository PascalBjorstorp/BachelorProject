# MPC_FPGA

Optimized MPC core prepared for FPGA/HLS conversion.

## What changed vs `MPC`

- Dedicated standalone CMake build (no ROS dependency required).
- QP solver projection path optimized:
  - Constraint metadata precomputed once per solve.
  - Per-iteration repeated constraint scanning removed.
  - Convergence check uses squared norm (no per-iteration `sqrt`).
- HLS preparation compile mode:
  - `MPC_HLS_TARGET` guard disables runtime `getenv/atof` tuning path in `mpc.c`.

## Build

```bash
cmake -S . -B build_fpga -DMPC_FPGA_BUILD_TESTS=ON -DMPC_FPGA_HLS_PREPARE=ON
cmake --build build_fpga -j
```

## Run tests

```bash
ctest --test-dir build_fpga --output-on-failure -R test_qp_solver_fpga
```

## Included tests

- `test_qp_solver_fpga`: solver feasibility, warm-start consistency, and runtime sanity benchmark.
- Existing tests from `MPC/test` are also available through CMake.
