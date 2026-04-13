# Kria MPC HLS Optimization Tracking

## Scope

- Target: `FPGA_Implementations/MPC_FPGA_Kria`
- Primary objective: maximize timing (goal >= 200 MHz in implementation)
- Secondary objective: reduce worst-case latency cycles

## Baseline (Before This Pass)

- HLS estimate: `~196 MHz` class, `~75.6k` cycles
- First measured post-route implementation point:
	- `CP achieved post-implementation: 8.569 ns` (`~116.7 MHz`)
	- `WNS: -3.569 ns`

## Experiments and Outcomes

| ID | Change | Timing/Fmax | Latency | Functional | Decision |
|---|---|---:|---:|---|
| E1 | Stage shared products in `compute_frenet_AB_hls` + vehicle mul budget tuning | HLS timing improved | Slight cycle win | OK | Keep |
| E2 | Force `fp_sin/fp_cos/fp_atan` out-of-line | HLS Fmax regressed (`~182 MHz`) | Mixed | OK | Revert |
| E3 | Vehicle mul cap `8` | HLS timing improved | Better cycles | OK | Keep |
| E4 | Vehicle mul cap `10` | Timing regressed | Cycles regressed | OK | Revert |
| E5 | Inline `compute_frenet_AB_hls` | **HLS est. `202.65 MHz`** | **`~75,486 cycles`** | OK | Keep |
| E6 | ADMM cap sweep `20 -> 16` | Timing unchanged | Big cycle reduction | **Bad (3/6)** | Reject |
| E7 | ADMM cap sweep `20 -> 18` | Timing unchanged | **`~68,316 cycles`** | **Acceptable (5/6)** | Keep |
| E8 | Enable post-route flow (`HLS_EXPORT_IMPL=1`) | First impl check: `8.569 ns` | N/A | N/A | Keep flow |
| E9 | Make global `fp_mul` out-of-line DSP latency-4 kernel | Impl `8.569 -> 7.193 ns` | Cycles up | 5/6 | Keep |
| E10 | Partition `p` vector + stage Riccati Step-2 M*B/S | Impl `7.193 -> 6.497 ns` | Cycles up (`~70.9k`) | 5/6 | Keep |
| E11 | Raw-acc guard bits sweep `10 -> 8` | Impl `6.497 -> 6.423 ns` | Slight cycle win | **Bad (3/6)** | Reject for default |
| E12 | Raw-acc guard bits sweep `10 -> 6` | Impl `6.497 -> 6.517 ns` | Similar | Bad | Reject |

## Final Selected Configuration

- `compute_frenet_AB_hls` inlined in vehicle model path
- Vehicle multiplier budget default: `8`
- ADMM max iterations default in HLS flow: `18`
- ADMM max iterations macro made override-friendly in constants header
- Global `fp_mul` implemented as out-of-line DSP-bound kernel (`latency=4`)
- Riccati `p` vector fully partitioned (register-based)
- Riccati Step-2 (`S = r + M*B`) staged into pipelined phases
- Raw accumulator guard bits kept at default (`10`) for functional stability

## Final Metrics (Current)

- HLS estimate:
	- `Estimated Fmax: 197.54 MHz`
	- `mpc_fpga_top` latency: `70,917 cycles`
- Post-route implementation (current default):
	- `CP achieved post-synthesis: 6.166 ns`
	- `CP achieved post-implementation: 6.497 ns` (`~153.9 MHz`)
	- `WNS: -1.497 ns`
- Best measured impl period (aggressive precision sweep): `6.423 ns` (`~155.7 MHz`), rejected for default due functional regression

## Functional Sanity (Current Default)

- Testbench: `testbench/test_fpga_sim_drive.c`
- Result profile: `5/6` checks passing
- No wall collisions in selected default (`MAX_ADMM_ITER=18`)
- Known remaining failure: driving-speed threshold check

## Notes

- `MPC_FPGA_MAX_ADMM_ITER` is now override-friendly (`#ifndef ... #endif`).
- HLS script default is set to 18 and can be overridden via `HLS_MAX_ADMM_ITER`.
- Raw accumulator guard bits are sweepable via `HLS_RAW_ACC_GUARD_BITS`.
- Despite large improvement (`8.569 ns -> 6.497 ns`), 200 MHz implementation closure is still not reached.
