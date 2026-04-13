# AP Fixed Stage-1 Data Notes

Source run:
- Command: MPC_SOLVER_TRACE=1 ./testbench/test_accuracy_kria_vs_ultra96.sh
- Log: build/accuracy_compare/kria.log
- Result: PASS accuracy check

Observed magnitudes (max abs):
- e_y: 0.092
- e_psi: 0.2895
- speed: 4.83
- cmd_accel: 7.308395
- cmd_steer: 0.289093
- x bounds in QP: 100.0
- q_vx term: 1944.463501

Derived integer-bit targets (signed):
- IO/control family: 6 bits
- Frenet family: 9 bits
- Riccati family: 13 bits
- Accumulator family: 20 bits

Stage-1 family widths introduced in fp_types_hls.hpp:
- fp_io_family_t: ap_fixed<22, 6>
- fp_frenet_family_t: ap_fixed<25, 9>
- fp_riccati_family_t: ap_fixed<29, 13>
- fp_accum_family_t: ap_fixed<40, 20>

Notes:
- Stage-1 only introduces family aliases + semantic mapping.
- Core compute path remains behavior-compatible while migration points are prepared.
- Next stage should migrate one hot path at a time with A/B timing checks.

Stage-3 experiment log:
- Tried AP-family arithmetic inside vehicle-model fp_mul_vm wrapper.
- Result: Estimated Fmax dropped to 186.96 MHz (from 204.47 MHz), despite small DSP reduction.
- Decision: reverted immediately; keep AP migration at explicit boundary quantization until a better arithmetic island strategy is identified.
