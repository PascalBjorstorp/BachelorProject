# Full-stack model-selection integration

`adaptive_odom_shadow.py` is the live reference implementation used during
candidate verification. It leaves the normal `VescToOdom` component running,
but publishes a separate candidate odometry stream and remaps
`AckermannToVesc` to consume it. This proves the estimator in the real command
and timing graph without permanently editing core C++ code.

The test therefore has two acceptance layers:

1. **Shadow acceptance** — the selected model beats all alternatives on
   complete LiDAR hold-outs and works in the live candidate stack.
2. **Production C++ port acceptance** — the port reproduces shadow output on
   replay and passes the same physical candidate verification.

This separation is deliberate: it is safer than automatically injecting a
nontrivial C++ patch into an arbitrary workspace and claiming it has been
validated.

See `PRODUCTION_PORT_CONTRACT.md` for the required production changes.


## Candidate ACCEL_TO_CURRENT shadow path

`candidate_accel_map.py` is active only for Stage 12. It independently maps
`/ackermann_cmd.drive.acceleration` to the selector's drive/brake current
inputs, using candidate odometry velocity and either a scalar map or the
full-envelope traction-surface inverse. `AckermannToVesc` remains active only
for steering; its motor outputs are intentionally remapped to unused topics.

