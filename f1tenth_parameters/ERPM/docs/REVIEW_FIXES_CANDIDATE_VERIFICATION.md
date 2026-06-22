# Candidate verification gates

The final candidate stages are strict validation, not presentation plots.

- Stage 11 enumerates every configured velocity condition and requires the
  requested number of accepted usable captures.
- Stage 12 enumerates every initial-speed × acceleration condition and requires
  full coverage before dynamic results count.
- Stage 13 enumerates every steering-angle × speed hold-out condition.
- Velocity, acceleration and cross-axis checks gate on signed bias as well as
  RMSE. Cross-axis validation also gates p95 absolute speed error.
- Candidate odometry acceleration, derived independently from its estimated
  speed, is compared with LiDAR acceleration in Stage 12.
- A partial, noisy or biased candidate becomes `completed_candidate_rejected`.
  Its MCAP bags and reports remain preserved; the original configuration is
  restored.
