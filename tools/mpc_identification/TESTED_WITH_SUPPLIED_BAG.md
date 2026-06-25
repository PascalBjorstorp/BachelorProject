# Tested with the supplied five-lap MCAP

The scripts were exercised against `shortened_baseline_0.mcap` and the supplied
raceline before this bundle was packaged.

Observed input inventory:

- 46.798 s recording;
- 1,868 `/scan` messages, each with 271 ranges;
- `/map` OccupancyGrid in frame `map`, 0.02 m cells, 487 × 581 cells;
- 9,343 `/ego_racecar/odom`, `/ekf_pose`, IMU and MPC timing cycles;
- 18,384 `/drive` commands;
- 17,901 `/sensors/servo_position_command` values.

The default scan-to-map ICP settings accepted all 1,868 scan registrations on
this bag. That indicates numerical convergence against the bag's map; it does
not prove that every ICP pose is perfect. Every sample retains its RMSE, inlier
count, and geometry-condition fields for filtering during fitting.

The operating track contains no long zero-steering straight. The longitudinal
stage therefore uses the least-coupled windows (`mean |steer| <= 0.14 rad` and
`mean |yaw rate| <= 1.0 rad/s`) rather than imposing a generic straight-line
threshold that would produce no usable segments.
