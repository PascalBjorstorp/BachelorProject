# OptiTrack Map Calibration

This tool calibrates the fixed transform between the ROS `map` frame and the
OptiTrack `world` frame. It launches `map_server`, lets you click known
landmarks in RViz, fits the transform, publishes it on `/tf_static`, and saves
the result to:

```text
f1tenth_localization/optitrackBenchmark/optitrack_map_transform.yaml
```

## Edit Landmarks

Edit:

```text
f1tenth_localization/optitrackBenchmark/optitrack_landmarks.yaml
```

For 3D calibration, use raw OptiTrack coordinates in meters:

```yaml
landmarks:
  - name: landmark_1
    optitrack: [x, y, z]
  - name: landmark_2
    optitrack: [x, y, z]
  - name: landmark_3
    optitrack: [x, y, z]
  - name: landmark_4
    optitrack: [x, y, z]
```

If OptiTrack is Y-up, keep the raw OptiTrack `[x, y, z]` values. Do not manually
swap axes for 3D calibration. The 3D fit estimates the full rotation into the
ROS map frame.

## Build

From the workspace root:

```bash
colcon build --packages-select f1tenth_localization
source install/setup.bash
```

## Run Calibration

Default map:

```bash
ros2 launch f1tenth_localization optitrack_calibration.launch.py
```

## RViz Steps

1. Open RViz.
2. Set `Fixed Frame` to `map`.
3. Add a `Map` display if needed, topic `/map`.
4. Select `Publish Point`.
5. Click the landmarks in the exact same order as `optitrack_landmarks.yaml`.

The node prints the next landmark to click. After the last click it saves the
calibration YAML and prints a `static_transform_publisher` command.

## Check Result

Good calibration should have low residuals:

```text
RMS < 0.05 m is good
max < 0.10 m is usually usable
```

Validate with one extra landmark not used in the fit if possible.

## Output

The output file contains:

```text
x, y, z
roll, pitch, yaw
qx, qy, qz, qw
rotation_matrix
rms_error_m
max_error_m
residuals per landmark
static_transform_publisher command
```

Use the printed static TF or keep the calibration node alive so it publishes
`map -> world` on `/tf_static`.

## Record Benchmark Bag

Keep the calibration TF running while recording. Either keep this launch alive
after calibration:

```bash
ros2 launch f1tenth_localization optitrack_calibration.launch.py
```

or publish the saved transform from `optitrack_map_transform.yaml`.

Then record the benchmark bag:

```bash
./f1tenth_localization/optitrackBenchmark/record_optitrack_benchmark_bag.sh
```

Optional output path:

```bash
./f1tenth_localization/optitrackBenchmark/record_optitrack_benchmark_bag.sh \
  $PWD/bags/MyOptitrackRun
```

The script records the same lateral-planner/localization topics as the existing
bag script, plus:

```text
/vrpn_mocap/Car2/pose
/tf
/tf_static
```

The `map -> world` calibration transform is stored in `/tf_static` as long as
the calibration node or equivalent static transform publisher is running.
