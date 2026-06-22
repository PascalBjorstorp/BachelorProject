# ERPM / speed calibration operator card

## Only command

```bash
cd ~/BachelorProject/f1tenth_parameters/ERPM
source ~/BachelorProject/install/setup.bash
python3 erpm_calibration.py --workspace ~/BachelorProject
```

Do **not** edit `vesc.yaml`, run `colcon`, start normal bringup, start MPC,
MPCC, teleoperation, planners, or `ros2 bag record`. The runner backs up the
source VESC configuration, changes controller profiles temporarily, builds,
launches only the calibration stack, bags every ROS topic, and restores the
original file plus rebuilds at the end.

## Before typing READY

- Install the **completed steering calibration** and confirm normal straight
  driving at zero steering command.
- Charge the standard Traxxas 3S battery.
- Use the fixed, asymmetric LiDAR room arrangement from the steering campaign.
- Clear an 8–10 m straight path and remove people, moving objects and loose
  cables.
- Ensure the car has normal tyres, fixed payload and normal gearing.
- Ensure an emergency stop is reachable.

## At every prompt

- `READY`: car is placed at the requested straight-line start.
- `ACCEPT`: manoeuvre was clean and safe.
- `REDO`: retain bagged attempt but exclude it from fitting; repeat without a
  retry cap.
- `SKIP`: retain bagged attempt but omit this condition. Analysis will fail
  coverage gates if too many required trials are skipped.
- `ABORT`: stop safely; raw bags are preserved and source VESC YAML is restored.

The run includes cooling pauses automatically. Do not move LiDAR features while
any capture is active.

## Recovery after power loss / forced stop

Do not drive. Run:

```bash
python3 erpm_calibration.py --recover --workspace ~/BachelorProject
```

The recovery command restores the byte-exact original VESC YAML and rebuilds.
