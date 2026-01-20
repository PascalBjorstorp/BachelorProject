# F1TENTH Gym Environment ROS2 Bridge

A ROS2 communication bridge for the F1TENTH gym environment that turns it into a simulation in ROS2 Jazzy.

## Requirements

- **Ubuntu 24.04** with ROS2 Jazzy installed
- **Python 3.10+**

## Quick Start (Recommended)

The easiest way to get started is using the provided setup script:

```bash
# Clone the repository
git clone <your-repo-url>
cd f1tenth_sim

# Run the setup script (creates venv, installs dependencies, builds package)
./setup.sh

# Configure the map path in config/sim.yaml
# Then run the simulation
./run.sh
```

That's it! The setup script will:
1. Create a Python virtual environment
2. Install all Python dependencies (including f1tenth-gym)
3. Install ROS2 dependencies
4. Build the workspace

## Scripts

| Script | Purpose |
|--------|---------|
| `./setup.sh` | First-time setup: creates venv, installs dependencies, builds package |
| `./build.sh` | Rebuild the package after making code changes |
| `./run.sh` | Launch the simulation |

## Manual Installation

If you prefer to set things up manually:

### 1. Install ROS2 Jazzy

Follow the official ROS2 Jazzy installation instructions:
https://docs.ros.org/en/jazzy/Installation.html

### 2. Create Virtual Environment

```bash
cd f1tenth_sim
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 3. Build the Package

```bash
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-path . --rosdistro jazzy -y
colcon build --symlink-install
```

### 4. Configure the Simulation

Edit the configuration file at `config/sim.yaml`:

```yaml
bridge:
  ros__parameters:
    map_path: '/full/path/to/your/map'  # Update this path!
    # ... other parameters
```

**Important:** Update `map_path` to point to the correct location of your map file (without the `.yaml` extension).

## Launching the Simulation

```bash
source /opt/ros/jazzy/setup.bash
source ~/sim_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

An RViz window should open showing the simulation.

## Configuration

The configuration file is located at `config/sim.yaml`. Key parameters:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `map_path` | Full path to map file (without extension) | `levine` |
| `map_img_ext` | Map image file extension | `.png` |
| `num_agent` | Number of agents (1 or 2) | `1` |
| `vehicle_params` | Vehicle type: `f1tenth`, `fullscale`, or `f1fifth` | `f1tenth` |
| `scale` | Scale factor for the simulation | `1.0` |
| `sx`, `sy`, `stheta` | Ego starting pose | `0.0, 0.0, 0.0` |
| `sx1`, `sy1`, `stheta1` | Opponent starting pose | `2.0, 0.5, 0.0` |
| `kb_teleop` | Enable keyboard teleop | `True` |
| `use_sim_time` | Use simulation time | `False` |
| `async_mode` | Asynchronous simulation mode | `True` |

## Topics

### Published Topics

**Single Agent:**
- `/scan` - Ego agent's laser scan (`sensor_msgs/LaserScan`)
- `/ego_racecar/odom` - Ego agent's odometry (`nav_msgs/Odometry`)
- `/map` - Environment map (`nav_msgs/OccupancyGrid`)

**Two Agents (additional):**
- `/opp_scan` - Opponent's laser scan
- `/ego_racecar/opp_odom` - Opponent's odometry for ego's planner
- `/opp_racecar/odom` - Opponent's odometry
- `/opp_racecar/opp_odom` - Ego's odometry for opponent's planner

### Subscribed Topics

**Single Agent:**
- `/drive` - Ego drive command (`ackermann_msgs/AckermannDriveStamped`)
- `/initialpose` - Reset ego pose (RViz 2D Pose Estimate)
- `/cmd_vel` - Keyboard teleop commands (`geometry_msgs/Twist`)
- `/pause_sim` - Pause/resume simulation (`std_msgs/Bool`)

**Two Agents (additional):**
- `/opp_drive` - Opponent drive command
- `/goal_pose` - Reset opponent pose (RViz 2D Goal Pose)

## Keyboard Teleop

When `kb_teleop` is enabled in `sim.yaml`, run in a separate terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controls:
- `i` - Move forward
- `u` / `o` - Move forward and turn
- `,` - Move backwards
- `m` / `.` - Move backwards and turn
- `k` - Stop

## Troubleshooting

### PEP 668 Error
If you encounter "This package is managed externally" error:
```bash
pip3 install --break-system-packages -e .
```
Or use a virtual environment.

### Missing Dependencies

Run the setup script to install all dependencies:
```bash
./setup.sh
```

### Package Build Issues

Clean and rebuild:
```bash
rm -rf build install log
./setup.sh
```

## License

MIT License - See [LICENSE](LICENSE) for details.
