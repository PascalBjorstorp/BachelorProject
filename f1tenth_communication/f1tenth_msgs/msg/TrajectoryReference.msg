# Trajectory Reference Message
# Optional: Send N waypoints ahead instead of just index
# Use this if you can't guarantee synced trajectory files

std_msgs/Header header

# Vehicle state (7-state dynamic bicycle model with wheel dynamics)
float64 x
float64 y
float64 theta
float64 velocity           # Longitudinal velocity v_x [m/s]
float64 lateral_velocity   # Lateral velocity v_y [m/s]
float64 yaw_rate           # Yaw rate ω [rad/s]
float64 wheel_speed        # Wheel rotational speed ω_w [rad/s]

# Reference waypoints (next N points)
float64[] waypoint_x         # X coordinates [meters]
float64[] waypoint_y         # Y coordinates [meters]
float64[] waypoint_v         # Target velocities [m/s]
float64[] waypoint_wheel_speed  # Target wheel speeds [rad/s]

uint32 start_index           # Index of first waypoint in trajectory
