# Trajectory Reference Message
# Optional: Send N waypoints ahead instead of just index
# Use this if you can't guarantee synced trajectory files

std_msgs/Header header

# Vehicle state
float64 x
float64 y
float64 theta
float64 velocity

# Reference waypoints (next N points)
float64[] waypoint_x         # X coordinates [meters]
float64[] waypoint_y         # Y coordinates [meters]
float64[] waypoint_v         # Target velocities [m/s]

uint32 start_index           # Index of first waypoint in trajectory
