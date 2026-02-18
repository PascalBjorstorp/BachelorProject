# MPC State Message (Fixed-Point Optimized)
# Contains vehicle state + waypoint index for trajectory tracking
# Uses Q16.16 fixed-point for FPGA compatibility and efficiency
#
# Q16.16 format: value = raw_int32 / 65536.0
#   - Range: -32768.0 to +32767.99998
#   - Precision: ~0.000015 (1/65536)

std_msgs/Header header

# Vehicle state in Q16.16 fixed-point (6-state dynamic bicycle model)
int32 x_fp           # Position X [meters], Q16.16: divide by 65536 for float
int32 y_fp           # Position Y [meters], Q16.16: divide by 65536 for float
int32 theta_fp       # Heading angle [radians], Q16.16
int32 velocity_fp    # Longitudinal velocity v_x [m/s], Q16.16 (body frame)
int32 vy_fp          # Lateral velocity v_y [m/s], Q16.16 (body frame)
int32 omega_fp       # Yaw rate ω [rad/s], Q16.16

# Trajectory reference (integer, no conversion needed)
uint32 waypoint_index

# For latency measurement / debugging
uint32 timestamp_ms  # Milliseconds since epoch
