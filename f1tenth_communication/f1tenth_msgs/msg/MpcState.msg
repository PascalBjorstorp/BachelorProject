# MPC State Message (Fixed-Point Optimized)
# Contains vehicle state + waypoint index for trajectory tracking
# Uses Q16.16 fixed-point for FPGA compatibility and efficiency
#
# Q16.16 format: value = raw_int32 / 65536.0
#   - Range: -32768.0 to +32767.99998
#   - Precision: ~0.000015 (1/65536)

std_msgs/Header header

# Vehicle state in Q16.16 fixed-point (7-state dynamic bicycle model with wheel dynamics)
int32 x_fp           # Position X [meters], Q16.16: divide by 65536 for float
int32 y_fp           # Position Y [meters], Q16.16: divide by 65536 for float
int32 theta_fp       # Heading angle [radians], Q16.16
int32 velocity_fp    # Longitudinal velocity v_x [m/s], Q16.16 (body frame)
int32 vy_fp          # Lateral velocity v_y [m/s], Q16.16 (body frame)
int32 omega_fp       # Yaw rate ω [rad/s], Q16.16
int32 wheel_speed_fp # Wheel rotational speed ω_w [rad/s], Q16.16
int32 steering_angle_fp  # Actual servo steering angle δ [rad], Q16.16

# Trajectory reference (integer, no conversion needed)
uint32 waypoint_index

# Streaming MPC horizon reference in Q16.16 (from state_publisher).
# Only the first horizon_length entries are valid.
uint32 horizon_length
int32[20] ref_x_fp
int32[20] ref_y_fp
int32[20] ref_psi_fp
int32[20] ref_vx_fp
int32[20] ref_kappa_fp
int32[20] ref_ax_fp

# For latency measurement / debugging
uint32 timestamp_ms  # Milliseconds since epoch
