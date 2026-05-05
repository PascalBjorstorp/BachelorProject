# @file MpcState.msg
# @brief Fixed-point transport message for vehicle state and MPC horizon data.
# @details Carries current vehicle state plus a streamed reference horizon from
#          publisher to receiver. All *_fp numeric fields use signed Q16.16,
#          where decoded_value = raw_int32 / 65536.0.
# @dependencies std_msgs/Header
#
# Data contract:
# - All fields ending with _fp use signed Q16.16 fixed-point encoding.
# - horizon_length defines how many entries in each ref_*_fp array are valid.
# - Receivers should validate each ref_*_fp array size >= horizon_length
#   before indexing.

std_msgs/Header header

# Vehicle state in Q16.16 fixed-point.
int32 x_fp           # Position X [meters], Q16.16: divide by 65536 for float
int32 y_fp           # Position Y [meters], Q16.16: divide by 65536 for float
int32 theta_fp       # Heading angle [radians], Q16.16
int32 velocity_fp    # Longitudinal velocity v_x [m/s], Q16.16 (body frame)
int32 vy_fp          # Lateral velocity v_y [m/s], Q16.16 (body frame)
int32 omega_fp       # Yaw rate ω [rad/s], Q16.16
int32 steering_angle_fp  # Actual servo steering angle δ [rad], Q16.16

# Horizon reference arrays and full horizon geometry (V2 message).
# All entries up to horizon_length are valid. Receivers should validate
# each ref_*_fp array size >= horizon_length before indexing.
uint32 horizon_length

# Per-step Frenet / geometric horizon data.
int32[] ref_ey_fp          # Reference lateral error e_y_ref [m], Q16.16
int32[] ref_epsi_fp        # Reference heading error e_psi_ref [rad], Q16.16

# Geometry used by the receiver to compute current Frenet errors.
int32[] ref_x_fp           # Waypoint X positions [meters], Q16.16
int32[] ref_y_fp           # Waypoint Y positions [meters], Q16.16
int32[] ref_psi_fp         # Waypoint heading [radians], Q16.16

# Dynamic/path references consumed by the FPGA kernel.
int32[] ref_vx_fp          # Reference longitudinal velocity [m/s], Q16.16
int32[] ref_vy_fp          # Reference lateral velocity [m/s], Q16.16
int32[] ref_omega_ref_fp   # Reference yaw rate [rad/s], Q16.16
int32[] ref_kappa_fp       # Reference curvature [1/m], Q16.16

# Corridor bounds.
int32[] ref_left_bound_fp  # Left track bound [m], Q16.16
int32[] ref_right_bound_fp # Right track bound [m], Q16.16
