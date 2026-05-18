# @file MpcState.msg
# @brief Fixed-point transport message for vehicle state and MPC horizon data.
# @details Carries current Frenet state plus a streamed reference horizon from
#          publisher to receiver. All *_fp numeric fields use signed raw QP,
#          where decoded_value = raw_int32 / MPC_FPGA_QP_SCALE_F64.
# @dependencies std_msgs/Header
#
# Data contract:
# - All fields ending with _fp use signed raw QP fixed-point encoding.
# - horizon_length defines how many entries in each ref_*_fp array are valid.
# - Receivers should validate each ref_*_fp array size >= horizon_length
#   before indexing.

std_msgs/Header header

# Current Frenet state in raw QP fixed-point.
int32 e_y_fp         # Current lateral error [meters], raw QP
int32 e_psi_fp       # Current heading error [radians], raw QP
int32 velocity_fp    # Longitudinal velocity v_x [m/s], raw QP (body frame)
int32 vy_fp          # Lateral velocity v_y [m/s], raw QP (body frame)
int32 omega_fp       # Yaw rate ω [rad/s], raw QP
int32 steering_angle_fp  # Actual servo steering angle δ [rad], raw QP

# Horizon reference arrays.
# All entries up to horizon_length are valid. Receivers should validate
# each ref_*_fp array size >= horizon_length before indexing.
uint32 horizon_length

# Per-step Frenet horizon data.
int32[] ref_ey_fp          # Reference lateral error e_y_ref [m], raw QP
int32[] ref_epsi_fp        # Reference heading error e_psi_ref [rad], raw QP

# Dynamic/path references consumed by the FPGA kernel.
int32[] ref_vx_fp          # Reference longitudinal velocity [m/s], raw QP
int32[] ref_vy_fp          # Reference lateral velocity [m/s], raw QP
int32[] ref_omega_ref_fp   # Reference yaw rate [rad/s], raw QP
int32[] ref_kappa_fp       # Reference curvature [1/m], raw QP

# Corridor bounds.
int32[] ref_left_bound_fp  # Left track bound [m], raw QP
int32[] ref_right_bound_fp # Right track bound [m], raw QP
