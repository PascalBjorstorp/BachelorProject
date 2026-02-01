#include "f1tenth_control/state_estimation/ekf.hpp"
#include <cmath>

namespace f1tenth_control {

ExtendedKalmanFilter::ExtendedKalmanFilter() : config_() {
    state_.setZero();
    covariance_.setIdentity();
    updateProcessNoise();
}

ExtendedKalmanFilter::ExtendedKalmanFilter(const Config& config) : config_(config) {
    state_.setZero();
    covariance_.setIdentity();
    updateProcessNoise();
}

void ExtendedKalmanFilter::initialize(double x, double y, double theta, double v, double omega) {
    state_(IDX_X) = x;
    state_(IDX_Y) = y;
    state_(IDX_THETA) = normalizeAngle(theta);
    state_(IDX_V) = v;
    state_(IDX_OMEGA) = omega;
    
    // Initialize covariance with reasonable uncertainty
    covariance_.setIdentity();
    covariance_(IDX_X, IDX_X) = 0.1;       // 0.1 m² initial position uncertainty
    covariance_(IDX_Y, IDX_Y) = 0.1;
    covariance_(IDX_THETA, IDX_THETA) = 0.1;  // ~18° initial heading uncertainty
    covariance_(IDX_V, IDX_V) = 0.5;       // 0.5 (m/s)² velocity uncertainty
    covariance_(IDX_OMEGA, IDX_OMEGA) = 0.1;
    
    initialized_ = true;
}

void ExtendedKalmanFilter::updateProcessNoise() {
    process_noise_.setZero();
    process_noise_(IDX_X, IDX_X) = config_.process_noise_x;
    process_noise_(IDX_Y, IDX_Y) = config_.process_noise_y;
    process_noise_(IDX_THETA, IDX_THETA) = config_.process_noise_theta;
    process_noise_(IDX_V, IDX_V) = config_.process_noise_v;
    process_noise_(IDX_OMEGA, IDX_OMEGA) = config_.process_noise_omega;
}

void ExtendedKalmanFilter::predict(double dt) {
    if (!initialized_ || dt <= 0.0) return;
    
    // Current state
    double x = state_(IDX_X);
    double y = state_(IDX_Y);
    double theta = state_(IDX_THETA);
    double v = state_(IDX_V);
    double omega = state_(IDX_OMEGA);
    
    // === Motion model (bicycle model / constant velocity) ===
    // For small dt, we use simple Euler integration
    // x' = x + v * cos(theta) * dt
    // y' = y + v * sin(theta) * dt
    // theta' = theta + omega * dt
    // v' = v (constant velocity assumption - updated by measurements)
    // omega' = omega (constant turn rate - updated by measurements)
    
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    
    // State prediction
    state_(IDX_X) = x + v * cos_theta * dt;
    state_(IDX_Y) = y + v * sin_theta * dt;
    state_(IDX_THETA) = normalizeAngle(theta + omega * dt);
    // v and omega remain unchanged in prediction (constant velocity model)
    
    // === Covariance prediction ===
    // P' = F * P * F^T + Q
    // where F is the Jacobian of the motion model
    
    StateCovariance F = computeStateJacobian(dt);
    covariance_ = F * covariance_ * F.transpose() + process_noise_ * dt;
    
    // Ensure covariance is symmetric and positive definite
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

ExtendedKalmanFilter::StateCovariance ExtendedKalmanFilter::computeStateJacobian(double dt) const {
    // Jacobian of motion model with respect to state
    // f(x, y, theta, v, omega) = [x + v*cos(theta)*dt, 
    //                             y + v*sin(theta)*dt, 
    //                             theta + omega*dt,
    //                             v,
    //                             omega]
    
    double theta = state_(IDX_THETA);
    double v = state_(IDX_V);
    double cos_theta = std::cos(theta);
    double sin_theta = std::sin(theta);
    
    StateCovariance F;
    F.setIdentity();
    
    // ∂x'/∂theta = -v * sin(theta) * dt
    F(IDX_X, IDX_THETA) = -v * sin_theta * dt;
    // ∂x'/∂v = cos(theta) * dt
    F(IDX_X, IDX_V) = cos_theta * dt;
    
    // ∂y'/∂theta = v * cos(theta) * dt
    F(IDX_Y, IDX_THETA) = v * cos_theta * dt;
    // ∂y'/∂v = sin(theta) * dt
    F(IDX_Y, IDX_V) = sin_theta * dt;
    
    // ∂theta'/∂omega = dt
    F(IDX_THETA, IDX_OMEGA) = dt;
    
    return F;
}

void ExtendedKalmanFilter::updateOdometry(double velocity, double omega_meas) {
    if (!initialized_) return;
    
    // Measurement model: z = [v, omega]
    // h(x) = [v, omega] (direct observation of velocity and angular velocity)
    
    constexpr int MEAS_DIM = 2;
    using MeasVector = Eigen::Matrix<double, MEAS_DIM, 1>;
    using MeasMatrix = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;
    using MeasCovariance = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;
    
    // Measurement
    MeasVector z;
    z << velocity, omega_meas;
    
    // Predicted measurement
    MeasVector z_pred;
    z_pred << state_(IDX_V), state_(IDX_OMEGA);
    
    // Innovation (measurement residual)
    MeasVector y = z - z_pred;
    
    // Measurement Jacobian H (dh/dx)
    // h = [v, omega], so H is [0, 0, 0, 1, 0; 0, 0, 0, 0, 1]
    MeasMatrix H;
    H.setZero();
    H(0, IDX_V) = 1.0;
    H(1, IDX_OMEGA) = 1.0;
    
    // Measurement noise R
    MeasCovariance R;
    R.setZero();
    R(0, 0) = config_.odom_velocity_variance;
    R(1, 1) = config_.odom_omega_variance;
    
    // Innovation covariance S = H * P * H^T + R
    MeasCovariance S = H * covariance_ * H.transpose() + R;
    
    // Kalman gain K = P * H^T * S^(-1)
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = covariance_ * H.transpose() * S.inverse();
    
    // State update: x' = x + K * y
    state_ += K * y;
    state_(IDX_THETA) = normalizeAngle(state_(IDX_THETA));
    
    // Covariance update: P' = (I - K * H) * P
    StateCovariance I = StateCovariance::Identity();
    covariance_ = (I - K * H) * covariance_;
    
    // Ensure symmetry
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void ExtendedKalmanFilter::updateIMU(double accel_x, double accel_y, double gyro_z) {
    if (!initialized_) return;
    
    // For now, we only use the gyroscope angular velocity
    // Accelerometer could be used to estimate velocity changes, but
    // wheel odometry is typically more reliable for ground vehicles
    
    // Simple update: just use gyro for angular velocity
    constexpr int MEAS_DIM = 1;
    using MeasVector = Eigen::Matrix<double, MEAS_DIM, 1>;
    using MeasMatrix = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;
    using MeasCovariance = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;
    
    // Measurement
    MeasVector z;
    z << gyro_z;
    
    // Predicted measurement
    MeasVector z_pred;
    z_pred << state_(IDX_OMEGA);
    
    // Innovation
    MeasVector y = z - z_pred;
    
    // Measurement Jacobian
    MeasMatrix H;
    H.setZero();
    H(0, IDX_OMEGA) = 1.0;
    
    // Measurement noise
    MeasCovariance R;
    R(0, 0) = config_.imu_omega_variance;
    
    // Innovation covariance
    MeasCovariance S = H * covariance_ * H.transpose() + R;
    
    // Kalman gain
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = covariance_ * H.transpose() * S.inverse();
    
    // State update
    state_ += K * y;
    state_(IDX_THETA) = normalizeAngle(state_(IDX_THETA));
    
    // Covariance update
    covariance_ = (StateCovariance::Identity() - K * H) * covariance_;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    
    // Optionally: Use accelerometer to update velocity estimate
    // This can help when wheels slip
    // For now, we trust wheel odometry more than accelerometer for velocity
    (void)accel_x;  // Suppress unused warning
    (void)accel_y;
}

void ExtendedKalmanFilter::updatePose(double x, double y, double theta,
                                       double x_var, double y_var, double theta_var) {
    if (!initialized_) {
        initialize(x, y, theta);
        return;
    }
    
    // Absolute pose measurement from MCL/localization
    constexpr int MEAS_DIM = 3;
    using MeasVector = Eigen::Matrix<double, MEAS_DIM, 1>;
    using MeasMatrix = Eigen::Matrix<double, MEAS_DIM, STATE_DIM>;
    using MeasCovariance = Eigen::Matrix<double, MEAS_DIM, MEAS_DIM>;
    
    // Measurement
    MeasVector z;
    z << x, y, theta;
    
    // Predicted measurement
    MeasVector z_pred;
    z_pred << state_(IDX_X), state_(IDX_Y), state_(IDX_THETA);
    
    // Innovation (with angle wrapping for theta)
    MeasVector y_innov;
    y_innov << x - state_(IDX_X),
               y - state_(IDX_Y),
               normalizeAngle(theta - state_(IDX_THETA));
    
    // Measurement Jacobian (direct observation of x, y, theta)
    MeasMatrix H;
    H.setZero();
    H(0, IDX_X) = 1.0;
    H(1, IDX_Y) = 1.0;
    H(2, IDX_THETA) = 1.0;
    
    // Measurement noise (use provided variance or config defaults)
    MeasCovariance R;
    R.setZero();
    R(0, 0) = (x_var > 0) ? x_var : config_.mcl_x_variance;
    R(1, 1) = (y_var > 0) ? y_var : config_.mcl_y_variance;
    R(2, 2) = (theta_var > 0) ? theta_var : config_.mcl_theta_variance;
    
    // Innovation covariance
    MeasCovariance S = H * covariance_ * H.transpose() + R;
    
    // Kalman gain
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = covariance_ * H.transpose() * S.inverse();
    
    // State update
    state_ += K * y_innov;
    state_(IDX_THETA) = normalizeAngle(state_(IDX_THETA));
    
    // Covariance update
    covariance_ = (StateCovariance::Identity() - K * H) * covariance_;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
}

void ExtendedKalmanFilter::getPoseWithCovariance(double& x, double& y, double& theta,
                                                   std::array<double, 9>& covariance_3x3) const {
    x = state_(IDX_X);
    y = state_(IDX_Y);
    theta = state_(IDX_THETA);
    
    // Extract 3x3 position/orientation covariance (row-major)
    // [xx, xy, xθ]
    // [yx, yy, yθ]
    // [θx, θy, θθ]
    covariance_3x3[0] = covariance_(IDX_X, IDX_X);
    covariance_3x3[1] = covariance_(IDX_X, IDX_Y);
    covariance_3x3[2] = covariance_(IDX_X, IDX_THETA);
    covariance_3x3[3] = covariance_(IDX_Y, IDX_X);
    covariance_3x3[4] = covariance_(IDX_Y, IDX_Y);
    covariance_3x3[5] = covariance_(IDX_Y, IDX_THETA);
    covariance_3x3[6] = covariance_(IDX_THETA, IDX_X);
    covariance_3x3[7] = covariance_(IDX_THETA, IDX_Y);
    covariance_3x3[8] = covariance_(IDX_THETA, IDX_THETA);
}

void ExtendedKalmanFilter::getVelocityWithCovariance(double& vx, double& vy, double& omega,
                                                       std::array<double, 9>& covariance_3x3) const {
    double theta = state_(IDX_THETA);
    double v = state_(IDX_V);
    
    // Transform velocity from body frame to world frame
    vx = v * std::cos(theta);
    vy = v * std::sin(theta);
    omega = state_(IDX_OMEGA);
    
    // Simplified covariance (just diagonal for now)
    // Full transformation would require Jacobian of velocity transformation
    double v_var = covariance_(IDX_V, IDX_V);
    covariance_3x3[0] = v_var;  // vx variance (simplified)
    covariance_3x3[1] = 0.0;
    covariance_3x3[2] = 0.0;
    covariance_3x3[3] = 0.0;
    covariance_3x3[4] = v_var;  // vy variance (simplified)
    covariance_3x3[5] = 0.0;
    covariance_3x3[6] = 0.0;
    covariance_3x3[7] = 0.0;
    covariance_3x3[8] = covariance_(IDX_OMEGA, IDX_OMEGA);
}

double ExtendedKalmanFilter::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}  // namespace f1tenth_control
