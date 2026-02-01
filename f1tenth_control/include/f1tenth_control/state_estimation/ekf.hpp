#ifndef F1TENTH_CONTROL_EKF_HPP_
#define F1TENTH_CONTROL_EKF_HPP_

#include <Eigen/Dense>
#include <array>
#include <cmath>

namespace f1tenth_control {

/**
 * @brief Extended Kalman Filter for F1Tenth state estimation
 * 
 * State vector [5x1]:
 *   x     - Position X [m]
 *   y     - Position Y [m]
 *   theta - Heading [rad]
 *   v     - Velocity [m/s]
 *   omega - Angular velocity [rad/s]
 * 
 * The EKF fuses:
 *   1. Wheel odometry (from VESC) - provides v and servo-based omega
 *   2. IMU - provides acceleration and angular velocity
 *   3. (Later) MCL pose updates from LiDAR
 */
class ExtendedKalmanFilter {
public:
    // State dimension
    static constexpr int STATE_DIM = 5;
    
    // State indices
    static constexpr int IDX_X = 0;
    static constexpr int IDX_Y = 1;
    static constexpr int IDX_THETA = 2;
    static constexpr int IDX_V = 3;
    static constexpr int IDX_OMEGA = 4;
    
    using StateVector = Eigen::Matrix<double, STATE_DIM, 1>;
    using StateCovariance = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;
    
    /**
     * @brief Configuration for EKF
     */
    struct Config {
        // Process noise (how much the state can change between updates)
        double process_noise_x{0.01};         // [m²/s²]
        double process_noise_y{0.01};         // [m²/s²]
        double process_noise_theta{0.01};     // [rad²/s²]
        double process_noise_v{0.1};          // [(m/s)²/s²]
        double process_noise_omega{0.1};      // [(rad/s)²/s²]
        
        // Measurement noise for wheel odometry
        double odom_velocity_variance{0.04};  // [m²/s²] - variance of velocity measurement
        double odom_omega_variance{0.01};     // [rad²/s²] - variance of angular vel from steering
        
        // Measurement noise for IMU
        double imu_accel_variance{0.1};       // [m²/s⁴] - variance of acceleration
        double imu_omega_variance{0.001};     // [rad²/s²] - variance of gyro angular vel
        
        // Measurement noise for MCL/localization pose updates
        double mcl_x_variance{0.05};          // [m²]
        double mcl_y_variance{0.05};          // [m²]
        double mcl_theta_variance{0.01};      // [rad²]
        
        // Vehicle parameters
        double wheelbase{0.3302};             // [m]
    };
    
    ExtendedKalmanFilter();
    explicit ExtendedKalmanFilter(const Config& config);
    
    /**
     * @brief Initialize the filter with a known pose
     */
    void initialize(double x, double y, double theta, double v = 0.0, double omega = 0.0);
    
    /**
     * @brief Prediction step using motion model
     * Call this at fixed rate (e.g., 200 Hz) between sensor updates
     * @param dt Time step [s]
     */
    void predict(double dt);
    
    /**
     * @brief Update with wheel odometry measurement (velocity and angular velocity)
     * @param velocity Linear velocity from wheel encoder [m/s]
     * @param omega Angular velocity (from steering angle + velocity) [rad/s]
     */
    void updateOdometry(double velocity, double omega);
    
    /**
     * @brief Update with IMU measurement
     * @param accel_x Linear acceleration in body frame [m/s²]
     * @param accel_y Lateral acceleration (typically ~0 for bicycle model) [m/s²]
     * @param gyro_z Angular velocity from gyroscope [rad/s]
     */
    void updateIMU(double accel_x, double accel_y, double gyro_z);
    
    /**
     * @brief Update with absolute pose from MCL/localization
     * @param x Position X [m]
     * @param y Position Y [m]
     * @param theta Heading [rad]
     * @param x_var Variance in X (optional, uses config default if 0)
     * @param y_var Variance in Y
     * @param theta_var Variance in theta
     */
    void updatePose(double x, double y, double theta, 
                    double x_var = 0.0, double y_var = 0.0, double theta_var = 0.0);
    
    // State getters
    double x() const { return state_(IDX_X); }
    double y() const { return state_(IDX_Y); }
    double theta() const { return state_(IDX_THETA); }
    double velocity() const { return state_(IDX_V); }
    double omega() const { return state_(IDX_OMEGA); }
    
    const StateVector& state() const { return state_; }
    const StateCovariance& covariance() const { return covariance_; }
    
    /**
     * @brief Get estimated pose with covariance (for publishing)
     */
    void getPoseWithCovariance(double& x, double& y, double& theta,
                                std::array<double, 9>& covariance_3x3) const;
    
    /**
     * @brief Get estimated velocity with covariance
     */
    void getVelocityWithCovariance(double& vx, double& vy, double& omega,
                                    std::array<double, 9>& covariance_3x3) const;
    
    void setConfig(const Config& config) { config_ = config; updateProcessNoise(); }
    const Config& config() const { return config_; }
    
private:
    Config config_;
    StateVector state_;
    StateCovariance covariance_;
    StateCovariance process_noise_;  // Q matrix
    
    bool initialized_{false};
    
    /**
     * @brief Update process noise matrix from config
     */
    void updateProcessNoise();
    
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalizeAngle(double angle);
    
    /**
     * @brief Compute state transition Jacobian
     */
    StateCovariance computeStateJacobian(double dt) const;
};

}  // namespace f1tenth_control

#endif  // F1TENTH_CONTROL_EKF_HPP_
