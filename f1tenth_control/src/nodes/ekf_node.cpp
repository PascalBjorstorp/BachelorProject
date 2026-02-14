#include "f1tenth_control/nodes/ekf_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace f1tenth_control {

EKFNode::EKFNode(const rclcpp::NodeOptions& options)
    : Node("ekf_node", options)
{
    RCLCPP_INFO(get_logger(), "Initializing EKF Node");
    
    declareParameters();
    loadParameters();
    
    // Initialize EKF
    ekf_ = std::make_unique<ExtendedKalmanFilter>(ekf_config_);
    
    // Subscribers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(),
        std::bind(&EKFNode::odomCallback, this, std::placeholders::_1)
    );
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::SensorDataQoS(),
        std::bind(&EKFNode::imuCallback, this, std::placeholders::_1)
    );
    
    mcl_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/mcl_pose", 10,
        std::bind(&EKFNode::mclPoseCallback, this, std::placeholders::_1)
    );
    
    // Publisher
    ekf_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ekf_odom", 10);
    
    // TF broadcaster
    if (publish_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    
    // Prediction timer
    auto period = std::chrono::duration<double>(1.0 / predict_rate_);
    predict_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&EKFNode::predictCallback, this)
    );
    
    last_predict_time_ = now();
    
    RCLCPP_INFO(get_logger(), "EKF Node initialized");
    RCLCPP_INFO(get_logger(), "  Prediction rate: %.1f Hz", predict_rate_);
    RCLCPP_INFO(get_logger(), "  Publish TF: %s", publish_tf_ ? "yes" : "no");
}

void EKFNode::declareParameters() {
    // Frame IDs
    declare_parameter("odom_frame", odom_frame_);
    declare_parameter("base_frame", base_frame_);
    declare_parameter("publish_tf", publish_tf_);
    declare_parameter("predict_rate", predict_rate_);
    
    // Process noise
    declare_parameter("process_noise_x", 0.01);
    declare_parameter("process_noise_y", 0.01);
    declare_parameter("process_noise_theta", 0.01);
    declare_parameter("process_noise_v", 0.1);
    declare_parameter("process_noise_omega", 0.1);
    
    // Measurement noise
    declare_parameter("odom_velocity_variance", 0.04);
    declare_parameter("odom_omega_variance", 0.01);
    declare_parameter("imu_omega_variance", 0.001);
    declare_parameter("mcl_x_variance", 0.05);
    declare_parameter("mcl_y_variance", 0.05);
    declare_parameter("mcl_theta_variance", 0.01);
    
    // Vehicle
    declare_parameter("wheelbase", 0.324);
}

void EKFNode::loadParameters() {
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    predict_rate_ = get_parameter("predict_rate").as_double();
    
    ekf_config_.process_noise_x = get_parameter("process_noise_x").as_double();
    ekf_config_.process_noise_y = get_parameter("process_noise_y").as_double();
    ekf_config_.process_noise_theta = get_parameter("process_noise_theta").as_double();
    ekf_config_.process_noise_v = get_parameter("process_noise_v").as_double();
    ekf_config_.process_noise_omega = get_parameter("process_noise_omega").as_double();
    
    ekf_config_.odom_velocity_variance = get_parameter("odom_velocity_variance").as_double();
    ekf_config_.odom_omega_variance = get_parameter("odom_omega_variance").as_double();
    ekf_config_.imu_omega_variance = get_parameter("imu_omega_variance").as_double();
    ekf_config_.mcl_x_variance = get_parameter("mcl_x_variance").as_double();
    ekf_config_.mcl_y_variance = get_parameter("mcl_y_variance").as_double();
    ekf_config_.mcl_theta_variance = get_parameter("mcl_theta_variance").as_double();
    
    ekf_config_.wheelbase = get_parameter("wheelbase").as_double();
}

void EKFNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Extract velocity from odometry
    double velocity = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;
    
    if (!initialized_) {
        // Initialize EKF with first odometry pose
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        
        ekf_->initialize(x, y, yaw, velocity, omega);
        initialized_ = true;
        
        last_odom_x_ = x;
        last_odom_y_ = y;
        last_odom_theta_ = yaw;
        last_odom_time_ = msg->header.stamp;
        
        RCLCPP_INFO(get_logger(), "EKF initialized at (%.2f, %.2f, %.1f°)",
                    x, y, yaw * 180.0 / M_PI);
        return;
    }
    
    // Update EKF with odometry measurement
    ekf_->updateOdometry(velocity, omega);
    last_odom_time_ = msg->header.stamp;
}

void EKFNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!initialized_) return;
    
    // Extract IMU data
    double accel_x = msg->linear_acceleration.x;
    double accel_y = msg->linear_acceleration.y;
    double gyro_z = msg->angular_velocity.z;
    
    // Update EKF with IMU measurement
    ekf_->updateIMU(accel_x, accel_y, gyro_z);
    last_imu_time_ = msg->header.stamp;
}

void EKFNode::mclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // Extract pose from MCL
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // Extract covariance (if provided)
    // Covariance is 6x6 row-major: [xx, xy, xz, xroll, xpitch, xyaw, ...]
    // We need xx (index 0), yy (index 7), yaw-yaw (index 35)
    double x_var = msg->pose.covariance[0];
    double y_var = msg->pose.covariance[7];
    double theta_var = msg->pose.covariance[35];
    
    // Update EKF with MCL pose
    ekf_->updatePose(x, y, yaw, x_var, y_var, theta_var);
    
    RCLCPP_DEBUG(get_logger(), "MCL update: (%.2f, %.2f, %.1f°)", x, y, yaw * 180.0 / M_PI);
}

void EKFNode::predictCallback() {
    if (!initialized_) return;
    
    // Calculate dt since last prediction
    rclcpp::Time current_time = now();
    double dt = (current_time - last_predict_time_).seconds();
    
    if (dt > 0 && dt < 0.1) {  // Sanity check on dt
        // Run prediction step
        ekf_->predict(dt);
        
        // Publish results
        publishOdometry();
        if (publish_tf_) {
            publishTF();
        }
    }
    
    last_predict_time_ = current_time;
}

void EKFNode::publishOdometry() {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    
    // Position
    odom_msg.pose.pose.position.x = ekf_->x();
    odom_msg.pose.pose.position.y = ekf_->y();
    odom_msg.pose.pose.position.z = 0.0;
    
    // Orientation (quaternion)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, ekf_->theta());
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    
    // Velocity
    odom_msg.twist.twist.linear.x = ekf_->velocity();
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = ekf_->omega();
    
    // Pose covariance (6x6 row-major)
    // [xx, xy, xz, xroll, xpitch, xyaw, yx, yy, ...]
    std::array<double, 9> pose_cov;
    double x, y, theta;
    ekf_->getPoseWithCovariance(x, y, theta, pose_cov);
    
    // Fill 6x6 covariance (we only have x, y, yaw)
    std::fill(odom_msg.pose.covariance.begin(), odom_msg.pose.covariance.end(), 0.0);
    odom_msg.pose.covariance[0] = pose_cov[0];   // xx
    odom_msg.pose.covariance[1] = pose_cov[1];   // xy
    odom_msg.pose.covariance[5] = pose_cov[2];   // x-yaw
    odom_msg.pose.covariance[6] = pose_cov[3];   // yx
    odom_msg.pose.covariance[7] = pose_cov[4];   // yy
    odom_msg.pose.covariance[11] = pose_cov[5];  // y-yaw
    odom_msg.pose.covariance[30] = pose_cov[6];  // yaw-x
    odom_msg.pose.covariance[31] = pose_cov[7];  // yaw-y
    odom_msg.pose.covariance[35] = pose_cov[8];  // yaw-yaw
    
    // Twist covariance (simplified)
    std::fill(odom_msg.twist.covariance.begin(), odom_msg.twist.covariance.end(), 0.0);
    const auto& cov = ekf_->covariance();
    odom_msg.twist.covariance[0] = cov(ExtendedKalmanFilter::IDX_V, ExtendedKalmanFilter::IDX_V);
    odom_msg.twist.covariance[35] = cov(ExtendedKalmanFilter::IDX_OMEGA, ExtendedKalmanFilter::IDX_OMEGA);
    
    ekf_odom_pub_->publish(odom_msg);
}

void EKFNode::publishTF() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    
    transform.transform.translation.x = ekf_->x();
    transform.transform.translation.y = ekf_->y();
    transform.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, ekf_->theta());
    transform.transform.rotation = tf2::toMsg(q);
    
    tf_broadcaster_->sendTransform(transform);
}

}  // namespace f1tenth_control

// Component registration
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_control::EKFNode)

// Main entry point
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_control::EKFNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
