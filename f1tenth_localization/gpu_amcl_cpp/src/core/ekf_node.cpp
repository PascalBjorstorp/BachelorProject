#include "gpu_amcl_cpp/core/ekf_node.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace gpu_amcl_cpp {

EkfNode::EkfNode(const rclcpp::NodeOptions& options)
    : Node("ekf_localization", options) {
    declare_all_parameters();
    load_parameters();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── Publisher ──────────────────────────────────────────────────
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        output_topic_, rclcpp::QoS(10));

    // ── Subscribers ───────────────────────────────────────────────
    amcl_sub_ = create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        amcl_topic_, rclcpp::QoS(10),
        std::bind(&EkfNode::amcl_callback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        odom_topic_, rclcpp::QoS(10),
        std::bind(&EkfNode::odom_callback, this, std::placeholders::_1));

    // No publish timer — event-driven publishing from callbacks (§10.2).

    RCLCPP_INFO(get_logger(),
                "EKF node started — fusing '%s' + '%s' → '%s' (event-driven)",
                amcl_topic_.c_str(), odom_topic_.c_str(),
                output_topic_.c_str());
}

// ─── Parameters ─────────────────────────────────────────────────────
void EkfNode::declare_all_parameters() {
    declare_parameter<std::string>("amcl_topic", "/amcl_pose");
    declare_parameter<std::string>("odom_topic", "/odom_pose");
    declare_parameter<std::string>("output_topic", "/ekf_pose");
    declare_parameter<std::string>("global_frame", "map");
    declare_parameter<std::string>("odom_frame", "ego_racecar/odom");
    declare_parameter<std::string>("base_frame", "ego_racecar/base_link");

    declare_parameter<double>("transform_tolerance", 0.1);
    declare_parameter<double>("process_noise_scale", 1.0);
}

void EkfNode::load_parameters() {
    amcl_topic_          = get_parameter("amcl_topic").as_string();
    odom_topic_          = get_parameter("odom_topic").as_string();
    output_topic_        = get_parameter("output_topic").as_string();
    global_frame_        = get_parameter("global_frame").as_string();
    odom_frame_          = get_parameter("odom_frame").as_string();
    base_frame_          = get_parameter("base_frame").as_string();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();
    process_noise_scale_ = get_parameter("process_noise_scale").as_double();
}

// ─── EKF Prediction ─────────────────────────────────────────────────
void EkfNode::predict(const Eigen::Vector3d& delta,
                      const Eigen::Matrix3d& Q) {
    // State prediction: compose SE(2) delta.
    state_ = math_utils::se2_compose(state_, delta);

    // Jacobian of the motion model w.r.t. state.
    // f(x, u) = x ⊕ u  →  df/dx = I + ...
    double c = std::cos(state_[2]);
    double s = std::sin(state_[2]);

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -delta[0] * s - delta[1] * c;
    F(1, 2) =  delta[0] * c - delta[1] * s;

    // Covariance prediction.
    P_ = F * P_ * F.transpose() + process_noise_scale_ * Q;
}

// ─── EKF Correction ─────────────────────────────────────────────────
void EkfNode::correct(const Eigen::Vector3d& z,
                      const Eigen::Matrix3d& R) {
    // Observation model: H = I  (we directly observe the pose).
    Eigen::Matrix3d H = Eigen::Matrix3d::Identity();

    // Innovation.
    Eigen::Vector3d y;
    y[0] = z[0] - state_[0];
    y[1] = z[1] - state_[1];
    y[2] = math_utils::angle_diff(z[2], state_[2]);

    // Innovation covariance.
    Eigen::Matrix3d S = H * P_ * H.transpose() + R;

    // Kalman gain.
    Eigen::Matrix3d K = P_ * H.transpose() * S.inverse();

    // State update.
    Eigen::Vector3d dx = K * y;
    state_[0] += dx[0];
    state_[1] += dx[1];
    state_[2]  = math_utils::normalize_angle(state_[2] + dx[2]);

    // Covariance update (Joseph form for numerical stability).
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d ImKH = I - K * H;
    P_ = ImKH * P_ * ImKH.transpose() + K * R * K.transpose();
}

// ─── Odom callback (prediction source) ─────────────────────────────
void EkfNode::odom_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    Eigen::Vector3d odom_pose = math_utils::pose_to_vec(msg->pose.pose);

    if (!odom_received_) {
        prev_odom_     = odom_pose;
        odom_received_ = true;

        if (!initialized_) {
            state_       = odom_pose;
            initialized_ = true;
        }
        return;
    }

    // Compute relative delta.
    Eigen::Vector3d delta = math_utils::se2_relative(prev_odom_, odom_pose);
    prev_odom_ = odom_pose;

    // Extract covariance from message (x, y, yaw → rows 0,1,5).
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    const auto& cov = msg->pose.covariance;
    Q(0, 0) = cov[0];   // xx
    Q(0, 1) = cov[1];   // xy
    Q(1, 0) = cov[6];   // yx
    Q(1, 1) = cov[7];   // yy
    Q(2, 2) = cov[35];  // yaw-yaw

    // Ensure minimum process noise.
    Q(0, 0) = std::max(Q(0, 0), 1e-6);
    Q(1, 1) = std::max(Q(1, 1), 1e-6);
    Q(2, 2) = std::max(Q(2, 2), 1e-6);

    predict(delta, Q);
    publish_and_broadcast();   // Publish immediately after prediction (§10.2)
}

// ─── AMCL callback (correction source) ─────────────────────────────
void EkfNode::amcl_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (!initialized_) {
        state_       = math_utils::pose_to_vec(msg->pose.pose);
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "EKF initialised from AMCL pose.");
    }

    Eigen::Vector3d z = math_utils::pose_to_vec(msg->pose.pose);

    // Extract measurement covariance.
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    const auto& cov = msg->pose.covariance;
    R(0, 0) = cov[0];
    R(0, 1) = cov[1];
    R(1, 0) = cov[6];
    R(1, 1) = cov[7];
    R(2, 2) = cov[35];

    R(0, 0) = std::max(R(0, 0), 1e-6);
    R(1, 1) = std::max(R(1, 1), 1e-6);
    R(2, 2) = std::max(R(2, 2), 1e-6);

    correct(z, R);
    publish_and_broadcast();   // Publish immediately after correction (§10.2)
}

// ─── Event-driven publish (called from callbacks that hold state_mutex_) ──
void EkfNode::publish_and_broadcast() {
    if (!initialized_) return;

    // Caller already holds state_mutex_ — read state directly.
    Eigen::Vector3d state = state_;
    Eigen::Matrix3d P     = P_;

    // ── Publish PoseWithCovarianceStamped ─────────────────────────
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp    = now();
    msg.header.frame_id = global_frame_;
    msg.pose.pose       = math_utils::vec_to_pose(state);

    auto& cov = msg.pose.covariance;
    std::fill(cov.begin(), cov.end(), 0.0);
    cov[0]  = P(0, 0);  cov[1]  = P(0, 1);  cov[5]  = P(0, 2);
    cov[6]  = P(1, 0);  cov[7]  = P(1, 1);  cov[11] = P(1, 2);
    cov[30] = P(2, 0);  cov[31] = P(2, 1);  cov[35] = P(2, 2);

    pose_pub_->publish(msg);

    // ── Broadcast map → odom TF ──────────────────────────────────
    broadcast_tf(msg.header.stamp);
}

// ─── TF broadcasting (caller holds state_mutex_) ────────────────────
void EkfNode::broadcast_tf(const rclcpp::Time& stamp) {
    // state_ IS the map → base_link pose.
    // prev_odom_ IS the odom → base_link pose (from the odom topic).

    // Caller already holds state_mutex_ — read directly.
    Eigen::Vector3d odom_base = prev_odom_;

    // map → odom = map→base ∘ (odom→base)⁻¹
    Eigen::Vector3d map_odom = math_utils::se2_compose(
        state_, math_utils::se2_inverse(odom_base));

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp + rclcpp::Duration::from_seconds(
        transform_tolerance_);
    tf.header.frame_id    = global_frame_;
    tf.child_frame_id     = odom_frame_;
    tf.transform.translation.x = map_odom[0];
    tf.transform.translation.y = map_odom[1];
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = math_utils::yaw_to_quaternion(map_odom[2]);

    tf_broadcaster_->sendTransform(tf);
}

}  // namespace gpu_amcl_cpp
