#include "gpu_amcl_cpp/core/ekf_node.hpp"
#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <Eigen/Cholesky>

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

    // Publish only from odom callbacks; AMCL callbacks correct the state.

    RCLCPP_INFO(get_logger(),
                "EKF node started — fusing '%s' + '%s' → '%s' (publishing on odom callbacks)",
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
    declare_parameter<double>("amcl_max_latency_sec", 0.08);
    declare_parameter<double>("odom_history_duration_s", 0.2);
    declare_parameter<bool>("amcl_jump_reset_enabled", true);
    declare_parameter<double>("amcl_jump_reset_distance_m", 1.0);
    declare_parameter<double>("amcl_jump_reset_yaw_rad", 1.2);
    declare_parameter<double>("amcl_jump_reset_covariance_scale", 1.0);
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
    amcl_max_latency_sec_ = get_parameter("amcl_max_latency_sec").as_double();
    odom_history_duration_s_ = std::max(
        0.0, get_parameter("odom_history_duration_s").as_double());
    amcl_jump_reset_enabled_ =
        get_parameter("amcl_jump_reset_enabled").as_bool();
    amcl_jump_reset_distance_m_ = std::max(
        0.0, get_parameter("amcl_jump_reset_distance_m").as_double());
    amcl_jump_reset_yaw_rad_ = std::max(
        0.0, get_parameter("amcl_jump_reset_yaw_rad").as_double());
    amcl_jump_reset_covariance_scale_ = std::max(
        0.0, get_parameter("amcl_jump_reset_covariance_scale").as_double());

    if (amcl_max_latency_sec_ <= 0.0) {
        RCLCPP_WARN(get_logger(),
                    "amcl_max_latency_sec <= 0.0, using 0.08 s");
        amcl_max_latency_sec_ = 0.08;
    }
}

void EkfNode::push_odom_sample(const rclcpp::Time& stamp,
                               const Eigen::Vector3d& pose) {
    odom_history_.push_back({stamp, pose[0], pose[1], pose[2]});

    while (odom_history_.size() > 2 &&
           (stamp - odom_history_.front().stamp).seconds() > odom_history_duration_s_) {
        odom_history_.pop_front();
    }
}

bool EkfNode::interpolate_odom_pose(const rclcpp::Time& stamp,
                                    Eigen::Vector3d& odom_pose_out) const {
    if (odom_history_.empty()) {
        return false;
    }

    const auto& first = odom_history_.front();
    if (stamp <= first.stamp) {
        odom_pose_out << first.x, first.y, first.theta;
        return true;
    }

    const auto& last = odom_history_.back();
    if (stamp >= last.stamp) {
        odom_pose_out << last.x, last.y, last.theta;
        return true;
    }

    for (size_t i = 1; i < odom_history_.size(); ++i) {
        const auto& a = odom_history_[i - 1];
        const auto& b = odom_history_[i];

        if (stamp <= b.stamp) {
            const double dt = (b.stamp - a.stamp).seconds();
            if (dt <= 1e-9) {
                odom_pose_out << b.x, b.y, b.theta;
                return true;
            }

            const double t = (stamp - a.stamp).seconds() / dt;
            odom_pose_out[0] = a.x + t * (b.x - a.x);
            odom_pose_out[1] = a.y + t * (b.y - a.y);

            const double dtheta = math_utils::angle_diff(b.theta, a.theta);
            odom_pose_out[2] = math_utils::normalize_angle(a.theta + t * dtheta);
            return true;
        }
    }

    return false;
}

// ─── EKF Prediction ─────────────────────────────────────────────────
void EkfNode::predict(const Eigen::Vector3d& delta,
                      const Eigen::Matrix3d& Q,
                      double dt) {
    // Linearize around the pre-update heading.
    double theta = state_[2];
    double c = std::cos(theta);
    double s = std::sin(theta);

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -delta[0] * s - delta[1] * c;
    F(1, 2) =  delta[0] * c - delta[1] * s;

    // State prediction: compose SE(2) delta.
    state_ = math_utils::se2_compose(state_, delta);

    // The odom message covariance describes pose uncertainty, not a single
    // high-rate callback increment. Scale it by the elapsed odom interval.
    const double noise_dt = std::clamp(dt, 0.0, 0.1);
    P_ = F * P_ * F.transpose() + process_noise_scale_ * noise_dt * Q;
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

    // Kalman gain via linear solve (more stable than explicit inverse).
    Eigen::Matrix3d PHt = P_ * H.transpose();
    Eigen::Matrix3d K = S.ldlt().solve(PHt.transpose()).transpose();

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

bool EkfNode::should_reset_from_amcl(const Eigen::Vector3d& z) const {
    if (!amcl_jump_reset_enabled_ || !initialized_) {
        return false;
    }

    const double dx = z[0] - state_[0];
    const double dy = z[1] - state_[1];
    const double distance = std::hypot(dx, dy);
    const double yaw = std::abs(math_utils::angle_diff(z[2], state_[2]));
    return distance > amcl_jump_reset_distance_m_ ||
           yaw > amcl_jump_reset_yaw_rad_;
}

void EkfNode::reset_from_amcl(const Eigen::Vector3d& z,
                              const Eigen::Matrix3d& R,
                              const rclcpp::Time& amcl_stamp,
                              rclcpp::Time& publish_stamp_out) {
    Eigen::Vector3d reset_state = z;
    publish_stamp_out = amcl_stamp;

    Eigen::Vector3d odom_at_amcl;
    if (odom_received_ && interpolate_odom_pose(amcl_stamp, odom_at_amcl)) {
        const Eigen::Vector3d odom_delta =
            math_utils::se2_relative(odom_at_amcl, prev_odom_);
        reset_state = math_utils::se2_compose(z, odom_delta);
        publish_stamp_out = prev_odom_stamp_;
    }

    state_ = reset_state;
    P_ = amcl_jump_reset_covariance_scale_ * R;
    P_(0, 0) = std::max(P_(0, 0), 1e-6);
    P_(1, 1) = std::max(P_(1, 1), 1e-6);
    P_(2, 2) = std::max(P_(2, 2), 1e-6);
}

// ─── Odom callback (prediction source) ─────────────────────────────
void EkfNode::odom_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(state_mutex_);

    const rclcpp::Time odom_stamp(msg->header.stamp);
    Eigen::Vector3d odom_pose = math_utils::pose_to_vec(msg->pose.pose);
    push_odom_sample(odom_stamp, odom_pose);

    if (!odom_received_) {
        prev_odom_       = odom_pose;
        prev_odom_stamp_ = odom_stamp;
        odom_received_   = true;
        return;
    }

    // Compute relative delta.
    Eigen::Vector3d delta = math_utils::se2_relative(prev_odom_, odom_pose);
    const double dt = (odom_stamp - prev_odom_stamp_).seconds();
    prev_odom_ = odom_pose;
    prev_odom_stamp_ = odom_stamp;

    if (!initialized_) {
        return;
    }

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

    predict(delta, Q, dt);
    lock.unlock();
    publish_and_broadcast(odom_stamp);   // Publish immediately after prediction (§10.2)
}

// ─── AMCL callback (correction source) ─────────────────────────────
void EkfNode::amcl_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    const auto clock_type = get_clock()->get_clock_type();
    const rclcpp::Time amcl_stamp(msg->header.stamp, clock_type);
    const double latency_sec = (now() - amcl_stamp).seconds();
    if (latency_sec > amcl_max_latency_sec_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping delayed AMCL update (latency %.4f s > %.4f s)",
            latency_sec, amcl_max_latency_sec_);
        return;
    }

    std::unique_lock<std::mutex> lock(state_mutex_);

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

    if (!initialized_) {
        state_       = z;
        P_           = R;
        initialized_ = true;
        RCLCPP_INFO(get_logger(), "EKF initialised from AMCL pose.");
        return;
    }

    if (should_reset_from_amcl(z)) {
        rclcpp::Time publish_stamp = amcl_stamp;
        reset_from_amcl(z, R, amcl_stamp, publish_stamp);
        RCLCPP_WARN(
            get_logger(),
            "EKF reset to accepted AMCL relocalization.");
        lock.unlock();
        publish_and_broadcast(publish_stamp);
        return;
    }

    correct(z, R);
}

// ─── Event-driven publish ───────────────────────────────────────────
void EkfNode::publish_and_broadcast(const rclcpp::Time& stamp) {
    Eigen::Vector3d state;
    Eigen::Matrix3d P;
    Eigen::Vector3d odom_base;

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!initialized_) {
            return;
        }

        state = state_;
        P = P_;
        if (!interpolate_odom_pose(stamp, odom_base)) {
            odom_base = prev_odom_;
        }
    }

    // ── Publish PoseWithCovarianceStamped ─────────────────────────
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp    = stamp;
    msg.header.frame_id = global_frame_;
    msg.pose.pose       = math_utils::vec_to_pose(state);

    auto& cov = msg.pose.covariance;
    std::fill(cov.begin(), cov.end(), 0.0);
    cov[0]  = P(0, 0);  cov[1]  = P(0, 1);  cov[5]  = P(0, 2);
    cov[6]  = P(1, 0);  cov[7]  = P(1, 1);  cov[11] = P(1, 2);
    cov[30] = P(2, 0);  cov[31] = P(2, 1);  cov[35] = P(2, 2);

    pose_pub_->publish(msg);

    // ── Broadcast map → odom TF ──────────────────────────────────
    broadcast_tf(msg.header.stamp, state, odom_base);
}

// ─── TF broadcasting ────────────────────────────────────────────────
void EkfNode::broadcast_tf(const rclcpp::Time& stamp,
                           const Eigen::Vector3d& map_base,
                           const Eigen::Vector3d& odom_base) {
    // map → odom = map→base ∘ (odom→base)⁻¹
    Eigen::Vector3d map_odom = math_utils::se2_compose(
        map_base, math_utils::se2_inverse(odom_base));

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
