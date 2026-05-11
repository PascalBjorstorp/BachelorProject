#pragma once

#include "gpu_amcl_cpp/helpers/math_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <mutex>
#include <deque>

namespace gpu_amcl_cpp {

/**
 * @brief Extended Kalman Filter node — fuses AMCL and odom-based
 *        pose estimates into a single localisation output.
 *
 * Prediction uses the odom model; correction uses AMCL.
 * Broadcasts the map → odom TF.
 *
 * State vector: [x, y, θ]  (SE(2) pose in the map frame).
 */
class EkfNode : public rclcpp::Node {
public:
    /**
     * @brief Construct EKF fusion node and initialize ROS interfaces/parameters.
     *
     * Input:
     *   - options: ROS2 node options for composition and runtime configuration.
     * Output:
     *   - EkfNode instance ready to subscribe/publish and run EKF callbacks.
     */
    explicit EkfNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    // ── Callbacks ──────────────────────────────────────────────────
    /**
     * @brief Handle incoming AMCL pose measurement updates.
     *
     * Input:
     *   - msg: AMCL pose-with-covariance measurement in map frame.
     * Output:
     *   - Triggers EKF correction/update path.
     */
    void amcl_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    
    /**
     * @brief Handle incoming odometry pose updates used for EKF prediction.
     *
     * Input:
     *   - msg: Odom-based pose-with-covariance message.
     * Output:
     *   - Triggers EKF prediction/update path.
     */
    void odom_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    // ── EKF steps ──────────────────────────────────────────────────
    /**
     * @brief Perform EKF prediction step from odometry delta.
     *
     * Input:
     *   - odom_delta: Relative odometry motion [dx, dy, dyaw].
     *   - Q: Process noise covariance for prediction.
     * Output:
     *   - Updates internal EKF state_ and covariance P_.
     */
    void predict(const Eigen::Vector3d& odom_delta,
                 const Eigen::Matrix3d& Q);

    /**
     * @brief Perform EKF correction step using AMCL measurement.
     *
     * Input:
     *   - z: Measurement vector [x, y, yaw].
     *   - R: Measurement noise covariance.
     * Output:
     *   - Updates internal EKF state_ and covariance P_.
     */
    void correct(const Eigen::Vector3d& z,
                 const Eigen::Matrix3d& R);

    // ── Helpers ────────────────────────────────────────────────────
    /**
     * @brief Declare all ROS parameters used by this node.
     *
     * Input:
     *   - None.
     * Output:
     *   - Parameter declarations are registered with the node.
     */
    void declare_all_parameters();
    /**
     * @brief Load parameter values from ROS parameter server into members.
     *
     * Input:
     *   - None.
     * Output:
     *   - Member configuration fields are updated from declared parameters.
     */
    void load_parameters();
    /**
     * @brief Broadcast map to odom transform using current EKF estimate.
     *
     * Input:
        *   - stamp: Timestamp for TF transform publication.
        *   - map_base: map -> base pose snapshot.
        *   - odom_base: odom -> base pose snapshot.
     * Output:
     *   - TF map -> odom transform is published.
     */
        void broadcast_tf(const rclcpp::Time& stamp,
                      const Eigen::Vector3d& map_base,
                      const Eigen::Vector3d& odom_base);
    /**
     * @brief Publish fused pose output and broadcast corresponding TF.
     *
     * Input:
        *   - stamp: Timestamp used for pose and TF publication.
     * Output:
     *   - Pose topic and TF outputs are emitted from current EKF state.
     */
        void publish_and_broadcast(const rclcpp::Time& stamp);

        /**
        * @brief Append odom pose sample to interpolation history buffer.
        *
        * Input:
        *   - stamp: Timestamp of the odom sample.
        *   - pose: Odom pose [x, y, yaw].
        * Output:
        *   - Odom history buffer updated (oldest entries dropped if needed).
        */
        void push_odom_sample(const rclcpp::Time& stamp,
                         const Eigen::Vector3d& pose);

        /**
        * @brief Interpolate odom pose at requested timestamp from history.
        *
        * Input:
        *   - stamp: Target timestamp.
        * Output:
        *   - odom_pose_out: Interpolated odom pose if successful.
        *   - Returns true when interpolation/nearest retrieval succeeds.
        */
        bool interpolate_odom_pose(const rclcpp::Time& stamp,
                             Eigen::Vector3d& odom_pose_out) const;

    // ── ROS I/O ────────────────────────────────────────────────────
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr odom_sub_;
    rclcpp::Publisher<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ── EKF state ──────────────────────────────────────────────────
    Eigen::Vector3d state_ = Eigen::Vector3d::Zero();   ///< [x, y, θ]
    Eigen::Matrix3d P_     = Eigen::Matrix3d::Identity(); ///< covariance

    bool initialized_ = false;

    // Previous odom for delta computation
    Eigen::Vector3d prev_odom_ = Eigen::Vector3d::Zero();
    bool odom_received_ = false;

    struct OdomSample {
        rclcpp::Time stamp;
        double x;
        double y;
        double theta;
    };

    std::deque<OdomSample> odom_history_;
    size_t odom_history_max_size_ = 500;

    // ── Parameters ─────────────────────────────────────────────────
    double transform_tolerance_ = 0.1;

    // Process noise scaling (multiplied onto odom covariance)
    double process_noise_scale_ = 1.0;
    double amcl_max_latency_sec_ = 0.08;

    std::string amcl_topic_;
    std::string odom_topic_;
    std::string output_topic_;
    std::string output_stamp_source_ = "odom";
    std::string global_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    rclcpp::Time last_amcl_stamp_;
    bool have_amcl_stamp_ = false;

    std::mutex state_mutex_;
};

}  // namespace gpu_amcl_cpp
