/**
 * @file mpc_receiver_mpc.cpp
 * @brief MPC Receiver — Software Frenet Controller (test/fallback, no FPGA)
 *
 * Runs on Ultra96 (or any platform for testing).
 * Subscribes to MpcState, computes Frenet-frame errors,
 * applies a proportional lateral + heading controller, publishes /drive.
 *
 * This is a TEST/FALLBACK node for validating the communication pipeline
 * without FPGA hardware. For production, use mpc_receiver_mpc_fpga_node.
 *
 * Controller:
 *   δ = -K_ey * e_y - K_epsi * e_psi
 *   speed = v_ref * (1 - K_slow * min(|e_y|, 1))
 *
 * ALL data arrives in Q16.16 fixed-point from MpcState.
 * Float conversions are done on the CPU for the controller math.
 */

#include <rclcpp/rclcpp.hpp>
#include <f1tenth_msgs/msg/mpc_state.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>

namespace f1tenth_communication {

/*===========================================================================
 * Fixed-Point Helpers (Q16.16)
 *===========================================================================*/

static constexpr int32_t FP_SCALE = 65536;

inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

inline int32_t float_to_fp(float f) {
    return static_cast<int32_t>(f * static_cast<float>(FP_SCALE));
}

/*===========================================================================
 * Waypoint Structure (Q16.16 fixed-point, loaded from CSV)
 *===========================================================================*/

struct WaypointFP {
    int32_t s_fp;      // Arc-length [m]
    int32_t x_fp;      // Position X [m]
    int32_t y_fp;      // Position Y [m]
    int32_t psi_fp;    // Heading [rad]
    int32_t kappa_fp;  // Curvature [1/m]
    int32_t vx_fp;     // Target velocity [m/s]
    int32_t ax_fp;     // Target acceleration [m/s²]
};

/*===========================================================================
 * MPC Horizon Reference (extracted from trajectory)
 *===========================================================================*/

static constexpr size_t MAX_HORIZON = 20;

struct MpcHorizon {
    float ref_x[MAX_HORIZON];
    float ref_y[MAX_HORIZON];
    float ref_psi[MAX_HORIZON];
    float ref_vx[MAX_HORIZON];
    float ref_kappa[MAX_HORIZON];
    float ref_ax[MAX_HORIZON];
    size_t length;
};

/*===========================================================================
 * MPC Receiver Software Node
 *===========================================================================*/

class MpcReceiverMpcNode : public rclcpp::Node {
public:
    MpcReceiverMpcNode() : Node("mpc_receiver_mpc") {
        // --- Declare parameters ---
        declare_parameter("trajectory_file", "");
        declare_parameter("input_topic", "/mpc_state");
        declare_parameter("drive_topic", "/drive");
        declare_parameter("horizon", static_cast<int>(MAX_HORIZON));

        // Controller gains
        declare_parameter("K_ey", 1.0);
        declare_parameter("K_epsi", 1.5);
        declare_parameter("K_slowdown", 0.5);

        // Vehicle limits
        declare_parameter("max_steering", 0.4189);  // ~24°
        declare_parameter("max_velocity", 6.0);
        declare_parameter("wheelbase", 0.324);

        // Control interval for acceleration estimate
        declare_parameter("control_dt", 0.02);  // [s] (default 50 Hz)

        // --- Read parameters ---
        auto trajectory_file = get_parameter("trajectory_file").as_string();
        auto input_topic     = get_parameter("input_topic").as_string();
        auto drive_topic     = get_parameter("drive_topic").as_string();
        horizon_             = static_cast<size_t>(get_parameter("horizon").as_int());

        K_ey_         = static_cast<float>(get_parameter("K_ey").as_double());
        K_epsi_       = static_cast<float>(get_parameter("K_epsi").as_double());
        K_slowdown_   = static_cast<float>(get_parameter("K_slowdown").as_double());
        max_steering_ = static_cast<float>(get_parameter("max_steering").as_double());
        max_velocity_ = static_cast<float>(get_parameter("max_velocity").as_double());
        control_dt_   = static_cast<float>(get_parameter("control_dt").as_double());

        if (trajectory_file.empty()) {
            RCLCPP_ERROR(get_logger(), "No trajectory file specified!");
            return;
        }

        if (!load_trajectory(trajectory_file)) {
            RCLCPP_ERROR(get_logger(), "Failed to load trajectory: %s",
                         trajectory_file.c_str());
            return;
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu waypoints (Q16.16) from %s",
                     trajectory_.size(), trajectory_file.c_str());

        // --- Create publisher (default QoS for drive) ---
        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);

        // --- Subscribe with Best Effort QoS (matches state_publisher) ---
        auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
        sub_ = create_subscription<f1tenth_msgs::msg::MpcState>(
            input_topic, qos,
            std::bind(&MpcReceiverMpcNode::state_callback, this,
                      std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                     "MPC Receiver (software) ready. %s → %s  (horizon=%zu)",
                     input_topic.c_str(), drive_topic.c_str(), horizon_);
    }

private:
    std::vector<WaypointFP> trajectory_;
    size_t horizon_ = MAX_HORIZON;

    float K_ey_, K_epsi_, K_slowdown_;
    float max_steering_, max_velocity_;
    float control_dt_ = 0.02f;  // Control interval for acceleration estimate [s]

    rclcpp::Subscription<f1tenth_msgs::msg::MpcState>::SharedPtr sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    uint64_t msg_count_       = 0;
    double   total_latency_ms_ = 0.0;

    /*-----------------------------------------------------------------------
     * Load trajectory CSV directly to Q16.16 fixed-point
     *---------------------------------------------------------------------*/
    bool load_trajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) return false;

        trajectory_.clear();
        std::string line;
        std::getline(file, line);  // skip header

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string tok;
            WaypointFP wp{};

            std::getline(ss, tok, ','); wp.s_fp     = float_to_fp(std::stof(tok));
            std::getline(ss, tok, ','); wp.x_fp     = float_to_fp(std::stof(tok));
            std::getline(ss, tok, ','); wp.y_fp     = float_to_fp(std::stof(tok));
            std::getline(ss, tok, ','); wp.psi_fp   = float_to_fp(std::stof(tok));
            std::getline(ss, tok, ','); wp.kappa_fp = float_to_fp(std::stof(tok));
            std::getline(ss, tok, ','); wp.vx_fp    = float_to_fp(std::stof(tok));
            std::getline(ss, tok, ','); wp.ax_fp    = float_to_fp(std::stof(tok));

            trajectory_.push_back(wp);
        }
        return !trajectory_.empty();
    }

    /*-----------------------------------------------------------------------
     * Extract N-step reference horizon starting from waypoint index
     *---------------------------------------------------------------------*/
    void extract_horizon(uint32_t start_idx, MpcHorizon& h) const {
        const size_t N   = trajectory_.size();
        h.length = std::min(horizon_, std::min(N, MAX_HORIZON));

        for (size_t i = 0; i < h.length; ++i) {
            size_t idx      = (start_idx + i) % N;
            h.ref_x[i]     = fp_to_float(trajectory_[idx].x_fp);
            h.ref_y[i]     = fp_to_float(trajectory_[idx].y_fp);
            h.ref_psi[i]   = fp_to_float(trajectory_[idx].psi_fp);
            h.ref_vx[i]    = fp_to_float(trajectory_[idx].vx_fp);
            h.ref_kappa[i] = fp_to_float(trajectory_[idx].kappa_fp);
            h.ref_ax[i]    = fp_to_float(trajectory_[idx].ax_fp);
        }
    }

    /*-----------------------------------------------------------------------
     * Wrap angle to [-π, π]
     *---------------------------------------------------------------------*/
    static float wrap_angle(float a) {
        while (a >  static_cast<float>(M_PI)) a -= 2.0f * static_cast<float>(M_PI);
        while (a < -static_cast<float>(M_PI)) a += 2.0f * static_cast<float>(M_PI);
        return a;
    }

    /*-----------------------------------------------------------------------
     * State callback  —  Frenet computation + proportional controller
     *---------------------------------------------------------------------*/
    void state_callback(const f1tenth_msgs::msg::MpcState::SharedPtr msg) {
        auto t_start = std::chrono::high_resolution_clock::now();

        const size_t N = trajectory_.size();
        if (N == 0) return;

        // --- 1. Decode vehicle state from fixed-point ---
        const float x     = fp_to_float(msg->x_fp);
        const float y     = fp_to_float(msg->y_fp);
        const float theta = fp_to_float(msg->theta_fp);
        const float vx    = fp_to_float(msg->velocity_fp);

        // --- 2. Get closest waypoint (from Jetson KD-tree lookup) ---
        const uint32_t wp_idx = msg->waypoint_index % N;

        // --- 3. Compute Frenet-frame errors at closest waypoint ---
        const float wx   = fp_to_float(trajectory_[wp_idx].x_fp);
        const float wy   = fp_to_float(trajectory_[wp_idx].y_fp);
        const float wpsi = fp_to_float(trajectory_[wp_idx].psi_fp);

        const float dx = x - wx;
        const float dy = y - wy;
        const float e_y   = -std::sin(wpsi) * dx + std::cos(wpsi) * dy;
        const float e_psi = wrap_angle(theta - wpsi);

        // --- 4. Extract N-step horizon (available for future MPC) ---
        MpcHorizon horizon;
        extract_horizon(wp_idx, horizon);

        // --- 5. Proportional lateral + heading controller ---
        float steering = -K_ey_ * e_y - K_epsi_ * e_psi;
        steering = std::clamp(steering, -max_steering_, max_steering_);

        // Reference velocity from a few steps ahead (smoother)
        const size_t look_steps = std::min(static_cast<size_t>(5), horizon.length);
        float v_ref = (look_steps > 0) ? horizon.ref_vx[look_steps - 1] : 0.0f;
        v_ref = std::min(v_ref, max_velocity_);

        // Slow down proportionally to cross-track error
        float speed = v_ref * (1.0f - K_slowdown_ * std::min(std::abs(e_y), 1.0f));
        speed = std::max(speed, 0.0f);

        // Estimate acceleration for downstream consumers (VESC may use this)
        float accel = (speed - vx) / control_dt_;

        // --- 6. Publish AckermannDriveStamped ---
        auto drive = ackermann_msgs::msg::AckermannDriveStamped();
        drive.header.stamp    = now();
        drive.header.frame_id = "base_link";
        drive.drive.steering_angle = steering;
        drive.drive.speed          = speed;
        drive.drive.acceleration   = accel;
        drive_pub_->publish(drive);

        // --- 7. Timing & logging ---
        auto t_end      = std::chrono::high_resolution_clock::now();
        auto compute_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              t_end - t_start).count();
        msg_count_++;

        auto now_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
        double latency_ms = static_cast<double>(now_ms - msg->timestamp_ms);
        total_latency_ms_ += latency_ms;

        if (msg_count_ % 100 == 0) {
            double avg = total_latency_ms_ / static_cast<double>(msg_count_);
            RCLCPP_INFO(get_logger(),
                "[SW-MPC] WP=%u  e_y=%.3f  e_psi=%.1f deg | "
                "delta=%.1f deg  v=%.1f m/s | %ld us | Lat %.1f ms (avg %.1f)",
                wp_idx, e_y, e_psi * 57.2958f,
                steering * 57.2958f, speed,
                compute_us, latency_ms, avg);
        }
    }
};

}  // namespace f1tenth_communication

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<f1tenth_communication::MpcReceiverMpcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
