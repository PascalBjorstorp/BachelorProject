/**
 * @file udp_control_bridge.cpp
 * @brief Jetson-side UDP control receiver and ROS drive publisher.
 * @details Receives control packets from Kria, validates packet integrity,
 *          publishes Ackermann commands, and enforces watchdog stop behavior.
 * @dependencies rclcpp, ackermann_msgs, state_transport_udp/state_packet.hpp,
 *               POSIX sockets
 */

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "state_transport_udp/state_packet.hpp"
#include "mpc_fpga_interface.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <string>

namespace state_transport_udp {

static constexpr int32_t FP_SCALE = MPC_FPGA_QP_SCALE_I32;

/**
 * @brief Convert raw QP fixed-point value to float.
 * @param fp Fixed-point input value.
 * @return Floating-point representation of `fp`.
 */
inline float fp_to_float(int32_t fp) {
    return static_cast<float>(fp) / static_cast<float>(FP_SCALE);
}

class UdpControlBridge : public rclcpp::Node {
public:
    /**
     * @brief Construct UDP control bridge and initialize socket + timers.
     * @return None
     */
    UdpControlBridge() : Node("udp_control_bridge") {
        declare_parameter<std::string>("drive_topic", "/drive");
        declare_parameter<int>("listen_port", 49001);
        declare_parameter<double>("watchdog_timeout_ms", 100.0);

        const std::string drive_topic = get_parameter("drive_topic").as_string();
        const int listen_port = get_parameter("listen_port").as_int();
        const double watchdog_ms = get_parameter("watchdog_timeout_ms").as_double();
        const int poll_period_us = std::max(50, static_cast<int>(MPC_FPGA_BRIDGE_POLL_PERIOD_US));

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, rclcpp::SystemDefaultsQoS());

        sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            throw std::runtime_error("Failed to create UDP socket for control bridge");
        }

        int flags = ::fcntl(sock_fd_, F_GETFL, 0);
        if (flags < 0 || ::fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            ::close(sock_fd_);
            sock_fd_ = -1;
            throw std::runtime_error("Failed to configure UDP socket as non-blocking");
        }

        sockaddr_in bind_addr{};
        bind_addr.sin_family = AF_INET;
        bind_addr.sin_port = htons(static_cast<uint16_t>(listen_port));
        bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (::bind(sock_fd_, reinterpret_cast<const sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0) {
            ::close(sock_fd_);
            sock_fd_ = -1;
            throw std::runtime_error("Failed to bind UDP control socket");
        }

        recv_timer_ = create_wall_timer(
            std::chrono::microseconds(poll_period_us),
            std::bind(&UdpControlBridge::pollSocket, this));

        watchdog_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(watchdog_ms)),
            std::bind(&UdpControlBridge::watchdogTick, this));

    }

    /**
     * @brief Destroy bridge and close UDP socket.
     * @return None
     */
    ~UdpControlBridge() override {
        if (sock_fd_ >= 0) {
            ::close(sock_fd_);
            sock_fd_ = -1;
        }
    }

private:
    /**
     * @brief Poll UDP socket and process all available control packets.
     * @return None
     */
    void pollSocket() {
        ControlPacket packet{};
        sockaddr_in peer_addr{};
        socklen_t peer_len = sizeof(peer_addr);

        while (true) {
            const ssize_t n = ::recvfrom(
                sock_fd_,
                &packet,
                sizeof(packet),
                0,
                reinterpret_cast<sockaddr*>(&peer_addr),
                &peer_len);

            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }
                break;
            }

            if (n != static_cast<ssize_t>(sizeof(packet))) {
                continue;
            }

            if (packet.magic != PACKET_MAGIC || packet.version != PACKET_VERSION) {
                continue;
            }

            const uint32_t rx_crc = packet.crc32;
            packet.crc32 = 0;
            const uint32_t calc_crc = crc32_ieee(
                reinterpret_cast<const uint8_t*>(&packet),
                sizeof(packet) - sizeof(packet.crc32));
            if (rx_crc != calc_crc) {
                continue;
            }

            handleControlPacket(packet);
        }
    }

    /**
     * @brief Decode validated control packet and publish ROS drive command.
     * @param packet Validated control packet payload.
     * @return None
     */
    void handleControlPacket(const ControlPacket& packet) {
        const bool bad_status =
            (packet.solver_status == MPC_FPGA_STATUS_ERROR) ||
            (packet.solver_status == MPC_FPGA_STATUS_NO_TRAJECTORY);

        float steering = 0.0f;
        float speed = 0.0f;
        float accel = 0.0f;

        if (bad_status) {
            steering = last_safe_steering_;
            speed = last_safe_speed_;
            accel = last_safe_accel_;
        } else {
            steering = std::clamp(
                fp_to_float(packet.steering_fp),
                -static_cast<float>(MPC_FPGA_MAX_STEER_RAD),
                static_cast<float>(MPC_FPGA_MAX_STEER_RAD));

            speed = std::clamp(
                fp_to_float(packet.speed_fp),
                0.0f,
                static_cast<float>(MPC_FPGA_MAX_VEL_MPS));

            accel = fp_to_float(packet.accel_fp);

            last_safe_steering_ = steering;
            last_safe_speed_ = speed;
            last_safe_accel_ = accel;
        }
        
        auto drive = ackermann_msgs::msg::AckermannDriveStamped();
        if (packet.source_stamp_sec != 0 || packet.source_stamp_nanosec != 0) {
            drive.header.stamp.sec = packet.source_stamp_sec;
            drive.header.stamp.nanosec = packet.source_stamp_nanosec;
        } else {
            drive.header.stamp = now();
        }
        drive.header.frame_id = "base_link";
        drive.drive.steering_angle = steering;
        drive.drive.speed = speed;
        drive.drive.acceleration = accel;
        drive_pub_->publish(drive);

        packet_count_++;
        last_packet_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief Publish stop command if control packets time out.
     * @return None
     */
    void watchdogTick() {
        if (packet_count_ == 0) {
            return;
        }

        const double timeout_ms = get_parameter("watchdog_timeout_ms").as_double();
        const auto now_tp = std::chrono::steady_clock::now();
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            now_tp - last_packet_time_).count();

        if (elapsed_ms > timeout_ms) {
            auto drive = ackermann_msgs::msg::AckermannDriveStamped();
            drive.header.stamp = now();
            drive.header.frame_id = "base_link";
            drive.drive.steering_angle = 0.0f;
            drive.drive.speed = 0.0f;
            drive.drive.acceleration = 0.0f;
            drive_pub_->publish(drive);
        }
    }

    int sock_fd_{-1};                       // UDP socket file descriptor
    uint64_t packet_count_{0};              // Total number of valid control packets received
    float last_safe_steering_{0.0f};
    float last_safe_speed_{0.0f};
    float last_safe_accel_{0.0f};
    std::chrono::steady_clock::time_point last_packet_time_{std::chrono::steady_clock::now()};

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr recv_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace state_transport_udp

/**
 * @brief Entry point for UDP control bridge process.
 * @param argc Argument count from process invocation.
 * @param argv Argument vector from process invocation.
 * @return Process exit code (0 on normal shutdown).
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<state_transport_udp::UdpControlBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
