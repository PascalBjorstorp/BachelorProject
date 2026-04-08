/**
 * @file udp_control_bridge.cpp
 * @brief Jetson-side UDP control receiver and ROS drive publisher.
 * @details Receives control packets from Ultra96, validates packet integrity,
 *          publishes Ackermann commands, and enforces watchdog stop behavior.
 * @dependencies rclcpp, ackermann_msgs, state_transport_udp/state_packet.hpp,
 *               POSIX sockets
 */

#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

#include "state_transport_udp/state_packet.hpp"

#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <limits>
#include <string>

namespace state_transport_udp {

static constexpr int32_t FP_SCALE = 65536;

/**
 * @brief Convert Q16.16 fixed-point value to float.
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
        stats_start_speed_mps_ = static_cast<double>(MPC_FPGA_BRIDGE_STATS_START_SPEED_MPS);

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

        RCLCPP_INFO(get_logger(),
                    "UDP control bridge ready: listening on %d, publishing %s (poll=%d us)",
                    listen_port,
                    drive_topic.c_str(),
                    poll_period_us);
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
     * @brief Get monotonic timestamp in nanoseconds.
     * @return Monotonic time in nanoseconds.
     */
    uint64_t monotonicNowNs() const {
        return static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

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
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "UDP recvfrom error: %s", std::strerror(errno));
                break;
            }

            if (n != static_cast<ssize_t>(sizeof(packet))) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Dropped control packet: expected %zu bytes, got %ld",
                                     sizeof(packet), static_cast<long>(n));
                continue;
            }

            if (packet.magic != PACKET_MAGIC || packet.version != PACKET_VERSION) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Dropped control packet: bad magic/version");
                continue;
            }

            const uint32_t rx_crc = packet.crc32;
            packet.crc32 = 0;
            const uint32_t calc_crc = crc32_ieee(
                reinterpret_cast<const uint8_t*>(&packet),
                sizeof(packet) - sizeof(packet.crc32));
            if (rx_crc != calc_crc) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Dropped control packet: CRC mismatch");
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
        // Set steering command
        const float steering = std::clamp(
            fp_to_float(packet.steering_fp),
            -static_cast<float>(MPC_FPGA_MAX_STEER_RAD),
            static_cast<float>(MPC_FPGA_MAX_STEER_RAD));
        
        // Set speed command
        const float speed = std::clamp(
            fp_to_float(packet.speed_fp),
            0.0f,
            static_cast<float>(MPC_FPGA_MAX_VEL_MPS));

        // Acceleration isn't used for direct control but is logged for monitoring.
        const float accel = fp_to_float(packet.accel_fp);

        
        auto drive = ackermann_msgs::msg::AckermannDriveStamped();
        drive.header.stamp = now();
        drive.header.frame_id = "base_link";
        drive.drive.steering_angle = steering;
        drive.drive.speed = speed;
        drive.drive.acceleration = accel;
        drive_pub_->publish(drive);

        const uint64_t now_ns = monotonicNowNs();
        if (!stats_started_ && speed >= static_cast<float>(stats_start_speed_mps_)) {
            stats_started_ = true;
            stats_start_seq_ = packet.sequence;
            stats_packet_count_ = 0;
            total_rtt_ms_ = 0.0;
            total_ultra_us_ = 0.0;
            min_rtt_ms_ = std::numeric_limits<double>::infinity();
            max_rtt_ms_ = 0.0;

            RCLCPP_INFO(get_logger(),
                        "UDP timing stats started at seq=%u (speed=%.2f m/s, threshold=%.2f m/s)",
                        packet.sequence,
                        speed,
                        static_cast<float>(stats_start_speed_mps_));
        }

        if (stats_started_) {
            if (packet.sender_mono_ns > 0 && now_ns >= packet.sender_mono_ns) {
                const double rtt_ms = static_cast<double>(now_ns - packet.sender_mono_ns) / 1e6;
                total_rtt_ms_ += rtt_ms;
                min_rtt_ms_ = std::min(min_rtt_ms_, rtt_ms);
                max_rtt_ms_ = std::max(max_rtt_ms_, rtt_ms);
            }
            total_ultra_us_ += static_cast<double>(packet.ultra_process_us);
            stats_packet_count_++;
        }

        packet_count_++;
        last_packet_time_ = std::chrono::steady_clock::now();

        if (packet_count_ == 1) {
            RCLCPP_INFO(get_logger(),
                        "First UDP control packet received: seq=%u steer=%.2f deg v=%.2f",
                        packet.sequence,
                        steering * 57.2958f,
                        speed);
        }

        if (packet_count_ % 100 == 0) {
            if (stats_started_ && stats_packet_count_ > 0) {
                const double avg_rtt = total_rtt_ms_ / static_cast<double>(stats_packet_count_);
                const double avg_ultra_us = total_ultra_us_ / static_cast<double>(stats_packet_count_);
                RCLCPP_INFO(get_logger(),
                            "[UDP] seq=%u steer=%.2f deg v=%.2f a=%.2f | status=%u iter=%u | RTT avg/min/max=%.2f/%.2f/%.2f ms | Ultra avg=%.1f us (stats_seq>=%u)",
                            packet.sequence,
                            steering * 57.2958f,
                            speed,
                            accel,
                            packet.solver_status,
                            packet.solver_iterations,
                            avg_rtt,
                            min_rtt_ms_,
                            max_rtt_ms_,
                            avg_ultra_us,
                            stats_start_seq_);
            } else {
                RCLCPP_INFO(get_logger(),
                            "[UDP] seq=%u steer=%.2f deg v=%.2f a=%.2f | status=%u iter=%u | timing stats waiting for speed >= %.2f m/s",
                            packet.sequence,
                            steering * 57.2958f,
                            speed,
                            accel,
                            packet.solver_status,
                            packet.solver_iterations,
                            static_cast<float>(stats_start_speed_mps_));
            }
        }
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

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "UDP watchdog: no control packet for %.0f ms, publishing zero drive",
                                 elapsed_ms);
        }
    }

    int sock_fd_{-1};                       // UDP socket file descriptor
    uint64_t packet_count_{0};              // Total number of valid control packets received
    double total_rtt_ms_{0.0};              // Cumulative round-trip time in milliseconds (Jetson send → Ultra96 receive → Ultra96 send → Jetson receive)
    double total_ultra_us_{0.0};            // Cumulative processing time reported by Ultra96 in microseconds
    double min_rtt_ms_{std::numeric_limits<double>::infinity()};
    double max_rtt_ms_{0.0};
    bool stats_started_{false};
    uint64_t stats_packet_count_{0};
    uint32_t stats_start_seq_{0};
    double stats_start_speed_mps_{0.5};
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
