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
#include <cstring>
#include <cstdio>
#include <ctime>
#include <limits>
#include <sys/stat.h>
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

        openTimingCsvFile();
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
        if (timing_csv_file_ != nullptr) {
            std::fclose(timing_csv_file_);
            timing_csv_file_ = nullptr;
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
     * @brief Open CSV file for per-packet RTT logging.
     * @return None
     */
    void openTimingCsvFile() {
        const char* log_dir = "log";
        ::mkdir(log_dir, 0755);

        const std::time_t now = std::time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        char timestamp[64];
        std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &tm_now);

        char csv_path[512];
        std::snprintf(csv_path, sizeof(csv_path), "%s/udp_roundtrip_%s.csv", log_dir, timestamp);

        timing_csv_file_ = std::fopen(csv_path, "w");
        if (timing_csv_file_ == nullptr) {
            RCLCPP_WARN(get_logger(), "Failed to open UDP timing CSV file: %s", csv_path);
            return;
        }

        std::fprintf(timing_csv_file_, "idx,rtt_us\n");
        std::fflush(timing_csv_file_);
        RCLCPP_INFO(get_logger(), "UDP timing CSV log: %s", csv_path);
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

            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "UDP control packet has bad solver status=%u, holding last safe command",
                                 packet.solver_status);
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
        drive.header.stamp = now();
        drive.header.frame_id = "base_link";
        drive.drive.steering_angle = steering;
        drive.drive.speed = speed;
        drive.drive.acceleration = accel;
        drive_pub_->publish(drive);

        const uint64_t now_ns = monotonicNowNs();
        if (packet.sender_mono_ns > 0 && now_ns >= packet.sender_mono_ns) {
            const double rtt_us = static_cast<double>(now_ns - packet.sender_mono_ns) / 1e3;

            if (timing_csv_file_ != nullptr) {
                timing_csv_idx_++;
                std::fprintf(timing_csv_file_, "%lu,%.1f\n", timing_csv_idx_, rtt_us);
                std::fflush(timing_csv_file_);
            }

            rtt_window_count_++;
            rtt_window_sum_us_ += rtt_us;
            rtt_window_min_us_ = std::min(rtt_window_min_us_, rtt_us);
            rtt_window_max_us_ = std::max(rtt_window_max_us_, rtt_us);

            const auto now_tp = std::chrono::steady_clock::now();
            const double elapsed_sec = std::chrono::duration<double>(now_tp - rtt_last_print_time_).count();
            if (elapsed_sec >= kStatsPrintIntervalSec && rtt_window_count_ > 0) {
                const double avg_us = rtt_window_sum_us_ / static_cast<double>(rtt_window_count_);
                RCLCPP_INFO(get_logger(),
                            "[UDP] RTT stats (last %.1fs, %lu samples): min=%.1f us, avg=%.1f us, max=%.1f us",
                            elapsed_sec,
                            rtt_window_count_,
                            rtt_window_min_us_,
                            avg_us,
                            rtt_window_max_us_);

                rtt_window_count_ = 0;
                rtt_window_sum_us_ = 0.0;
                rtt_window_min_us_ = std::numeric_limits<double>::infinity();
                rtt_window_max_us_ = 0.0;
                rtt_last_print_time_ = now_tp;
            }
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
            RCLCPP_INFO(get_logger(),
                        "[UDP] seq=%u steer=%.2f deg v=%.2f a=%.2f | status=%u iter=%u",
                        packet.sequence,
                        steering * 57.2958f,
                        speed,
                        accel,
                        packet.solver_status,
                        packet.solver_iterations);
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
    uint64_t timing_csv_idx_{0};
    uint64_t rtt_window_count_{0};
    double rtt_window_sum_us_{0.0};
    double rtt_window_min_us_{std::numeric_limits<double>::infinity()};
    double rtt_window_max_us_{0.0};
    std::chrono::steady_clock::time_point rtt_last_print_time_{std::chrono::steady_clock::now()};
    static constexpr double kStatsPrintIntervalSec = 5.0;
    FILE* timing_csv_file_{nullptr};
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
