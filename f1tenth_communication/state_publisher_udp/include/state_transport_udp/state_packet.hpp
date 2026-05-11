#ifndef STATE_TRANSPORT_UDP_STATE_PACKET_HPP_
#define STATE_TRANSPORT_UDP_STATE_PACKET_HPP_

/**
 * @file state_packet.hpp
 * @brief Packet definitions and checksum helpers for UDP state/control transport.
 * @details Defines packed wire-format structs exchanged between Jetson and Kria
 *          and utility helpers for checksum and layout validation.
 * @dependencies array, cstdint, cstddef, mpc_fpga_constants.h
 */

#include <array>
#include <cstdint>
#include <cstddef>
#include "mpc_fpga_constants.h"
namespace state_transport_udp {

constexpr uint32_t PACKET_MAGIC = 0x53545550;  // 'STUP'
constexpr uint16_t PACKET_VERSION = 3;
constexpr size_t MPC_HORIZON = MPC_FPGA_HORIZON_STEPS;

#pragma pack(push, 1)
struct StatePacket {
    uint32_t magic;            // Packet magic for quick framing/validation.
    uint16_t version;          // Wire-format version.
    uint16_t flags;            // Reserved transport flags.
    uint32_t sequence;         // Monotonic packet sequence from sender.
    uint32_t sender_time_ms;   // Sender wall-clock timestamp (ms, wrap-safe).
    uint64_t sender_mono_ns;   // Sender monotonic timestamp for RTT measurement.
    int32_t source_stamp_sec;   // Original ROS stamp sec used as pipeline token.
    uint32_t source_stamp_nanosec; // Original ROS stamp nanosec used as pipeline token.

    int32_t x_fp;              // Vehicle x [m], Q16.16.
    int32_t y_fp;              // Vehicle y [m], Q16.16.
    int32_t theta_fp;          // Vehicle yaw [rad], Q16.16.
    int32_t velocity_fp;       // Longitudinal speed vx [m/s], Q16.16.
    int32_t vy_fp;             // Lateral speed vy [m/s], Q16.16.
    int32_t omega_fp;          // Yaw rate [rad/s], Q16.16.
    int32_t steering_angle_fp; // Current steering angle [rad], Q16.16.

    uint32_t horizon_length;   // Number of valid horizon entries in arrays below.

    /* Full per-step geometry arrays (V2): publisher fills these samples for
     * robust receiver-side projection. All arrays have at least horizon_length
     * valid entries (pad/truncate as appropriate). */
    std::array<int32_t, MPC_HORIZON> ref_x_fp;           // Waypoint X positions [m], Q16.16
    std::array<int32_t, MPC_HORIZON> ref_y_fp;           // Waypoint Y positions [m], Q16.16
    std::array<int32_t, MPC_HORIZON> ref_psi_fp;         // Waypoint heading [rad], Q16.16

    std::array<int32_t, MPC_HORIZON> ref_ey_fp;          // Horizon lateral error reference [m], Q16.16.
    std::array<int32_t, MPC_HORIZON> ref_epsi_fp;        // Horizon heading error reference [rad], Q16.16.
    std::array<int32_t, MPC_HORIZON> ref_vx_fp;          // Horizon target speed [m/s], Q16.16.
    std::array<int32_t, MPC_HORIZON> ref_vy_fp;          // Horizon target lateral speed [m/s], Q16.16.
    std::array<int32_t, MPC_HORIZON> ref_omega_ref_fp;   // Horizon target yaw rate [rad/s], Q16.16.
    std::array<int32_t, MPC_HORIZON> ref_kappa_fp;       // Horizon curvature [rad/m], Q16.16.
    std::array<int32_t, MPC_HORIZON> ref_left_bound_fp;  // Left wall bound [m], Q16.16.
    std::array<int32_t, MPC_HORIZON> ref_right_bound_fp; // Right wall bound [m], Q16.16.

    uint32_t crc32;            // IEEE CRC32 over packet bytes with this field zeroed.
};

struct ControlPacket {
    uint32_t magic;            // Packet magic for quick framing/validation.
    uint16_t version;          // Wire-format version.
    uint16_t flags;            // Transport/diagnostic flags.
    uint32_t sequence;         // Echoed sequence from matching StatePacket.
    uint32_t receiver_time_ms; // Receiver wall-clock timestamp (ms, wrap-safe).
    uint64_t sender_mono_ns;   // Echoed sender monotonic timestamp for RTT computation.
    int32_t source_stamp_sec;   // Echoed ROS pipeline token stamp sec.
    uint32_t source_stamp_nanosec; // Echoed ROS pipeline token stamp nanosec.

    int32_t steering_fp;       // Steering command [rad], Q16.16.
    int32_t speed_fp;          // Speed command [m/s], Q16.16.
    int32_t accel_fp;          // Acceleration command [m/s^2], Q16.16.
    uint32_t solver_status;    // Solver status code.
    uint32_t solver_iterations;// Solver iterations for this cycle.
    uint32_t ultra_process_us; // Kria-side processing duration [us].
    uint32_t reserved;         // Reserved for future metadata.

    uint32_t crc32;            // IEEE CRC32 over packet bytes with this field zeroed.
};
#pragma pack(pop)

/**
 * @brief Compute IEEE CRC32 checksum for a byte sequence.
 * @param data Pointer to input byte buffer.
 * @param len Number of bytes to include in the checksum.
 * @return Computed CRC32 checksum value.
 */
inline uint32_t crc32_ieee(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]);
        for (int j = 0; j < 8; ++j) {
            uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1u)));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

/**
 * @brief Validate packet struct size assumptions for UDP transport.
 * @return true when `StatePacket` remains below transport size limit.
 */
inline bool is_packet_layout_valid() {
    return sizeof(StatePacket) < 2048;
}

}  // namespace state_transport_udp

#endif  // STATE_TRANSPORT_UDP_STATE_PACKET_HPP_
