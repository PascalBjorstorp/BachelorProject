#ifndef STATE_TRANSPORT_UDP_STATE_PACKET_HPP_
#define STATE_TRANSPORT_UDP_STATE_PACKET_HPP_

#include <array>
#include <cstdint>
#include <cstddef>

#include "mpc_fpga_constants.h"

namespace state_transport_udp {

constexpr uint32_t PACKET_MAGIC = 0x53545550;  // 'STUP'
constexpr uint16_t PACKET_VERSION = 1;
constexpr size_t MPC_HORIZON = MPC_FPGA_HORIZON_STEPS;

#pragma pack(push, 1)
struct StatePacket {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t sequence;
    uint32_t sender_time_ms;
    uint64_t sender_mono_ns;

    int32_t x_fp;
    int32_t y_fp;
    int32_t theta_fp;
    int32_t velocity_fp;
    int32_t vy_fp;
    int32_t omega_fp;
    int32_t wheel_speed_fp;
    int32_t steering_angle_fp;

    uint32_t waypoint_index;
    uint32_t horizon_length;

    std::array<int32_t, MPC_HORIZON> ref_x_fp;
    std::array<int32_t, MPC_HORIZON> ref_y_fp;
    std::array<int32_t, MPC_HORIZON> ref_psi_fp;
    std::array<int32_t, MPC_HORIZON> ref_vx_fp;
    std::array<int32_t, MPC_HORIZON> ref_kappa_fp;
    std::array<int32_t, MPC_HORIZON> ref_ax_fp;

    uint32_t crc32;
};

struct ControlPacket {
    uint32_t magic;
    uint16_t version;
    uint16_t flags;
    uint32_t sequence;
    uint32_t receiver_time_ms;
    uint64_t sender_mono_ns;

    int32_t steering_fp;
    int32_t speed_fp;
    int32_t accel_fp;
    uint32_t solver_status;
    uint32_t solver_iterations;
    uint32_t ultra_process_us;
    uint32_t reserved;

    uint32_t crc32;
};
#pragma pack(pop)

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

inline bool is_packet_layout_valid() {
    return sizeof(StatePacket) < 2048;
}

}  // namespace state_transport_udp

#endif  // STATE_TRANSPORT_UDP_STATE_PACKET_HPP_
