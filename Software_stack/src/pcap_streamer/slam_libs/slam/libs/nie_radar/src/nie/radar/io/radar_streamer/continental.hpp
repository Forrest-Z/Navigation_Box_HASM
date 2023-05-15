/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

namespace nie::io::continental {

// TODO(DCo): Probably in the future we should re-use this (nie/core/endian?)

inline constexpr std::uint8_t BigToLittle8(std::uint8_t const v) { return v; }

inline constexpr std::uint16_t BigToLittle16(std::uint16_t const v) {
    std::uint8_t const* data = reinterpret_cast<std::uint8_t const*>(&v);
    return (data[0] << 8) + data[1];
}

inline constexpr std::uint32_t BigToLittle32(std::uint32_t const v) {
    std::uint8_t const* data = reinterpret_cast<std::uint8_t const*>(&v);
    return (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];
}

// Structure taken from: https://confluence.navinfo.eu/display/LIGHTYEAR/Radar
// NOTE: SomeIpHeader and E2EP06Header packets are big endian!

enum class ShapeQualifier : std::uint8_t { kInvalid = 0, kValid = 7 };

enum class ServiceId { kObjects = 230 };

enum class MethodId { kObjects1 = 1, kObjects2 = 2 };

// Resolution of Object values
// Further info at: https://confluence.navinfo.eu/display/LIGHTYEAR/Radar -> ARS430DI_EthInterface.pdf
namespace obj_const_res {
double constexpr kPosition = 0.009155553;
double constexpr kVelocity = 0.004577776;
double constexpr kAcceleration = 0.001525925;
double constexpr kStdDeviationPos = 0.000457778;
double constexpr kStdDeviationVel = 0.000457778;
double constexpr kStdDeviationAcc = 0.118110236;
double constexpr kDistance = 0.001525925;
double constexpr kYaw = 9.58767E-05;

}  // namespace obj_const_res

#pragma pack(push, 1)

// Structure taken from: https://confluence.navinfo.eu/display/LIGHTYEAR/Radar
// Header in Big Endian!
struct SomeIpHeader {
    // Byte 0/1
    std::uint16_t service_id;

    // Byte 2/3
    std::uint16_t method_id;

    // Byte 4/5/6/7: SOME/IP Payload + 8
    std::uint32_t length_someip_h;

    // Byte 8/9
    std::uint16_t client_id;  // it is 0 in conti_radar packet

    // Byte 10/11
    std::uint16_t session_id;  // it is 0 in conti_radar packet

    // Byte 12
    std::uint8_t protocol_version;  // it is 1 in conti_radar packet

    // Byte 13
    std::uint8_t interface_version;  // it is 1 in conti_radar packet

    // Byte 14
    std::uint8_t message_type;  // it is 1 in conti_radar packet

    // Byte 15
    std::uint8_t return_code;  // it is 0 in conti_radar packet
};

// E2EP06Header is the Autograd header inside SOMEIP payload.
// For further info see: https://confluence.navinfo.eu/display/LIGHTYEAR/Radar -> ARS430DI_EthInterface.pdf
struct E2EP06Header {
    std::uint16_t crc;
    std::uint16_t length_someip_pl;
    std::uint8_t sqc;
};

// Page 9
// object struct data is little endian
struct Object {
    std::uint8_t f_obj_id;
    std::uint16_t f_dist_x;
    std::uint16_t f_dist_y;
    std::uint16_t f_v_abs_x;
    std::uint16_t f_v_abs_y;
    std::uint16_t f_acc_abs_x;
    std::uint16_t f_acc_abs_y;
    std::uint16_t f_dist_x_std;
    std::uint16_t f_dist_y_std;
    std::uint16_t f_v_abs_x_std;
    std::uint16_t f_v_abs_y_std;
    std::uint8_t f_a_abs_x_std;
    std::uint8_t f_a_abs_y_std;
    std::array<std::uint16_t, 3> a_l_delta_x;
    std::array<std::uint16_t, 3> a_l_delta_y;
    std::uint8_t e_shape_qualifier;
    std::uint16_t f_obj_orientation;
    std::uint16_t f_rcs;
    std::uint8_t u_score;
    std::uint16_t u_life_cycles;
    std::uint8_t e_dynamic_property;
    std::uint8_t e_obj_state;
};

// Page 8
struct Objects {
    std::uint16_t crc;                       // big endian
    std::uint16_t length_obj_msg;            // big endian
    std::uint8_t sqc;                        // big endian, sequence counter
    std::uint8_t sensors_id;                 // little endian
    std::uint8_t message_counter;            // little endian
    std::uint16_t global_PTP_MSB;            // little endian
    std::uint32_t global_PTP_LSB;            // little endian
    std::uint32_t global_PTP_nanoseconds;    // little endian
    std::uint8_t status_PTP_synch;           // little endian
    std::uint32_t ARS_int_timestamp;         // little endian
    std::uint32_t internal_component_count;  // little endian
    std::uint8_t signal_status;              // little endian
    std::int16_t objects_ego_vx;             // little endian
    std::int16_t objects_ego_yaw_rate;       // little endian
    std::uint16_t available_objects_num;     // little endian
    // obstacle list of the conti radar ARS430DI is always 31
    std::array<Object, 31> list;
};

#pragma pack(pop)

static_assert(sizeof(SomeIpHeader) == 16);
static_assert(sizeof(E2EP06Header) == 5);
static_assert(sizeof(Object) == 45);
static_assert(sizeof(Objects) == 1428);

}  // namespace nie::io::continental
