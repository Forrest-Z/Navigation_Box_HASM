/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

// Standard includes
#include <array>

// NIE includes
#include <nie/core/endian.hpp>

// Local includes
#include "fixed_angle.hpp"

namespace nie {

namespace io {

namespace velodyne {

// For a description of the Velodyne packet format for laser and time packets see:
//   e.g. Velodyne HDL-32E User Manual Appendix B

// Ensure that the target architecture is little-endian
static_assert(nie::Endian::kNative == nie::Endian::kLittle);

constexpr static std::size_t kBlocksPerPacket = 12;
constexpr static std::size_t kLasersPerBlock = 32;
constexpr static std::size_t kLasersPerPacket = kLasersPerBlock * kBlocksPerPacket;

// Ensure that packet structures are memory mappable
#pragma pack(push, 1)

struct LaserReturn {
    std::uint16_t distance;
    std::uint8_t intensity;
};
static_assert(sizeof(LaserReturn) == 3);

struct Block {
    // NOTE: [EDD] I don't understand why Velodyne has chosen to allocate 16 bits in every block to specify the block
    //             type while there are currently only 5 possible values (0xffff, 0xeeff, 0xddff, 0xccff, 0xbbff). This
    //             seems to waste 12-18 bytes per packet, while they could have used the bits for instance to:
    //               - allow for more accurate azimuth
    //               - give a full GPS timestamp instead of microseconds since whole hour
    std::uint16_t type;
    FixedAngle<std::uint16_t> azimuth;
    std::array<LaserReturn, kLasersPerBlock> laser_returns;
};

struct LaserPacket {
    std::array<Block, kBlocksPerPacket> blocks;
    std::uint32_t gps_time_since_hour;  // Time of first laser return in first block [in microseconds since GPS hour]
    std::uint8_t mode;
    std::uint8_t sensor_type;
};
static_assert(sizeof(LaserPacket) == 1206);
static_assert(std::is_trivially_copyable_v<LaserPacket>);

struct TimePacket {
    std::array<std::uint8_t, 198> reserved_1;
    std::uint32_t gps_time_since_hour;  // Time when NMEA string was received [in microseconds since GPS hour]
    std::array<std::uint8_t, 4> reserved_2;
    std::array<char, 72> nmea;  // See: https://en.wikipedia.org/wiki/NMEA_0183
    std::array<std::uint8_t, 234> reserved_3;
};
static_assert(sizeof(TimePacket) == 512);
static_assert(std::is_trivially_copyable_v<TimePacket>);
#pragma pack(pop)

using Packet = std::vector<std::uint8_t>;

}  // namespace velodyne

}  // namespace io

}  // namespace nie
