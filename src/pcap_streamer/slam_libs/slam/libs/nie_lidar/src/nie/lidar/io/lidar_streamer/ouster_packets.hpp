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

namespace ouster {

// For a description of the Ouster packet formats see:
//   e.g. OS2-User-Guide-v1.14.0-beta.12 section 9.1

// Ensure that the target architecture is little-endian
static_assert(nie::Endian::kNative == nie::Endian::kLittle);

constexpr static std::size_t kAzimuthBlocksPerPacket = 16;

// Helper struct to compare struct sizes and show the actual sizes in the compilation error message
template <std::size_t size_a, std::size_t size_b, bool condition = size_a == size_b>
struct EqualSizeof : public std::bool_constant<condition> {
    static_assert(condition);
};

// Ensure that packet structures are memory mappable
// TODO: pragmas are not portable. Prefer performing manual (de-)serialization
#pragma pack(push, 1)
struct ChannelDataBlock {
    std::uint32_t range;  // only the 20 LSB are used
    std::uint16_t signal_photons;
    std::uint16_t reflectivity;
    std::uint16_t reserved;
    std::uint16_t ambient_photons;
};

template <std::size_t Channels>
struct AzimuthBlock {
    std::uint64_t timestamp;
    std::uint16_t measurement_id;
    std::uint16_t frame_id;
    std::uint32_t encoder_count;
    std::array<ChannelDataBlock, Channels> channel_data_blocks;
    std::uint32_t block_status;
};
// Values from section 9.2 of the manual
static_assert(EqualSizeof<sizeof(AzimuthBlock<32>), 404>::value);
static_assert(EqualSizeof<sizeof(AzimuthBlock<64>), 788>::value);
static_assert(EqualSizeof<sizeof(AzimuthBlock<128>), 1556>::value);

template <std::size_t Channels>
struct LidarDataPacket {
    std::array<AzimuthBlock<Channels>, kAzimuthBlocksPerPacket> azimuth_blocks;
};
// Values from section 9.2 of the manual
static_assert(EqualSizeof<sizeof(LidarDataPacket<32>), 6464>::value);
static_assert(EqualSizeof<sizeof(LidarDataPacket<64>), 12608>::value);
static_assert(EqualSizeof<sizeof(LidarDataPacket<128>), 24896>::value);
#pragma pack(pop)

using Packet = std::vector<std::uint8_t>;

}  // namespace ouster

}  // namespace io

}  // namespace nie
