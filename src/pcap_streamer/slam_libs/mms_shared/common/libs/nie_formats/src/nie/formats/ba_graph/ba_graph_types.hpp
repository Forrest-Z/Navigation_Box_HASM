/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <array>
#include <cstdint>

#include <glog/logging.h>
#include <Eigen/Geometry>

namespace nie {

namespace io {

namespace detail {

using ChunkType = std::array<std::uint8_t, 4>;
using SignatureType = std::array<std::uint8_t, 9>;

constexpr ChunkType kChunkTypeHeader{'I', 'H', 'D', 'R'};
constexpr ChunkType kChunkTypeEndOfFile{'I', 'E', 'O', 'F'};

// Signature type
// (decimal)              137  xx  xx  xx  xx  13  10  26  10
// (ASCII C notation)    \211   X   X   X   X  \r  \n \032 \n
inline constexpr SignatureType CreateSignature(ChunkType const& s) {
    return {137, s[0], s[1], s[2], s[3], 13, 10, 26, 10};
}

}  // namespace detail

// TODO: Should this namespace be added to more formerly ba_graph, now graph related code?
namespace graph {

// File extensions
template <typename T>
inline std::string Extension() {
    LOG(FATAL) << "No extension known for supplied type.";
    return "";
}

}  // namespace graph

// Major.Minor version for per 2 bytes.
using VersionType = std::int32_t;

// There is a dumb ass public define major and minor in sysmacros
inline constexpr VersionType BaGraphVersion(std::int32_t vmajor, std::int32_t vminor) {
    return static_cast<VersionType>((vmajor << 16) | vminor);
}

inline constexpr std::pair<std::int32_t, std::int32_t> BaGraphMajorMinor(VersionType version) {
    return {static_cast<std::int32_t>((version >> 16) & 0xffff), static_cast<std::int32_t>(version & 0xffff)};
}

}  // namespace io

}  // namespace nie

