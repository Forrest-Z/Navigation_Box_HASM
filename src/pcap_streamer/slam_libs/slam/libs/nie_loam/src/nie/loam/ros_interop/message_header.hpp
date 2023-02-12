/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LOAM_MESSAGE_HEADER_H
#define LOAM_MESSAGE_HEADER_H

#include <cmath>
#include <cstdint>
#include <string>

#include <nie/core/time.hpp>

namespace loam {

struct MessageHeader {
    MessageHeader() : seq(-1), stamp(std::chrono::gps_clock::time_point()), frame_id("unknown") {}
    MessageHeader(uint32_t seq, nie::Timestamp_ns const& stamp, std::string const& frame_id)
        : seq(seq), stamp(stamp), frame_id(frame_id) {}

    uint32_t seq;
    nie::Timestamp_ns stamp;
    std::string frame_id;
};

}  // end namespace loam

#endif  // LOAM_MESSAGE_HEADER_H
