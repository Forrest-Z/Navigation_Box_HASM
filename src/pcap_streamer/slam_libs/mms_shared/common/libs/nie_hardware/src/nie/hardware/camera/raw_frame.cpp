/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "raw_frame.hpp"

namespace nie {

RawFrame::RawFrame(const std::uint32_t& width, const std::uint32_t& height)
    : width_(width), height_(height), data_(width * height) {}

RawFrame::RawFrame(const std::uint32_t& width, const std::uint32_t& height, std::vector<std::uint8_t>&& data)
    : width_(width), height_(height), data_(std::move(data)) {}

void RawFrame::Create(const std::uint32_t& width, const std::uint32_t& height) {
    if (width_ != width || height_ != height) {
        *this = RawFrame(width, height);
    }
}

void RawFrame::Create(const std::uint32_t& width, const std::uint32_t& height, std::vector<std::uint8_t>&& data) {
    *this = RawFrame(width, height, std::move(data));
}

std::uint32_t RawFrame::width() const { return width_; }

std::uint32_t RawFrame::height() const { return height_; }

const std::vector<std::uint8_t>& RawFrame::data() const { return data_; }

}  // namespace nie
