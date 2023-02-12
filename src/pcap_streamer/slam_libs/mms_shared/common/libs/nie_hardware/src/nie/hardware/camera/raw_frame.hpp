/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_CAMERA_RAW_FRAME_HPP
#define NIE_HARDWARE_CAMERA_RAW_FRAME_HPP

#include <cstdint>
#include <vector>

#include <iostream>

namespace nie {

class RawFrame {
public:
    RawFrame() = default;
    RawFrame(const std::uint32_t& width, const std::uint32_t& height);
    RawFrame(const std::uint32_t& width, const std::uint32_t& height, std::vector<std::uint8_t>&& data);

    void Create(const std::uint32_t& width, const std::uint32_t& height);
    void Create(const std::uint32_t& width, const std::uint32_t& height, std::vector<std::uint8_t>&& data);

    std::uint32_t width() const;
    std::uint32_t height() const;
    const std::vector<std::uint8_t>& data() const;

    template <typename T>
    T* ptr() {
        return static_cast<T*>(&data_[0]);
    }

private:
    std::uint32_t width_;
    std::uint32_t height_;
    std::vector<std::uint8_t> data_;
};

}  // namespace nie

#endif  // NIE_HARDWARE_CAMERA_RAW_FRAME_HPP
