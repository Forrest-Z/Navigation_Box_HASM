/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_CAMERA_CAMERA_HPP
#define NIE_HARDWARE_CAMERA_CAMERA_HPP

#include <cstdint>  // for std::uint32_t and std::int64_t types
#include <string>

#include <opencv2/core.hpp>

namespace nie {
enum class GrabStrategy { kOneByOne, kLatestImageOnly };

class Camera {
public:
    virtual ~Camera() = default;
    virtual void StartGrabbing(GrabStrategy strategy = GrabStrategy::kOneByOne) = 0;
    virtual void StopGrabbing() = 0;
    virtual bool IsGrabbing() const = 0;

    virtual bool WaitForTriggerReady(std::uint32_t const& timeout_ms, bool throws_on_timeout) = 0;

    // Outputs an image and its corresponding time of capture as system time in nanoseconds
    virtual bool GetFrame(
            std::uint32_t const& timeout_ms,
            cv::Mat* frame,
            std::int64_t* system_time,
            bool const& throws_on_timeout) = 0;

    virtual std::string id() const = 0;
    virtual cv::Size image_size() const = 0;

protected:
    Camera() = default;
};

}  // namespace nie

#endif  // NIE_HARDWARE_CAMERA_CAMERA_HPP
