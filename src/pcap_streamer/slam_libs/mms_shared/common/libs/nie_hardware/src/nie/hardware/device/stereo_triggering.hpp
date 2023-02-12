/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <nie/core/container/mt_circular_buffer.hpp>
#include <nie/hardware/camera/basler_camera.hpp>
#include <opencv2/core.hpp>

namespace nie {

// TODO(jbr): Use a resource pool for the images
class StereoTriggering {
public:
    using Pair = std::pair<std::int64_t, cv::Mat>;
    using PairPtr = std::unique_ptr<Pair>;
    using Buffer = nie::mt::MtCircularBuffer<PairPtr>;

    StereoTriggering(
        int const gpio_port,
        int const trigger_speed_ms,
        std::function<void(PairPtr, PairPtr)> processor,
        nie::BaslerCamera* camera_left,
        nie::BaslerCamera* camera_right);

    ~StereoTriggering();

private:
    static constexpr std::uint32_t kTimeOutMs = 2500;

    void RunTriggering(std::uint32_t const gpio_id, std::uint32_t const trigger_speed_ms);

    void RunGrabber(
        nie::BaslerCamera* camera,
        Buffer* buffer,
        std::vector<std::chrono::time_point<std::chrono::system_clock>>* t_camera);

    void RunProcessor();

    nie::BaslerCamera *camera_left_, *camera_right_;
    std::function<void(PairPtr, PairPtr)> processor_;
    Buffer buffer_left_;
    Buffer buffer_right_;

    std::atomic_bool run_process_;
    std::thread thread_trigger_;
    std::thread thread_left_;
    std::thread thread_right_;
    std::thread thread_processor_;

    // TODO(jbr) IMPROVED DEBUGGING
    std::size_t test_count_;
    std::vector<std::chrono::time_point<std::chrono::system_clock>> t_trigger_;
    std::vector<std::chrono::time_point<std::chrono::system_clock>> t_left_;
    std::vector<std::chrono::time_point<std::chrono::system_clock>> t_right_;
    std::mutex debug_mutex_;
};

}  // namespace nie
