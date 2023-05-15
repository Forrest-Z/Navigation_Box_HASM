/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
//
// Created by jonathan.broere on 12/19/18.
//

#include "stereo_triggering.hpp"

#include <glog/logging.h>
#include <nie/core/glog.hpp>
#include <nie/hardware/device.hpp>

namespace nie {

void TryWaitForTrigger(const std::uint32_t& timeout_ms, const std::size_t& max_attempts, nie::BaslerCamera* camera) {
    for (std::size_t i = 0; i < max_attempts; ++i) {
        try {
            camera->WaitForTriggerReady(timeout_ms);
            return;
        } catch (const std::exception& e) {
            nie::LogException(e);
        }
    }

    throw std::runtime_error("failed to trigger: " + camera->id());
}

// FIXME(jbr) Many hardcoded values that should be based on time
StereoTriggering::StereoTriggering(
    int const gpio_port,
    int const trigger_speed_ms,
    std::function<void(PairPtr, PairPtr)> processor,
    nie::BaslerCamera* camera_left,
    nie::BaslerCamera* camera_right)
    : camera_left_(camera_left),
      camera_right_(camera_right),
      processor_(processor),
      buffer_left_(150),
      buffer_right_(150),
      run_process_(true),
      test_count_(50),
      t_trigger_(50),
      t_left_(50),
      t_right_(50) {
    thread_left_ = std::thread(&StereoTriggering::RunGrabber, this, camera_left_, &buffer_left_, &t_left_);
    thread_right_ = std::thread(&StereoTriggering::RunGrabber, this, camera_right_, &buffer_right_, &t_right_);
    thread_processor_ = std::thread(&StereoTriggering::RunProcessor, this);
    thread_trigger_ = std::thread(&StereoTriggering::RunTriggering, this, gpio_port, trigger_speed_ms);
}

StereoTriggering::~StereoTriggering() {
    run_process_ = false;

    if (thread_trigger_.joinable()) {
        thread_trigger_.join();
    }

    if (thread_processor_.joinable()) {
        thread_processor_.join();
    }

    if (thread_right_.joinable()) {
        thread_right_.join();
    }

    if (thread_left_.joinable()) {
        thread_left_.join();
    }

    // BEGIN DEBUGGING
    //    std::chrono::milliseconds::rep acc_left = 0;
    //    std::chrono::milliseconds::rep acc_right = 0;
    //    std::chrono::milliseconds::rep acc_delta = 0;
    //
    //    for (std::size_t i = 0; i < test_count_; ++i)
    //    {
    //        auto msd_left = std::chrono::duration_cast<std::chrono::microseconds>(t_left_[i] - t_trigger_[i]);
    //        auto ms_left = msd_left.count();
    //        acc_left += ms_left;
    //
    //        auto msd_right = std::chrono::duration_cast<std::chrono::microseconds>(t_right_[i] - t_trigger_[i]);
    //        auto ms_right = msd_right.count();
    //        acc_right += ms_right;
    //
    //        auto msd_delta = std::chrono::duration_cast<std::chrono::microseconds>(t_right_[i] - t_left_[i]);
    //        auto ms_delta = msd_delta.count();
    //        acc_delta += std::abs(ms_delta);
    //
    //        std::cout << "l-t: " << (ms_left /  1000.0) << " r-t: " << (ms_right / 1000.0) << " l-r: " << (ms_delta /
    //        1000.0) << std::endl;
    //    }
    // END DEBUGGING
}

// TODO(Jbr): Slightly improve triggering because of the comment below
// If the trigger speed is close to identical of the maximum capability of the cameras, the buffer of the left (first)
// will slowly fill up (right one is always a bit slower because the left one is handle first).
void StereoTriggering::RunTriggering(std::uint32_t const gpio_id, std::uint32_t const trigger_speed_ms) {
    nie::Gpio gpio(gpio_id);
    std::size_t index = 0;

    VLOG(4) << "Triggering every ms: " << trigger_speed_ms;

    while (run_process_) {
        std::chrono::time_point<std::chrono::system_clock> begin_trigger_loop = std::chrono::system_clock::now();

        // The interesting buffers queued equals max (default 10) when nothing happened and the ready buffers 0
        // When ready buffers equals the max there can be a buffer overflow
        VLOG(5) << "NumQueuedBuffers (" << camera_left_->id()
                << "): " << camera_left_->camera().NumQueuedBuffers.GetValue() << std::endl;
        VLOG(5) << "NumReadyBuffers (" << camera_left_->id()
                << "): " << camera_left_->camera().NumReadyBuffers.GetValue() << std::endl;
        VLOG(5) << "NumQueuedBuffers (" << camera_right_->id()
                << "): " << camera_right_->camera().NumQueuedBuffers.GetValue() << std::endl;
        VLOG(5) << "NumReadyBuffers (" << camera_right_->id()
                << "): " << camera_right_->camera().NumReadyBuffers.GetValue() << std::endl;

        TryWaitForTrigger(kTimeOutMs, 3, camera_left_);
        TryWaitForTrigger(kTimeOutMs, 3, camera_right_);

        auto dt =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - begin_trigger_loop)
                .count();
        auto ms = trigger_speed_ms - 1 - dt;

        VLOG(5) << "Waited for trigger ready in ms: " << dt;

        if (ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }

        gpio.value(nie::GpioValue::kHigh);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        gpio.value(nie::GpioValue::kLow);

        // DEBUGGING
        //        {
        //            std::lock_guard<std::mutex> lock(debug_mutex_);
        //            t_trigger_[index] = std::chrono::system_clock::now();
        //        }
        index = (index + 1) % test_count_;

        std::chrono::time_point<std::chrono::system_clock> end_trigger = std::chrono::system_clock::now();

        VLOG(5) << "Trigger delta 1: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(end_trigger - begin_trigger_loop).count();
        // DEBUGGING
        // VLOG(5) << "Trigger delta 2: " << std::chrono::duration_cast<std::chrono::milliseconds>(t_trigger_[(index -
        // 1) % test_count_] - t_trigger_[(index - 2) % test_count_]).count();
    }

    VLOG(4) << "StereoTriggering::RunTriggering(): Stopped.";
}

void StereoTriggering::RunGrabber(
    nie::BaslerCamera* camera,
    Buffer* buffer,
    std::vector<std::chrono::time_point<std::chrono::system_clock>>* t_camera) {
    std::size_t index = 0;

    camera->StartGrabbing();

    bool has_grab_result = false;

    do {
        std::chrono::milliseconds::rep ms;
        std::chrono::time_point<std::chrono::system_clock> begin_frame_loop = std::chrono::system_clock::now();

        // TODO(jbr): Use an updated version of the resource pool
        cv::Mat frame;
        std::int64_t system_time;

        try {
            has_grab_result = camera->GetFrame(kTimeOutMs, &frame, &system_time);
        }
        // Theoretically handled by the wrapper :D
        //        catch (const Pylon::GenericException& e)
        //        {
        //            throw std::runtime_error("GenericException(): failed to copy frame from grabbed buffer: " +
        //            camera->id() + " - " + e.what());
        //        }
        catch (const std::exception& e) {
            nie::LogException(e);
            throw std::runtime_error("failed to copy frame from grabbed buffer: " + camera->id());
        }

        (*t_camera)[index] = std::chrono::time_point<std::chrono::system_clock>(std::chrono::nanoseconds(system_time));

        ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - begin_frame_loop)
                 .count();

        VLOG(6) << "Camera " << camera->id() << " copied frame in ms: " << ms;

        // BEGIN DEBUGGING
        //        {
        //            std::lock_guard<std::mutex> lock(debug_mutex_);
        //            ms = std::chrono::duration_cast<std::chrono::milliseconds>( (*t_camera)[index] - t_trigger_[index]
        //            ).count();
        //        }

        // VLOG(4) << "Camera " << camera->id() << " trigger copy delta in ms: " << ms;

        //        if ( ms > 15000 )
        //        {
        //            std::cout << ("delta time too big(): " + camera->id() + " index: " + std::to_string(index) + " ms:
        //            " + std::to_string(ms / 1000.0)) << std::endl; run_process_ = false;
        //        }
        // END DEBUGGING

        buffer->BlockingPushBack(std::make_unique<Pair>(system_time, frame));

        index = (index + 1) % test_count_;

        ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - begin_frame_loop)
                 .count();

        VLOG(6) << "Camera " << camera->id() << " frame loop frame in ms: " << ms;
        /**/
    } while (has_grab_result);

    camera->StopGrabbing();

    // If the receiving thread is blocking it cannot get out of it without some sort of notification.
    // A nullptr is used.
    buffer->PushBack(PairPtr());

    VLOG(4) << "StereoTriggering::RunGrabber(): Camera " << camera->id() << " stopped.";
}

void StereoTriggering::RunProcessor() {
    while (run_process_) {
        PairPtr grabbed_left;
        buffer_left_.BlockingPopFront(&grabbed_left);
        // Camera thread stop check and signal break to writer
        if (!grabbed_left) {
            break;
        }

        PairPtr grabbed_right;
        buffer_right_.BlockingPopFront(&grabbed_right);
        // Camera thread stop check and signal break to writer
        if (!grabbed_right) {
            break;
        }

        // TODO(jbr): Adjust FPS based on processing time and/or camera buffer size to prevent buffer overflow.
        std::chrono::time_point<std::chrono::system_clock> begin_processing = std::chrono::system_clock::now();

        processor_(std::move(grabbed_left), std::move(grabbed_right));

        VLOG(6) << "Processed pair in ms: "
                << std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::system_clock::now() - begin_processing)
                       .count();
    }

    VLOG(4) << "StereoTriggering::RunProcessor(): Stopped.";
}

}  // namespace nie
