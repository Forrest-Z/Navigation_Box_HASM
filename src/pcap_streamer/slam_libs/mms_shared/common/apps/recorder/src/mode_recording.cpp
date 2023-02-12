/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "mode_recording.hpp"

#include <atomic>
#include <chrono>
#include <thread>

#include <glog/logging.h>
#include <nie/core/gflags.hpp>
#include <nie/core/work_pool.hpp>
#include <nie/hardware/camera/basler_camera.hpp>
#include <nie/hardware/device/stereo_triggering.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "checkerboard_overlap_filter.hpp"

void WriteImage(std::unique_ptr<std::pair<std::string, cv::Mat>> const& work) {
    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

    cv::imwrite(work->first, work->second);

    std::chrono::time_point<std::chrono::system_clock> after = std::chrono::system_clock::now();

    DVLOG(3) << "Written image in ms: "
             << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
}

cv::Size GetPreviewSizeFromTargetWidth(nie::BaslerCamera const& camera, int preview_target_width) {
    cv::Size preview_size = camera.image_size();
    preview_size.height = static_cast<int>(std::ceil(
            static_cast<double>(preview_size.height) * static_cast<double>(preview_target_width) /
            static_cast<double>(preview_size.width)));
    preview_size.width = preview_target_width;

    return preview_size;
}

std::size_t RunModeRecording(
        std::string const& out_path_images, nie::BaslerCamera* camera, std::unique_ptr<nie::BaseFilter> const& filter) {
    cv::Size preview_size = GetPreviewSizeFromTargetWidth(*camera, nie::detail::kPreviewTargetWidth);
    cv::Mat preview;
    bool recording = false;
    std::size_t recorded = 0;

    {
        std::uint32_t processor_count =
                static_cast<std::uint32_t>(std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1));

        VLOG(2) << "Processors used for writing images: " << processor_count;

        nie::WorkPool<std::pair<std::string, cv::Mat>> pool_image_writer(
                processor_count, processor_count * 2, &WriteImage);

        std::cout << "Press 'Esc' to quit." << std::endl;
        std::cout << "Press 'Enter' to start or stop recording." << std::endl;
        std::cout << std::endl;

        camera->StartGrabbing(nie::GrabStrategy::kLatestImageOnly);

        bool run_process = true;
        std::uint32_t timeout_ms = 5000;

        while (run_process) {
            cv::Mat frame;
            std::int64_t system_time;

            camera->GetFrame(timeout_ms, &frame, &system_time, true);

            cv::resize(frame, preview, preview_size, cv::INTER_AREA);

            if (recording) {
                // So people can see they are recording
                cv::circle(preview, cv::Point(preview_size.width - 15, 15), 3, cv::Scalar(0, 0, 255), 5);
                if (filter->Filter(frame).second) {
                    std::string filename = out_path_images + std::to_string(recorded) + ".png";
                    pool_image_writer.queue().BlockingPushBack(
                            std::make_unique<std::pair<std::string, cv::Mat>>(filename, frame));
                    ++recorded;
                }
            }

            cv::imshow("Recorded Image", preview);

            // 10 FPS is fine, probably still giving too many images :D
            int key = cv::waitKey(nie::detail::kPreviewCaptureSpeedMs);

            switch (key) {
                case 13:  // Enter
                {
                    recording = !recording;
                } break;
                case 27:  // Esc
                {
                    run_process = false;
                } break;
                default:
                    break;
            }
        }
    }

    std::cout << "Pressed 'Esc'. Quiting ..." << std::endl;

    return recorded;
}

std::size_t RunModeRecording(
        StereoFolderStructure const& out_folder_struct,
        cv::Size const& pattern_size,
        int const gpio_port,
        nie::BaslerCamera* camera_left,
        nie::BaslerCamera* camera_right) {
    CHECK_EQ(camera_left->image_size(), camera_right->image_size());

    cv::Size preview_size = GetPreviewSizeFromTargetWidth(*camera_left, nie::detail::kPreviewTargetWidth);
    cv::Mat preview(preview_size.height, preview_size.width * 2, CV_8UC3);
    std::atomic_bool recording(false);
    std::size_t recorded = 0;
    nie::CheckerboardOverlapFilter filter_left(pattern_size, nie::detail::kOnlineOverlapThreshold);
    nie::CheckerboardOverlapFilter filter_right(pattern_size, nie::detail::kOnlineOverlapThreshold);

    std::uint32_t processor_count =
            static_cast<std::uint32_t>(std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 4));

    VLOG(2) << "Processors used for writing images: " << processor_count;

    nie::WorkPool<std::pair<std::string, cv::Mat>> pool_image_writer(processor_count, processor_count * 2, &WriteImage);

    std::function<void(nie::StereoTriggering::PairPtr left, nie::StereoTriggering::PairPtr right)> stereo_processor =
            [&preview_size,
             &preview,
             &pool_image_writer,
             &recording,
             &out_folder_struct,
             &recorded,
             &filter_left,
             &filter_right](nie::StereoTriggering::PairPtr left, nie::StereoTriggering::PairPtr right) -> void {
        cv::Mat small_left = preview(cv::Rect(0, 0, preview_size.width, preview_size.height));
        cv::Mat small_right = preview(cv::Rect(preview_size.width, 0, preview_size.width, preview_size.height));
        cv::resize(left->second, small_left, preview_size);
        cv::resize(right->second, small_right, preview_size);

        if (recording) {
            std::pair<bool, bool> checkerboard_x_keep = filter_left.Filter(left->second);

            if (checkerboard_x_keep.second) {
                std::string filename_left = out_folder_struct.root_folder + out_folder_struct.path_relative_left +
                                            std::to_string(recorded) + ".png";
                pool_image_writer.queue().BlockingPushBack(
                        std::make_unique<std::pair<std::string, cv::Mat>>(filename_left, left->second));
                std::string filename_right = out_folder_struct.root_folder + out_folder_struct.path_relative_right +
                                             std::to_string(recorded) + ".png";
                pool_image_writer.queue().BlockingPushBack(
                        std::make_unique<std::pair<std::string, cv::Mat>>(filename_right, right->second));
                recorded++;
                return;
            }

            if (!checkerboard_x_keep.first && filter_right.Filter(right->second).second) {
                std::string filename_left = out_folder_struct.root_folder + out_folder_struct.path_relative_left +
                                            std::to_string(recorded) + ".png";
                pool_image_writer.queue().BlockingPushBack(
                        std::make_unique<std::pair<std::string, cv::Mat>>(filename_left, left->second));
                std::string filename_right = out_folder_struct.root_folder + out_folder_struct.path_relative_right +
                                             std::to_string(recorded) + ".png";
                pool_image_writer.queue().BlockingPushBack(
                        std::make_unique<std::pair<std::string, cv::Mat>>(filename_right, right->second));
                recorded++;
                return;
            }
        }
    };

    nie::StereoTriggering triggering(
            gpio_port, nie::detail::kPreviewCaptureSpeedMs, stereo_processor, camera_left, camera_right);

    std::cout << "Press 'Esc' to quit." << std::endl;
    std::cout << "Press 'Enter' to start or stop recording." << std::endl;
    std::cout << std::endl;

    bool run_process = true;

    while (run_process) {
        // The output of left and right is not synced with respect to the window, there is probably some minor
        // delta between the two. If the CPU load for the thread is too high frames probably jump too.
        cv::imshow("Stereo Pair", preview);

        if (recording) {
            // So people can see they are recording
            cv::circle(preview, cv::Point(preview_size.width * 2 - 15, 15), 3, cv::Scalar(0, 0, 255), 5);
        }

        int key = cv::waitKey(10);

        switch (key) {
            case 13:  // Enter
            {
                recording = !recording;
            } break;
            case 27:  // Esc
            {
                run_process = false;
            } break;
            default:
                break;
        }
    }

    std::cout << "Pressed 'Esc'. Quiting ..." << std::endl;

    return recorded;
}