/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/core/string.cpp>

#include "checkerboard_overlap_filter.hpp"
#include "mode_recording.hpp"

DEFINE_string(out_path_images, "", "The directory (including trailing /) where camera images are written to.");

DEFINE_string(out_path_flags, "", "The name of the file to write the command-line arguments to.");

DEFINE_string(path_relative_left, "l/", "The sub-directory where left camera images are written to.");

DEFINE_string(path_relative_right, "r/", "The sub-directory where right camera images are written to.");

DEFINE_bool(
        stereo,
        false,
        "Use a stereo setup. Directories such as <path_images> are appended with l/ and r/ for the left and right "
        "camera "
        "respectively.");

DEFINE_string(
        checkerboard_size,
        "",
        "The amount of squares in two dimensions as a comma separated list. E.g., 12,8."
        "If no argument is passed, it means that there is no checkerboard so it will record without any filtering");

DEFINE_string(camera_ids, "", "The identifiers of the cameras in order.");

DEFINE_int32(gpio, -1, "GPIO for triggering the cameras in case of stereo setup.");

DEFINE_validator(camera_ids, nie::ValidateStringNotEmpty);

DEFINE_validator(out_path_images, nie::ValidateStringNotEmpty);

StereoFolderStructure CreateOutputStructure() {
    return {FLAGS_out_path_images, FLAGS_path_relative_left, FLAGS_path_relative_right};
}

void CreateOutputDirectories() {
    if (!boost::filesystem::is_directory(FLAGS_out_path_images)) {
        if (!FLAGS_stereo) {
            boost::filesystem::create_directories(FLAGS_out_path_images);
        } else {
            boost::filesystem::create_directories(FLAGS_out_path_images + FLAGS_path_relative_left);
            boost::filesystem::create_directories(FLAGS_out_path_images + FLAGS_path_relative_right);
        }
    }
}

cv::Size GetPatternSize() {
    std::vector<int> checkerboard_vector = nie::Split<int>(FLAGS_checkerboard_size);
    return cv::Size(checkerboard_vector[0] - 1, checkerboard_vector[1] - 1);
}

void Record() {
    VLOG(6) << "Recording mode..." << std::endl;

    // FIXME(jbr): Currently Basler specific
    nie::BaslerParameters p;
    p.auto_exposure_roi = cv::Rect(0, 400, 1920, 400);
    p.auto_exposure_brightness = 0.25;

    if (FLAGS_stereo) {
        p.trigger_mode = Basler_UsbCameraParams::TriggerMode_On;
        p.trigger_source = Basler_UsbCameraParams::TriggerSource_Line4;
    }

    CreateOutputDirectories();

    std::size_t image_count;
    if (!FLAGS_stereo) {
        nie::BaslerCamera camera(p);
        std::unique_ptr<nie::BaseFilter> filter;
        if (!FLAGS_checkerboard_size.empty()) {
            CHECK(nie::Split<int>(FLAGS_checkerboard_size).size() == 2)
                    << "<checkerboard_size> is invalid: " << FLAGS_checkerboard_size << std::endl;

            filter = nie::CheckerboardOverlapFilter::Create(GetPatternSize(), nie::detail::kOnlineOverlapThreshold);
        } else {
            filter = nie::NoOpFilter::Create();
        }
        image_count = RunModeRecording(FLAGS_out_path_images, &camera, filter);

    } else {
        CHECK(nie::Split<int>(FLAGS_checkerboard_size).size() == 2)
                << "<checkerboard_size> is invalid: " << FLAGS_checkerboard_size << std::endl;

        std::vector<std::string> ids = nie::Split<std::string>(FLAGS_camera_ids);
        CHECK(ids.size() == 2) << "Missing identifier for a camera.";
        CHECK(FLAGS_gpio > 0) << "GPIO flag not set for stereo triggering.";

        nie::BaslerCamera camera_left(ids[0], p);
        nie::BaslerCamera camera_right(ids[1], p);

        image_count =
                RunModeRecording(CreateOutputStructure(), GetPatternSize(), FLAGS_gpio, &camera_left, &camera_right);
    }

    VLOG(6) << "Recorded image count: " << image_count << std::endl;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    if (!FLAGS_out_path_flags.empty() && !nie::WriteFlagsFile(FLAGS_out_path_flags)) return 0;

    Record();

    return 0;
}