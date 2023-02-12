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

#include "mode_calibration.hpp"
#include "mode_optimization.hpp"
#include "mode_rectification.hpp"
#include "mode_validation.hpp"

DEFINE_string(in_path_images, "", "The directory (including trailing /) where camera images are read from.");
DEFINE_string(out_path_images, "", "The directory (including trailing /) where camera images are written to.");

DEFINE_string(path_relative_left, "l/", "The sub-directory where left camera images are read/written.");
DEFINE_string(path_relative_right, "r/", "The sub-directory where right camera images are read/written.");

DEFINE_string(in_path_intrinsics, "", "The camera intrinsics .json file that is read from.");
DEFINE_string(out_path_intrinsics, "", "The camera intrinsics .json file that is written to.");

DEFINE_string(
    in_path_extended_data,
    "",
    "The .json file where the extended calibration data (extrinsics, residuals, etc.) is read from.");
DEFINE_string(
    out_path_extended_data,
    "",
    "The .json file where the extended calibration data (extrinsics, residuals, etc.) is written to.");

DEFINE_string(out_path_validated_data, "", "The .json file where the validated calibration data is written to.");

DEFINE_string(out_path_flags, "", "The name of the file to write the command-line arguments to.");

DEFINE_string(image_size, "", "The pixel size of the input images as a comma separated list. E.g. 1920,1200");

DEFINE_bool(
    calibrate,
    false,
    "Calibrate a camera using images read from <path_images>. The intrinsics will be stored in <path_intrinsics>.");

DEFINE_bool(
    optimize,
    false,
    "Using the extended data from calibration, optimize the spatial coverage of the dataset and calibrate again.");

DEFINE_bool(rectify, false, "Rectify images stored in <path_images> using the intrinsics read from <path_intrinsics>.");

DEFINE_bool(
    validate,
    false,
    "Validate rectified images stored in <path_images> using the intrinsics read from <path_intrinsics>.");

DEFINE_bool(
    stereo,
    false,
    "Use a stereo setup. Directories such as <path_images> are appended with l/ and r/ for the left and right camera "
    "respectively.");

DEFINE_bool(write_corner_images, false, "Debug option. If true, outputs images with the detected corners drawn.");

DEFINE_string(checkerboard_size, "", "The amount of squares in two dimensions as a comma separated list. E.g., 12,8");
DEFINE_double(checkerboard_square_size, 1.0, "The physical size of a square in meters.");

DEFINE_string(camera_ids, "", "The identifiers of the cameras in order.");

bool CheckFlagCheckerboardSize() {
    if (FLAGS_checkerboard_size.empty()) {
        std::cout << "<checkerboard_size> is not set." << std::endl;
        return false;
    }

    if (nie::Split<int>(FLAGS_checkerboard_size).size() != 2) {
        std::cout << "<checkerboard_size> is invalid: " << FLAGS_checkerboard_size << std::endl;
        return false;
    }

    return true;
}

bool CheckFlagInputImages() {
    bool success = boost::filesystem::is_directory(FLAGS_in_path_images);

    if (!success) {
        std::cout << "<in_path_images> does not exist: " << FLAGS_in_path_images << std::endl;
    }

    return success;
}

bool CheckFlagOutputImages() {
    if (FLAGS_out_path_images.empty()) {
        std::cout << "<out_path_images> is not set." << std::endl;
        return false;
    }

    return true;
}

bool CheckFlagsInputImagesAndIntrinsics() {
    if (!CheckFlagInputImages()) return false;

    if (FLAGS_in_path_intrinsics.empty()) {
        std::cout << "<in_path_intrinsics> is not set." << std::endl;
        return false;
    }

    return true;
}

void TryCreateOutputDirectories() {
    if (!boost::filesystem::is_directory(FLAGS_out_path_images)) {
        if (!FLAGS_stereo) {
            boost::filesystem::create_directories(FLAGS_out_path_images);
        } else {
            boost::filesystem::create_directories(FLAGS_out_path_images + FLAGS_path_relative_left);
            boost::filesystem::create_directories(FLAGS_out_path_images + FLAGS_path_relative_right);
        }
    }
}

bool CheckFlagOutputIntrinsics() {
    if (FLAGS_out_path_intrinsics.empty()) {
        std::cout << "<out_path_intrinsics> is not set." << std::endl;
        return false;
    }

    return true;
}

bool CheckFlagOutputImagesAndCreateDirectories() {
    if (!CheckFlagOutputImages()) return false;

    TryCreateOutputDirectories();

    return true;
}

bool CheckFlagsOutputImagesIntrinsicsAndCreateDirectories() {
    return CheckFlagOutputImagesAndCreateDirectories() && CheckFlagOutputIntrinsics();
}

cv::Size GetPatternSize() {
    std::vector<int> checkerboard_vector = nie::Split<int>(FLAGS_checkerboard_size);
    return cv::Size(checkerboard_vector[0] - 1, checkerboard_vector[1] - 1);
}

void Calibrate() {
    std::cout << "Calibration mode..." << std::endl;

    if (!CheckFlagInputImages() || !CheckFlagOutputIntrinsics()) return;

    if (FLAGS_image_size.empty()) {
        std::cout << "<image_size> is not set (needed for calibration)." << std::endl;
        return;
    }

    std::vector<int> image_size = nie::Split<int>(FLAGS_image_size);

    if (image_size.size() != 2) {
        std::cout << "'" << FLAGS_image_size << "' is not a valid format for <image_size>. Expected width,height"
                  << std::endl;
        return;
    }

    if (FLAGS_out_path_extended_data.empty()) {
        std::cout << "<out_path_extended_data> is not set." << std::endl;
        return;
    }

    if (!CheckFlagCheckerboardSize()) {
        return;
    }

    std::vector<std::string> ids = nie::Split<std::string>(FLAGS_camera_ids);

    if (!FLAGS_stereo) {
        CHECK(ids.size() == 1) << "Missing identifier for a camera.";
        RunModeCalibration(
            ids[0],
            FLAGS_in_path_images,
            FLAGS_out_path_intrinsics,
            FLAGS_out_path_extended_data,
            cv::Size(image_size[0], image_size[1]),
            GetPatternSize(),
            FLAGS_checkerboard_square_size,
            FLAGS_write_corner_images);
    } else {
        CHECK(ids.size() == 2) << "Missing identifier for a camera.";
        RunModeCalibration(
            ids[0],
            ids[1],
            FLAGS_in_path_images,
            FLAGS_path_relative_left,
            FLAGS_path_relative_right,
            FLAGS_out_path_intrinsics,
            FLAGS_out_path_extended_data,
            cv::Size(image_size[0], image_size[1]),
            GetPatternSize(),
            FLAGS_checkerboard_square_size,
            FLAGS_write_corner_images);
    }
}

void Rectify() {
    std::cout << "Rectification mode..." << std::endl;

    if (!CheckFlagsInputImagesAndIntrinsics()) return;

    if (!CheckFlagsOutputImagesIntrinsicsAndCreateDirectories()) return;

    RunModeRectification(
        FLAGS_in_path_images,
        FLAGS_path_relative_left,
        FLAGS_path_relative_right,
        FLAGS_in_path_intrinsics,
        FLAGS_out_path_images,
        FLAGS_out_path_intrinsics,
        FLAGS_stereo);
}

void Validate() {
    std::cout << "Validation mode..." << std::endl;

    if (!CheckFlagsInputImagesAndIntrinsics()) return;

    if (FLAGS_in_path_extended_data.empty()) {
        std::cout << "<in_path_extended_data> is not set." << std::endl;
        return;
    }

    if (FLAGS_out_path_validated_data.empty()) {
        std::cout << "<out_path_validated_data> is not set." << std::endl;
        return;
    }

    RunModeValidation(
        FLAGS_in_path_images,
        FLAGS_path_relative_left,
        FLAGS_path_relative_right,
        FLAGS_in_path_intrinsics,
        FLAGS_in_path_extended_data,
        FLAGS_out_path_validated_data,
        FLAGS_write_corner_images,
        FLAGS_stereo);
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    if (!FLAGS_out_path_flags.empty() && !nie::WriteFlagsFile(FLAGS_out_path_flags)) return 0;

    if (FLAGS_calibrate) {
        Calibrate();
    } else if (FLAGS_optimize) {
        if (FLAGS_in_path_extended_data.empty()) {
            std::cout << "<in_path_extended_data> is not set." << std::endl;
            return 0;
        }

        RunModeOptimization(FLAGS_in_path_extended_data, FLAGS_stereo);
    } else if (FLAGS_rectify) {
        Rectify();
    } else if (FLAGS_validate) {
        Validate();
    } else {
        std::cout << "Use with a mode such as -calibrate ..." << std::endl;
    }

    return 0;
}