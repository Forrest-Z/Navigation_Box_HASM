/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
// Option to show all images
// NOTE: This is currently a preprocessor flag because the ladybug library is not supported on Ubuntu 18.04
//#define SHOW_IMAGES

// Standard includes
#include <string>

// Google includes
#include <gflags/gflags.h>

// OpenCV includes
#include <opencv2/opencv.hpp>

// NIE includes
#include <nie/core/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ladybug.hpp>

// Local includes
#include "trigger_timestamp_reader.hpp"

// Define command line parameters
DEFINE_string(in_file_ladybug_pgr, "", "File that contains Ladybug stream recording, in pgr format.");
DEFINE_string(in_file_camera_sta, "", "File that contains timestamps of camera triggers, in sta format.");
DEFINE_bool(show_timestamps, false, "Show timestamps for all images");

// Validate command line parameters
DEFINE_validator(in_file_ladybug_pgr, nie::ValidateIsFile);
DEFINE_validator(in_file_camera_sta, nie::ValidateIsFile);

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    // Obtain the timestamps for all images
    boost::filesystem::path sta_file(FLAGS_in_file_camera_sta);
    VLOG(1) << "Opening camera trigger file " << sta_file;
    std::unordered_map<unsigned, nie::Timestamp_us> timestamps = ReadTriggerTimestamps(sta_file);
    VLOG(5) << "Input file contains " << timestamps.size() << " image triggers";

    // Open the camera stream
    boost::filesystem::path pgr_file(FLAGS_in_file_ladybug_pgr);
    VLOG(1) << "Opening Ladybug camera stream " << pgr_file;
    nie::io::ladybug::StreamReader camera_stream(pgr_file);
    VLOG(5) << "Stream contains " << camera_stream.frame_count() << " frames";

    if (timestamps.size() != camera_stream.frame_count()) {
        std::stringstream ss;
        ss << "The number of camera triggers in the .sta file "
           << "(" << timestamps.size() << ")"
           << " is not the same as the amount of images in the camera stream "
           << "(" << camera_stream.frame_count() << "). "
           << "Unable to match correct timestamps to image.";

        throw std::runtime_error(ss.str());
    }

#ifdef SHOW_IMAGES
    // Save the file with calibration parameters from the stream to a temporary file
    boost::filesystem::path cal_file = nie::TemporaryFile(pgr_file.stem().string() + ".cal");
    VLOG(5) << "Saving Ladybug calibration parameters to temporary file" << cal_file;
    camera_stream.SaveCalibrationFile(cal_file);

    // Load the file with calibration parameters and initialize processor
    VLOG(5) << "Creating image processor with calibration parameters from the camera stream";
    nie::io::ladybug::ImageProcessor processor(cal_file);

    // Set field-of-view, and looking direction (Rotation order Z-Y-X)
    processor.SetViewParameters(60.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    // Create a window for display
    cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
#endif

    // Process frames in the range
    for (unsigned frame = 0; frame < camera_stream.frame_count(); frame++) {
        // Show timestamps
        if (FLAGS_show_timestamps) {
            VLOG(1) << "Frame #" << std::setw(5) << std::setfill('0') << frame << " -- "
                    << "Timestamp: " << timestamps.at(frame);
        }
#ifdef SHOW_IMAGES
        // Read an image from the stream
        nie::io::ladybug::Image raw_image = camera_stream.Read();

        // Process image
        cv::Mat image = processor.Process(raw_image, LADYBUG_SPHERICAL, 1000, 1000, LADYBUG_BGR, LADYBUG_HQLINEAR);

        // Show image
        imshow("Display window", image);

        // Check for key presses
        int key = cv::waitKey(1);
        switch (key) {
            case -1:  // No key pressed
                break;

            case 27:  // Escape
            case 'q':
                return EXIT_SUCCESS;

            default:  // Any other key
                std::cout << "Key press with code: " << key << std::endl;
                break;
        }
#endif
    }
}
