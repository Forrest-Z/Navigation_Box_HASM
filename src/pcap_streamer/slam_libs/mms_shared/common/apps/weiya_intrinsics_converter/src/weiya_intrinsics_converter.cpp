/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>
#include <nie/formats/weiya/rectified_parameters.hpp>

DEFINE_string(in_file_extrinsics, "", "The input xml file with the weiya camera calibration extrinsics.");
DEFINE_string(in_file_boresight, "", "The input yaml file with the weiya camera set up.");
DEFINE_string(out_file_intrinsics, "", "The output json file with the intrinsics.");

DEFINE_validator(in_file_extrinsics, nie::ValidateIsFile);
DEFINE_validator(in_file_boresight, nie::ValidateIsFile);
DEFINE_validator(out_file_intrinsics, nie::ValidateStringNotEmpty);

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    nie::io::weiya::RectifiedCameraParametersFromWeiya(FLAGS_in_file_extrinsics, FLAGS_in_file_boresight)
        .Write(FLAGS_out_file_intrinsics);

    return 0;
}
