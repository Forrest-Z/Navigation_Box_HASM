/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/gflags.hpp>

#include "evaluate_odometry.hpp"

DEFINE_string(in_file_reference, "", "The first/reference KITTI pose file to be used in the comparison.");
DEFINE_string(in_file_test, "", "The second/test KITTI pose file to be used in the comparison.");
DEFINE_bool(
        correct_scale,
        false,
        "When an automatic scaling should be applied to the test poses in order to let the start and end of both"
        "trajectories coincide.");
DEFINE_string(out_dir_result, "", "The output directory where the comparison results are generated.");
DEFINE_string(
        out_file_name,
        "",
        "The output files will all get this name. [optional] When omitted or left empty then the reference and test "
        "file names will be used like '<reference>-<test>'. Note that result will be overwritten when already "
        "present.");

DEFINE_validator(in_file_reference, &nie::ValidateIsFile);
DEFINE_validator(in_file_test, &nie::ValidateIsFile);
DEFINE_validator(out_dir_result, &nie::ValidateIsDirectory);

/// The application will take one KITTI pose file as reference and a KITTI pose file or binary pose file as test, to
/// generate the error statistics and plots.
///
/// In case of a KITTI pose file as test file, then the number of poses is expected to be equal to the reference one. In
/// case of a binary pose file as test file (recognized by extension .pose), then the pose id will be used as index
/// referring to the corresponding pose in the reference one. This might be handy in case of keyframing in which not all
/// image poses will be supplied.
///
/// This app requires the following tools: gnuplot, ps2pdf, pdfcrop

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    // Check if required tolls are installed
    for (std::string const& tool : {"gnuplot", "ps2pdf", "pdfcrop"}) {
        std::string const command = "command -v " + tool + " >/dev/null 2>&1";
        CHECK(system(command.c_str()) == 0) << "Tool '" << tool << "' should be installed to use this app.";
    }

    // Run evaluation
    LOG(INFO) << "Comparing '" << FLAGS_in_file_reference << "' with '" << FLAGS_in_file_test << "'." << std::endl;
    EvaluateFile(
            FLAGS_in_file_reference,
            FLAGS_in_file_test,
            FLAGS_correct_scale,
            FLAGS_out_dir_result,
            FLAGS_out_file_name);

    return 0;
}
