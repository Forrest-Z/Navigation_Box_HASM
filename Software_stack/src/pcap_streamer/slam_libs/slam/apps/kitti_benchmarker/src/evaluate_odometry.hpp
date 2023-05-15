/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef KITTI_BENCHMARKER_EVALUATE_ODOMETRY_HPP
#define KITTI_BENCHMARKER_EVALUATE_ODOMETRY_HPP

#include <string>

bool EvaluateFile(
    std::string const& ref_file,
    std::string const& tst_file,
    bool const correct_scale,
    std::string const& result_dir,
    std::string file_name);

#endif  // KITTI_BENCHMARKER_EVALUATE_ODOMETRY_HPP
