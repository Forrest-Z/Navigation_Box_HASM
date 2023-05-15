/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/cv/vo/visual_odometry.hpp>

TEST(ComputeRelativeFactor, Test) {
    double const kFactor = 3.;

    // Tetrahedron has 4 points, all at equal distances from each other, so the sampling done in ComputeRelativeFactor
    // will not have any influence
    std::vector<Eigen::Vector3d> objects_a = {{0., 0., 0.}, {0., 1., 1.}, {1., 0., 1.}, {1., 1., 0.}};
    std::vector<Eigen::Vector3d> objects_b{objects_a};
    std::for_each(objects_b.begin(), objects_b.end(), [&kFactor](auto& p) mutable { p *= kFactor; });

    nie::MatchVector matches;
    matches.emplace_back(0, 0);
    matches.emplace_back(1, 1);
    matches.emplace_back(2, 2);
    matches.emplace_back(3, 3);

    CHECK_NEAR(nie::ComputeRelativeFactor(matches, matches, objects_a, objects_b), 1. / kFactor, 1.e-15);
}
