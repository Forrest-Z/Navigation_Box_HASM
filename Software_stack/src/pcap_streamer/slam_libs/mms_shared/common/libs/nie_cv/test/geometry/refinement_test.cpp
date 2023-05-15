/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/cv/geometry/refinement.hpp>

TEST(RefinementTest, Centered) {
    Eigen::Matrix3f samples;
    samples.setConstant(1.0f);
    samples(1, 1) = 2.0;
    Eigen::Vector2f x{0, 0};

    EXPECT_TRUE(nie::RefineQuadratic2D(samples, &x));
    EXPECT_NEAR(0.0f, x(0), std::numeric_limits<float>::epsilon());
    EXPECT_NEAR(0.0f, x(1), std::numeric_limits<float>::epsilon());
}

TEST(RefinementTest, ToX) {
    Eigen::Matrix3f samples;
    samples << 0.8, 1.0, 1.2, 0.8, 2.0, 1.2, 0.8, 1.0, 1.2;
    Eigen::Vector2f x{0, 0};

    EXPECT_TRUE(nie::RefineQuadratic2D(samples, &x));
    EXPECT_TRUE(x(0) > 0.0f);
    EXPECT_TRUE(x(0) < 1.0f);
    EXPECT_NEAR(0.0f, x(1), std::numeric_limits<float>::epsilon());
}

TEST(RefinementTest, ToY) {
    Eigen::Matrix3f samples;
    samples << 0.8, 0.8, 0.8, 1.0, 2.0, 1.0, 1.2, 1.2, 1.2;
    Eigen::Vector2f x{0, 0};

    EXPECT_TRUE(nie::RefineQuadratic2D(samples, &x));
    EXPECT_TRUE(x(1) > 0.0f);
    EXPECT_TRUE(x(1) < 1.0f);
    EXPECT_NEAR(0.0f, x(0), std::numeric_limits<float>::epsilon());
}