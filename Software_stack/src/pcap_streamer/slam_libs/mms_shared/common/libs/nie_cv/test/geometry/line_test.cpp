/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/cv/geometry/line.hpp>
#include <opencv2/core.hpp>

struct LinePair {
    // In
    Eigen::Vector3d p0, p1, p2, p3;
    // Out
    Eigen::Vector3d x;
    double s0, s1;
};

LinePair kPairZ{{0, 0, 1}, {2, 2, 1}, {2, 0, 0}, {-2, 4, 0}, {1, 1, 0.5}, 0.5, 0.25};
LinePair kPairY{{0, 1, 0}, {2, 1, 2}, {2, 0, 0}, {-2, 0, 4}, {1, 0.5, 1}, 0.5, 0.25};
LinePair kPairP{{0, 1, 0}, {0, 2, 0}, {1, 1, 0}, {1, 4, 0}, {0, 0, 0}, 0, 0};

void LineLineIntersectTestLinePairTrue(LinePair const& pair, bool const with_s) {
    Eigen::Vector3d x;
    double s0 = 98234, s1 = 1231;

    ASSERT_TRUE(nie::LineLineIntersect(
            pair.p0, (pair.p1 - pair.p0).eval(), pair.p2, (pair.p3 - pair.p2).eval(), &x, &s0, &s1));
    ASSERT_NEAR((x - pair.x).norm(), 0.0, 1.0e-012);

    if (with_s) {
        ASSERT_NEAR(pair.s0, s0, 1.0e-012);
        ASSERT_NEAR(pair.s1, s1, 1.0e-012);
    }
}

void LineLineIntersectTestLinePairFalse(LinePair const& pair) {
    Eigen::Vector3d x;
    double s0 = 98234, s1 = 1231;

    ASSERT_FALSE(nie::LineLineIntersect(
            pair.p0, (pair.p1 - pair.p0).eval(), pair.p2, (pair.p3 - pair.p2).eval(), &x, &s0, &s1));
}

TEST(LineTest, LineLineIntersect) {
    LineLineIntersectTestLinePairTrue(kPairY, true);
    LineLineIntersectTestLinePairTrue(kPairZ, true);
    LineLineIntersectTestLinePairFalse(kPairP);
}

void LinesIntersectTestLinePairTrue(LinePair const& pair) {
    std::vector<Eigen::Vector3d> origins{pair.p0, pair.p2};
    std::vector<Eigen::Vector3d> vectors{pair.p1 - pair.p0, pair.p3 - pair.p2};
    Eigen::Vector3d x;
    ASSERT_TRUE(nie::LinesIntersect(origins, vectors, &x));
    ASSERT_NEAR((x - pair.x).norm(), 0.0, 1.0e-012);
}

void LinesIntersectTestLinePairFalse(LinePair const& pair) {
    std::vector<Eigen::Vector3d> origins{pair.p0, pair.p2};
    std::vector<Eigen::Vector3d> vectors{pair.p1 - pair.p0, pair.p3 - pair.p2};
    Eigen::Vector3d x;
    ASSERT_FALSE(nie::LinesIntersect(origins, vectors, &x));
}

TEST(LineTest, LinesIntersect) {
    LinesIntersectTestLinePairTrue(kPairY);
    LinesIntersectTestLinePairTrue(kPairZ);
    LinesIntersectTestLinePairFalse(kPairP);
}