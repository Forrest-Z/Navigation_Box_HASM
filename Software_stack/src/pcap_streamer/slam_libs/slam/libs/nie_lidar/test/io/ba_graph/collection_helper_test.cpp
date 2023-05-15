/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/geometry/rotation.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>

// Unit test by conforming to both implementations.
TEST(CollectionHelperTest, InformationMatrixFromIsometry) {
    double const angle = nie::kDeg2Rad<double> * 10.0;
    Eigen::Vector3d t{2.0, 1.0, 3.0};
    Eigen::Quaterniond r{Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())};
    nie::Isometry3qd ei(t, r);

    double const w = 0.5;
    double const delta_percentage = 0.05;

    Eigen::Matrix<double, 6, 6> informationf = nie::io::InformationMatrixFromIsometry(ei, w, delta_percentage);

    // The duplication of implementation.
    double lt = ei.translation().norm();
    double lr = Eigen::AngleAxisd{ei.rotation()}.angle();
    lt *= delta_percentage;
    lr *= delta_percentage;

    double wt = w / (lt * lt);
    double wr = w / (lr * lr);

    Eigen::Matrix<double, 6, 6> informationt = Eigen::Matrix<double, 6, 6>::Zero();
    informationt.diagonal().head<3>().setConstant(wt);
    informationt.diagonal().tail<3>().setConstant(wr);

    EXPECT_TRUE(informationf.isApprox(informationt));
}