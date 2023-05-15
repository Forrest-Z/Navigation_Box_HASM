// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// The changes made in this file, of which a summary is listed below, are copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
// Information classification: Confidential
// This content is protected by international copyright laws.
// Reproduction and distribution is prohibited without written permission.
//
// List of changes:
// * Changed namespace from autoware to aiim
// * Adding aiim prefix to includes

#include <gtest/gtest.h>

#include <aiim_autoware_common/types.hpp>
#include <aiim_helper_functions/mahalanobis_distance.hpp>

TEST(MahalanobisDistanceTest, basic_test) {
    Eigen::Matrix<aiim::common::types::float32_t, 2, 1> mean;
    mean << 2.F, 2.F;
    Eigen::Matrix<aiim::common::types::float32_t, 2, 1> sample;
    sample << 2.F, 3.F;
    Eigen::Matrix<aiim::common::types::float32_t, 2, 2> cov;
    cov << 0.1F, 0.0F, 0.0F, 0.6F;

    // the two states are independent and one has more variance than the other. With samples
    // equidistant from mean but on two different axes will have vastly different
    // mahalanobis distance values
    EXPECT_FLOAT_EQ(aiim::common::helper_functions::calculate_mahalanobis_distance(sample, mean, cov), 1.666666666F);

    sample << 3.F, 2.F;
    EXPECT_FLOAT_EQ(aiim::common::helper_functions::calculate_mahalanobis_distance(sample, mean, cov), 10.0F);
}
