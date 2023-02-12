/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "detector_tests_base.hpp"

#include <gtest/gtest.h>

#include <nie/cv/vo/feature/detector_lv_corner.hpp>

/*
 * It is assumed that the files with the same name, will also output the same
 * result. For now only the feature detection count is considered.
 */
std::map<std::string, int> const kExpectationCorners = {{"20190214_140759163_09475", 3927},
                                                        {"20190214_140759163_09485", 3935}};

TEST_F(VoFeatureDetectorTest, LvCorner) {
    nie::DetectorLvCorner::Parameters params;
    params.feature_distance = 15;
    nie::DetectorPtr detector = std::make_unique<nie::DetectorLvCorner>(params);
    ProcessImages(detector, kExpectationCorners);
}
