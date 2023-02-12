/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "detector_tests_base.hpp"

#include <gtest/gtest.h>

#include <nie/cv/vo/feature/detector_lv_blob.hpp>

/*
 * It is assumed that the files with the same name, will also output the same
 * result. For now only the feature detection count is considered.
 */
std::map<std::string, int> const kExpectationBlobs = {{"20190214_140759163_09475", 3877},
                                                      {"20190214_140759163_09485", 3873}};

TEST_F(VoFeatureDetectorTest, LvBlob) {
    nie::DetectorLvBlob::Parameters params;
    params.feature_distance = 15;
    nie::DetectorPtr detector = std::make_unique<nie::DetectorLvBlob>(params);
    ProcessImages(detector, kExpectationBlobs);
}
