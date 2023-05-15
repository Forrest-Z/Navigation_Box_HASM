/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "resample.hpp"

#include <nie/core/gflags.hpp>

#include "../common.hpp"
#include "pose_resampler.hpp"

DEFINE_double(sample_interval, 0.0, "Sampling interval between poses in meters.");
DEFINE_validator(sample_interval, nie::ValidateLargerOrEqualToZero);

void Resample() {
    nie::io::PoseCollection pose_collection;
    CHECK(ReadData(&pose_collection));

    double const max_delta = GetMaximumDistanceThreshold(pose_collection);

    WriteData(ResampleDistance(pose_collection, FLAGS_sample_interval, max_delta));
}
