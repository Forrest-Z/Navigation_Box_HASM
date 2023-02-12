/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include "io.hpp"

void AddGroundTruthStatistics(
        std::vector<PoseSet> const& pose_sets,
        ClosureGroundTruthMap const& ground_truth,
        double translation_threshold,
        double rotation_threshold,
        Statistics* stats);
