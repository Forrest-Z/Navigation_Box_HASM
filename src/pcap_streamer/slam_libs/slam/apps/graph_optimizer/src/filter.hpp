/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/formats/ba_graph/pose_collection.hpp>

void Filter(
        double const& mahalanobis_threshold,
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::PoseEdgeRecord>* p_edges);

double DetermineMahalanobisDistance(
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::PoseEdgeRecord> const& edges,
        double const& percentage);

void WriteMahalanobisDistances(
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::PoseEdgeRecord> const& edges,
        std::string const& output_filename);
