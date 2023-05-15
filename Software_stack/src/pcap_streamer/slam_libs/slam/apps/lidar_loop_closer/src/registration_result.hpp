/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Eigen>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/lidar/geometry/pose_bbox.hpp>

class RegistrationResult {
public:
    std::pair<nie::io::PoseId, nie::io::PoseId> ids;
    nie::Isometry3qd T;
};
