/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/formats/ba_graph.hpp>

nie::io::PoseCollection Transform(
        nie::io::PoseCollection const& pose_gps,
        nie::io::PoseCollection const& pose_odom,
        nie::io::PoseCollection const& pose_bbox_odom);
