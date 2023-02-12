/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/formats/ba_graph/pose_collection.hpp>

#include "tool/io.hpp"

void AddFixture() {
    nie::io::PoseCollection pose_collection;
    CHECK(ReadData(&pose_collection));

    pose_collection.fixes.push_back({pose_collection.poses.front().id});

    WriteData(pose_collection);
}
