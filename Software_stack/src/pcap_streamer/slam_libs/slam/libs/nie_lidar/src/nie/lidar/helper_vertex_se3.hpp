/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/formats/g2o_types.hpp>

namespace nie {

io::MapOfPoses CalcVertexDeltas(io::MapOfPoses const& vertices_a, io::MapOfPoses const& vertices_b);

}  // namespace nie
