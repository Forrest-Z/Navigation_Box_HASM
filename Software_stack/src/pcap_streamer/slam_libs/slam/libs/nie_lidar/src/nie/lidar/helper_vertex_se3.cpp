/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <glog/logging.h>

#include "helper_vertex_se3.hpp"

namespace nie {

io::MapOfPoses CalcVertexDeltas(io::MapOfPoses const& vertices_a, io::MapOfPoses const& vertices_b) {
    io::MapOfPoses deltas;

    for (auto const& vertices_a_it : vertices_a) {
        int const& id = vertices_a_it.first;
        auto const& vertex_a = vertices_a_it.second;
        auto const& vertices_b_it = vertices_b.find(id);
        // In theory never happens and this check is non-sense
        CHECK(vertices_b_it != vertices_b.end()) << "In- and output do not share the same id.";
        auto const& vertex_b = vertices_b_it->second;

        deltas.insert({id, {id, vertex_b.v.Delta(vertex_a.v)}});
    }

    return deltas;
}

}  // namespace nie
