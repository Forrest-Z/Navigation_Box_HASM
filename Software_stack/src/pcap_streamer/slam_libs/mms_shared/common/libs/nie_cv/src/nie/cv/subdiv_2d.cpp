/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "subdiv_2d.hpp"

#include <glog/logging.h>

namespace nie {

void Subdiv2D::getConnectionList(std::vector<std::pair<int, int>>* p_connections) const {
    /*
     * Functionality is essentially copied from getEdgeList except that vertex
     * id's are returned, not the vertex locations.
     */

    CHECK_NOTNULL(p_connections);
    auto& connections = *p_connections;

    connections.clear();
    connections.reserve(qedges.size());

    for (size_t i = 4; i < qedges.size(); i++) {
        if (qedges[i].isfree()) {
            continue;
        }

        if (qedges[i].pt[0] > 0 and qedges[i].pt[2] > 0) {
            connections.emplace_back(qedges[i].pt[0], qedges[i].pt[2]);
        }
    }

    connections.shrink_to_fit();
}

}  // namespace nie
