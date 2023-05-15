/* Copyright (C) {} by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <vector>

#include <nie/formats/ba_graph/pose_collection.hpp>

namespace nie {

class PoseRecordKdTreeAdapter {
public:
    using Scalar = double;

    explicit PoseRecordKdTreeAdapter(std::vector<io::PoseRecord> const& poses) : poses_{poses} {}

    inline std::size_t kdtree_get_point_count() const { return poses_.size(); }

    // Returns the dim'th component of the idx'th point in the class.
    inline double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
        return poses_[idx].isometry.translation().head<3>()[dim];
    }

    inline Scalar const* Ptr(io::PoseRecord const& p) const { return p.isometry.translation().head<3>().data(); }
    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it
    //   again. Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
        return false;
    }

private:
    std::vector<io::PoseRecord> const& poses_;
};

}  // namespace nie