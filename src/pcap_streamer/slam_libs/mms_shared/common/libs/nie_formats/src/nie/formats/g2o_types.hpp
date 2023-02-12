/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_G2O_TYPES
#define NIE_FORMATS_G2O_TYPES

// https://github.com/RainerKuemmerle/g2o/wiki/File-Format

#include <map>
#include <string>
#include <vector>

#include <nie/core/geometry/isometry3.hpp>

namespace nie {

namespace io {

struct G2oVertexSe3Quat {
    int id;

    // The transformation that represents the vertex w.r.t. the origin (it simply contains the position and orientation
    // of the vertex in the world). A more formal name would be t_ov.
    nie::Isometry3qd v;

    // The name of the data type in the g2o file format.
    static std::string Tag() { return "VERTEX_SE3:QUAT"; }
};

using MapOfPoses = std::map<int, G2oVertexSe3Quat, std::less<> >;

// The constraint between two vertices in the pose graph. The constraint is the transformation from vertex id_begin to
// vertex id_end.
struct G2oEdgeSe3Quat {
    int id_begin;
    int id_end;

    // The transformation that represents the pose of the end frame E w.r.t. the begin frame B. In other words, it
    // transforms a vector in the E frame to the B frame.
    nie::Isometry3qd t_be;

    // The inverse of the covariance matrix for the measurement. The order of the entries are x, y, z, delta
    // orientation.
    Eigen::Matrix<double, 6, 6> information;

    // The name of the data type in the g2o file format.
    static std::string Tag() { return "EDGE_SE3:QUAT"; }
};

using VectorOfConstraints = std::vector<G2oEdgeSe3Quat>;

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_G2O_TYPES
