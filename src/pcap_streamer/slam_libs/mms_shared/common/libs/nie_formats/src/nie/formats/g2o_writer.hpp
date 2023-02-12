/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_G2O_WRITER
#define NIE_FORMATS_G2O_WRITER

#include <fstream>
#include <iomanip>
#include <map>
#include <string>

#include <glog/logging.h>

#include "g2o_types.hpp"

namespace nie {

namespace io {

inline std::ostream& operator<<(std::ostream& output, Isometry3qd const& pose) {
    auto& t = pose.translation();
    auto& r = pose.rotation();

    output << t.x() << " " << t.y() << " " << t.z() << " " << r.x() << " " << r.y() << " " << r.z() << " " << r.w();

    return output;
}

inline std::ostream& operator<<(std::ostream& output, G2oVertexSe3Quat const& pose) {
    output << pose.id << " " << pose.v;

    return output;
}

inline std::ostream& operator<<(std::ostream& output, G2oEdgeSe3Quat const& constraint) {
    const Isometry3qd& t_be = constraint.t_be;
    output << constraint.id_begin << " " << constraint.id_end << " " << t_be;

    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            output << " " << constraint.information(i, j);
        }
    }

    return output;
}

template <typename Pose, typename Constraint>
bool WriteG2oFile(
    const std::string& filename,
    const std::map<int, Pose, std::less<>>& poses,
    const std::vector<Constraint>& constraints) {
    std::ofstream outfile(filename.c_str());
    if (!outfile) {
        return false;
    }

    outfile << std::setprecision(10);

    for (const auto& pose : poses) {
        outfile << Pose::Tag() << " " << pose.second << std::endl;
    }

    for (const auto& constraint : constraints) {
        outfile << Constraint::Tag() << " " << constraint << std::endl;
    }

    return true;
}

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_G2O_WRITER
