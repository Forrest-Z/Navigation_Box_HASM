/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_G2O_READER
#define NIE_FORMATS_G2O_READER

#include <fstream>
#include <iomanip>
#include <map>
#include <string>

#include <glog/logging.h>

#include "g2o_types.hpp"

namespace nie {

namespace io {

inline std::istream& operator>>(std::istream& input, nie::Isometry3qd& v) {
    auto& t = v.translation();
    auto& r = v.rotation();

    input >> t.x() >> t.y() >> t.z() >> r.x() >> r.y() >> r.z() >> r.w();

    // Normalize the quaternion to account for precision loss due to
    // serialization.
    r.normalize();

    return input;
}

inline std::istream& operator>>(std::istream& input, G2oVertexSe3Quat& pose) {
    input >> pose.id >> pose.v;

    return input;
}

// Helper function to fill the information matrix using the stream
template <typename T, int Rows, int Cols>
inline void ReadInformationMatrix(std::istream& input_stream, Eigen::Matrix<T, Rows, Cols>* p_matrix) {
    Eigen::Matrix<T, Rows, Cols>& matrix = *p_matrix;

    // Firstly, read the upper triangle
    for (int row = 0; row < matrix.rows(); ++row) {
        for (int col = row; col < matrix.cols(); ++col) {
            input_stream >> matrix(row, col);
        }
    }

    // Secondly, copy the upper to the lower triangle
    matrix.template triangularView<Eigen::StrictlyLower>() = matrix.transpose();
}

inline std::istream& operator>>(std::istream& input, G2oEdgeSe3Quat& constraint) {
    Isometry3qd& t_be = constraint.t_be;
    input >> constraint.id_begin >> constraint.id_end >> t_be;
    ReadInformationMatrix(input, &constraint.information);
    return input;
}

// Reads a single pose from the input and inserts it into the map. Returns false
// if there is a duplicate entry.
template <typename Pose>
bool ReadVertex(std::ifstream* infile, std::map<int, Pose, std::less<>>* poses) {
    Pose pose;
    *infile >> pose;

    // Ensure we don't have duplicate poses.
    if (poses->find(pose.id) != poses->end()) {
        LOG(ERROR) << "Duplicate vertex with ID: " << pose.id;
        return false;
    }

    (*poses)[pose.id] = pose;

    return true;
}

// Reads the contraints between two vertices in the pose graph
template <typename Constraint, typename Allocator>
void ReadConstraint(std::ifstream* infile, std::vector<Constraint, Allocator>* constraints) {
    Constraint constraint;
    *infile >> constraint;

    constraints->push_back(constraint);
}

// Reads a file in the g2o filename format that describes a pose graph
// problem. The g2o format consists of two entries, vertices and constraints.
//
// In 2D, a vertex is defined as follows:
//
// VERTEX_SE2 ID x_meters y_meters yaw_radians
//
// A constraint is defined as follows:
//
// EDGE_SE2 ID_A ID_B A_x_B A_y_B A_yaw_B I_11 I_12 I_13 I_22 I_23 I_33
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement.
//
//
// In 3D, a vertex is defined as follows:
//
// VERTEX_SE3:QUAT ID x y z q_x q_y q_z q_w
//
// where the quaternion is in Hamilton form.
// A constraint is defined as follows:
//
// EDGE_SE3:QUAT ID_a ID_b x_ab y_ab z_ab q_x_ab q_y_ab q_z_ab q_w_ab I_11 I_12 I_13 ... I_16 I_22 I_23 ... I_26 ...
// I_66 // NOLINT
//
// where I_ij is the (i, j)-th entry of the information matrix for the
// measurement. Only the upper-triangular part is stored. The measurement order
// is the delta position followed by the delta orientation.
template <typename Pose, typename Constraint>
bool ReadG2oFile(
    const std::string& filename, std::map<int, Pose, std::less<>>* poses, std::vector<Constraint>* constraints) {
    CHECK_NOTNULL(poses);
    CHECK_NOTNULL(constraints);

    poses->clear();
    constraints->clear();

    std::ifstream infile(filename.c_str());
    if (!infile) {
        return false;
    }

    std::string data_type;
    while (infile.good()) {
        // Read whether the type is a node or a constraint.
        infile >> data_type;
        if (data_type == Pose::Tag()) {
            if (!ReadVertex(&infile, poses)) {
                return false;
            }
        } else if (data_type == Constraint::Tag()) {
            ReadConstraint(&infile, constraints);
        } else {
            LOG(ERROR) << "Unknown data type: " << data_type;
            return false;
        }

        // Clear any trailing whitespace from the line.
        infile >> std::ws;
    }

    return true;
}

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_G2O_READER