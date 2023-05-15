/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_INERTIAL_EXPLORER_ISOMETRY_HPP
#define NIE_FORMATS_INERTIAL_EXPLORER_ISOMETRY_HPP

#include <nie/core/ceres/error_propagation.hpp>
#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/core/geometry/isometry3.hpp>

#include "export_profile.hpp"

/// @file isometry.hpp
/// @brief Creates an Isometry and corresponding covariances from Inertial Explorer PosT records.
/// @details PosT records are given in the Aircraft frame:
///
/// x (Right)
/// y (Forward)
/// z (Up)
/// Euler angles based matrix as: z (yaw = -heading) * x (pitch) * y (roll)
///
/// See the Inertial Explorer User Manual 8.8.0, section 3.8.2.4 for the basic source of rotation ordering.
/// https://docs.novatel.com/Waypoint/Content/PDFs/Waypoint_Software_User_Manual_OM-20000166.pdf
///
/// The Intertial Eplorer User Guide 8.1.0, section 4.4.10 provides more in depth information.
/// http://www.sokkiatopcon.tw/NOVATEL/Documents/Manuals/InertialExplorer810.pdf
///
/// Note that the map or meridian convergence is NOT taken into account yet.

namespace nie {
namespace io {
namespace inertial_explorer {

namespace detail {

template <typename T>
inline Eigen::Matrix<T, 3, 1> GridToVectorAircraft(T const& easting, T const& northing, T const& h_ell) {
    return Eigen::Matrix<T, 3, 1>{easting, northing, h_ell};
}

template <typename T>
inline Eigen::Quaternion<T> EulerToQuaternionAircraft(T const& roll, T const& pitch, T const& heading) {
    return EulerToQuaternion<Axis::Z, Axis::X, Axis::Y>(Deg2Rad(-heading), Deg2Rad(pitch), Deg2Rad(roll));
}

template <typename T>
Isometry3q<T> IsometryAircraftFromPosT(
    T const& easting, T const& northing, T const& h_ell, T const& roll, T const& pitch, T const& heading) {
    return {GridToVectorAircraft(easting, northing, h_ell), EulerToQuaternionAircraft(roll, pitch, heading)};
}

struct EulerToQuaternionAircraftFunctor {
    template <typename T>
    bool operator()(T const* const euler, T* quaternion) const {
        Eigen::Map<Eigen::Quaternion<T>>{quaternion} = EulerToQuaternionAircraft(euler[0], euler[1], euler[2]);
        return true;
    }
};

inline void CovQuaternionAircraftFromEuler(
    Eigen::Matrix3d const& cov_e, Eigen::Vector3d const& e, Eigen::Matrix3d* cov_q, Eigen::Quaterniond* q) {
    Eigen::Matrix<double, 4, 4> Q_q;
    nie::AutoDiffErrorPropagator<EulerToQuaternionAircraftFunctor, 4, 3>().Propagate(
        cov_e.data(), e.data(), Q_q.data(), q->coeffs().data());
    // We discard the w part of the quaternion.
    *cov_q = Q_q.block<3, 3>(0, 0);
}

// The order of rotation equals mc * z * x * y.
// Both the z component and the mc are a rotation around z. Because both are the last rotations to be applied, they
// can simply be summed together. Nothing needs to be done for the Error Propagation as it happens that J = f'(z+mc)
// = Quaternion_z(z) == g'(z) = QuaternionZ(z+mc)
// Note: z + mc == yaw + mc == -heading + mc
// yaw' = yaw + mc = -heading + mc
// heading' = -yaw' = -(-heading + mc) = heading - mc
template <typename RowProfile>
inline double HeadingGridNorth(RowProfile const& row) {
    return row.heading - row.map_convergence;
}

// Just returns the heading as the classic PosT file did not contain a column for the map convergence.
template <>
inline double HeadingGridNorth(PosTClassic::Row const& row) {
    return row.heading;
}

}  // namespace detail

template <typename RowProfile>
inline Eigen::Vector3d VectorAircraftFromPosT(RowProfile const& row) {
    return detail::GridToVectorAircraft(row.easting, row.northing, row.height);
}

template <typename RowProfile>
inline nie::Isometry3qd IsometryAircraftFromPosT(RowProfile const& row) {
    return detail::IsometryAircraftFromPosT(
        row.easting, row.northing, row.height, row.roll, row.pitch, detail::HeadingGridNorth(row));
}

inline Eigen::Matrix3d CovVectorAircraftFromPosT(AiimFile::Row const& row) {
    Eigen::Matrix3d cov_t;
    // clang-format off
    cov_t <<
        row.cov_t_ee, row.cov_t_en, row.cov_t_eh,
        row.cov_t_en, row.cov_t_nn, row.cov_t_nh,
        row.cov_t_eh, row.cov_t_nh, row.cov_t_hh;
    // clang-format on
    return cov_t;
}

inline void CovQuaternionAircraftFromPosT(
        AiimFile::Row const& row, Eigen::Matrix3d* cov_q, Eigen::Quaterniond* q) {
    Eigen::Vector3d e;
    e << row.roll, row.pitch, detail::HeadingGridNorth(row);

    Eigen::Matrix3d cov_e;
    // clang-format off
    cov_e <<
        row.cov_r_rr, row.cov_r_pr, row.cov_r_rh,
        row.cov_r_pr, row.cov_r_pp, row.cov_r_ph,
        row.cov_r_rh, row.cov_r_ph, row.cov_r_hh;
    // clang-format on

    detail::CovQuaternionAircraftFromEuler(cov_e, e, cov_q, q);
}

inline Eigen::Matrix3d CovQuaternionAircraftFromPosT(AiimFile::Row const& row) {
    Eigen::Quaterniond q;
    Eigen::Matrix3d cov_q;
    CovQuaternionAircraftFromPosT(row, &cov_q, &q);
    return cov_q;
}

inline Eigen::Matrix<double, 6, 6> CovIsometryAircraftFromPosT(AiimFile::Row const& row) {
    Eigen::Matrix<double, 6, 6> cov_i;
    cov_i.block<3, 3>(0, 0) = CovVectorAircraftFromPosT(row);
    cov_i.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    cov_i.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    cov_i.block<3, 3>(3, 3) = CovQuaternionAircraftFromPosT(row);
    return cov_i;
}

/// Most efficient implementation if we want both the isometry and covariance.
inline void CovIsometryAircraftFromPosT(
            AiimFile::Row const& row, Eigen::Matrix<double, 6, 6>* covariance, nie::Isometry3qd* isometry) {
    Eigen::Matrix3d cov_q;
    isometry->translation() = VectorAircraftFromPosT(row);
    CovQuaternionAircraftFromPosT(row, &cov_q, &isometry->rotation());
    covariance->block<3, 3>(0, 0) = CovVectorAircraftFromPosT(row);
    covariance->block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
    covariance->block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
    covariance->block<3, 3>(3, 3) = cov_q;
}

}  // namespace inertial_explorer
}  // namespace io
}  // namespace nie

#endif  // NIE_FORMATS_INERTIAL_EXPLORER_ISOMETRY_HPP
