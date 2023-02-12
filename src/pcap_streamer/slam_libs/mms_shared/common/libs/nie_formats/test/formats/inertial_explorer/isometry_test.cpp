/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/time.hpp>
#include <nie/formats/inertial_explorer/isometry.hpp>

// clang-format off
// /data/aiim/Data_Coremap/Project1001-0003-190403-03/Output/1001-0003-190403-03-base.post
// SeqNum        GPSTime      Northing       Easting        H-Ell        Latitude       Longitude   HzSpeed      Roll     Pitch   Heading Project Name             Q
// Following record:
// 44054       14969.540  3369676.0501   545325.1411       20.769  30.44623634501 114.47188022882    5.7092   0.12462  -4.74309 -88.14294 1001-0003-190403-03-base 1
const nie::io::inertial_explorer::PosTClassic::Row kRow44054 {
    44054, 14969.540, 3369676.0501, 545325.1411, 20.769, 0.12462, -4.74309, -88.14294, 5.7092
};

// /data/aiim/Data_Coremap/weiyacollection-10fps/1002-0001-200117-00.aiim
// SeqNum         Week     GPSTime     Northing      Easting        H-Ell       Latitude     Longitude        Roll       Pitch     Heading         Convg          CxEE          CxEH          CxEN          CxHH          CxNH          CxNN         CxVHH         CxVPP         CxVPH         CxVPR         CxVRR         CxVRH
// Following record:
// 0        2088.00000 26019.00000  3371904.611   537587.191       23.063   30.466604750 114.391402482  -1.3031123  -4.0245415  87.6287381  0.1984575043   1.81730E-05  -1.37058E-06  -9.13177E-08   1.09181E-04  -2.52030E-06   2.44757E-05   2.12234E-03   8.80370E-05   1.57590E-09   1.15405E-11   8.76386E-05  -7.01288E-10
const nie::io::inertial_explorer::AiimFile::Row kRow0 {
    0, std::chrono::weeks{2088}, 26019.00000 , 3371904.611, 537587.191, 23.063, -1.3031123, -4.0245415, 87.6287381, 0.1984575043, 1.81730E-05, -1.37058E-06, -9.13177E-08, 1.09181E-04, -2.52030E-06, 2.44757E-05, 2.12234E-03, 8.80370E-05, 1.57590E-09, 1.15405E-11, 8.76386E-05, -7.01288E-10, {}
};

// clang-format on

/// Reference from 4.4.10 of user guide:
/// http://www.sokkiatopcon.tw/NOVATEL/Documents/Manuals/InertialExplorer810.pdf
/// See also Relating the Body Frame to the Local Level Frame:
/// https://www.novatel.com/assets/Documents/Bulletins/apn037.pdf
template <typename T>
inline Eigen::Matrix<T, 3, 3> EulerPostToMatrixAircraft_IE_UserGuide(T const& roll, T const& pitch, T const& heading) {
    T r = nie::Deg2Rad(roll);
    T p = nie::Deg2Rad(pitch);
    T y = nie::Deg2Rad(-heading);

    using namespace std;

    Eigen::Matrix<T, 3, 3> R_rpy;
    R_rpy(0, 0) = cos(y) * cos(r) - sin(y) * sin(p) * sin(r);
    R_rpy(1, 0) = sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
    R_rpy(2, 0) = -cos(p) * sin(r);

    R_rpy(0, 1) = -sin(y) * cos(p);
    R_rpy(1, 1) = cos(y) * cos(p);
    R_rpy(2, 1) = sin(p);

    R_rpy(0, 2) = cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
    R_rpy(1, 2) = sin(y) * sin(r) - cos(y) * sin(p) * cos(r);
    R_rpy(2, 2) = cos(p) * cos(r);

    DCHECK(R_rpy.determinant() > T(0));

    return R_rpy;
}

TEST(InertialExplorerIsometryTest, EulerToQuaterion) {
    auto isometry = nie::io::inertial_explorer::IsometryAircraftFromPosT(kRow44054);

    Eigen::Matrix3d R_i{isometry.rotation()};
    Eigen::Matrix3d R_r = EulerPostToMatrixAircraft_IE_UserGuide(kRow44054.roll, kRow44054.pitch, kRow44054.heading);

    EXPECT_TRUE(R_i.isApprox(R_r, 0.0000001));
}

TEST(InertialExplorerIsometryTest, EulerToQuaterionErrorPropagation) {
    // Effectively a duplicate implementation to see if it matches.
    // clang-format off
    Eigen::Matrix3d cov_t;
    cov_t <<
        kRow0.cov_t_ee, kRow0.cov_t_en, kRow0.cov_t_eh,
        kRow0.cov_t_en, kRow0.cov_t_nn, kRow0.cov_t_nh,
        kRow0.cov_t_eh, kRow0.cov_t_nh, kRow0.cov_t_hh;

    // This doesn't come from the row directly. Euler angles were converted to quaternion from which this test was
    // created. We only test stability, not correctness.
    Eigen::Matrix3d cov_q;
    cov_q <<
         6.7171062152146520998e-09,  7.0833922164490129898e-11,  -2.184804620700887553e-09,
         7.0833922164490789057e-11,   6.863254772061697016e-09, -3.8906954917795556163e-09,
        -2.1848046207008871394e-09, -3.8906954917795556163e-09,  8.4399831578561669524e-08;
    // clang-format on

    // Test the composition of the transformed covariance.
    Eigen::Matrix<double, 6, 6> cov_i = Eigen::Matrix<double, 6, 6>::Zero();
    cov_i.block<3, 3>(0, 0) = cov_t;
    cov_i.block<3, 3>(3, 3) = cov_q;

    EXPECT_TRUE(cov_i.isApprox(nie::io::inertial_explorer::CovIsometryAircraftFromPosT(kRow0)));

    nie::Isometry3qd isometry;
    Eigen::Matrix<double, 6, 6> cov;
    nie::io::inertial_explorer::CovIsometryAircraftFromPosT(kRow0, &cov, &isometry);
    EXPECT_TRUE(cov_i.isApprox(cov));
}
