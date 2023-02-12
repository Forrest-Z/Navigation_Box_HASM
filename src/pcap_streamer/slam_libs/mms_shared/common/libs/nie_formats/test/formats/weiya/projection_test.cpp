/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/geometry/conversion.hpp>
#include <nie/core/geometry/interpolation.hpp>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/time.hpp>
#include <nie/cv/geometry/triangulation.hpp>
#include <nie/formats/inertial_explorer/export_profile.hpp>
#include <nie/formats/inertial_explorer/isometry.hpp>
#include <nie/formats/opencv.hpp>
#include <nie/formats/weiya/camera_boresight.hpp>
#include <nie/formats/weiya/image_time_in_day.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common.hpp"

// clang-format off
std::string kFilename675Left =
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/650-750/LeftCamera/20190403-040911538069-0000000675_L.jpg";
std::string kFilename675Right =
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/650-750/RightCamera/20190403-040911538069-0000000675_R.jpg";
std::string kFilename676Left =
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/650-750/LeftCamera/20190403-040912222386-0000000676_L.jpg";
std::string kFilename95Left =
    "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/80-110/LeftCamera/20190403-040450530251-0000000095_L.jpg";

// Measured in the field.
//      easting,        northing,       height
const Eigen::Matrix<double, 3, 6> kGroundTruthAirplane = (
    Eigen::Matrix<double, 6, 3>{} <<
        545302.631,     3369674.4305,   25.944,
        545302.8585,    3369672.7745,   25.911,
        545271.757,     3369674.784,    26.576,
        545271.8235,    3369675.7775,   26.564,
        545271.8925,    3369676.8025,   26.572,
        545271.959,     3369677.8325,   26.552
).finished().transpose();

const Eigen::Matrix<double, 2, 6> kGroundTruthLeft675 = (
    Eigen::Matrix<double, 6, 2>{} <<
        1955.51,    348.757,
        1770.95,    346.892,
        1987.1,     634.276,
        2032.71,    634.374,
        2079.83,    633.526,
        2127.25,    633.989
).finished().transpose();

const Eigen::Matrix<double, 2, 6> kGroundTruthRight675 = (
    Eigen::Matrix<double, 6, 2>{} <<
        1845.62,    348.757,
        1659.71,    346.892,
        1941.26,    634.276,
        1986.83,    634.374,
        2033.91,    633.526,
        2081.3,     633.989
).finished().transpose();

const Eigen::Matrix<double, 2, 2> kGroundTruthLeft95 = (
    Eigen::Matrix<double, 2, 2>{} <<
        2977.472, 214.030,
        2771.662, 265.317
).finished().transpose();

const std::vector<cv::Rect> kRects675Left{
        {1932,  246,    89,    191},
        {1703,  246,    91,    190},
        {1954,  603,    48,    46},
        {2003,  599,    41,    51},
        {2048,  597,    44,    54},
        {2098,	597,    40,    52}
};

const std::vector<cv::Rect> kRects676Left{
        {1899,  126,    102,    219},
        {1616,  118,    106,    227},
        {1941,  583,    48,     50},
        {1993,  581,    44,     52},
        {2041,  577,    47,     57},
        {2093,  578,    47,     55}
};

// IMU to GNSS Antenna Lever Arms:
// /data/aiim/Data_Coremap/Project1001-0003-190403-03/Output/1001-0003-190403-03-base.post

//
// /data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/650-750/LeftCamera/20190403-040911538069-0000000675_L.jpg
// /data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/650-750/LeftCamera/20190403-040912222386-0000000676_L.jpg

//
// /data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/650-750/RightCamera/20190403-040911538069-0000000675_R.jpg
// /data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03/650-750/RightCamera/20190403-040912222386-0000000676_R.jpg

// /data/aiim/Data_Coremap/Project1001-0003-190403-03/Output/1001-0003-190403-03-base.post
// SeqNum        GPSTime      Northing       Easting        H-Ell        Latitude       Longitude   HzSpeed      Roll     Pitch   Heading Project Name             Q

// 675
// Timestamp conversions: 20190403-040911538069-0000000675_L - 20190403 = 14969.538000000000466
// Following records:
// 44053       14969.530  3369676.0487   545325.1982       20.768  30.44623632994 114.47188082325    5.7072   0.12724  -4.73070 -88.14472 1001-0003-190403-03-base 1
// 44054       14969.540  3369676.0501   545325.1411       20.769  30.44623634501 114.47188022882    5.7092   0.12462  -4.74309 -88.14294 1001-0003-190403-03-base 1

const double kTimeInDay675 = 14969.538000000000466;
const nie::io::inertial_explorer::PosTClassic::Row kRow44053 {
        44053, 14969.530, 3369676.0487, 545325.1982, 20.768, 0.12724, -4.73070, -88.14472, 5.7072
};
const nie::io::inertial_explorer::PosTClassic::Row kRow44054 {
        44054, 14969.540, 3369676.0501, 545325.1411, 20.769, 0.12462, -4.74309, -88.14294, 5.7092
};

// 676
// Timestamp conversions: 20190403-040912222386-0000000676_L.jpg - 20190403 = 14970.221999999999753
// 44122       14970.220  3369676.1633   545321.1757       20.819  30.44623751499 114.47183895115    5.9685   0.50521  -4.75014 -88.07474 1001-0003-190403-03-base 2
// 44123       14970.230  3369676.1646   545321.1160       20.820  30.44623752903 114.47183832960    5.9710   0.49462  -4.74727 -88.07371 1001-0003-190403-03-base 2

const double kTimeInDay676 = 14970.221999999999753;
const nie::io::inertial_explorer::PosTClassic::Row kRow44122 {
        44122, 14970.220, 3369676.1633, 545321.1757, 20.819, 0.50521, -4.75014, -88.07474, 5.9685
};
const nie::io::inertial_explorer::PosTClassic::Row kRow44123 {
        44123, 14970.230, 3369676.1646, 545321.1160, 20.820, 0.49462, -4.74727, -88.07371, 5.9710
};

// 95
// Timestamp conversions: 20190403-040450530251-0000000095_L.jpg - 20190403 = 14708.530251000000135
// 17953       14708.530  3369686.5660   545316.2756       20.726  30.44633153325 114.47178838970    7.2998  -0.77607  -4.27728 -147.5325 1001-0003-190403-03-base 3
// 17954       14708.540  3369686.5060   545316.2342       20.727  30.44633099325 114.47178795567    7.3040  -0.77190  -4.27508 -147.3606 1001-0003-190403-03-base 3

const double kTimeInDay95 = 14708.530251000000135;
const nie::io::inertial_explorer::PosTClassic::Row kRow17953 {
        17953, 14708.530, 3369686.5660, 545316.2756, 20.726, -0.77607, -4.27728, -147.5325, 7.2998
};
const nie::io::inertial_explorer::PosTClassic::Row kRow17954 {
        17954, 14708.540, 3369686.5060, 545316.2342, 20.727, -0.77190, -4.27508, -147.3606, 7.3040
};

// clang-format on

// Print time in day since the start of the GPS date
void PrintTimeInDay(std::string const& filename) {
    // Obtain GPS time from the filename
    auto const gps_time = nie::io::weiya::GpsTimeFromFilename(filename);

    // Convert to alternative representation of GPS date / time in date
    auto const gps_daytime = nie::ToGPSDayTime(gps_time);

    // Represent time in date as double [in seconds]
    double const seconds_in_day = nie::RepresentDurationAsDouble(gps_daytime.time_in_day);

    std::cout << std::endl << std::setprecision(20) << "GPS Time in day: " << seconds_in_day << std::endl << std::endl;
}

nie::Isometry3qd IsometryAirplane(
        double time_in_day,
        nie::io::inertial_explorer::PosTClassic::Row min,
        nie::io::inertial_explorer::PosTClassic::Row max) {
    double ratio = (time_in_day - min.time_in_day) / (max.time_in_day - min.time_in_day);
    CHECK(ratio >= 0 && ratio <= 1.0) << "Ratio should be in the range [0...1]";
    return nie::Interpolate(IsometryAircraftFromPosT(min), IsometryAircraftFromPosT(max), ratio);
}

nie::Isometry3qd IsometryAirplane675() { return IsometryAirplane(kTimeInDay675, kRow44053, kRow44054); }

nie::Isometry3qd IsometryAirplane676() { return IsometryAirplane(kTimeInDay676, kRow44122, kRow44123); }

nie::Isometry3qd IsometryAirplane95() { return IsometryAirplane(kTimeInDay95, kRow17953, kRow17954); }

Eigen::Matrix<double, 2, 6> ProjectPoints(
        nie::Isometry3qd const& C, Eigen::Matrix3d const& K, Eigen::Matrix<double, 3, 6> const& points3d) {
    Eigen::Matrix<double, 3, 4> P = K * C.Inversed().ToTransform().matrix().block<3, 4>(0, 0);
    return (P * points3d.colwise().homogeneous()).colwise().hnormalized();
}

void PrintPoints(Eigen::Matrix<double, 2, 6> const& points_2d, bool with_norm = false) {
    std::cout << std::endl;
    for (int i = 0; i < points_2d.cols(); ++i) {
        std::cout << std::setprecision(20) << points_2d.col(i).transpose()
                  << ((with_norm) ? " [" + std::to_string(points_2d.col(i).norm()) + "]" : "") << std::endl;
    }
    std::cout << std::endl;
}

void PrintPoints(Eigen::Matrix<double, 3, 6> const& points_3d, bool with_norm = false) {
    std::cout << std::endl;
    for (int i = 0; i < points_3d.cols(); ++i) {
        std::cout << std::setprecision(20) << points_3d.col(i).transpose()
                  << ((with_norm) ? " [" + std::to_string(points_3d.col(i).norm()) + "]" : "") << std::endl;
    }
    std::cout << std::endl;
}

cv::Point Center(cv::Rect const& r) { return (r.tl() + r.br()) / 2; }

void DrawAndShowPoints(std::string const& filename, Eigen::Matrix<double, 2, 6> const& points_2d) {
    cv::Mat img = cv::imread(filename);

    for (int i = 0; i < points_2d.cols(); ++i) {
        cv::circle(img, cv::Point(nie::ConvertPoint(points_2d.col(i).eval())), 5, {0, 0, 255}, 5);
    }

    cv::Mat img_small;
    cv::resize(img, img_small, {}, 0.35, 0.35);
    cv::imshow("AllBs", img_small);
    cv::waitKey(0);
}

void DrawAndShowPoints(
        std::string const& filename, Eigen::Matrix<double, 2, 6> const& points_2d, std::vector<cv::Rect> const& rects) {
    cv::Mat img = cv::imread(filename);

    for (int i = 0; i < points_2d.cols(); ++i) {
        cv::circle(img, cv::Point(nie::ConvertPoint(points_2d.col(i).eval())), 5, {0, 0, 255}, 5);

        cv::rectangle(img, rects[i], {0, 255, 0}, 3);
        cv::circle(img, Center(rects[i]), 5, {0, 255, 0}, 5);
    }

    cv::Mat img_small;
    cv::resize(img, img_small, {}, 0.35, 0.35);
    cv::imshow("AllBs", img_small);
    cv::waitKey(0);
}

void DrawAndShowPoints(
        std::string const& filename,
        Eigen::Matrix<double, 2, 6> const& points_2d,
        std::vector<cv::Rect> const& rects,
        Eigen::Matrix<double, 2, 6> const& bp_points_2d) {
    cv::Mat img = cv::imread(filename);

    for (int i = 0; i < points_2d.cols(); ++i) {
        cv::circle(img, cv::Point(nie::ConvertPoint(points_2d.col(i).eval())), 5, {0, 0, 255}, 5);

        cv::rectangle(img, rects[i], {0, 255, 0}, 3);
        cv::circle(img, Center(rects[i]), 5, {0, 255, 0}, 5);

        cv::circle(img, cv::Point(nie::ConvertPoint(bp_points_2d.col(i).eval())), 5, {255, 0, 0}, 5);
    }

    cv::Mat img_small;
    cv::resize(img, img_small, {}, 0.35, 0.35);
    cv::imshow("AllBs", img_small);
    cv::waitKey(0);
}

Eigen::Matrix<float, 2, 6> GetPoints(std::vector<cv::Rect> const& rects) {
    CHECK(rects.size() == 6) << "Rect!";
    Eigen::Matrix<float, 2, 6> points_2d;

    for (std::size_t i = 0; i < rects.size(); ++i) {
        points_2d.col(i) = nie::ConvertPoint(Center(rects[i])).cast<float>();
    }

    return points_2d;
}

Eigen::Matrix<double, 3, 6> TriangulatePoints(
        nie::Isometry3qd const& cam0,
        nie::Isometry3qd const& cam1,
        Eigen::Matrix3d const& K,
        Eigen::Matrix<float, 2, 6> const& points0,
        Eigen::Matrix<float, 2, 6> const& points1) {
    Eigen::Matrix<double, 3, 6> points3d;
    std::vector<nie::Isometry3qd> cams{cam0, cam1};

    for (int i = 0; i < points3d.cols(); ++i) {
        std::vector<Eigen::Vector2f> image_points{points0.col(i), points1.col(i)};

        Eigen::Vector3d x;
        CHECK(TriangulateNonLinear(cams, image_points, K, &x)) << "TriangulateNonLinear :(";
        points3d.col(i) = x;
    }

    return points3d;
}

TEST(ProjectionTest, BackProjectionGroundTruth) {
    nie::io::weiya::CameraCalibration extrinsics_xml = nie::io::weiya::CameraCalibration::Read(kFilenameExtrinsicsXml);
    nie::Isometry3qd T_imu_l = nie::io::weiya::CameraBoresight::Read(kFilenameAllBs);

    nie::Isometry3qd T_675_g_gnss = IsometryAirplane675();
    nie::Isometry3qd T_95_g_gnss = IsometryAirplane95();

    nie::Isometry3qd cam_l_675 = T_675_g_gnss * GetCamToGnssLeft(extrinsics_xml, T_imu_l);
    nie::Isometry3qd cam_r_675 = T_675_g_gnss * GetCamToGnssRight(extrinsics_xml, T_imu_l);
    nie::Isometry3qd cam_l_95 = T_95_g_gnss * GetCamToGnssLeft(extrinsics_xml, T_imu_l);

    Eigen::Matrix<double, 2, 6> points_2d_l_675 = ProjectPoints(cam_l_675, extrinsics_xml.K, kGroundTruthAirplane);
    Eigen::Matrix<double, 2, 6> points_2d_r_675 = ProjectPoints(cam_r_675, extrinsics_xml.K, kGroundTruthAirplane);
    Eigen::Matrix<double, 2, 6> points_2d_l_95 = ProjectPoints(cam_l_95, extrinsics_xml.K, kGroundTruthAirplane);

    // If somehow this test fails, these prints and/or visualizations can be enabled.
    // PrintPoints(points_2d_l_675);
    // PrintPoints(points_2d_r_675);
    // PrintPoints(points_2d_l_95);
    // DrawAndShowPoints(kFilename675Left, points_2d_l_675);
    // DrawAndShowPoints(kFilename675Right, points_2d_r_675);
    // DrawAndShowPoints(kFilename95Left, points_2d_l_95);

    EXPECT_NEAR((kGroundTruthLeft675 - points_2d_l_675).norm(), 0, 1.5);
    EXPECT_NEAR((kGroundTruthRight675 - points_2d_r_675).norm(), 0, 1.5);
    // The first two points are the traffic lights, the traffic signs are not seen (outside of the image)
    EXPECT_NEAR((kGroundTruthLeft95 - points_2d_l_95.block<2, 2>(0, 0)).norm(), 0, 1.5);
}

TEST(ProjectionTest, DISABLED_TriangulationDemo) {
    nie::io::weiya::CameraCalibration extrinsics_xml = nie::io::weiya::CameraCalibration::Read(kFilenameExtrinsicsXml);
    nie::Isometry3qd T_imu_l = nie::io::weiya::CameraBoresight::Read(kFilenameAllBs);

    nie::Isometry3qd T_675_g_gnss = IsometryAirplane675();
    nie::Isometry3qd T_676_g_gnss = IsometryAirplane676();

    nie::Isometry3qd cam_l_675 = T_675_g_gnss * GetCamToGnssLeft(extrinsics_xml, T_imu_l);
    nie::Isometry3qd cam_r_675 = T_675_g_gnss * GetCamToGnssRight(extrinsics_xml, T_imu_l);
    nie::Isometry3qd cam_l_676 = T_676_g_gnss * GetCamToGnssLeft(extrinsics_xml, T_imu_l);

    Eigen::Matrix<double, 2, 6> points_2d_l_675 = ProjectPoints(cam_l_675, extrinsics_xml.K, kGroundTruthAirplane);
    Eigen::Matrix<double, 2, 6> points_2d_r_675 = ProjectPoints(cam_r_675, extrinsics_xml.K, kGroundTruthAirplane);
    Eigen::Matrix<double, 2, 6> points_2d_l_676 = ProjectPoints(cam_l_676, extrinsics_xml.K, kGroundTruthAirplane);

    Eigen::Matrix<float, 2, 6> centers_2d_l_675 = GetPoints(kRects675Left);
    Eigen::Matrix<float, 2, 6> centers_2d_l_676 = GetPoints(kRects676Left);
    Eigen::Matrix<double, 3, 6> triangulated =
            TriangulatePoints(cam_l_675, cam_l_676, extrinsics_xml.K, centers_2d_l_675, centers_2d_l_676);
    Eigen::Matrix<double, 2, 6> bp_points_2d_l_675 = ProjectPoints(cam_l_675, extrinsics_xml.K, triangulated);
    Eigen::Matrix<double, 2, 6> bp_points_2d_l_676 = ProjectPoints(cam_l_676, extrinsics_xml.K, triangulated);

    DrawAndShowPoints(kFilename675Left, points_2d_l_675, kRects675Left, bp_points_2d_l_675);
    DrawAndShowPoints(kFilename675Right, points_2d_r_675);
    DrawAndShowPoints(kFilename676Left, points_2d_l_676, kRects676Left, bp_points_2d_l_676);

    // Residuals
    PrintPoints((bp_points_2d_l_675 - centers_2d_l_675.cast<double>()).eval(), true);
    PrintPoints((bp_points_2d_l_676 - centers_2d_l_676.cast<double>()).eval(), true);
    PrintPoints((triangulated - kGroundTruthAirplane).eval(), true);

    PrintPoints(triangulated);
}

/// Example on how to get the timestamp used for sampling within PosT files and how to generate an interpolated
/// position.
TEST(ProjectionTest, DISABLED_GetTimeInDayDemo) {
    PrintTimeInDay(kFilename95Left);
    // Equals IsometryAirplane95()
    [[maybe_unused]] nie::Isometry3qd T_95_g_gnss = IsometryAirplane(kTimeInDay95, kRow17953, kRow17954);
}

/// Example to show how to generate new local extrinsics file for position_estimator.
TEST(ProjectionTest, DISABLED_GenerateLocalExtrinsicsDemo) {
    // clang-format off
    std::vector<std::string> dirs = {
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1001-0003-190403-03",
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1002-0001-190828-00-rectified-1m1frame",
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1002-0001-190828-00-rectified-1m1frame_halfed",
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1002-0001-190828-01-rectified-2m1frame",
        "/data/aiim/unit_tests_data/position_estimator/regression/coremap-1002-0001-190828-01-rectified-2m1frame_halfed"};
    std::string out_left = "/local_extrinsics_left.json";
    std::string out_right = "/local_extrinsics_right.json";
    // clang-format on

    nie::io::weiya::CameraCalibration extrinsics_xml = nie::io::weiya::CameraCalibration::Read(kFilenameExtrinsicsXml);
    nie::Isometry3qd T_imu_l = nie::io::weiya::CameraBoresight::Read(kFilenameAllBs);

    for (auto const& dir : dirs) {
        std::string out_filename_left = dir + out_left;
        std::string out_filename_right = dir + out_right;

        nie::Isometry3qd lel = GetCamToGnssLeft(extrinsics_xml, T_imu_l);
        nie::Isometry3qd ler = GetCamToGnssRight(extrinsics_xml, T_imu_l);

        nie::io::OpenCvSerializer<nie::Isometry3qd>::Write(out_filename_left, lel);
        nie::io::OpenCvSerializer<nie::Isometry3qd>::Write(out_filename_right, ler);
    }
}