/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_HELPER_CALIBRATION_IO_HPP
#define NIE_FORMATS_CALIB3D_HELPER_CALIBRATION_IO_HPP

#include <string>
#include <vector>

#include "nie/formats/opencv.hpp"

namespace nie {

namespace io {

class CalibratedParameters {
public:
    std::string focal_length, skew, distortion_radial, distortion_tangential, distortion_thin_prism;
};

class CalibratedPinhole {
public:
    std::string id;
    std::vector<double> intrinsics;
    Eigen::MatrixXd cov;
};

// Another way of looking at the baseline, is considering this to be the right camera
// having the left one as the origin.
class CalibratedBaseline {
public:
    nie::Isometry3qd estimate;
    Eigen::Matrix<double, 6, 6> cov;
};

class CalibratedStereo {
public:
    CalibratedPinhole lens_left;
    CalibratedPinhole lens_right;
    CalibratedBaseline baseline;
};

class ExtendedCalibrationData {
public:
    std::vector<std::string> image_ids;
    std::vector<std::vector<Eigen::Vector2f>> image_points;
    std::vector<nie::Isometry3qd> extrinsics;
    std::vector<std::vector<Eigen::Vector2f>> residuals;
    std::vector<Eigen::Matrix<double, 6, 6>> cov_extrinsics;
};

class ValidatedCalibrationData {
public:
    std::vector<std::string> image_ids;
    std::vector<std::vector<Eigen::Vector2f>> image_points;
    std::vector<std::vector<Eigen::Vector2f>> residuals;
};

// Fixed-size covariance I/O (take 4)
template <typename T, int Size>
void WriteCov(cv::FileStorage& fs, Eigen::Matrix<T, Size, Size> const& cov) {
    fs << "[";
    for (int row = 0; row < Size; ++row) {
        for (int col = row; col < Size; ++col) {
            fs << cov(row, col);
        }
    }
    fs << "]";
}

template <typename T, int Size>
void ReadCov(cv::FileNode const& node, Eigen::Matrix<T, Size, Size>& cov) {
    size_t idx = 0;
    for (int row = 0; row < Size; ++row) {
        for (int col = row; col < Size; ++col) {
            node[idx++] >> cov(row, col);
        }
    }
    cov.template triangularView<Eigen::StrictlyLower>() =
        cov.template triangularView<Eigen::StrictlyUpper>().transpose();
}

}  // namespace io

}  // namespace nie

namespace cv {

// NOTE: Deviation from the google code guide because this interface is needed for OpenCV serialization.
void write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedPinhole const& p);
void read(
    cv::FileNode const& node,
    nie::io::CalibratedPinhole& x,
    nie::io::CalibratedPinhole const& default_value = nie::io::CalibratedPinhole());

void write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedParameters const& p);
void read(
    cv::FileNode const& node,
    nie::io::CalibratedParameters& x,
    nie::io::CalibratedParameters const& default_value = nie::io::CalibratedParameters());

void write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedBaseline const& p);
void read(
    cv::FileNode const& node,
    nie::io::CalibratedBaseline& x,
    nie::io::CalibratedBaseline const& default_value = nie::io::CalibratedBaseline());

void write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedStereo const& p);
void read(
    cv::FileNode const& node,
    nie::io::CalibratedStereo& x,
    nie::io::CalibratedStereo const& default_value = nie::io::CalibratedStereo());

void write(cv::FileStorage& fs, std::string const&, nie::io::ExtendedCalibrationData const& p);
void read(
    cv::FileNode const& node,
    nie::io::ExtendedCalibrationData& x,
    nie::io::ExtendedCalibrationData const& default_value = nie::io::ExtendedCalibrationData());

void write(cv::FileStorage& fs, std::string const&, nie::io::ValidatedCalibrationData const& p);
void read(
    cv::FileNode const& node,
    nie::io::ValidatedCalibrationData& x,
    nie::io::ValidatedCalibrationData const& default_value = nie::io::ValidatedCalibrationData());

}  // namespace cv

#endif  // NIE_FORMATS_CALIB3D_HELPER_CALIBRATION_IO_HPP
