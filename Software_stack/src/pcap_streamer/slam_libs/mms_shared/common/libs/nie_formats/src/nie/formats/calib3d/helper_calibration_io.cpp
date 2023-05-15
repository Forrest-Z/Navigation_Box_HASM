/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "helper_calibration_io.hpp"

// NOTE: Deviation from the google code guide because this interface is needed for OpenCV serialization.
void cv::write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedPinhole const& p) {
    fs << "{";
    fs << "id" << p.id;
    fs << "intrinsics" << p.intrinsics;
    fs << "cov" << p.cov;
    fs << "}";
}

void cv::read(
    cv::FileNode const& node, nie::io::CalibratedPinhole& x, nie::io::CalibratedPinhole const& default_value) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["id"] >> x.id;
        node["intrinsics"] >> x.intrinsics;
        node["cov"] >> x.cov;
    }
}

void cv::write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedParameters const& p) {
    fs << "{";
    fs << "focal_length" << p.focal_length;
    fs << "skew" << p.skew;
    fs << "distortion_radial" << p.distortion_radial;
    fs << "distortion_tangential" << p.distortion_tangential;
    fs << "distortion_thin_prism" << p.distortion_thin_prism;
    fs << "}";
}

void cv::read(
    cv::FileNode const& node, nie::io::CalibratedParameters& x, nie::io::CalibratedParameters const& default_value) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["focal_length"] >> x.focal_length;
        node["skew"] >> x.skew;
        node["distortion_radial"] >> x.distortion_radial;
        node["distortion_tangential"] >> x.distortion_tangential;
        node["distortion_thin_prism"] >> x.distortion_thin_prism;
    }
}

void cv::write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedBaseline const& p) {
    fs << "{";
    fs << "estimate" << p.estimate;
    fs << "cov";
    nie::io::WriteCov(fs, p.cov);
    fs << "}";
}

// The reason we do this is because reading INTO a triangularView is not that
// straightforward through the opencv API, after several attempts I ran into
// problems with either the 'transient' nature of triangularView(), and then if
// we bind to an lvalue, thus making it 'permanent', we find that TriangularView
// does not have a default constructor, which OpenCV needs when de-serializing.
void cv::read(
    cv::FileNode const& node, nie::io::CalibratedBaseline& x, nie::io::CalibratedBaseline const& default_value) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["estimate"] >> x.estimate;
        nie::io::ReadCov(node["cov"], x.cov);
    }
}

void cv::write(cv::FileStorage& fs, std::string const&, nie::io::CalibratedStereo const& p) {
    fs << "{";
    fs << "lens_left" << p.lens_left;
    fs << "lens_right" << p.lens_right;
    fs << "baseline" << p.baseline;
    fs << "}";
}

void cv::read(cv::FileNode const& node, nie::io::CalibratedStereo& x, nie::io::CalibratedStereo const& default_value) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["lens_left"] >> x.lens_left;
        node["lens_right"] >> x.lens_right;
        node["baseline"] >> x.baseline;
    }
}

void cv::write(cv::FileStorage& fs, std::string const&, nie::io::ExtendedCalibrationData const& p) {
    fs << "{";
    fs << "image_ids" << p.image_ids;
    fs << "image_points" << p.image_points;
    fs << "extrinsics" << p.extrinsics;
    fs << "residuals" << p.residuals;
    fs << "cov_extrinsics";
    fs << "[";
    for (auto const& cov_extrinsics : p.cov_extrinsics) {
        nie::io::WriteCov(fs, cov_extrinsics);
    }
    fs << "]";
    fs << "}";
}

void cv::read(
    cv::FileNode const& node,
    nie::io::ExtendedCalibrationData& x,
    nie::io::ExtendedCalibrationData const& default_value) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["image_ids"] >> x.image_ids;
        node["image_points"] >> x.image_points;
        node["extrinsics"] >> x.extrinsics;
        node["residuals"] >> x.residuals;
        x.cov_extrinsics.reserve(node["cov_extrinsics"].size());
        for (size_t i = 0; i < node["cov_extrinsics"].size(); ++i) {
            x.cov_extrinsics.emplace_back(default_value.cov_extrinsics.front().Zero());
            nie::io::ReadCov(node["cov_extrinsics"][i], x.cov_extrinsics.back());
        }
    }
}

void cv::write(cv::FileStorage& fs, std::string const&, nie::io::ValidatedCalibrationData const& p) {
    fs << "{";
    fs << "image_ids" << p.image_ids;
    fs << "image_points" << p.image_points;
    fs << "residuals" << p.residuals;
    fs << "}";
}

void cv::read(
    cv::FileNode const& node,
    nie::io::ValidatedCalibrationData& x,
    nie::io::ValidatedCalibrationData const& default_value) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["image_ids"] >> x.image_ids;
        node["image_points"] >> x.image_points;
        node["residuals"] >> x.residuals;
    }
}
