/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "mode_calibration.hpp"

#include <iomanip>
#include <iostream>
#include <thread>
#include <utility>

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <nie/core/container/mt_vector.hpp>
#include <nie/core/geometry/conversion.hpp>
#include <nie/core/work_pool.hpp>
#include <nie/cv/calib3d/camera_calibration.hpp>
#include <nie/cv/calib3d/pinhole_distortion_model.hpp>
#include <nie/formats/calib3d/calibrated_mono_parameters.hpp>
#include <nie/formats/calib3d/calibrated_stereo_parameters.hpp>
#include <nie/formats/calib3d/extended_mono_parameters.hpp>
#include <nie/formats/calib3d/extended_stereo_parameters.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common.hpp"

constexpr double kTargetWidth = 1280.0;
constexpr double kTargetHeight = 720.0;

double GetXfromPoint3d(const cv::Point3d& p) { return p.x; }

double GetZfromPoint3d(const cv::Point3d& p) { return -p.z; }

double GetYfromPoint3d(const cv::Point3d& p) { return p.y; }

struct Transform {
    std::function<double(const cv::Point3d&)> f_x;
    std::function<double(const cv::Point3d&)> f_y;
    double scale;
    cv::Point3d offset;

    cv::Point operator()(const cv::Point3d d) {
        return {static_cast<int>(f_x(d) * scale + offset.x), static_cast<int>(f_y(d) * scale + offset.y)};
    }
};

template <typename T>
void GetVecMinMaxCoordinates(
    const std::vector<cv::Point3_<T>>& v, Transform const* const t, cv::Point3d& min, cv::Point3d& max) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        min.x = std::min(min.x, t->f_x(v[i]));
        min.y = std::min(min.y, t->f_y(v[i]));

        max.x = std::max(max.x, t->f_x(v[i]));
        max.y = std::max(max.y, t->f_y(v[i]));
    }
}

void CalculateResolution(
    double const target_width,
    double const target_height,
    std::vector<cv::Point3f> const& object_points,
    std::vector<cv::Point3d> const& translations,
    Transform* t) {
    cv::Point3d min{}, max{};

    min.x = t->f_x(object_points[0]);
    min.y = t->f_y(object_points[0]);
    max.x = t->f_x(object_points[0]);
    max.y = t->f_y(object_points[0]);

    GetVecMinMaxCoordinates(object_points, t, min, max);
    GetVecMinMaxCoordinates(translations, t, min, max);

    double width = max.x - min.x;
    double height = max.y - min.y;
    double ratio = width / height;
    double target_ratio = target_width / target_height;
    double target_margin = 20.0;

    if (ratio >= target_ratio) {
        // width dominant
        t->scale = (target_width - target_margin * 2.0) / width;
        t->offset.x = -min.x * t->scale + target_margin;
        t->offset.y = -min.y * t->scale + (target_height - (max.y - min.y) * t->scale) / 2;
    } else {
        // height dominant
        t->scale = (target_height - target_margin * 2.0) / height;
        t->offset.x = -min.x * t->scale + (target_width - (max.x - min.x) * t->scale) / 2;
        t->offset.y = -min.y * t->scale + target_margin;
    }
}

void DrawCheckerboard(
    cv::Mat d,
    std::vector<cv::Point3f> const& object_points,
    Transform& t,
    cv::Scalar circle_color,  //(255, 255, 255)
    cv::Scalar text_color     //(255, 0, 0)
) {
    for (std::size_t i = 0; i < object_points.size(); ++i) {
        cv::Point p = t(object_points[i]);
        cv::circle(d, p, 2, circle_color, 2);

        if (i == 0 || i == object_points.size() - 1) {
            cv::putText(
                d,
                std::to_string((int)(object_points[i].x)) + ", " + std::to_string((int)(object_points[i].y)),
                p,
                cv::FONT_HERSHEY_PLAIN,
                1.0,
                text_color);
        }
    }
}

cv::Point3d TransformPoint(cv::Point3d const& t, cv::Matx33d const& R, cv::Point3d const& p) { return R * p + t; }

void DrawCameraPositions(
    cv::Mat d,
    const std::vector<std::string>& ids,
    const std::vector<cv::Point3d>& translations,
    const std::vector<cv::Matx33d>& rotations,
    Transform& t,
    const cv::Scalar& color,
    const double square_size) {
    for (std::size_t i = 0; i < translations.size(); ++i) {
        cv::Point p0 = t(translations[i]);

        cv::Point3d x_3d = TransformPoint(translations[i], rotations[i], {square_size, 0.0, 0.0});
        cv::Point3d y_3d = TransformPoint(translations[i], rotations[i], {0.0, square_size, 0.0});
        cv::Point3d z_3d = TransformPoint(translations[i], rotations[i], {0.0, 0.0, square_size});

        cv::Point px = t(x_3d);
        cv::Point py = t(y_3d);
        cv::Point pz = t(z_3d);

        cv::line(d, p0, px, cv::Scalar(0, 0, 255));
        cv::line(d, p0, py, cv::Scalar(0, 255, 0));
        cv::line(d, p0, pz, cv::Scalar(255, 0, 0));
        cv::circle(d, p0, 2, color, 2);
    }

    for (std::size_t i = 0; i < translations.size(); ++i) {
        if (i % 10 == 0) {
            cv::Point p0 = t(translations[i]) + cv::Point{5, 5};

            boost::filesystem::path path(ids[i]);
            cv::putText(d, path.filename().stem().string(), p0, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
        }
    }
}

void ToOpenCv(
    const std::vector<nie::Isometry3qd>& extrinsics,
    std::vector<cv::Point3d>* translations,
    std::vector<cv::Matx33d>* rotations) {
    translations->resize(extrinsics.size());
    rotations->resize(extrinsics.size());
    for (std::size_t i = 0; i < extrinsics.size(); ++i) {
        (*translations)[i] = nie::ConvertPoint(extrinsics[i].translation());
        (*rotations)[i] = nie::ConvertMat(extrinsics[i].rotation().toRotationMatrix().eval());
    }
}

void DrawExtrinsics(
    const std::string& path_out,
    const std::vector<Eigen::Vector3f>& object_points,
    const std::vector<std::string>& ids,  // abuse as path
    const std::vector<nie::Isometry3qd>& extrinsics,
    float const& square_size,
    std::function<double(const cv::Point3d&)> map_y) {
    Transform t{GetXfromPoint3d, map_y, 1.0, {}};

    std::vector<cv::Point3f> board = nie::ConvertPoints(object_points);
    std::vector<cv::Point3d> translations;
    std::vector<cv::Matx33d> rotations;
    ToOpenCv(extrinsics, &translations, &rotations);

    CalculateResolution(kTargetWidth, kTargetHeight, board, translations, &t);

    cv::Mat d(kTargetHeight, kTargetWidth, CV_8UC3, cv::Scalar(0));

    DrawCheckerboard(d, board, t, cv::Scalar(255, 255, 255), cv::Scalar(255, 0, 0));

    DrawCameraPositions(d, ids, translations, rotations, t, cv::Scalar(0, 255, 255), square_size);

    cv::imwrite(path_out, d);
}

void DrawExtrinsicsYZ(
    std::string const& path_out,
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<std::string> const& ids,  // abuse as path
    std::vector<nie::Isometry3qd> const& extrinsics,
    float const& square_size) {
    DrawExtrinsics(path_out + ".y_view.png", object_points, ids, extrinsics, square_size, GetYfromPoint3d);
    DrawExtrinsics(path_out + ".z_view.png", object_points, ids, extrinsics, square_size, GetZfromPoint3d);
}

void DrawExtrinsicsStereo(
    std::string const& path_out,
    std::vector<Eigen::Vector3f> const& object_points,
    std::size_t const pair_count,
    std::vector<std::string> const& ids_left,   // abuse as path
    std::vector<std::string> const& ids_right,  // abuse as path
    const std::vector<nie::Isometry3qd>& extrinsics_left,
    const std::vector<nie::Isometry3qd>& extrinsics_right,
    float const square_size,
    std::function<double(const cv::Point3d&)> const map_y) {
    Transform t;
    t.f_x = GetXfromPoint3d;
    t.f_y = map_y;

    std::vector<cv::Point3f> board = nie::ConvertPoints(object_points);
    std::vector<cv::Point3d> translations_left;
    std::vector<cv::Point3d> translations_right;
    std::vector<cv::Matx33d> rotations_left;
    std::vector<cv::Matx33d> rotations_right;
    ToOpenCv(extrinsics_left, &translations_left, &rotations_left);
    ToOpenCv(extrinsics_right, &translations_right, &rotations_right);

    {
        auto translations = translations_left;
        translations.insert(translations.end(), translations_right.begin(), translations_right.end());
        CalculateResolution(kTargetWidth, kTargetHeight, board, translations, &t);
    }

    cv::Mat d(kTargetHeight, kTargetWidth, CV_8UC3, cv::Scalar(0));

    DrawCheckerboard(d, board, t, cv::Scalar(255, 255, 255), cv::Scalar(255, 0, 0));

    {
        auto ids_pairs = std::vector<std::string>(ids_left.begin(), ids_left.begin() + pair_count);
        auto positions_pairs =
            std::vector<cv::Point3d>(translations_left.begin(), translations_left.begin() + pair_count);
        auto orientation_pairs = std::vector<cv::Matx33d>(rotations_left.begin(), rotations_left.begin() + pair_count);

        DrawCameraPositions(d, ids_pairs, positions_pairs, orientation_pairs, t, cv::Scalar(0, 255, 255), square_size);

        auto ids_l = std::vector<std::string>(ids_left.begin() + pair_count, ids_left.end());
        auto pos_l = std::vector<cv::Point3d>(translations_left.begin() + pair_count, translations_left.end());
        auto ori_l = std::vector<cv::Matx33d>(rotations_left.begin() + pair_count, rotations_left.end());

        DrawCameraPositions(d, ids_l, pos_l, ori_l, t, cv::Scalar(255, 255, 0), square_size);
    }

    DrawCameraPositions(d, ids_right, translations_right, rotations_right, t, cv::Scalar(255, 0, 255), square_size);

    cv::imwrite(path_out, d);
}

void DrawExtrinsicsStereoYZ(
    std::string const& path_out,
    std::vector<Eigen::Vector3f> const& object_points,
    std::size_t const pair_count,
    std::vector<std::string> const& ids_left,   // abuse as path
    std::vector<std::string> const& ids_right,  // abuse as path
    const std::vector<nie::Isometry3qd>& extrinsics_left,
    const std::vector<nie::Isometry3qd>& extrinsics_right,
    float const square_size) {
    DrawExtrinsicsStereo(
        path_out + ".y_view.png",
        object_points,
        pair_count,
        ids_left,
        ids_right,
        extrinsics_left,
        extrinsics_right,
        square_size,
        GetYfromPoint3d);
    DrawExtrinsicsStereo(
        path_out + ".z_view.png",
        object_points,
        pair_count,
        ids_left,
        ids_right,
        extrinsics_left,
        extrinsics_right,
        square_size,
        GetZfromPoint3d);
}

// TODO(jbr) Read from parameter file or something
nie::DistortionModelParameters GetDistortionParameters() {
    nie::DistortionModelParameters parameters;
    parameters.focal_length = nie::ParameterFocalLength::XY_F;
    parameters.skew = nie::ParameterSkew::NO_SKEW;
    parameters.distortion_radial = nie::ParameterDistortionRadial::K3;
    parameters.distortion_tangential = nie::ParameterDistortionTangential::P2;
    parameters.distortion_thin_prism = nie::ParameterDistortionThinPrism::NO_DISTORTION;

    return parameters;
}

std::vector<Eigen::Vector3f> GenerateObjectPoints(cv::Size const& pattern_size, float const& square_size) {
    std::vector<Eigen::Vector3f> object_points(pattern_size.width * pattern_size.height);
    std::generate(object_points.begin(), object_points.end(), [&pattern_size, &square_size, n = 0]() mutable {
        // In opencv, the order of the image points in x and y goes from
        // high to low for the detected corners.
        Eigen::Vector3f point{// We want image x and world to be aligned.
                              static_cast<float>(n % pattern_size.width) * square_size,
                              // We want image y and world y to be flipped. That is, image
                              // down and world up.
                              static_cast<float>(n / pattern_size.width) * square_size,
                              0.0f};

        n++;

        return point;
    });

    return object_points;
}

void RunModeCalibration(
    std::string const& camera_id,
    std::string const& in_path_images,
    std::string const& out_path_intrinsics,
    std::string const& out_path_extended_calibration_data,
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    float const& square_size,
    bool write_corner_images) {
    nie::mt::MtVector<ImageCorners> ids_and_image_points;

    GetCornerData(in_path_images, pattern_size, write_corner_images, &ids_and_image_points);

    VLOG(2) << "Processed all images...";

    if (ids_and_image_points.size() < 2) {
        VLOG(2) << "Not enough results to calibrate.";
        return;
    }

    VLOG(2) << "Starting calibration...";

    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

    std::vector<Eigen::Vector3f> object_points = GenerateObjectPoints(pattern_size, square_size);

    nie::DistortionModelParameters parameters = GetDistortionParameters();
    std::vector<std::string> ids(ids_and_image_points.size());
    std::vector<std::vector<Eigen::Vector2f>> image_points(ids_and_image_points.size());
    std::vector<double> intrinsics;
    std::vector<nie::Isometry3qd> extrinsics;
    std::vector<std::vector<Eigen::Vector2f>> residuals;
    Eigen::MatrixXd cov_intrinsics;
    std::vector<Eigen::Matrix<double, 6, 6>> cov_extrinsics;
    double var_residuals;

    for (std::size_t i = 0; i < ids_and_image_points.size(); ++i) {
        ids[i] = std::move(ids_and_image_points.vector()[i].first);
        image_points[i] = std::move(ids_and_image_points.vector()[i].second);
    }

    nie::EstimateCameraParametersCeres(
        parameters,
        object_points,
        image_points,
        &intrinsics,
        &extrinsics,
        &residuals,
        &cov_intrinsics,
        &cov_extrinsics,
        &var_residuals);

    DrawExtrinsicsYZ(out_path_intrinsics, object_points, ids, extrinsics, square_size);

    nie::io::CalibratedMonoParameters::Write(
        out_path_intrinsics,
        image_size,
        CalibratedParametersFromDistortionParameters(parameters),
        camera_id,
        intrinsics,
        cov_intrinsics);

    VLOG(2) << "Successfully wrote intrinsics to " << out_path_intrinsics << ".";

    nie::io::ExtendedMonoParameters::Write(
        out_path_extended_calibration_data,
        image_size,
        CalibratedParametersFromDistortionParameters(parameters),
        pattern_size,
        square_size,
        object_points,
        var_residuals,
        camera_id,
        ids,
        image_points,
        intrinsics,
        extrinsics,
        residuals,
        cov_intrinsics,
        cov_extrinsics);

    VLOG(2) << "Successfully wrote extended calibration data to " << out_path_extended_calibration_data << ".";

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - before).count();

    VLOG(3) << "Calibrated camera using " << ids_and_image_points.size() << " images in ms: " << ms;
}

void RunModeCalibration(
    std::string const& camera_id_left,
    std::string const& camera_id_right,
    std::string const& in_path_images,
    std::string const& path_relative_left,
    std::string const& path_relative_right,
    std::string const& out_path_intrinsics,
    std::string const& out_path_extended_calibration_data,
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    float const& square_size,
    bool write_corner_images) {
    nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>> ids_and_image_points_pairs;
    nie::mt::MtVector<ImageCorners> ids_and_image_points_left;
    nie::mt::MtVector<ImageCorners> ids_and_image_points_right;

    GetCornerData(
        in_path_images,
        path_relative_left,
        path_relative_right,
        pattern_size,
        write_corner_images,
        &ids_and_image_points_pairs,
        &ids_and_image_points_left,
        &ids_and_image_points_right);

    VLOG(2) << "Processed all images...";

    if (ids_and_image_points_pairs.size() < 2) {
        VLOG(2) << "Not enough results to calibrate.";
        return;
    }

    VLOG(2) << "Starting calibration...";

    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

    std::vector<std::vector<Eigen::Vector2f>> image_points_left;
    std::vector<std::vector<Eigen::Vector2f>> image_points_right;
    std::vector<std::string> ids_left;
    std::vector<std::string> ids_right;

    MoveStereoInput(
        &ids_and_image_points_pairs,
        &ids_and_image_points_left,
        &ids_and_image_points_right,
        &image_points_left,
        &image_points_right,
        &ids_left,
        &ids_right);

    std::vector<Eigen::Vector3f> object_points = GenerateObjectPoints(pattern_size, square_size);
    nie::DistortionModelParameters parameters = GetDistortionParameters();
    std::size_t pair_count = ids_and_image_points_pairs.size();
    std::vector<double> intrinsics_left;
    std::vector<double> intrinsics_right;

    std::vector<nie::Isometry3qd> extrinsics_left;
    std::vector<nie::Isometry3qd> extrinsics_right;

    nie::Isometry3qd baseline;

    std::vector<std::vector<Eigen::Vector2f>> residuals_left;
    std::vector<std::vector<Eigen::Vector2f>> residuals_right;
    Eigen::MatrixXd cov_intrinsics_left;
    Eigen::MatrixXd cov_intrinsics_right;
    std::vector<Eigen::Matrix<double, 6, 6>> cov_extrinsics_left;
    std::vector<Eigen::Matrix<double, 6, 6>> cov_extrinsics_right;
    Eigen::Matrix<double, 6, 6> cov_baseline;
    double var_residuals;

    nie::EstimateStereoParametersCeres(
        parameters,
        object_points,
        image_points_left,
        image_points_right,
        pair_count,
        &intrinsics_left,
        &intrinsics_right,
        &extrinsics_left,
        &extrinsics_right,
        &baseline,
        &residuals_left,
        &residuals_right,
        &cov_intrinsics_left,
        &cov_intrinsics_right,
        &cov_extrinsics_left,
        &cov_extrinsics_right,
        &cov_baseline,
        &var_residuals);

    // extract pairs from left
    std::vector<std::string> ids_pairs(ids_left.begin(), ids_left.begin() + pair_count);
    std::vector<nie::Isometry3qd> extrinsics_pairs(extrinsics_left.begin(), extrinsics_left.begin() + pair_count);
    DrawExtrinsicsYZ(out_path_intrinsics + ".pairs", object_points, ids_pairs, extrinsics_pairs, square_size);

    // create left_only vectors
    std::vector<std::string> ids_left_only(ids_left.begin() + pair_count, ids_left.end());
    std::vector<nie::Isometry3qd> extrinsics_left_only(extrinsics_left.begin() + pair_count, extrinsics_left.end());
    DrawExtrinsicsYZ(out_path_intrinsics + ".left", object_points, ids_left_only, extrinsics_left_only, square_size);

    // same as right_only
    DrawExtrinsicsYZ(out_path_intrinsics + ".right", object_points, ids_right, extrinsics_right, square_size);

    // all of it
    DrawExtrinsicsStereoYZ(
        out_path_intrinsics + ".stereo",
        object_points,
        pair_count,
        ids_left,
        ids_right,
        extrinsics_left,
        extrinsics_right,
        square_size);

    nie::io::CalibratedStereoParameters::Write(
        out_path_intrinsics,
        image_size,
        CalibratedParametersFromDistortionParameters(parameters),
        camera_id_left,
        intrinsics_left,
        cov_intrinsics_left,
        camera_id_right,
        intrinsics_right,
        cov_intrinsics_right,
        baseline,
        cov_baseline);

    VLOG(2) << "Sucessfully wrote intrinsics to " << out_path_intrinsics << ".";

    nie::io::ExtendedStereoParameters::Write(
        out_path_extended_calibration_data,
        image_size,
        CalibratedParametersFromDistortionParameters(parameters),
        pattern_size,
        square_size,
        object_points,
        var_residuals,
        camera_id_left,
        camera_id_right,
        ids_left,
        ids_right,
        image_points_left,
        image_points_right,
        intrinsics_left,
        intrinsics_right,
        extrinsics_left,
        extrinsics_right,
        residuals_left,
        residuals_right,
        cov_intrinsics_left,
        cov_intrinsics_right,
        cov_extrinsics_left,
        cov_extrinsics_right,
        pair_count,
        baseline,
        cov_baseline);

    VLOG(2) << "Successfully wrote extended calibration data to " << out_path_extended_calibration_data << ".";

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - before).count();

    VLOG(3) << "Used: " << pair_count << " camera pairs for the baseline.";
    VLOG(3) << "Used: " << extrinsics_left.size() << " images for the left camera parameters.";
    VLOG(3) << "Used: " << extrinsics_right.size() << " images for the right camera parameters.";
    VLOG(3) << "Calibrated cameras in ms: " << ms;
}
