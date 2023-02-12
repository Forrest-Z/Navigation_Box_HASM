/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "mode_validation.hpp"

#include <unordered_map>

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <nie/formats/calib3d/extended_mono_parameters.hpp>
#include <nie/formats/calib3d/extended_stereo_parameters.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>
#include <nie/formats/calib3d/validated_mono_parameters.hpp>
#include <nie/formats/calib3d/validated_stereo_parameters.hpp>
#include <opencv2/calib3d.hpp>

#include "common.hpp"

Eigen::Matrix4d GetP(Eigen::Matrix3d const& K, nie::Isometry3qd const& extrinsics) {
    Eigen::Matrix4d P = extrinsics.Inversed().ToTransform().matrix();
    P.block<3, 3>(0, 0).applyOnTheLeft(K);
    P.block<3, 1>(0, 3).applyOnTheLeft(K);
    return P;
}

void GetProjectionMatrices(
    nie::io::ExtendedMonoParameters const& extended,
    nie::io::RectifiedCameraParameters const& intrinsics,
    std::unordered_map<std::string, Eigen::Matrix4d>* p_extrinsics) {
    Eigen::Matrix3d K = intrinsics.K;
    std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics = *p_extrinsics;

    for (std::size_t i = 0; i < extended.extended_data().extrinsics.size(); ++i) {
        std::string id = boost::filesystem::path(extended.extended_data().image_ids[i]).stem().string();
        nie::Isometry3qd const& isometry = extended.extended_data().extrinsics[i];
        extrinsics.insert(std::make_pair(id, GetP(K, isometry)));
    }
}

void GetProjectionMatrices(
    nie::io::ExtendedStereoParameters const& extended,
    nie::io::RectifiedCameraParameters const& intrinsics,
    std::unordered_map<std::string, Eigen::Matrix4d>* p_extrinsics_left,
    std::unordered_map<std::string, Eigen::Matrix4d>* p_extrinsics_right) {
    Eigen::Matrix3d K = intrinsics.K;
    std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_left = *p_extrinsics_left;
    std::unordered_map<std::string, Eigen::Matrix4d>& extrinsics_right = *p_extrinsics_right;

    Eigen::Matrix3d K_c;
    Eigen::Vector3d cw_baseline_t_c;
    Eigen::Matrix3d cw_R_c_left;
    Eigen::Matrix3d cw_R_c_right;

    GetRectifiedParametersStereoCw(
        extended.stereo_parameters().baseline().estimate.Inversed(), &cw_baseline_t_c, &cw_R_c_left, &cw_R_c_right);

    nie::Isometry3qd isometry_c_left = nie::Isometry3qd::FromRotation(Eigen::Quaterniond{cw_R_c_left.transpose()});
    nie::Isometry3qd isometry_c_right = nie::Isometry3qd::FromRotation(Eigen::Quaterniond{cw_R_c_right.transpose()});

    // cw_baseline_t_c equals -intrinsics.baseline_translation()
    nie::Isometry3qd isometry_relative_left =
        isometry_c_right * nie::Isometry3qd::FromTranslation(-intrinsics.frames[1].baseline.translation());
    nie::Isometry3qd isometry_relative_right = isometry_c_left * intrinsics.frames[1].baseline;

    // ALL LEFT (from pairs and left only)
    // THERE ARE NO RIGHT EXTRINSICS SO THEY ARE GENERATED USING THE BASELINE
    for (std::size_t i = 0; i < extended.extended_data_left().extrinsics.size(); ++i) {
        std::string id = boost::filesystem::path(extended.extended_data_left().image_ids[i]).stem().string();
        // Left
        nie::Isometry3qd const& isometry_left = extended.extended_data_left().extrinsics[i];
        extrinsics_left.emplace(id, GetP(K, isometry_left * isometry_c_left));

        // Right
        extrinsics_right.emplace(id, GetP(K, isometry_left * isometry_relative_right));
    }

    // Below the special cases because of the pair count
    const std::size_t pair_count = extended.pair_count();

    // RIGHT ONLY
    // THERE ARE NO LEFT EXTRINSICS SO THEY ARE GENERATED USING THE BASELINE
    for (std::size_t i = 0; i < extended.extended_data_right().extrinsics.size(); ++i) {
        std::string id =
            boost::filesystem::path(extended.extended_data_right().image_ids[i + pair_count]).stem().string();

        // Right
        nie::Isometry3qd const& isometry_right = extended.extended_data_right().extrinsics[i];
        extrinsics_right.emplace(id, GetP(K, isometry_right * isometry_c_right));

        // Left
        extrinsics_left.emplace(id, GetP(K, isometry_right * isometry_relative_left));
    }
}

void GetEpipolarErrors(
    nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>> const& ids_and_image_points_pairs,
    cv::Size const& pattern_size,
    std::vector<std::vector<double>>* p_epipolar_errors) {
    std::size_t const pair_count = ids_and_image_points_pairs.size();
    std::size_t const point_count = pattern_size.area();

    std::vector<std::vector<double>>& epipolar_errors = *p_epipolar_errors;
    epipolar_errors.resize(pair_count);

    for (std::size_t i = 0; i < pair_count; ++i) {
        std::vector<Eigen::Vector2f> const& points_left = ids_and_image_points_pairs.vector()[i].first.second;
        std::vector<Eigen::Vector2f> const& points_right = ids_and_image_points_pairs.vector()[i].second.second;

        epipolar_errors[i].resize(point_count);

        for (std::size_t j = 0; j < point_count; ++j) {
            epipolar_errors[i][j] = points_left[j].y() - points_right[j].y();
        }
    }
}

void GetProjectionErrors(
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<Eigen::Vector2f> const& image_points,
    Eigen::Matrix4d const& P,
    std::vector<Eigen::Vector2f>* p_projection_errors) {
    std::vector<Eigen::Vector2f>& projection_errors = *p_projection_errors;
    projection_errors.resize(object_points.size());

    for (std::size_t i = 0; i < object_points.size(); ++i) {
        Eigen::Vector4d u4 = P * object_points[i].cast<double>().homogeneous();
        projection_errors[i] = u4.head<3>().hnormalized().cast<float>() - image_points[i];
    }
}

void TryGetProjectionErrors(
    std::vector<Eigen::Vector3f> const& object_points,
    ImageCorners const& data,
    std::unordered_map<std::string, Eigen::Matrix4d> const& extrinsics,
    std::vector<Eigen::Vector2f>* projection_errors) {
    std::vector<Eigen::Vector2f> const& image_points = data.second;
    std::string id = boost::filesystem::path(data.first).stem().string();

    // There isn't always a camera available. The calibration run may not have found any corners in the original
    // image while after rectification there may all of a sudden be corners. These will remain empty lists.
    auto it = extrinsics.find(id);
    if (it != extrinsics.end()) {
        GetProjectionErrors(object_points, image_points, it->second, projection_errors);
    }
}

void GetProjectionErrors(
    nie::mt::MtVector<ImageCorners> const& ids_and_image_points,
    nie::io::ExtendedMonoParameters const& extended,
    std::unordered_map<std::string, Eigen::Matrix4d> const& extrinsics,
    std::vector<std::vector<Eigen::Vector2f>>* p_projection_errors) {
    std::vector<std::vector<Eigen::Vector2f>>& projection_errors = *p_projection_errors;
    std::size_t const count = ids_and_image_points.size();
    projection_errors.resize(count);

    for (std::size_t i = 0; i < count; ++i) {
        TryGetProjectionErrors(
            extended.object_points(), ids_and_image_points.vector()[i], extrinsics, &projection_errors[i]);
    }
}

void GetProjectionErrors(
    nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>> const& ids_and_image_points_pairs,
    nie::mt::MtVector<ImageCorners> const& ids_and_image_points_left,
    nie::mt::MtVector<ImageCorners> const& ids_and_image_points_right,
    nie::io::ExtendedStereoParameters const& extended,
    std::unordered_map<std::string, Eigen::Matrix4d> const& extrinsics_left,
    std::unordered_map<std::string, Eigen::Matrix4d> const& extrinsics_right,
    std::vector<std::vector<Eigen::Vector2f>>* p_projection_errors_left,
    std::vector<std::vector<Eigen::Vector2f>>* p_projection_errors_right) {
    std::vector<std::vector<Eigen::Vector2f>>& projection_errors_left = *p_projection_errors_left;
    std::vector<std::vector<Eigen::Vector2f>>& projection_errors_right = *p_projection_errors_right;
    projection_errors_left.resize(ids_and_image_points_pairs.size() + ids_and_image_points_left.size());
    projection_errors_right.resize(ids_and_image_points_pairs.size() + ids_and_image_points_right.size());
    std::size_t pair_count = ids_and_image_points_pairs.size();
    std::size_t left_count = ids_and_image_points_left.size();
    std::size_t right_count = ids_and_image_points_right.size();

    for (std::size_t i = 0; i < pair_count; ++i) {
        TryGetProjectionErrors(
            extended.object_points(),
            ids_and_image_points_pairs.vector()[i].first,
            extrinsics_left,
            &projection_errors_left[i]);

        TryGetProjectionErrors(
            extended.object_points(),
            ids_and_image_points_pairs.vector()[i].second,
            extrinsics_right,
            &projection_errors_right[i]);
    }

    for (std::size_t i = 0; i < left_count; ++i) {
        TryGetProjectionErrors(
            extended.object_points(),
            ids_and_image_points_left.vector()[i],
            extrinsics_left,
            &projection_errors_left[i + pair_count]);
    }

    for (std::size_t i = 0; i < right_count; ++i) {
        TryGetProjectionErrors(
            extended.object_points(),
            ids_and_image_points_right.vector()[i],
            extrinsics_right,
            &projection_errors_right[i + pair_count]);
    }
}

void RunModeValidationMono(
    std::string const& in_path_images,
    std::string const& in_path_intrinsics,
    std::string const& in_path_extended,
    std::string const& out_path_validated,
    bool write_corner_images) {
    const auto extended = nie::io::ExtendedMonoParameters::Read(in_path_extended);

    nie::mt::MtVector<ImageCorners> ids_and_image_points;
    cv::Size pattern_size = extended.pattern_size();

    GetCornerData(in_path_images, pattern_size, write_corner_images, &ids_and_image_points);

    VLOG(2) << "Processed all images...";

    const auto intrinsics = nie::io::RectifiedCameraParameters::Read(in_path_intrinsics);
    std::unordered_map<std::string, Eigen::Matrix4d> extrinsics;
    GetProjectionMatrices(extended, intrinsics, &extrinsics);

    std::vector<std::vector<Eigen::Vector2f>> projection_errors;
    GetProjectionErrors(ids_and_image_points, extended, extrinsics, &projection_errors);

    std::vector<std::string> ids(ids_and_image_points.size());
    std::vector<std::vector<Eigen::Vector2f>> image_points(ids_and_image_points.size());
    for (std::size_t i = 0; i < ids_and_image_points.size(); ++i) {
        ids[i] = std::move(ids_and_image_points.vector()[i].first);
        image_points[i] = std::move(ids_and_image_points.vector()[i].second);
    }

    nie::io::ValidatedMonoParameters::Write(
        out_path_validated, extended.image_size(), pattern_size, ids, image_points, projection_errors);
}

void RunModeValidationStereo(
    std::string const& in_path_images,
    std::string const& path_relative_left,
    std::string const& path_relative_right,
    std::string const& in_path_intrinsics,
    std::string const& in_path_extended,
    std::string const& out_path_validated,
    bool write_corner_images) {
    const auto extended = nie::io::ExtendedStereoParameters::Read(in_path_extended);

    nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>> ids_and_image_points_pairs;
    nie::mt::MtVector<ImageCorners> ids_and_image_points_left;
    nie::mt::MtVector<ImageCorners> ids_and_image_points_right;
    cv::Size pattern_size = extended.pattern_size();

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

    std::vector<std::vector<double>> epipolar_errors;
    GetEpipolarErrors(ids_and_image_points_pairs, pattern_size, &epipolar_errors);

    const auto intrinsics = nie::io::RectifiedCameraParameters::Read(in_path_intrinsics);
    std::unordered_map<std::string, Eigen::Matrix4d> extrinsics_left;
    std::unordered_map<std::string, Eigen::Matrix4d> extrinsics_right;
    GetProjectionMatrices(extended, intrinsics, &extrinsics_left, &extrinsics_right);

    std::vector<std::vector<Eigen::Vector2f>> projection_errors_left;
    std::vector<std::vector<Eigen::Vector2f>> projection_errors_right;
    GetProjectionErrors(
        ids_and_image_points_pairs,
        ids_and_image_points_left,
        ids_and_image_points_right,
        extended,
        extrinsics_left,
        extrinsics_right,
        &projection_errors_left,
        &projection_errors_right);

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

    nie::io::ValidatedStereoParameters::Write(
        out_path_validated,
        extended.image_size(),
        pattern_size,
        ids_left,
        ids_right,
        image_points_left,
        image_points_right,
        projection_errors_left,
        projection_errors_right,
        epipolar_errors,
        ids_and_image_points_pairs.size());
}

void RunModeValidation(
    std::string const& in_path_images,
    std::string const& path_relative_left,
    std::string const& path_relative_right,
    std::string const& in_path_intrinsics,
    std::string const& in_path_extended,
    std::string const& out_path_validated,
    bool write_corner_images,
    bool stereo) {
    if (!stereo) {
        RunModeValidationMono(
            in_path_images, in_path_intrinsics, in_path_extended, out_path_validated, write_corner_images);
    } else {
        RunModeValidationStereo(
            in_path_images,
            path_relative_left,
            path_relative_right,
            in_path_intrinsics,
            in_path_extended,
            out_path_validated,
            write_corner_images);
    }
}
