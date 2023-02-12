/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <boost/filesystem.hpp>
#include <nie/core/geometry/covariance.hpp>
#include <nie/cv/calib3d/distortion_model_factory.hpp>
#include <nie/cv/geometry/triangulation.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/calib3d/extended_mono_parameters.hpp>

#include "tool/io.hpp"

auto constexpr kPoseEstimates = "/estimates.pose";
auto constexpr kObjtEstimates = "/estimates.objt";
auto constexpr kObjtConstraints = "/constraints.objt";
auto constexpr kKpntConstraints = "/constraints.kpnt";

inline nie::DistortionModelParameters DistortionParametersFromCalibratedParameters(
    nie::io::CalibratedParameters const& parameters) {
    return nie::DistortionModelParameters{nie::ParameterFocalLengthToEnum(parameters.focal_length),
                                          nie::ParameterSkewToEnum(parameters.skew),
                                          nie::ParameterDistortionRadialToEnum(parameters.distortion_radial),
                                          nie::ParameterDistortionTangentialToEnum(parameters.distortion_tangential),
                                          nie::ParameterDistortionThinPrismToEnum(parameters.distortion_thin_prism)};
}

void ConvertFromExtendedMonoCalibration() {
    InPathExistsOrFatal();

    nie::io::ExtendedMonoParameters emp = nie::io::ExtendedMonoParameters::Read(FLAGS_in_paths);

    boost::filesystem::path out_directory_ba_graph(FLAGS_out_paths);
    if (!boost::filesystem::is_directory(out_directory_ba_graph)) {
        boost::filesystem::create_directories(out_directory_ba_graph);
    }

    {
        nie::io::PoseHeader pose_header{};
        pose_header.flags = nie::io::PoseHeader::Flag::kHasPoseInformationPerRecord;
        nie::io::SetNieAuthority(&pose_header);
        nie::io::PoseCollectionStreamWriter pcs(out_directory_ba_graph.string() + kPoseEstimates, pose_header);

        for (std::size_t i = 0; i < emp.extended_data().extrinsics.size(); ++i) {
            nie::io::PoseRecord pose_record{};
            pose_record.isometry = emp.extended_data().extrinsics[i];
            nie::CovarianceToInformation(emp.extended_data().cov_extrinsics[i], &pose_record.information);

            pose_record.id = static_cast<nie::io::PoseId>(i);

            // Don't use the device_id here
            pose_record.device_id = 0;
            pcs.Write(pose_record);
        }
    }

    // objt estimates
    {
        nie::io::ObjectHeader object_header{};
        object_header.flags = 0;

        nie::io::ObjectCollectionStreamWriter ocs(out_directory_ba_graph.string() + kObjtEstimates, object_header);

        for (std::size_t i = 0; i < emp.object_points().size(); ++i) {
            nie::io::ObjectRecord object_record{};
            object_record.position = emp.object_points()[i].cast<double>();
            object_record.id = static_cast<std::int32_t>(i);
            ocs.Write(object_record);
        }
    }

    std::vector<std::vector<Eigen::Vector2f>> image_points(emp.object_points().size());
    {
        nie::io::KeypointHeader keypoint_header{};
        keypoint_header.flags = nie::io::ObjectHeader::Flag::kHasInformation;
        nie::CovarianceToInformation(
            Eigen::Vector2d::Constant(emp.var_residuals()).asDiagonal().toDenseMatrix(), &keypoint_header.information);

        nie::io::KeypointCollectionStreamWriter kcs(
            out_directory_ba_graph.string() + kKpntConstraints, keypoint_header);

        for (std::size_t i = 0; i < emp.extended_data().image_points.size(); ++i) {
            for (std::size_t j = 0; j < emp.extended_data().image_points[i].size(); ++j) {
                nie::io::KeypointRecord keypoint_record{};
                keypoint_record.position = emp.extended_data().image_points[i][j];
                keypoint_record.pose_id = static_cast<std::int32_t>(i);
                keypoint_record.frame_id = 0;
                keypoint_record.object_id = static_cast<std::int32_t>(j);
                kcs.Write(keypoint_record);
                image_points[j].push_back(keypoint_record.position);
            }
        }
    }

    // objt constraints
    {
        // Not the correct ones as they are non-rectified.
        Eigen::Matrix3d K = nie::DistortionModelFactory::Create(
                                DistortionParametersFromCalibratedParameters(emp.distortion_parameters()),
                                emp.mono_parameters().lens().intrinsics)
                                ->K();

        nie::io::ObjectHeader object_header{};
        object_header.flags = nie::io::ObjectHeader::Flag::kHasInformationPerRecord;

        nie::io::ObjectCollectionStreamWriter ocs(out_directory_ba_graph.string() + kObjtConstraints, object_header);

        for (std::size_t i = 0; i < emp.object_points().size(); ++i) {
            nie::io::ObjectRecord object_record{};
            Eigen::Matrix3d cov;

            TriangulateNonLinear(
                emp.extended_data().extrinsics,
                image_points[i],
                emp.extended_data().cov_extrinsics,
                emp.var_residuals(),
                K,
                &object_record.position,
                &cov);

            nie::CovarianceToInformation(cov, &object_record.information);

            object_record.id = static_cast<std::int32_t>(i);
            ocs.Write(object_record);
        }
    }
}
