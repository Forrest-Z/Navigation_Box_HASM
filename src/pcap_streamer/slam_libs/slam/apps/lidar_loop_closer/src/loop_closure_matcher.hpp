/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <unordered_set>

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <nie/core/container/mt_vector.hpp>
#include <nie/core/hash.hpp>
#include <nie/core/time.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/lidar/cloud.hpp>
#include <nie/lidar/cloud_filter.hpp>
#include <nie/lidar/cloud_matcher.hpp>
#include <nie/lidar/helper_cloud.hpp>
#include <nie/lidar/io/las_reader.hpp>

#include "covariance_filter.hpp"
#include "helper_transformation.hpp"
#include "registration_result.hpp"

struct Candidate {
    nie::io::PoseId id_begin;
    nie::io::PoseId id_end;
    std::chrono::weeks gps_week_begin;
    std::chrono::weeks gps_week_end;
    std::string path_begin;
    std::string path_end;
    nie::PoseBbox obounds_begin;
    nie::PoseBbox obounds_end;

    // The transformation bringing points from system of end cloud to the system of begin cloud
    nie::Isometry3qd T_begin_end;
};
using Candidates = std::vector<Candidate>;

template <typename PointT>
class LoopClosureMatcher {
public:
    LoopClosureMatcher(
            typename nie::CloudFilter<PointT>::Parameters const& filter_parameters,
            std::vector<typename nie::CloudMatcher<PointT>::Parameters> const& match_parameters,
            std::vector<nie::io::PoseRecord> const& trace,
            bool dry_run,
            nie::mt::MtVector<RegistrationResult>* registration_results,
            nie::DebugIo* debug_io)
        : filter_parameters_(filter_parameters),
          matcher_parameters_(match_parameters),
          trace_(trace),
          dry_run_(dry_run),
          registration_results_(registration_results),
          debug_io_(debug_io) {}

    void operator()(std::unique_ptr<Candidate> const& candidate) {
        // Convenience variables
        nie::io::PoseId const& pose_id_a = candidate->id_begin;
        nie::io::PoseId const& pose_id_b = candidate->id_end;
        std::string const& file_a = candidate->path_begin;
        std::string const& file_b = candidate->path_end;
        std::pair<nie::io::PoseId, nie::io::PoseId> const ids = {pose_id_a, pose_id_b};

        LOG(INFO) << "Processing match candidate with ids: " << pose_id_a << ", " << pose_id_b
                  << " representing las files " << boost::filesystem::path(file_a).filename().string() << ", "
                  << boost::filesystem::path(file_b).filename().string();

        // Read the clouds
        nie::Cloud<PointT> cloud_a = nie::io::ReadLas<PointT>(file_a, candidate->gps_week_begin);
        nie::Cloud<PointT> cloud_b = nie::io::ReadLas<PointT>(file_b, candidate->gps_week_end);
        debug_io_->SetClouds(ids, cloud_a, cloud_b, trace_);

        if (!dry_run_) {
            nie::PoseBbox bounds_a = candidate->obounds_begin;
            nie::PoseBbox bounds_b = candidate->obounds_end;
            debug_io_->SetOrientedBoundingBoxes(ids, {bounds_a, bounds_b});

            // Increase the bounding boxes to incorporate a possible error
            bounds_a.bbox().Inflate(10.0f);
            bounds_b.bbox().Inflate(10.0f);

            // Only keep the overlapping region (intersection of oriented bounding boxes) between candidate point clouds
            {
                // Transformation to bring the local axis-aligned points of cloud b to the local oriented bounding box a
                auto const T_ab = nie::Isometry3qd::FromRotation(bounds_a.origin().rotation().conjugate()) *
                                  candidate->T_begin_end;
                Filter(T_ab, bounds_a.bbox(), &cloud_b);
            }
            {
                // Transformation to bring the local axis-aligned points of cloud a to the local oriented bounding box b
                auto const T_ba = nie::Isometry3qd::FromRotation(bounds_b.origin().rotation().conjugate()) *
                                  candidate->T_begin_end.Inversed();
                Filter(T_ba, bounds_b.bbox(), &cloud_a);
            }

            // Filter points based on covariances
            if (!cloud_a.point_cloud().empty() && !cloud_b.point_cloud().empty()) {
                FilterWithCovariance(&cloud_a);
                FilterWithCovariance(&cloud_b);
            }

            // Reduce the number of points based on a voxel grid
            if (!cloud_a.point_cloud().empty() && !cloud_b.point_cloud().empty()) {
                nie::CloudFilter<PointT> filter_(filter_parameters_);
                cloud_a = filter_.Filter(cloud_a);
                cloud_b = filter_.Filter(cloud_b);
            }

            if (!cloud_a.point_cloud().empty() && !cloud_b.point_cloud().empty()) {
                debug_io_->SetFilteredClouds(ids, cloud_a, cloud_b);

                // Matrix format of the transformation and its inverse
                nie::Isometry3qd T_ab = candidate->T_begin_end;
                nie::CloudMatcher<PointT> matcher(matcher_parameters_);
                bool converged = matcher.Match(cloud_a, cloud_b, &T_ab);
                if (converged) {
                    VLOG(1) << "Matcher converged for ids: " << pose_id_a << ", " << pose_id_b;

                    // The input transformation was: candidate->T_begin_end
                    // This is "subtracted" after ICP to get the difference/error that was solved by ICP
                    // If the result seems reasonable we store it.
                    nie::Isometry3qd T_error = T_ab * candidate->T_begin_end.Inversed();
                    if (IsValidSolution(T_error)) {
                        registration_results_->PushBack(RegistrationResult{ids, T_ab});
                    } else {
                        VLOG(1) << "Result of matcher is not within expected limits for ids: " << pose_id_a << ", "
                                << pose_id_b;
                    }

                    debug_io_->SetRegistrationError(ids, matcher.GetFitnessScore(), T_error);
                } else {
                    VLOG(1) << "The matcher did not converge for ids: " << pose_id_a << ", " << pose_id_b;
                }
                debug_io_->SetCorrespondences(ids, matcher.GetCorrespondences());
            } else {
                VLOG(1) << "One of the clouds has all points filtered out, no matching can be performed for ids: "
                        << pose_id_a << ", " << pose_id_b;
            }
        } else {
            VLOG(1) << "Dry run output for ids: " << pose_id_a << ", " << pose_id_b;
            registration_results_->PushBack(RegistrationResult{ids, candidate->T_begin_end});
        }

        // Let the debug object know this candidate is done
        debug_io_->SetDone(ids);
    }

private:
    bool IsValidSolution(nie::Isometry3qd const& T) {
        // When the transformation (translation and rotation) falls below these values, then the result is valid.
        constexpr double kTranslationSizeLimit = 7.5;  // [meter]
        constexpr double kRotationSizeLimit = 2.;      // [degree]

        double const translation_size = nie::CalcTranslationNorm(T);
        double const rotation_size = nie::Rad2Deg(nie::CalcRotationAngle(T));
        if (translation_size > kTranslationSizeLimit) {
            VLOG(2) << "Resulting translation is too large: " << translation_size << " > " << kTranslationSizeLimit;
            return false;
        } else if (rotation_size > kRotationSizeLimit) {
            VLOG(2) << "Resulting rotation is too large: " << rotation_size << " > " << kRotationSizeLimit;
            return false;
        }
        return true;
    }

    typename nie::CloudFilter<PointT>::Parameters filter_parameters_;
    std::vector<typename nie::CloudMatcher<PointT>::Parameters> matcher_parameters_;

    std::vector<nie::io::PoseRecord> const& trace_;

    bool dry_run_;
    nie::mt::MtVector<RegistrationResult>* registration_results_;
    nie::DebugIo* debug_io_;
};
