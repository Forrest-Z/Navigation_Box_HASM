/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <numeric>
#include <random>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/constants.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/collection_writer.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <variant>

// This tool will introduce some artificial error to the poses (not edges), based on the provided settings. The
// modifications are applied such that toward the middle the effect becomes larger in a linear fashion. There will be no
// effect on the end points. Setting options:
//  - "drift abs": Apply drift in a fixed global/overall direction. The first parameter supplied is the maximum drift
//                 in meters in the middle of the trajectory. The remaining 3 parameters specify the direction, like
//                 "1., 2., 0.". Note that the direction vector will be normalized.
//  - "drift rel": Apply drift in a relative local direction. The direction supplied is treated as a local to the pose.
//                 The 4 parameters are the same as for the absolute drift, maximum drift and direction specification.
//  - "uncertain": Percentage increase the uncertainty by scaling the information matrix values. When a value of 10 is
//                 provided, then the values are scaled with a factor 0.9 (= 1 - 10/100).
//  - "noise":     Introduce noise being a translation in a random 3D direction with a size taken from a gaussian
//                 distribution. When one parameter is given like "noise,.1", then the the gaussian has a sigma of 0.1
//                 in all directions. With three values (comma-separated), then the x, y and z coordinates are taken
//                 from respective gaussian distributions.

// clang-format off
DEFINE_string(in_file_pose, "",
        "Filepath to input .pose file to resampled. Coordinate system is expected to be aircraft.");
DEFINE_string(settings, "",
        "What actions need to be done, in order, ';' separated. Options: 'drift abs', 'drift rel', 'uncertain', 'noise'");
DEFINE_string(out_file_pose, "",
        "Filepath to output .pose file.");
// clang-format on

DEFINE_validator(in_file_pose, nie::ValidateIsFile);
DEFINE_validator(settings, nie::ValidateStringNotEmpty);

// Object to generate random points on the unit sphere taken from a uniform distribution
class RandomUnitVector {
public:
    Eigen::Vector3d Get() {
        double const theta = Pi2 * GetRandom(dist_param_0_1_);
        double const phi = std::acos(GetRandom(dist_param_1_1_));
        double const sin_phi = std::sin(phi);
        return {std::cos(theta) * sin_phi, std::sin(theta) * sin_phi, std::cos(phi)};
    }

private:
    double GetRandom(std::uniform_real_distribution<>::param_type const params) { return dist_(generator_, params); }

    std::default_random_engine generator_{12345};
    std::uniform_real_distribution<> dist_{};

    std::uniform_real_distribution<>::param_type dist_param_0_1_{0., 1.};   // Between 0 and 1
    std::uniform_real_distribution<>::param_type dist_param_1_1_{-1., 1.};  // Between -1 and 1

    double const Pi2 = 2. * nie::kPi<>;
};

double GetTraceLength(nie::io::PoseCollection const& poses) {
    Eigen::Vector3d prev_position = poses.poses.front().isometry.translation();
    return std::accumulate(
            poses.poses.cbegin() + 1,
            poses.poses.cend(),
            0.,
            [&prev_position](double length, nie::io::PoseRecord const& p) {
                length += (prev_position - p.isometry.translation()).norm();
                prev_position = p.isometry.translation();
                return length;
            });
}

std::string const kDriftAbs = "drift abs";
std::string const kDriftRel = "drift rel";
std::string const kIncrUncertainty = "uncertain";
std::string const kNoise = "noise";

void AddDrift(std::string const& option, std::vector<std::string> const& settings, nie::io::PoseCollection* p_poses) {
    CHECK(settings.size() == 4) << option << " settings should be one parameter, drift at end of trajectory in meters.";

    nie::io::PoseCollection& poses = *p_poses;
    double const half_length = GetTraceLength(poses) / 2.;

    double const drift = std::stod(settings.front());
    Eigen::Vector3d const unit_error =
            Eigen::Vector3d{
                    std::stod(*(settings.begin() + 1)),
                    std::stod(*(settings.begin() + 2)),
                    std::stod(*(settings.begin() + 3))}
                    .normalized() *
            drift;

    Eigen::Vector3d prev_position = poses.poses.front().isometry.translation();
    double length = 0;
    for (auto pose_it = poses.poses.begin() + 1; pose_it != poses.poses.end(); ++pose_it) {
        length += (prev_position - pose_it->isometry.translation()).norm();
        double factor = (length > half_length ? 2. * half_length - length : length) / half_length;
        prev_position = pose_it->isometry.translation();

        if (option == kDriftAbs) {
            pose_it->isometry.translation() += unit_error * factor;
        } else if (option == kDriftRel) {
            pose_it->isometry.translation() += pose_it->isometry.rotation() * unit_error * factor;
        }
    }
}

void AddNoise(std::vector<std::string> const& settings, nie::io::PoseCollection* p_poses) {
    CHECK(settings.size() == 1 || settings.size() == 3)
            << kNoise
            << " settings should be 1 or 3 parameters, "
               "1 or 3 sigmas of noise size in meters, in all directions or every direction, respectively.";
    nie::io::PoseCollection& poses = *p_poses;
    double const half_length = GetTraceLength(poses) / 2.;

    auto const std_devs = (settings.size() == 1) ? Eigen::Array3d::Constant(std::stod(settings.front()))
                                                 : Eigen::Array3d{
                                                           std::stod(*settings.begin()),
                                                           std::stod(*(settings.begin() + 1)),
                                                           std::stod(*(settings.begin() + 2))};

    RandomUnitVector random_vector;
    std::default_random_engine gen{1234};
    std::normal_distribution<> dis{0., 1.};
    auto const get_noise = [&random_vector, &gen, &dis, &std_devs]() -> Eigen::Vector3d {
        auto noise = random_vector.Get();
        noise.array() *= std_devs * Eigen::Array3d{dis(gen), dis(gen), dis(gen)};
        return noise;
    };
    Eigen::Vector3d prev_position = poses.poses.front().isometry.translation();
    double length = 0;
    for (auto pose_it = poses.poses.begin() + 1; pose_it != poses.poses.end(); ++pose_it) {
        length += (prev_position - pose_it->isometry.translation()).norm();
        prev_position = pose_it->isometry.translation();

        double const factor = (length > half_length ? 2. * half_length - length : length) / half_length;
        pose_it->isometry.translation() += pose_it->isometry.rotation() * get_noise() * factor;
    }
}

void IncreaseUncertainty(std::vector<std::string> const& settings, nie::io::PoseCollection* p_poses) {
    CHECK(settings.size() == 1) << kIncrUncertainty << " settings should be one parameter, percentage increase of "
                                << "uncertainty. 10 means that the information matrix values are scaled with 0.9.";
    nie::io::PoseCollection& poses = *p_poses;
    double const half_length = GetTraceLength(poses) / 2.;

    double const factor = std::stod(settings.front()) / 100.;
    Eigen::Vector3d prev_position = poses.poses.front().isometry.translation();
    double length = 0;
    for (auto pose_it = poses.poses.begin() + 1; pose_it != poses.poses.end(); ++pose_it) {
        length += (prev_position - pose_it->isometry.translation()).norm();
        prev_position = pose_it->isometry.translation();

        pose_it->information *= 1. - (length > half_length ? 2. * half_length - length : length) / half_length * factor;
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Reading pose file: " << FLAGS_in_file_pose;
    auto poses = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose);

    std::vector<std::string> const configurations = nie::Split<std::string>(FLAGS_settings, ';');
    for (auto const& configuration : configurations) {
        if (configuration.empty()) {
            continue;
        }
        std::vector<std::string> const configuration_parts = nie::Split<std::string>(configuration, ',');
        std::string const& option = nie::ToLower(nie::Trim(configuration_parts.front()));
        std::vector<std::string> const settings = {configuration_parts.cbegin() + 1, configuration_parts.cend()};

        if (option == kDriftAbs || option == kDriftRel) {
            AddDrift(option, settings, &poses);
        } else if (option == kNoise) {
            AddNoise(settings, &poses);
        } else if (option == kIncrUncertainty) {
            IncreaseUncertainty(settings, &poses);
        } else {
            LOG(FATAL) << "unknown setting option: " << option;
        }
    }

    LOG(INFO) << "Writing updated pose to file: " << FLAGS_out_file_pose;
    nie::io::Write(poses, FLAGS_out_file_pose);
}
