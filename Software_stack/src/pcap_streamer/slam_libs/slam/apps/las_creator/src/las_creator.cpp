/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <iomanip>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
#include <nie/formats/calib3d/lidar_parameters.hpp>
#include <nie/lidar/filter_ground_plane.hpp>
#include <nie/lidar/io/filenamer.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_bbox_writer.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_las_writer.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_point_interpolator.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_stream_consumer.hpp>
#include <nie/lidar/io/lidar_streamer/velodyne_streamer.hpp>

#include "lidar_iref_writer.hpp"

DEFINE_string(pose_file, "", "Filepath to input .pose file.");
DEFINE_string(source_list_file, "", "Filepath to txt file with source filepaths.");
DEFINE_string(lidar_extrinsics_file, "", "Filepath to input lidar extrinsics calibration parameters.");
DEFINE_string(lidar_intrinsics_file, "", "Filepath to input lidar intrinsics calibration parameters.");
DEFINE_string(output_dir, ".", "Directory for output .las and .iref files.");
DEFINE_string(output_prefix, "las", "Prefix used in naming the output .las and .iref files.");
DEFINE_double(traveled_distance, 200, "Traveled distance per output las file.");
DEFINE_bool(lidar_odometry, false, "Is the input .pose coming from LOAM?");
DEFINE_string(lidar_id, "", "Identifier of lidar to be used.");
DEFINE_bool(output_iref, false, "Output .iref files.");
DEFINE_bool(output_bbox, false, "Output .bbox and related files.");
DEFINE_double(min_ray_length, 0, "Minimum length of a lidar ray to be considered a valid lidar point.");
DEFINE_double(
        max_ray_length,
        std::numeric_limits<float>::max(),
        "Maximum length of a lidar ray to be considered a valid lidar point.");
DEFINE_bool(filter_ground_plane, false, "Filter the ground plane.");

DEFINE_validator(pose_file, nie::ValidateIsFile);
DEFINE_validator(source_list_file, nie::ValidateIsFile);
DEFINE_validator(lidar_intrinsics_file, nie::ValidateIsFile);
DEFINE_validator(output_dir, nie::ValidatePathExists);
DEFINE_validator(output_prefix, nie::ValidateStringNotEmpty);
DEFINE_validator(traveled_distance, nie::ValidateLargerThanZero);
DEFINE_validator(min_ray_length, nie::ValidateLargerOrEqualToZero);
DEFINE_validator(max_ray_length, nie::ValidateLargerThanZero);

struct Options {
    double slice_size;
    float min_ray_length;
    float max_ray_length;
    boost::filesystem::path output_dir;
    std::string output_prefix;
    bool output_iref;
    bool output_bbox;
    bool filter_ground_plane;
};

// Class that handles all components for and around the las creation
template <typename Streamer>
class LasCreator {
private:
    using PointT = pcl::PointXYZI;

public:
    LasCreator(
            Streamer&& streamer,
            std::size_t const& num_lasers,
            nie::Isometry3qd const& lidar_extr,
            nie::io::PoseCollection const& trace_pose,
            Options const& options)
        : lidar_streamer_{std::move(streamer)},
          ground_plane_filter_{
                  options.filter_ground_plane ? std::make_unique<nie::GroundPlaneFilter>(
                                                        num_lasers, lidar_extr, nie::GroundPlaneFilter::Options())
                                              : nullptr},
          lidar_stream_consumer_{
                  options.slice_size,
                  [this](auto const& points, auto const& offset, auto const& timestamps) {
                      ProcessSlice(points, offset, timestamps);
                  }},
          las_filenamer_{options.output_dir, options.output_prefix, ".las"},
          interpolator_{trace_pose.poses.cbegin(), trace_pose.poses.cend()},
          bounds_calculator_{options.output_bbox ? std::make_unique<nie::PoseBboxCalculator>() : nullptr},
          las_writer_{las_filenamer_},
          iref_writer_{
                  options.output_iref ? std::make_unique<nie::LidarIrefWriter>(las_filenamer_, trace_pose.poses)
                                      : nullptr},
          bbox_writer_{
                  options.output_bbox ? std::make_unique<nie::LidarBboxWriter>(las_filenamer_, *bounds_calculator_)
                                      : nullptr} {
        lidar_streamer_.packet_consumer.SetDistanceThreshold(
                nie::io::lidar::DistanceThresholds{options.min_ray_length, options.max_ray_length});
        // Using kSweep to be consistent with LOAM. However, for las creation both should work.
        if (options.filter_ground_plane) {
            lidar_streamer_.template AddCallback<nie::io::lidar::LidarCallbackTags::kSweepAndAngles>(
                    [this](auto returns, auto angles) { ForwardSweep(std::move(returns), std::move(angles)); });
        } else {
            lidar_streamer_.template AddCallback<nie::io::lidar::LidarCallbackTags::kSweep>(
                    [this](auto returns) { ForwardSweep(std::move(returns)); });
        }
    }

    void Run() {
        lidar_streamer_.Start();
        while (lidar_streamer_.IsRunning()) {
            std::this_thread::sleep_for(std::chrono::microseconds{1});
        }
        lidar_streamer_.Stop();

        lidar_stream_consumer_.Stop();
    }

private:
    // This function is used to find a pose using the interpolator based on the raw lidar returns.
    // The angles are only used when the ground filter is applied.
    void ForwardSweep(nie::io::lidar::Returns returns, nie::io::lidar::Angles angles = nie::io::lidar::Angles{}) {
        if (returns.points.empty()) {
            // Discard empty sweep.
            return;
        }

        if (ground_plane_filter_) {
            CHECK(!angles.hor_angles.empty()) << "Angles are required to perform ground filtering.";

            std::vector<bool> const filter = ground_plane_filter_->filter(returns, angles);
            CHECK(!std::all_of(filter.cbegin(), filter.cend(), [](bool b) { return b; }))
                    << "All points are labeled as ground.";
            returns.points.points.erase(nie::RemoveIf(filter, &returns.points.points), returns.points.points.end());
            returns.timestamps.erase(nie::RemoveIf(filter, &returns.timestamps), returns.timestamps.end());
        }

        auto [lidar_isometry, success] = interpolator_(returns.timestamps.front());
        if (!success) return;
        auto lidar_pose = nie::io::PoseRecord{
                0,
                nie::io::PoseRecord::Category::kOdom,
                0,
                returns.timestamps.front(),
                lidar_isometry,
                Eigen::Matrix<double, 6, 6>::Zero()};
        lidar_stream_consumer_(std::move(returns), std::move(lidar_pose));
    }

    void ProcessSlice(
            pcl::PointCloud<PointT> const& points,
            Eigen::Vector3d const& offset,
            std::vector<nie::Timestamp_ns> const& timestamps) {
        if (bounds_calculator_) {
            bounds_calculator_->Calculate(points);

            las_writer_.ProcessSliceWithBounds(points, offset, timestamps, bounds_calculator_->GetBounds());
        } else {
            las_writer_.ProcessSlice(points, offset, timestamps);
        }

        if (iref_writer_) {
            iref_writer_->ProcessSlice(timestamps);
        }
        if (bbox_writer_) {
            bbox_writer_->ProcessSlice(offset, timestamps);
        }

        las_filenamer_.Next();
    }

    // As some processing steps and outputting of files are optional, the corresponding processors and writers/streamers
    // are also conditional, for which a unique_ptr is used.

    Streamer lidar_streamer_;
    std::unique_ptr<nie::GroundPlaneFilter> const ground_plane_filter_;
    nie::LidarStreamConsumer<PointT> lidar_stream_consumer_;

    nie::Filenamer las_filenamer_;
    nie::LidarPointInterpolator interpolator_;
    std::unique_ptr<nie::PoseBboxCalculator> bounds_calculator_;

    nie::LidarLasWriter las_writer_;
    std::unique_ptr<nie::LidarIrefWriter> iref_writer_;
    std::unique_ptr<nie::LidarBboxWriter> bbox_writer_;
};

void ExecuteLasCreator(
        nie::io::LidarParameters const& params,
        std::string const& intr_path,
        nie::io::PoseCollection const& trace_pose,
        std::vector<boost::filesystem::path> source_paths,
        Options const& options) {
    switch (params.type) {
        case nie::io::LidarType::kVelodyneVLP16:
        case nie::io::LidarType::kVelodyneHDL32: {
            auto intrinsics = nie::io::velodyne::LoadCalibrationFromFile(params.type, intr_path);
            auto streamer = nie::io::velodyne::CreatePcapFileStreamer(intrinsics, std::move(source_paths));
            auto creator =
                    LasCreator(std::move(streamer), intrinsics.num_lasers(), params.extrinsics, trace_pose, options);
            creator.Run();
            break;
        }
        case nie::io::LidarType::kOusterOS1_32:
        case nie::io::LidarType::kOusterOS1_128:
        case nie::io::LidarType::kOusterOS2_128: {
            auto intrinsics = nie::io::ouster::LoadCalibrationFromFile(intr_path);
            auto streamer = nie::io::ouster::CreatePcapFileStreamer(intrinsics, std::move(source_paths));
            auto creator = LasCreator(
                    std::move(streamer),
                    intrinsics.beam_altitude_radians.size(),
                    params.extrinsics,
                    trace_pose,
                    options);
            creator.Run();
            break;
        }
        case nie::io::LidarType::kKitti: {
            auto streamer = nie::io::kitti::CreateTextFileStreamer(std::move(source_paths));
            auto creator = LasCreator(std::move(streamer), 64, params.extrinsics, trace_pose, options);
            creator.Run();
            break;
        }
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    // Read the required files
    VLOG(3) << "Reading source list file: " << FLAGS_source_list_file;
    auto source_paths =
            nie::ReadLines(FLAGS_source_list_file, [](std::string const& s) { return boost::filesystem::path{s}; });
    CHECK(!source_paths.empty()) << "No pcap files to run loam for.";

    VLOG(3) << "Reading pose file " << FLAGS_pose_file;
    auto trace_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_pose_file);
    VLOG(3) << "Read " << trace_pose.poses.size() << " poses.";

    // Determine the type of lidar and read the extrinsics when required
    bool const extr_required = !FLAGS_lidar_odometry || FLAGS_filter_ground_plane ||
                               (!FLAGS_lidar_id.empty() && nie::ToLower(FLAGS_lidar_id) != "kitti");
    nie::io::LidarParameters lidar_parameters{};
    if (extr_required) {
        CHECK(nie::ValidateIsFile("lidar_extrinsics_file", FLAGS_lidar_extrinsics_file));
    }
    if (FLAGS_lidar_id.empty()) {
        lidar_parameters.type = nie::io::LidarType::kVelodyneHDL32;
        if (extr_required) {
            lidar_parameters.extrinsics = nie::ReadLidarExtrinsics(FLAGS_lidar_extrinsics_file);
        }
    } else if (nie::ToLower(FLAGS_lidar_id) == "kitti") {
        lidar_parameters.type = nie::io::LidarType::kKitti;
        if (extr_required) {
            lidar_parameters.extrinsics =
                    nie::kIsometryFromTo<nie::Frame::kVehicle, nie::Frame::kAircraft, Eigen::Quaterniond> *
                    nie::io::kitti::ReadExtrinsics(FLAGS_lidar_extrinsics_file);
        }
    } else {
        lidar_parameters = nie::io::ReadLidarParametersByIdentifier(FLAGS_lidar_extrinsics_file, FLAGS_lidar_id);
    }

    if (!FLAGS_lidar_odometry) {
        // Multiply all poses with lidar extrinsics. We want the poses to be transformed from a lidar to world
        // coordinate system.
        for (auto& pose : trace_pose.poses) {
            pose.isometry *= lidar_parameters.extrinsics;
        }
    }

    // Create and run the object that handles the las creation
    Options const options{
            FLAGS_traveled_distance,
            static_cast<float>(FLAGS_min_ray_length),
            static_cast<float>(FLAGS_max_ray_length),
            FLAGS_output_dir,
            FLAGS_output_prefix,
            FLAGS_output_iref,
            FLAGS_output_bbox,
            FLAGS_filter_ground_plane};
    ExecuteLasCreator(lidar_parameters, FLAGS_lidar_intrinsics_file, trace_pose, source_paths, options);

    return 0;
}
