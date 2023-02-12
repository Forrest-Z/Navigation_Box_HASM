/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
#include <nie/formats/kitti/readers.hpp>
#include <nie/lidar/filter_ground_plane.hpp>
#include <nie/lidar/io/filenamer.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_bbox_writer.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_las_writer.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_stream_consumer.hpp>
#include <nie/lidar/io/lidar_stream_consumer/lidar_stream_trimmer.hpp>
#include <nie/lidar/io/lidar_streamer.hpp>
#include <nie/lidar/time_intervals/time_intervals.hpp>
#include <nie/lidar/time_intervals/time_intervals_reader.hpp>
#include <nie/loam/loam_manager.hpp>

#include "lidar_iref_writer.hpp"
#include "lidar_pose_writer.hpp"

DEFINE_string(in_source_list_file, "", "Filepath to txt file with source filepaths.");
DEFINE_string(in_lidar_intrinsics_file, "", "Path to input lidar intrinsics calibration parameters.");
DEFINE_string(in_lidar_extrinsics_file, "", "Path to input lidar extrinsics calibration parameters.");
DEFINE_string(
        in_stationary_intervals, "", "Filepath to csv file with list of intervals when the vehicle was not moving.");
DEFINE_string(lidar_id, "", "Identifier of lidar to be used.");
DEFINE_double(
        pose_trim_size,
        1.5,
        "Number of poses to be removed from the beginning and end of the output pose file. In radius from the first "
        "and last poses of the trace.");
DEFINE_double(traveled_distance, 200, "Traveled distance per output las/iref file.");
DEFINE_double(loam_sd_scale, 1.0, "Scaling factor to be applied to the standard deviations computed by LOAM.");
DEFINE_string(output_dir, ".", "Output directory.");
DEFINE_string(output_prefix, "lidar_odometry", "Prefix used in naming the output files (LAS, Iref, pose).");

DEFINE_double(min_ray_length, 0, "Minimum length of a lidar ray to be considered a valid lidar point.");
DEFINE_double(
        max_ray_length,
        std::numeric_limits<float>::max(),
        "Maximum length of a lidar ray to be considered a valid lidar point.");
DEFINE_bool(filter_ground_plane, false, "Filter the ground plane.");

DEFINE_validator(in_source_list_file, nie::ValidateIsFile);
DEFINE_validator(pose_trim_size, nie::ValidateLargerOrEqualToZero);
DEFINE_validator(traveled_distance, nie::ValidateLargerThanZero);
DEFINE_validator(loam_sd_scale, nie::ValidateLargerOrEqualToZero);
DEFINE_validator(output_dir, nie::ValidatePathExists);
DEFINE_validator(output_prefix, nie::ValidateStringNotEmpty);

struct Options {
    double slice_size;
    double trim_size;
    double loam_sd_scale;
    float min_ray_length;
    float max_ray_length;
    bool filter_ground_plane;
    boost::filesystem::path output_dir;
    std::string output_prefix;
    std::vector<std::pair<nie::Timestamp_ns, nie::Timestamp_ns>> intervals;
};

// Class that handles all components for and around loam
template <typename Streamer>
class LoamHandler {
private:
    using PointT = pcl::PointXYZI;

public:
    LoamHandler(
            Streamer&& streamer,
            std::size_t const& num_lasers,
            nie::io::LidarParameters const& lidar_params,
            Options const& options)
        : options_{options},
          lidar_streamer_{std::move(streamer)},
          loam_manager_{GetMultiScanMapper(lidar_params.type), GetMultiScanConfig(lidar_params.type)},
          ground_plane_filter_{
                  options.filter_ground_plane
                          ? std::make_unique<nie::GroundPlaneFilter>(
                                    num_lasers, lidar_params.extrinsics, nie::GroundPlaneFilter::Options())
                          : nullptr},
          lidar_stream_trimmer_{
                  options.trim_size > 0.0
                          ? std::make_unique<nie::LidarStreamTrimmer>(
                                    options.trim_size,
                                    [this](auto const& points, auto const& pose) { ProcessSweep(points, pose); })
                          : nullptr},
          lidar_stream_consumer_{
                  options.slice_size,
                  [this](auto const& points, auto const& offset, auto const& timestamps) {
                      ProcessSlice(points, offset, timestamps);
                  }},
          las_filenamer_{options.output_dir, options.output_prefix, ".las"},
          bounds_calculator_{},
          las_writer_{las_filenamer_},
          iref_writer_{las_filenamer_},
          pose_writer_{
                  las_filenamer_.FullBasename() + nie::io::graph::Extension<nie::io::PoseCollection>(),
                  options.loam_sd_scale},
          bbox_writer_{las_filenamer_, bounds_calculator_},
          T_gps_lidar_{lidar_params.extrinsics},
          T_lidar_gps_{T_gps_lidar_.Inversed()},
          stationary_ranges_{options.intervals, 1.0},
          pose_id_{0} {
        // No ground filtering with data from kitti
        CHECK(!(options.filter_ground_plane && lidar_params.type == nie::io::LidarType::kKitti));

        if (options.filter_ground_plane) {
            lidar_streamer_.template AddCallback<nie::io::lidar::LidarCallbackTags::kSweepAndAngles>(
                    [this](auto returns, auto angles) { ForwardSweep(std::move(returns), std::move(angles)); });
        } else {
            lidar_streamer_.template AddCallback<nie::io::lidar::LidarCallbackTags::kSweep>(
                    [this](auto returns) { ForwardSweep(std::move(returns)); });
        }
        lidar_streamer_.packet_consumer.SetDistanceThreshold(
                nie::io::lidar::DistanceThresholds{options.min_ray_length, options.max_ray_length});
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
    // This function is used to retrieve the pose calculated by loam based on the raw lidar returns.
    void ForwardSweep(nie::io::lidar::Returns returns, nie::io::lidar::Angles angles = nie::io::lidar::Angles{}) {
        if (returns.points.empty()) {
            // Discard empty sweep.
            return;
        }
        nie::TimeIntervals::TimeRange const sweep_time_range{returns.timestamps.front(), returns.timestamps.back()};
        if (stationary_ranges_.IsMostlyContained(sweep_time_range)) {
            // Discard if the vehicle is stationary.
            LOG(INFO) << "Discarding sweep during stationary interval: " << sweep_time_range.first << ", "
                      << sweep_time_range.second;
            return;
        }

        // Let loam determine the new pose
        loam_manager_.ProcessLidarReturns(returns);
        nie::Isometry3qd loam_isometry =
                T_gps_lidar_ * LoamOdometryToIsometry(loam_manager_.MappedOdometry()) * T_lidar_gps_;

        if (ground_plane_filter_) {
            CHECK(!angles.hor_angles.empty()) << "Angles are required to perform ground filtering.";

            std::vector<bool> const filter = ground_plane_filter_->filter(returns, angles);
            CHECK(!std::all_of(filter.cbegin(), filter.cend(), [](bool b) { return b; }))
                    << "All points are labeled as ground.";
            returns.points.points.erase(nie::RemoveIf(filter, &returns.points.points), returns.points.points.end());
            returns.timestamps.erase(nie::RemoveIf(filter, &returns.timestamps), returns.timestamps.end());
        }

        for (auto& p : returns.points) {
            p.getVector3fMap() = (T_gps_lidar_ * p.getVector3fMap().cast<double>()).cast<float>();
        }

        nie::io::PoseRecord loam_pose{
                pose_id_++,
                nie::io::PoseRecord::Category::kOdom,
                0,
                returns.timestamps.front(),
                std::move(loam_isometry),
                Eigen::Matrix<double, 6, 6>::Zero()};

        // If there is no trimming to be done, then just by pass the trimmer.
        if (options_.trim_size > 0.0) {
            (*lidar_stream_trimmer_)(std::move(returns), std::move(loam_pose));
        } else {
            ProcessSweep(returns, loam_pose);
        }
    }

    void ProcessSweep(nie::io::lidar::Returns const& points, nie::io::PoseRecord const& pose) {
        pose_writer_.ProcessSweep(pose);
        iref_writer_.ProcessSweep(pose);

        lidar_stream_consumer_(points, pose);
    }

    void ProcessSlice(
            pcl::PointCloud<PointT> const& points,
            Eigen::Vector3d const& offset,
            std::vector<nie::Timestamp_ns> const& timestamps) {
        bounds_calculator_.Calculate(points);

        las_writer_.ProcessSliceWithBounds(points, offset, timestamps, bounds_calculator_.GetBounds());
        bbox_writer_.ProcessSlice(offset, timestamps);

        las_filenamer_.Next();
    }

    static loam::MultiScanMapper GetMultiScanMapper(nie::io::LidarType const type) {
        switch (type) {
            case nie::io::LidarType::kVelodyneVLP16:
                return loam::MultiScanMapper::Velodyne_VLP_16();
            case nie::io::LidarType::kVelodyneHDL32:
                return loam::MultiScanMapper::Velodyne_HDL_32();
            case nie::io::LidarType::kOusterOS1_32:
                return loam::MultiScanMapper::Ouster_OS1_32();
            case nie::io::LidarType::kOusterOS1_128:
                return loam::MultiScanMapper::Ouster_OS1_128();
            case nie::io::LidarType::kOusterOS2_128:
                return loam::MultiScanMapper::Ouster_OS2_128();
            // Removes a warning. The last is just:
            // case nie::io::LidarType::kKitti:
            default:
                return loam::MultiScanMapper::Velodyne_HDL_64E();
        }
    }

    // TODO More inline with GetMultiScanMapper() would be to move the defaults into the class.
    static loam::RegistrationParams GetMultiScanConfig(nie::io::LidarType const type) {
        switch (type) {
            case nie::io::LidarType::kOusterOS1_32:
            case nie::io::LidarType::kOusterOS1_128:
            case nie::io::LidarType::kOusterOS2_128: {
                // TODO(jbr): This has to come from somewhere.
                return loam::RegistrationParams(0.05);
            }
            default: {
                return loam::RegistrationParams();
            }
        }
    }

    static nie::Isometry3qd LoamOdometryToIsometry(loam::Odometry const& loam_odometry) {
        return {loam_odometry.pose.pose.position, loam_odometry.pose.pose.orientation.normalized()};
    }

    // As some processing steps and outputting of files are optional, the corresponding processors and writers/streamers
    // are also conditional, for which a unique_ptr is used.

    Options const& options_;

    Streamer lidar_streamer_;
    nie::LoamManager loam_manager_;
    std::unique_ptr<nie::GroundPlaneFilter> const ground_plane_filter_;
    std::unique_ptr<nie::LidarStreamTrimmer> lidar_stream_trimmer_;
    nie::LidarStreamConsumer<PointT> lidar_stream_consumer_;

    nie::Filenamer las_filenamer_;
    nie::PoseBboxCalculator bounds_calculator_;

    nie::LidarLasWriter las_writer_;
    nie::LidarIrefWriter iref_writer_;
    nie::LidarPoseWriter pose_writer_;
    nie::LidarBboxWriter bbox_writer_;

    nie::Isometry3qd const T_gps_lidar_;  // Lidar extrinsics
    nie::Isometry3qd const T_lidar_gps_;
    nie::TimeIntervals stationary_ranges_;
    std::atomic<nie::io::PoseId> pose_id_;
};

void ExecuteLoam(
        nie::io::LidarParameters const& params,
        std::string const& intr_path,
        std::vector<boost::filesystem::path> source_paths,
        Options const& options) {
    switch (params.type) {
        case nie::io::LidarType::kVelodyneVLP16:
        case nie::io::LidarType::kVelodyneHDL32: {
            auto intrinsics = nie::io::velodyne::LoadCalibrationFromFile(params.type, intr_path);
            auto streamer = nie::io::velodyne::CreatePcapFileStreamer(intrinsics, std::move(source_paths));
            auto loam = LoamHandler(std::move(streamer), intrinsics.num_lasers(), params, options);
            loam.Run();
            break;
        }
        case nie::io::LidarType::kOusterOS1_32:
        case nie::io::LidarType::kOusterOS1_128:
        case nie::io::LidarType::kOusterOS2_128: {
            auto intrinsics = nie::io::ouster::LoadCalibrationFromFile(intr_path);
            auto streamer = nie::io::ouster::CreatePcapFileStreamer(intrinsics, std::move(source_paths));
            auto loam = LoamHandler(std::move(streamer), intrinsics.beam_altitude_radians.size(), params, options);
            loam.Run();
            break;
        }
        case nie::io::LidarType::kKitti: {
            auto streamer = nie::io::kitti::CreateTextFileStreamer(std::move(source_paths));
            auto loam = LoamHandler(std::move(streamer), 64, params, options);
            loam.Run();
            break;
        }
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    bool const is_kitti = nie::ToLower(FLAGS_lidar_id) == "kitti";

    // Read the required files
    VLOG(3) << "Reading source list file: " << FLAGS_in_source_list_file;
    auto source_paths =
            nie::ReadLines(FLAGS_in_source_list_file, [](std::string const& s) { return boost::filesystem::path{s}; });
    if (is_kitti) {
        CHECK(source_paths.size() >
              2) << "No files to run loam for, expecting a start and end timestamp file and the point data files.";
    } else {
        CHECK(!source_paths.empty()) << "No files to run loam for.";
    }

    nie::io::LidarParameters lidar_parameters;
    if (FLAGS_lidar_id.empty()) {
        lidar_parameters.type = nie::io::LidarType::kVelodyneHDL32;
        lidar_parameters.extrinsics = nie::ReadLidarExtrinsics(FLAGS_in_lidar_extrinsics_file);

    } else if (is_kitti) {
        lidar_parameters.type = nie::io::LidarType::kKitti;
        lidar_parameters.extrinsics =
                nie::kIsometryFromTo<nie::Frame::kVehicle, nie::Frame::kAircraft, Eigen::Quaterniond> *
                nie::io::kitti::ReadExtrinsics(FLAGS_in_lidar_extrinsics_file);
    } else {
        lidar_parameters = nie::io::ReadLidarParametersByIdentifier(FLAGS_in_lidar_extrinsics_file, FLAGS_lidar_id);
    }

    if (!is_kitti) {
        CHECK(nie::ValidateIsFile("in_lidar_intrinsics_file", FLAGS_in_lidar_intrinsics_file));
    }

    Options options{};
    options.slice_size = FLAGS_traveled_distance;
    options.trim_size = FLAGS_pose_trim_size;
    options.loam_sd_scale = FLAGS_loam_sd_scale;
    options.min_ray_length = static_cast<float>(FLAGS_min_ray_length);
    options.max_ray_length = static_cast<float>(FLAGS_max_ray_length);
    options.filter_ground_plane = FLAGS_filter_ground_plane;
    options.output_dir = FLAGS_output_dir;
    options.output_prefix = FLAGS_output_prefix;
    if (!FLAGS_in_stationary_intervals.empty()) {
        options.intervals = nie::io::ReadIntervalsList(FLAGS_in_stationary_intervals);
    }

    // Create and run the object that handles loam
    ExecuteLoam(lidar_parameters, FLAGS_in_lidar_intrinsics_file, std::move(source_paths), options);
}
