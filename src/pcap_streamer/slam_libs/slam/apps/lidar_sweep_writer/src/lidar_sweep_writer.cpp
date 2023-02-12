/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <chrono>
#include <iomanip>
#include <sstream>
#include <thread>

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
#include <nie/formats/calib3d/lidar_parameters.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// TODO: Add support for ouster_streamer
// This can be done via a flag
#include <nie/lidar/io/lidar_streamer/velodyne_calibration.hpp>
#include <nie/lidar/io/lidar_streamer/velodyne_streamer.hpp>

DEFINE_string(in_file_calib_intrinsics, "", "Lidar calibration intrinsics");
DEFINE_string(in_file_calib_extrinsics, "", "Lidar calibration extrinsics");
DEFINE_string(in_file_path_to_pcap, "", "Path to PCAP file");
DEFINE_string(in_file_output_folder, "", "Folder path output images");
DEFINE_double(x_min, -20, "Maximum range in meters in negative x direction.");
DEFINE_double(x_max, 20, "Maximum range in meters  in positive x direction.");
DEFINE_double(y_min, -20, "Maximum range in meters  in negative y direction.");
DEFINE_double(y_max, 20, "Maximum range in meters  in positive y direction.");
DEFINE_double(z_min, 0, "Maximum range in meters  in the negative Z direction.");
DEFINE_double(z_max, 3, "Maximum range in meters  in the positive Z direction.");
DEFINE_double(lidar_height, 1.7, "Height offset in meters of the lidar compared to the ground.");
DEFINE_double(xy_resolution, 0.1, "Distance in meters which corresponds to the height and width of a pixel.");

DEFINE_validator(in_file_calib_intrinsics, nie::ValidateIsFile);
DEFINE_validator(in_file_calib_extrinsics, nie::ValidateIsFile);
DEFINE_validator(in_file_path_to_pcap, nie::ValidateIsFile);
DEFINE_validator(in_file_output_folder, nie::ValidatePathExists);

struct ImageConfig {
    double x_min, x_max;
    double y_min, y_max;
    double z_min, z_max;
    double lidar_height;
    double xy_resolution;
    int x_size = (x_max - x_min) / xy_resolution;
    int y_size = (y_max - y_min) / xy_resolution;
};

class SweepWriter {
public:
    SweepWriter(boost::filesystem::path root, nie::Isometry3qd lidar_extrinsics, ImageConfig const& config)
        : root_(std::move(root)), sweep_counter_(0), lidar_extrinsics_(lidar_extrinsics), config_(config) {}

    void operator()(pcl::PointCloud<pcl::PointXYZI> const& points) {
        cv::Mat image(config_.x_size, config_.y_size, CV_8UC3);

        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << sweep_counter_;

        CreateTopViewImage(points, &image);

        boost::filesystem::path filename = root_ / (ss.str() + ".png");
        cv::imwrite(filename.string(), image);

        sweep_counter_++;
    }

private:
    void CreateTopViewImage(pcl::PointCloud<pcl::PointXYZI> const& points, cv::Mat* image) {
        /* Create top view image from an input point cloud. The output image has 3 channels (RGB image) which means that
         * for every pixel location only three values can be saved. */

        for (pcl::PointXYZI point : points) {
            // Get lidar point.
            point.getVector3fMap() = (lidar_extrinsics_ * point.getVector3fMap().cast<double>()).cast<float>();

            // Get pixel location in the top view map of the lidar point.
            int x, y;
            x = static_cast<int>((point.x - config_.x_min) / config_.xy_resolution);
            y = static_cast<int>((point.y - config_.y_min) / config_.xy_resolution);

            // Check if point is inside the image range.
            if (x >= 0 && y >= 0 && x < config_.x_size && y < config_.y_size) {
                // Add lidar offset so 0 height is located at the bottom of the ego vehicle.
                double z = point.z + config_.lidar_height;

                // Check if z falls inside the defined height range, otherwise the point is skipped.
                if (z < config_.z_min || z > config_.z_max) continue;

                double z_delta = config_.z_max - config_.z_min;

                // 0 <= z <= z_delta
                z = z - config_.z_min;

                // Distribute height over the different color channels.
                // 0 <= z <= 3
                double normalized = (z / z_delta) * 3.0;
                int color_channel = static_cast<int>(normalized);
                DCHECK(color_channel >= 0 && color_channel <= 3);

                // Create color saw tooth.
                // z [0...1) = blue
                // z [1...2) = green
                // z [2...3) = red
                auto value = static_cast<std::uint8_t>((normalized - color_channel) * 255.0);

                if (value > image->at<cv::Vec3b>(x, y)[color_channel]) {
                    image->at<cv::Vec3b>(x, y)[color_channel] = value;
                }
            }
        }
    }

    boost::filesystem::path const root_;
    std::size_t sweep_counter_;
    nie::Isometry3qd const lidar_extrinsics_;
    ImageConfig const config_;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    LOG(INFO) << "Reading Lidar calibration extrinsics: " << FLAGS_in_file_calib_extrinsics;
    nie::Isometry3qd const lidar_extrinsics = nie::ReadLidarExtrinsics(FLAGS_in_file_calib_extrinsics);

    LOG(INFO) << "Reading path to pcap file: " << FLAGS_in_file_path_to_pcap;
    std::vector<boost::filesystem::path> pcap_files{FLAGS_in_file_path_to_pcap};

    LOG(INFO) << "Reading folder to output images: " << FLAGS_in_file_output_folder;
    std::string const output_folder = FLAGS_in_file_output_folder;

    ImageConfig image_config{
            FLAGS_x_min,
            FLAGS_x_max,
            FLAGS_y_min,
            FLAGS_y_max,
            FLAGS_z_min,
            FLAGS_z_max,
            FLAGS_lidar_height,
            FLAGS_xy_resolution};

    SweepWriter writer(output_folder, lidar_extrinsics, image_config);

    auto streamer = nie::io::velodyne::CreatePcapFileStreamer(
            nie::io::LidarType::kVelodyneVLP16, FLAGS_in_file_calib_intrinsics, std::move(pcap_files));
    streamer.AddCallback<nie::io::lidar::LidarCallbackTags::kSweep>(
            [&writer](nie::io::lidar::Returns const& returns) { writer(returns.points); });

    streamer.Start();
    while (streamer.IsRunning()) {
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }
    streamer.Stop();

    return 0;
}
