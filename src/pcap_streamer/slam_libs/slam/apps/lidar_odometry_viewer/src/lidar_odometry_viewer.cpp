/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
// Standard includes
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// Google includes
#include <gflags/gflags.h>
#include <glog/logging.h>

// Eigen includes
#include <Eigen/Eigen>

// VTK includes
#include <vtkRenderWindowCollection.h>

// PCL includes
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

// NIE includes
#include <nie/core/geometry/rotation.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
#include <nie/lidar/io/lidar_streamer/velodyne_streamer.hpp>

// Local includes
#include "circle_grid.hpp"
#include "frame.hpp"

// Define command line parameters
DEFINE_string(in_pcap_list_file, "", "Filepath to txt file with pcap filepaths.");
DEFINE_string(in_file_lidar_cal, "", "File that contains Velodyne lidar calibration parameters (Optional).");

// Validate command line parameters
DEFINE_validator(in_pcap_list_file, nie::ValidateIsFile);

#define FPS_CALC(_WHAT_)                                                                                          \
    do {                                                                                                          \
        static unsigned count = 0;                                                                                \
        ++count;                                                                                                  \
        using Clock = std::chrono::high_resolution_clock;                                                         \
        static Clock::time_point last = Clock::now();                                                             \
        Clock::time_point const now = Clock::now();                                                               \
        using Duration = std::chrono::duration<double>;                                                           \
        auto elapsed = std::chrono::duration_cast<Duration>(now - last);                                          \
        using namespace std::chrono_literals;                                                                     \
        if (elapsed >= 1s) {                                                                                      \
            VLOG(6) << "Average framerate (" << _WHAT_ << "): " << count / elapsed.count() << " Hz" << std::endl; \
            count = 0;                                                                                            \
            last = now;                                                                                           \
        }                                                                                                         \
    } while (false)

class OdometryViewer {
public:
    using PointT = pcl::PointXYZI;
    //    using CloudConstPtr = typename pcl::PointCloud<PointT>::ConstPtr;

    OdometryViewer(
            nie::io::velodyne::PcapFileStreamer* streamer, pcl::visualization::PointCloudColorHandler<PointT>* handler)
        : viewer_(new pcl::visualization::PCLVisualizer("Lidar Odometry Viewer")),
          renderer_(*viewer_->getRenderWindow()->GetRenderers()->GetFirstRenderer()),
          handler_(*handler),
          streamer_(*streamer),

          T_road_(Eigen::Isometry3d::Identity()),
          T_road_ref_(Eigen::Isometry3d::Identity()),
          // Original ##6_CPT
          // T_ref_lidar_ (nie::CalibrationTransform<double>(-0.0354721, -0.303846, +0.247825, -42.6373, -0.879673,
          // +91.3008).ToTransform()),
          // Shanghai dataset
          T_ref_lidar_(Eigen::Isometry3d::Identity()),
          T_ref_camera_(Eigen::Isometry3d::Identity()),
          T_ref_gnss_(Eigen::Isometry3d::Identity()),
          T_ref_imu_(Eigen::Isometry3d::Identity())

    {
        // Initialize the viewer
        viewer_->setPosition(100, 100);
        viewer_->setSize(1800, 1000);
        viewer_->setBackgroundColor(0.2, 0.2, 0.35);
        viewer_->setCameraPosition(-10, -20, 5, 0, 0, 2, 0, 0, 1);

        // Add sensor frames to VTK renderer
        // clang-format off
        ShowFrame(renderer_, "road",      T_road_);
//      ShowFrame(renderer_, "reference", T_road_ref_);
        ShowFrame(renderer_, "lidar",     T_road_ref_ * T_ref_lidar_);
        ShowFrame(renderer_, "camera",    T_road_ref_ * T_ref_camera_);
        ShowFrame(renderer_, "gnss",      T_road_ref_ * T_ref_gnss_);
        ShowFrame(renderer_, "imu",       T_road_ref_ * T_ref_imu_);
        // clang-format on

        // Add grid with circles to VTK renderer
        renderer_.AddActor(CreateCircleGrid(10, 1.0, {0, 0, 0}, {0, 0, 1}, {0.4, 0.4, 0.4}));
    }

    void Run() {
        streamer_.AddCallback<nie::io::lidar::CallbackTags::kSweep>(
                boost::bind(&OdometryViewer::ReceiveSweep, this, _1));

        streamer_.Start();

        while (!viewer_->wasStopped()) {
            pcl::PointCloud<PointT>::Ptr sweep_ptr = boost::make_shared<pcl::PointCloud<PointT>>();

            // See if we can get a sweep
            if (sweep_mutex_.try_lock()) {
                sweep_.swap(*sweep_ptr);
                sweep_mutex_.unlock();
            }

            if (!sweep_ptr->points.empty()) {
                DrawSweep(sweep_ptr);
            }

            viewer_->spinOnce();

            std::this_thread::yield();
        }

        streamer_.Stop();

        // streamer_.RemoveCallbacks<CallbackTags::kLidarSweep>();
    }

private:
    void ReceiveSweep(nie::io::lidar::Returns const& sweep) {
#define REAL_TIME_FACTOR
#ifdef REAL_TIME_FACTOR
        using clock = std::chrono::steady_clock;

        // Define sweep rate in Hz
        static double const sweep_rate = 15.0;

        // Define the deadline when the next sweep is expected; For the first sweep, the deadline is now
        static clock::time_point deadline = clock::now();

        // Convert sweep rate to sweep duration
        static clock::duration const sweep_duration(
                std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(1 / sweep_rate)));

        // Wait until the deadline
        std::this_thread::sleep_until(deadline);
#endif
        // Set current sweep
        {
            std::lock_guard<std::mutex> lock(sweep_mutex_);
            sweep_ = std::move(sweep.points);
        }

#ifdef REAL_TIME_FACTOR
        // Set the next deadline
        deadline += sweep_duration;
#endif
        // Log the actual sweep update frequency
        FPS_CALC("Receiving sweep");
    }

    void DrawSweep(pcl::PointCloud<PointT>::ConstPtr sweep_ptr) {
        static std::string id = "Velodyne";

        handler_.setInputCloud(sweep_ptr);
        if (!viewer_->updatePointCloud(sweep_ptr, handler_, id)) {
            viewer_->addPointCloud(sweep_ptr, handler_, id);
            viewer_->updatePointCloudPose(id, (T_road_ref_ * T_ref_lidar_).template cast<float>());
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, id);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, id);
#if 1
            viewer_->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_GREY, id);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE, -12, 32, id);
#else
            viewer_->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_VIRIDIS, id);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LUT_RANGE, 0, 32, id);
#endif
        }

        // Log the actual draw frequency
        FPS_CALC("Drawing sweep");
    }

    pcl::visualization::PCLVisualizer::Ptr viewer_;
    vtkRenderer& renderer_;
    pcl::visualization::PointCloudColorHandler<PointT>& handler_;

    nie::io::velodyne::PcapFileStreamer& streamer_;

    std::mutex sweep_mutex_;
    pcl::PointCloud<PointT> sweep_;

    // TODO: [EDD] Replace later with a more generic approach similar to ROS::TF (Tree of named nodes)
    //             See, http://wiki.ros.org/tf
    Eigen::Isometry3d const T_road_;
    Eigen::Isometry3d const T_road_ref_;
    Eigen::Isometry3d const T_ref_lidar_;
    Eigen::Isometry3d const T_ref_camera_;
    Eigen::Isometry3d const T_ref_gnss_;
    Eigen::Isometry3d const T_ref_imu_;
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    nie::io::velodyne::PcapFileStreamer streamer(
            nie::io::velodyne::LoadCalibrationFromFile(nie::io::LidarType::kVelodyneHDL32, FLAGS_in_file_lidar_cal),
            nie::ReadLines(FLAGS_in_pcap_list_file, [](std::string const& s) { return boost::filesystem::path{s}; }));

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> color_handler("intensity");
    OdometryViewer viewer(&streamer, &color_handler);
    viewer.Run();

    return EXIT_SUCCESS;
}
