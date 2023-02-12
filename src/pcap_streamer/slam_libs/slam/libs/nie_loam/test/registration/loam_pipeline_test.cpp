/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
// TODO: Add tests for ouster_streamer
#include <nie/lidar/io/lidar_streamer/velodyne_streamer.hpp>

#include "nie/loam/loam_manager.hpp"

class LoamPipelineTest : public testing::Test {
protected:
    boost::filesystem::path const kLoamTestData = "/data/aiim/unit_tests_data/loam/pipeline/";
    boost::filesystem::path const kPathRecording = kLoamTestData / "10021005180120113920.pcap";
    boost::filesystem::path const kPathCalibration = kLoamTestData / "HDL-32E.xml";
    boost::filesystem::path const kPathExpectedPointCloud = kLoamTestData / "shangai_overpass_loamed_control.pcd";
    boost::filesystem::path const kPathImuData = kLoamTestData / "10021005180120104131.imr";

    size_t const kTestPacketsNum = 30;
    double const kIcpFitnessEpsilon = 0.1;
};

TEST_F(LoamPipelineTest, RunPipeline) {
    auto streamer = nie::io::velodyne::CreatePcapFileStreamer(
            nie::io::LidarType::kVelodyneHDL32, kPathCalibration.string(), {kPathRecording});
    nie::LoamManager loam_manager(loam::MultiScanMapper::Velodyne_HDL_32());

    std::atomic<size_t> processed_returns = 0;

    streamer.AddCallback<nie::io::lidar::LidarCallbackTags::kSweep>(
            [&loam_manager, &processed_returns](nie::io::lidar::Returns const& returns) {
                loam_manager.ProcessLidarReturns(returns);
                ++processed_returns;
            });

    streamer.Start();

    while (streamer.IsRunning()) {
        if (processed_returns >= kTestPacketsNum) {
            streamer.Stop();
        }
    }

    auto const& generated_point_cloud = loam_manager.MappingLaserCloud();

    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZI> expected_point_cloud;
    pcl::PointCloud<pcl::PointXYZI> aligned;
    reader.read(kPathExpectedPointCloud.string(), expected_point_cloud);
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaximumIterations(1);
    icp.setInputSource(generated_point_cloud.makeShared());
    icp.setInputTarget(expected_point_cloud.makeShared());
    icp.align(aligned);

    LOG(INFO) << "ICP \"fitness\" score (generated -> expected) after 1 iteration: " << icp.getFitnessScore();
    ASSERT_LE(icp.getFitnessScore(), kIcpFitnessEpsilon);
}

TEST_F(LoamPipelineTest, RunPipelineWithImu) {
    auto streamer = nie::io::velodyne::CreatePcapFileStreamer(
            nie::io::LidarType::kVelodyneHDL32, kPathCalibration.string(), {kPathRecording});
    nie::LoamManager loam_manager(
            loam::MultiScanMapper::Velodyne_HDL_32(), loam::RegistrationParams(), kPathImuData.string());

    std::atomic<size_t> processed_returns = 0;

    streamer.AddCallback<nie::io::lidar::LidarCallbackTags::kSweep>(
            [&loam_manager, &processed_returns](nie::io::lidar::Returns const& returns) {
                loam_manager.ProcessLidarReturns(returns);
                ++processed_returns;
            });

    streamer.Start();

    while (streamer.IsRunning()) {
        if (processed_returns >= kTestPacketsNum) {
            streamer.Stop();
        }
    }

    auto const& generated_point_cloud = loam_manager.MappingLaserCloud();

    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZI> expected_point_cloud;
    pcl::PointCloud<pcl::PointXYZI> aligned;
    reader.read(kPathExpectedPointCloud.string(), expected_point_cloud);
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaximumIterations(1);
    icp.setInputSource(generated_point_cloud.makeShared());
    icp.setInputTarget(expected_point_cloud.makeShared());
    icp.align(aligned);

    LOG(INFO) << "ICP \"fitness\" score (generated -> expected) after 1 iteration: " << icp.getFitnessScore();
    ASSERT_LE(icp.getFitnessScore(), kIcpFitnessEpsilon);
}
