/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <thread>

#include <boost/filesystem.hpp>
#include <nie/lidar/io/lidar_streamer/velodyne_streamer.hpp>

#include "lidar_tests_common.hpp"

class VelodyneStreamerTest : public ::testing::Test {
protected:
    boost::filesystem::path const kBasePath{"/data/aiim/unit_tests_data/lidar/io/lidar_streamer"};

    nie::io::LidarType const kType = nie::io::LidarType::kVelodyneHDL32;
    boost::filesystem::path const kFilepathIntrinsics{kBasePath / "velodyne_calibration_HDL-32E.xml"};

    boost::filesystem::path const kFilepathBeijingRampMerged{kBasePath / "beijing_ramp_merged.pcap"};
    boost::filesystem::path const kPathShanghaiOverpassPcap{kBasePath / "Shanghai-overpass" / "pcap" / "subset"};

    nie::io::velodyne::PcapFileStreamer CreateStreamer(std::vector<boost::filesystem::path> files) {
        return nie::io::velodyne::CreatePcapFileStreamer(kType, kFilepathIntrinsics.string(), std::move(files));
    };
};

TEST_F(VelodyneStreamerTest, OnePcap) { lidar_tests::StreamTest(CreateStreamer({kFilepathBeijingRampMerged})); }

TEST_F(VelodyneStreamerTest, TwoPcaps) {
    auto const all_pcap{nie::FindFiles(kPathShanghaiOverpassPcap)};
    ASSERT_TRUE(all_pcap.size() >= 2);
    lidar_tests::StreamTest(CreateStreamer({all_pcap[0], all_pcap[1]}));
}

TEST_F(VelodyneStreamerTest, TwoPcapsCountSweeps) {
    auto const pcap_files{nie::FindFiles(kPathShanghaiOverpassPcap)};
    ASSERT_TRUE(pcap_files.size() >= 2);
    lidar_tests::TwoPcapsCountSweepsTest(
            [&](std::vector<boost::filesystem::path> files) { return CreateStreamer(std::move(files)); },
            {pcap_files[0], pcap_files[1]});
}
