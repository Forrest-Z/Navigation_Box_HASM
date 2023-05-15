/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <thread>

#include <boost/filesystem.hpp>
#include <nie/lidar/io/lidar_streamer/ouster_streamer.hpp>

#include "lidar_tests_common.hpp"

class OusterStreamerTest : public ::testing::Test {
protected:
    boost::filesystem::path const kBasePath{"/data/aiim/unit_tests_data/lidar/io/lidar_streamer"};

    boost::filesystem::path const kFilepathIntrinsics{kBasePath / "OusterTest_08_09_2020.json"};

    boost::filesystem::path const kFilepathOusterTest_08_09_2020_1{kBasePath / "OusterTest_08_09_2020_1.pcap"};
    boost::filesystem::path const kPathTUeCampusPcap{kBasePath / "TUe-campus" / "pcap" / "subset"};

    nie::io::ouster::PcapFileStreamer CreateStreamer(std::vector<boost::filesystem::path> files) {
        return nie::io::ouster::CreatePcapFileStreamer(kFilepathIntrinsics.string(), std::move(files));
    };
};

TEST_F(OusterStreamerTest, OnePcap) { lidar_tests::StreamTest(CreateStreamer({kFilepathOusterTest_08_09_2020_1})); }

TEST_F(OusterStreamerTest, TwoPcaps) {
    auto const all_pcap_files{nie::FindFiles(kPathTUeCampusPcap)};
    ASSERT_TRUE(all_pcap_files.size() >= 2);
    lidar_tests::StreamTest(CreateStreamer({all_pcap_files[0], all_pcap_files[1]}));
}

TEST_F(OusterStreamerTest, TwoPcapsCountSweeps) {
    auto const pcap_files{nie::FindFiles(kPathTUeCampusPcap)};
    ASSERT_TRUE(pcap_files.size() >= 2);
    lidar_tests::TwoPcapsCountSweepsTest(
            [&](std::vector<boost::filesystem::path> files) { return CreateStreamer(std::move(files)); },
            {pcap_files[0], pcap_files[1]});
}
