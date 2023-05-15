/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <thread>

#include <nie/radar/io/radar_streamer/radar_streamer.hpp>

// TODO(davide.congiu): test against real life data. Right now it just receives objects
void GetObjectsList(std::vector<nie::io::radar::Object> objects) {
    if (objects.empty()) {
        return;
    }
}

template <typename Streamer>
void RunStreamer(Streamer&& streamer) {
    streamer.Start();
    while (streamer.IsRunning()) {
        std::this_thread::sleep_for(std::chrono::microseconds{1});
    }
    streamer.Stop();
}

template <typename Streamer>
void StreamTest(Streamer&& streamer) {
    streamer.template AddCallback<nie::io::radar::RadarCallbackTags::kObjects>(
            [](auto objects) { GetObjectsList(std::move(objects)); });
    RunStreamer(std::forward<Streamer>(streamer));
}

class RadarStreamerTest : public ::testing::Test {
protected:
    boost::filesystem::path const kBasePath{"/data/aiim/unit_tests_data/radar/io/radar_streamer"};

    boost::filesystem::path const kFilepathContinentalTest{kBasePath / "ContinentalTest.pcap"};

    nie::io::radar::PcapFileStreamer CreateStreamer(std::vector<boost::filesystem::path> files) {
        return nie::io::radar::CreatePcapFileStreamer(std::move(files));
    }
};

TEST_F(RadarStreamerTest, OnePcap) { StreamTest(CreateStreamer({kFilepathContinentalTest})); }
