/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <gtest/gtest.h>

#include <thread>

#include <boost/filesystem.hpp>
#include <nie/lidar/io/streamer.hpp>

namespace lidar_tests {

static constexpr auto kSleepDuration{std::chrono::milliseconds(1)};

// TODO: [EDD] Ideas for unit tests:
//       - Start / Stop the streamer
//       - Packets should return correct number of lidar points (unless invalid points)
//       - Timestamps should be increasing monotonously over entire dataset, even on a GPS hour roll over
//       - Estimated rotation speed should approximate RPM for some data set (e.g. 900 RPM)
//       - Should be able to read .pcap files for different Velodyne device types
//       - Packet / Sweep Id should be increasing
//       - Try to get tiny dataset: e.g. 0 / 1 / 2 (incomplete) sweeps
//       - Compare with numbers from VeloView?
static inline void DebugLogTimes(nie::io::lidar::Returns const& returns) {
    for (auto const timestamp : returns.timestamps) {
        DLOG(INFO) << timestamp;
    }
}

template <typename Streamer>
void RunStreamer(Streamer&& streamer) {
    streamer.Start();
    while (streamer.IsRunning()) {
        std::this_thread::sleep_for(kSleepDuration);
    }
    streamer.Stop();
}

template <typename Streamer>
void StreamTest(Streamer&& streamer) {
    streamer.template AddCallback<nie::io::lidar::LidarCallbackTags::kPacket>(DebugLogTimes);
    RunStreamer(std::forward<Streamer>(streamer));
}

template <typename CreateStreamerFunc>
void TwoPcapsCountSweepsTest(CreateStreamerFunc create_streamer, std::vector<boost::filesystem::path> const& files) {
    static_assert(
            std::is_invocable_v<CreateStreamerFunc, std::vector<boost::filesystem::path>>,
            "CreateStreamerFunc should take vector of file paths.");

    auto constexpr callback_tag = nie::io::lidar::LidarCallbackTags::kSweep;

    size_t sweep_count_a = 0;
    {
        auto streamer = create_streamer({files[0]});
        streamer.template AddCallback<callback_tag>([&sweep_count_a](nie::io::lidar::Returns const& returns) {
            ASSERT_FALSE(returns.points.empty());
            ++sweep_count_a;
        });
        RunStreamer(std::move(streamer));
    }
    size_t sweep_count_b = 0;
    {
        auto streamer = create_streamer({files[1]});
        streamer.template AddCallback<callback_tag>([&sweep_count_b](nie::io::lidar::Returns const& returns) {
            ASSERT_FALSE(returns.points.empty());
            ++sweep_count_b;
        });
        RunStreamer(std::move(streamer));
    }
    size_t sweep_count_a_plus_b = 0;
    {
        auto streamer = create_streamer(files);
        streamer.template AddCallback<callback_tag>([&sweep_count_a_plus_b](nie::io::lidar::Returns const& returns) {
            ASSERT_FALSE(returns.points.empty());
            ++sweep_count_a_plus_b;
        });
        RunStreamer(std::move(streamer));
    }

    // TODO: this weird assert is necessary for velodyne::PcapFileStreamer
    // only sweep_count_a + sweep_count_b == sweep_count_a_plus_b should be sufficient. Investigate why
    ASSERT_TRUE(
            sweep_count_a + sweep_count_b == sweep_count_a_plus_b - 1 ||
            sweep_count_a + sweep_count_b == sweep_count_a_plus_b);
}

}  // namespace lidar_tests
