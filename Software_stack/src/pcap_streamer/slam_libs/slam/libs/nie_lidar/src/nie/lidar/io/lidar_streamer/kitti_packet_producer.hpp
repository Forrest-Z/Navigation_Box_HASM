/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <memory>

#include <glog/logging.h>
#include <pcap/pcap.h>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <nie/core/filesystem.hpp>
#include <nie/formats/kitti/readers.hpp>
#include "nie/lidar/helper_point_types.hpp"

#include "kitti_packets.hpp"
#include "nie/lidar/io/packet_producer.hpp"

namespace nie {

namespace io {

namespace kitti {

class TextFileProducer : public PacketProducer<Packet> {
public:
    // The vector of paths is assumed contain the start timestamps file, end timestamps file and then all the points
    // data files.
    explicit TextFileProducer(std::vector<boost::filesystem::path> paths)
        : start_times_{ReadTimestamps(paths.cbegin()->string())},
          end_times_{ReadTimestamps((paths.cbegin() + 1)->string())},
          point_data_files_{StripFirstTwo(std::move(paths))} {
        CHECK(start_times_.size() == end_times_.size()) << start_times_.size() << " vs. " << end_times_.size();
        CHECK(start_times_.size() >= point_data_files_.size())
                << start_times_.size() << " vs. " << point_data_files_.size();
    }

    ~TextFileProducer() override = default;

    void Start() override {}

    void Stop() override {}

    [[nodiscard]] bool IsOpen() const override { return sweep_data_index_ < point_data_files_.size(); }

    [[nodiscard]] bool Produce(std::unique_ptr<Packet>* packet) override {
        bool const result = sweep_data_index_ < point_data_files_.size();
        if (result) {
            CHECK(boost::filesystem::is_regular_file(point_data_files_[sweep_data_index_]))
                    << "Supplied file " << point_data_files_[sweep_data_index_] << " does not exist.";
            *packet = std::make_unique<Packet>(
                    Packet{start_times_[sweep_data_index_],
                           end_times_[sweep_data_index_],
                           ReadPointData(point_data_files_[sweep_data_index_])});
        }
        ++sweep_data_index_;
        return result;
    }

private:
    static std::vector<boost::filesystem::path> StripFirstTwo(std::vector<boost::filesystem::path> paths) {
        CHECK(paths.size() >= 2);
        std::vector<boost::filesystem::path> result;
        result.reserve(paths.size() - 2);
        std::move(paths.begin() + 2, paths.end(), std::back_inserter(result));
        return result;
    }

    static pcl::PointCloud<pcl::PointXYZI> ReadPointData(boost::filesystem::path const& path) {
        std::vector<Eigen::Vector4f> const points = ReadPoints(path.string());

        pcl::PointCloud<pcl::PointXYZI> result(points.size(), 1);
        std::transform(
                points.begin(),
                points.end(),
                std::back_inserter(result.points),
                [](Eigen::Vector4f const& values) -> pcl::PointXYZI {
                    // Values given are reflectance and so on [0-1] interval, but we expect std::uint16_t
                    CHECK(0.f <= values(3) && values(3) <= 1.f);
                    pcl::PointXYZI p{std::round(values(3) * std::numeric_limits<std::uint16_t>::max())};
                    p.x = values[0];
                    p.y = values[1];
                    p.z = values[2];
                    return p;
                });

        return result;
    }

    std::vector<nie::Timestamp_ns> const start_times_;
    std::vector<nie::Timestamp_ns> const end_times_;
    std::vector<boost::filesystem::path> const point_data_files_;

    std::size_t sweep_data_index_{0};
};

}  // namespace kitti

}  // namespace io

}  // namespace nie
