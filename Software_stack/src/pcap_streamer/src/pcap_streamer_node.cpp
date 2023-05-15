/**
 * Copyright (C) 2021, 2022 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#include "pcap_streamer_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <boost/algorithm/string.hpp>
#include <nie/core/filesystem.hpp>
#include <nie/formats/calib3d/lidar_parameters.hpp>
#include <nie/lidar/io/lidar_streamer/ouster_streamer.hpp>
#include <nie/lidar/io/lidar_streamer/velodyne_streamer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {
enum class StreamType { kPcap = 0, kNetwork };
std::vector<std::string> const kStreamTypeStrings = {"Pcap", "Network"};

StreamType StreamTypeStringToEnum(std::string const& stream_type_string) {
    auto it = std::find(kStreamTypeStrings.begin(), kStreamTypeStrings.end(), stream_type_string);
    CHECK(it != kStreamTypeStrings.cend()) << "Unknown stream type: " << stream_type_string;
    return static_cast<StreamType>(std::distance(kStreamTypeStrings.begin(), it));
}
}  // namespace

template <typename Streamer>
struct StreamerTraits;
template <typename Packet, typename PacketConsumer>
struct StreamerTraits<nie::io::Streamer<nie::io::PcapFileProducer<Packet>, PacketConsumer>> {
    static constexpr bool kNetwork = false;
};
template <typename Packet, typename PacketConsumer>
struct StreamerTraits<nie::io::Streamer<nie::io::NetworkProducer<Packet>, PacketConsumer>> {
    static constexpr bool kNetwork = true;
};

template <typename PcapStreamer>
class PcapPublisherImpl : public PcapPublisher {
public:
    PcapPublisherImpl(
            PcapStreamer streamer,
            int hertz,
            std::string const& output_topic_points,
            std::string const& output_topic_stamps,
            rclcpp::Node* node)
        : streamer_(std::move(streamer)),
          publisher_sweep_points_(node->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_points, 10)),
          // publisher_sweep_stamps_(node->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_stamps, 10)),
          node_(node) {
        Run(hertz);
    }

private:
    void Run(int hertz) {
        auto prev = std::chrono::high_resolution_clock::now();
        auto curr = prev;
        auto delta_time_threshold = std::chrono::duration<double>(1.0 / static_cast<double>(hertz));

        if (hertz < 0 || StreamerTraits<PcapStreamer>::kNetwork) {
            streamer_.template AddCallback<nie::io::lidar::LidarCallbackTags::kSweep>(
                    [this](nie::io::lidar::Returns returns) { Publish(returns); });
        } else if (hertz > 0) {
            streamer_.template AddCallback<nie::io::lidar::LidarCallbackTags::kSweep>(
                    [this, &prev, &curr, delta_time_threshold](nie::io::lidar::Returns returns) {
                        curr = std::chrono::high_resolution_clock::now();

                        auto delta_time = std::chrono::duration<double>(curr - prev);
                        if (delta_time <= delta_time_threshold) {
                            std::this_thread::sleep_for(delta_time_threshold - delta_time);
                        }

                        Publish(returns);
                        prev = std::chrono::high_resolution_clock::now();
                    });
        }

        streamer_.Start();
        while (streamer_.IsRunning()) {
            std::this_thread::sleep_for(std::chrono::microseconds{1});
        }
        streamer_.Stop();
    }

    void Publish(nie::io::lidar::Returns const& returns) const {
        // Publish will create a unique pointer. So, to prevent a copy we create it as a unique pointer
        // here as well.
        auto message_sweep_points = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(returns.points, *message_sweep_points);
        message_sweep_points->header.frame_id = "pcap_streamer_frame";
        message_sweep_points->header.stamp = node_->now();

        publisher_sweep_points_->publish(std::move(message_sweep_points));
    }

    PcapStreamer streamer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_sweep_points_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_sweep_stamps_;
    rclcpp::Node* node_;
};

PcapStreamerNode::PcapStreamerNode() : rclcpp::Node("pcap_streamer") {
    std::string output_topic_points_param;
    std::string output_topic_stamps_param;
    std::string lidar_type_param;
    std::string lidar_intrinsics_path_param;
    std::string source_param;
    int lidar_hertz;
    std::string stream_type_param;

    GetParameter("output_topic_points", &output_topic_points_param);
    GetParameter("output_topic_stamps", &output_topic_stamps_param);
    GetParameter("lidar_type", &lidar_type_param);
    GetParameter("lidar_intrinsics_path", &lidar_intrinsics_path_param);
    GetParameter("lidar_hertz", &lidar_hertz);
    GetParameter("source", &source_param);
    GetParameter("stream_type", &stream_type_param);

    RCHECK(get_logger(), "source " << source_param << "is empty.", !source_param.empty());
    RCHECK(get_logger(), "lidar_hertz " << lidar_hertz << " cannot be 0.", lidar_hertz != 0);

    nie::io::LidarType lidar_type = nie::io::LidarTypeStringToEnum(lidar_type_param);
    StreamType stream_type = StreamTypeStringToEnum(stream_type_param);

    // TODO(gab): better to flatten nested switch statement for ease of code readablity
    switch (lidar_type) {
        case nie::io::LidarType::kVelodyneVLP16:
        case nie::io::LidarType::kVelodyneHDL32:
            switch (stream_type) {
                case StreamType::kPcap: {
                    std::vector<boost::filesystem::path> source_paths = nie::ReadLines(
                            source_param, [](std::string const& s) { return boost::filesystem::path{s}; });
                    RCHECK(get_logger(), "source_paths file " << source_param << " is empty.", !source_paths.empty());
                    // Design issue where the velodyne streamer is dependent on an enum with things in it that have
                    // nothing to do with the velodyne.
                    publisher_ = std::make_unique<PcapPublisherImpl<nie::io::velodyne::PcapFileStreamer>>(
                            nie::io::velodyne::CreatePcapFileStreamer(
                                    lidar_type, lidar_intrinsics_path_param, source_paths),
                            lidar_hertz,
                            output_topic_points_param,
                            output_topic_stamps_param,
                            this);
                } break;
                case StreamType::kNetwork: {
                    std::vector<std::string> result;
                    boost::split(result, source_param, boost::is_any_of(":"));
                    std::string ip_address = result.at(0);
                    uint16_t port = std::stoul(result.at(1));
                    // Design issue where the velodyne streamer is dependent on an enum with things in it that have
                    // nothing to do with the velodyne.
                    publisher_ = std::make_unique<PcapPublisherImpl<nie::io::velodyne::NetworkStreamer>>(
                            nie::io::velodyne::CreateNetworkStreamer(
                                    lidar_type, lidar_intrinsics_path_param, ip_address, port),
                            lidar_hertz,
                            output_topic_points_param,
                            output_topic_stamps_param,
                            this);
                } break;
                default:
                    RLOG_FATAL(get_logger(), "Unsupported stream type.");
                    break;
            }
            break;
        case nie::io::LidarType::kOusterOS1_32:
        case nie::io::LidarType::kOusterOS1_128:
        case nie::io::LidarType::kOusterOS2_128:
            switch (stream_type) {
                case StreamType::kPcap: {
                    std::vector<boost::filesystem::path> source_paths = nie::ReadLines(
                            source_param, [](std::string const& s) { return boost::filesystem::path{s}; });
                    RCHECK(get_logger(), "source_paths file " << source_param << " is empty.", !source_paths.empty());
                    publisher_ = std::make_unique<PcapPublisherImpl<nie::io::ouster::PcapFileStreamer>>(
                            nie::io::ouster::CreatePcapFileStreamer(lidar_intrinsics_path_param, source_paths),
                            lidar_hertz,
                            output_topic_points_param,
                            output_topic_stamps_param,
                            this);
                } break;
                case StreamType::kNetwork: {
                    std::vector<std::string> result;
                    boost::split(result, source_param, boost::is_any_of(":"));
                    std::string ip_address = result.at(0);
                    uint16_t port = std::stoul(result.at(1));
                    publisher_ = std::make_unique<PcapPublisherImpl<nie::io::ouster::NetworkStreamer>>(
                            nie::io::ouster::CreateNetworkStreamer(lidar_intrinsics_path_param, ip_address, port),
                            lidar_hertz,
                            output_topic_points_param,
                            output_topic_stamps_param,
                            this);
                } break;
                default:
                    RLOG_FATAL(get_logger(), "Unsupported stream type.");
                    break;
            }
            break;
        default:
            // We skip "kitty" and "unknown". Though unknown should not have existed in the enum.
            RLOG_FATAL(get_logger(), "Unsupported LiDAR type.");
            break;
    }
}
