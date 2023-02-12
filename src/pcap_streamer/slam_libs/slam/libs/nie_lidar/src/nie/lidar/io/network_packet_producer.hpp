/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <glog/logging.h>
#include <boost/asio.hpp>
#include <memory>
#include <utility>

#include "nie/lidar/io/packet_producer.hpp"

namespace nie {

namespace io {

// TODO(jbr): This class is based on code stripped from lidar_streamer. Note that this code was never practically
// called. It is preserved because it may be useful.
template <typename Packet>
class NetworkProducer : public PacketProducer<Packet> {
public:
    // Maximum UDP packet size from Lidar
    static constexpr std::size_t kMaxPacketSize = 24896;

    // TODO(jbr): These defaults were never used, but now they are: are they good?
    explicit NetworkProducer(
            std::string const& ip_address,
            std::uint16_t const& port,
            boost::asio::ip::address source_address_filter = {},
            std::uint16_t source_port_filter = {})
        : udp_listener_endpoint_{boost::asio::ip::address::from_string(ip_address), port},
          io_service_{},
          socket_{nullptr},
          source_address_filter_{std::move(source_address_filter)},
          source_port_filter_{source_port_filter} {}

    NetworkProducer(NetworkProducer const&) = delete;

    NetworkProducer& operator=(NetworkProducer const&) = delete;

    NetworkProducer(NetworkProducer&& other)
        : udp_listener_endpoint_(std::move(other.udp_listener_endpoint_)),
          socket_(std::move(other.socket_)),
          buffer_(std::move(other.buffer_)),
          sender_endpoint_(std::move(other.sender_endpoint_)),
          source_address_filter_(std::move(other.source_address_filter_)),
          source_port_filter_(std::move(other.source_port_filter_)) {}

    NetworkProducer& operator=(NetworkProducer&& other) {
        CHECK(io_service_.stopped() && other.io_service_.stopped())
                << "It is not allowed to move when io service is running";
        if (this != &other) {
            udp_listener_endpoint_ = std::move(other.udp_listener_endpoint_);
            socket_ = std::move(other.socket_);
            buffer_ = std::move(other.buffer_);
            source_address_filter_ = std::move(other.source_address_filter_);
            source_port_filter_ = std::move(other.source_port_filter_);
            sender_endpoint_ = std::move(other.sender_endpoint_);
        }
        return *this;
    }

    ~NetworkProducer() noexcept { Stop(); }

    void Start() {
        if (IsOpen()) return;

        try {
            try {
                socket_ = std::make_unique<boost::asio::ip::udp::socket>(io_service_, udp_listener_endpoint_);
            } catch (boost::system::system_error const&) {
                auto const endpoint = boost::asio::ip::udp::endpoint(
                        boost::asio::ip::address_v4::any(), udp_listener_endpoint_.port());

                socket_ = std::make_unique<boost::asio::ip::udp::socket>(io_service_, endpoint);
            }

            io_service_.run();
        } catch (std::exception& e) {
            LOG(FATAL) << "Unable to bind to socket: " << e.what();
        }
    }

    void Stop() {
        if (IsOpen()) socket_.reset();
    }

    [[nodiscard]] bool IsOpen() const { return socket_ && socket_->is_open(); }

    [[nodiscard]] bool Produce(std::unique_ptr<Packet>* packet) {
        std::size_t length = socket_->receive_from(boost::asio::buffer(buffer_), sender_endpoint_);

        if (source_address_filter_.is_unspecified() ||
            (source_address_filter_ == sender_endpoint_.address() && source_port_filter_ == sender_endpoint_.port())) {
            // Create a copy of the packet data
            *packet = std::make_unique<Packet>(buffer_.data(), buffer_.data() + length);
            return true;
        }

        return false;
    }

private:
    boost::asio::ip::udp::endpoint udp_listener_endpoint_;
    boost::asio::io_service io_service_;
    std::unique_ptr<boost::asio::ip::udp::socket> socket_;

    // Expect packets to fit in a single Ethernet frame
    std::array<std::uint8_t, kMaxPacketSize> buffer_;
    boost::asio::ip::udp::endpoint sender_endpoint_;

    boost::asio::ip::address source_address_filter_;
    std::uint16_t source_port_filter_;
};

}  // namespace io

}  // namespace nie
