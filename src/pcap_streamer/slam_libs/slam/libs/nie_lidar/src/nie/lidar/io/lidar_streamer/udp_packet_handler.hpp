/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include "nie/lidar/io/packet_producer.hpp"

namespace nie {

namespace io {

struct UdpHeader {
private:
    std::uint8_t const* data_;

public:
    UdpHeader(std::uint8_t const* data) : data_(data) {}
    // Structure taken from: https://en.wikipedia.org/wiki/User_Datagram_Protocol#IPv4_pseudo_header
    // NOTE: networking packets are big endian!
    // Byte 0/1: source port
    uint port_src() const { return (data_[0] << 8) + data_[1]; };
    // Byte 2/3: destination port
    uint port_dst() const { return (data_[2] << 8) + data_[3]; };
    // Bytes 4/5: Size of the udp packet in bytes (including udp header)
    uint packet_size() const { return (data_[4] << 8) + data_[5]; };

    // Bytes 6/7 contain the checksum
    // So a header size of 8 bytes
    uint constexpr header_size() { return 8; };
    // Size of the data contained within ip packet.
    uint data_size() { return packet_size() - header_size(); };
};

/// \brief Generates a packet based on input ipv4 data
/// \param data [in] pointer to start of udp byte data
/// \param packet [out] generated network packet
/// \return [TRUE] if packet was created, [FALSE] if it failed
template <typename Packet>
bool GenerateUdpPacket(std::uint8_t const* data, std::unique_ptr<Packet>* packet) {
    UdpHeader header(data);

    // Create a copy of the packet data (excluding UDP header at start of data)
    *packet = std::make_unique<Packet>(data + header.header_size(), data + header.packet_size());

    return true;
}

}  // namespace io

}  // namespace nie
