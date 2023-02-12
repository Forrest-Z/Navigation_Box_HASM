/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <iomanip>
#include <iostream>
#include <memory>

#include <netinet/if_ether.h>

#include "ipv4_packet_handler.hpp"
#include "nie/lidar/io/packet_producer.hpp"

namespace nie {

namespace io {

/// \brief Converts a mac adress in memory to string
/// \param data [in] pointer to start of byte data
/// \return mac adress in format "00:00:00:00:00:00"
static std::string BytesToMAC(std::uint8_t const* data) {
    std::stringstream ss;
    ss << std::hex;
    // Mac is 6 long
    for (size_t i = 0; i < 6; ++i) {
        // setw and setfill are needed to print leading 0's
        ss << std::setw(2) << std::setfill('0') << uint(data[i]);
        if (i != 5) {
            ss << ":";
        }
    }
    return ss.str();
}

struct EthHeader {
private:
    std::uint8_t const* data_;

public:
    EthHeader(std::uint8_t const* data) : data_(data) { CheckIfValid(); }
    // Structure taken from: https://en.wikipedia.org/wiki/Ethernet_frame#Header
    // NOTE: networking packets are big endian!
    // Byte 0-5: Destination MAC
    std::string mac_dst() const { return BytesToMAC(data_); };
    // Byte 6-11: Source MAC
    std::string mac_src() const { return BytesToMAC(data_ + 6); };
    // Byte 12/13: EtherType
    uint ether_type() const { return (data_[12] << 8) + data_[13]; };

    // Header has a size of 14 bytes
    uint constexpr header_size() { return 14; };

    void CheckIfValid() {
        CHECK_EQ(ether_type(), ETHERTYPE_IP)
                << "Given ethernet_type is not IPv4 (0x0800) but 0x" << std::hex << std::setw(4) << std::setfill('0')
                << ether_type() << " which is not supported.";
    };
};

class EthPacketHandler {
public:
    EthPacketHandler() {}

    /// \brief Generates a packet based on input ipv4 data
    /// \param data [in] pointer to start of byte data
    /// \param packet [out] generated network packet
    /// \return [TRUE] if packet was created, [FALSE] if it is fragmented and incomplete
    /// \remark the packet will remain empty while it is still fragmented
    template <typename Packet>
    bool GeneratePacket(std::uint8_t const* data, std::unique_ptr<Packet>* packet) {
        EthHeader header(data);

        // Process the ipv4 packet and forward the result
        return ipv4_packet_handler_.GeneratePacket(data + header.header_size(), packet);
    }

private:
    Ipv4PacketHandler ipv4_packet_handler_;
};

}  // namespace io

}  // namespace nie
