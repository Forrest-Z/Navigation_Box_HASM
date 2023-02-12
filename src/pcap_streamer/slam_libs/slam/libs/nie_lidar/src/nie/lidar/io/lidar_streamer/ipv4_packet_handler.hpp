/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

/// TODO: Handle packages coming in out of order
/// TODO: Do not output error messages when reading fragmented packets at start of pcap.
#pragma once

#include <memory>
#include <vector>

#include <netinet/in.h>

#include "nie/lidar/io/packet_producer.hpp"
#include "udp_packet_handler.hpp"

namespace nie {

namespace io {

struct Ipv4Header {
private:
    std::uint8_t const* data_;

public:
    Ipv4Header(std::uint8_t const* data) : data_(data) { CheckIfValid(); }
    // Structure taken from: https://en.wikipedia.org/wiki/IPv4#Header
    // NOTE: networking packets are big endian!
    // Byte 0:
    // xxxx .... Version of ip protocol (assumed to be 4)
    // .... xxxx Size of the ipv4 header in in 32 bit words. E.g. minimum 5: 5 x 32 / 8 = 20 bytes
    uint ip_version() const { return data_[0] >> 4; };
    uint header_size() const { return (data_[0] & 0x0f) * 4; };
    // Byte 1:
    // xxxx xx.. Differentiated Services Code Point
    // .... ..xx Explicit Congestion Notification
    uint dscp() const { return data_[1] >> 2; };
    uint ecn() const { return data_[1] & 0x03; };
    // Bytes 2/3:
    // xxxx xxxx xxxx xxxx Size of the ipv4 packet in bytes (including ipv4 header)
    uint packet_size() const { return (data_[2] << 8) + data_[3]; };
    // Bytes 4/5:
    // xxxx xxxx xxxx xxxx ID of the ipv4 packet
    uint packet_id() const { return (data_[4] << 8) + data_[5]; };
    // Bytes 6/7:
    // x... .... .... .... reserved
    // .x.. .... .... .... don't fragment (DF)
    // ..x. .... .... .... more fragments (MF)
    // ...x xxxx xxxx xxxx fragment offset in 8 byte blocks
    // Reserved flag must always be 0.
    bool reserved() const { return 1 & (data_[6] >> 7); };
    // Flag whether the packet is allowed to be fragmented or not
    bool dont_fragment() const { return 1 & (data_[6] >> 6); };
    // Flag whether more fragments are expected to be coming
    // If fragmented, the MF flag is set for all packets except the last one.
    // For that one, the MF flag is 0, but the offset still is nonzero, distincting it from regular packets.
    bool more_fragments() const { return 1 & (data_[6] >> 5); };
    // Offset of the total packet data this fragment contains in 8 byte blocks.
    uint fragment_offset() const { return (((data_[6] & 0x1f) << 8) + data_[7]) << 3; };
    // Byte 8: Time to Live
    uint time_to_live() const { return data_[8]; };
    // Byte 9: Protocol used in data partition
    uint protocol() const { return data_[9]; };

    void CheckIfValid() {
        CHECK_EQ(ip_version(), 4) << "Packet IP header version is not equal to 4. This IP version is not supported.";
        CHECK(!reserved()) << "Reserved flag is set, but it should always be 0. Something is wrong with this packet.";
        CHECK_EQ(protocol(), IPPROTO_UDP)
                << "Given protocol is not UDP (17) but " << protocol() << " which is not supported.";
    };

    // Size of the data contained within ip packet.
    uint data_size() const { return packet_size() - header_size(); };
};

class Ipv4PacketHandler {
public:
    Ipv4PacketHandler() : has_fragmented_data{false}, last_packet_id{0}, expected_data_offset(0), output_data{{}} {}

    /// \brief Generates a packet based on input ipv4 data
    /// \param data [in] pointer to start of byte data
    /// \param packet [out] generated network packet
    /// \return [TRUE] if packet was created, [FALSE] if it is fragmented and incomplete
    /// \remark the packet will remain empty while it is still fragmented
    template <typename Packet>
    bool GeneratePacket(std::uint8_t const* data, std::unique_ptr<Packet>* packet) {
        Ipv4Header header(data);

        // Check if packet is fragmented
        if (header.more_fragments() || header.fragment_offset()) {
            CheckPacketID(header);
            if (!CheckDataOffset(header)) {
                return false;
            }

            last_packet_id = header.packet_id();
        } else {
            // Not fragmented, so we clear previous data
            output_data.clear();
        }

        AddDataToOutput(data, header);

        // Check if we have a complete packet
        if (!CheckPacketCompleted(header)) {
            return false;
        }

        // Process the udp packet
        GenerateUdpPacket(output_data.data(), packet);

        return true;
    }

private:
    /// \brief Checks whether the fragmented packet has the same id as the previous
    /// \param header ipv4 header information
    /// \note Will clear the stored data if it is different
    void CheckPacketID(Ipv4Header const& header) {
        if (header.packet_id() != last_packet_id) {
            if (has_fragmented_data) {
                LOG(INFO) << "Incoming packet has different packet_id than previous fragmented packet. Previous packet "
                             "thrown away.";
            }
            output_data.clear();
            expected_data_offset = 0;
        }
    }

    /// \brief Checks whether the data offset corresponds with the last packet received
    /// \param header ipv4 header information
    /// \return [TRUE] if offset is correct, [FALSE] if it is wrong
    bool CheckDataOffset(Ipv4Header const& header) {
        if (header.fragment_offset() != expected_data_offset) {
            LOG(INFO) << "Incoming fragment offset has different packet offset than last packet. "
                      << header.fragment_offset() << " vs " << expected_data_offset << ".Throwing away old data.";
            if (header.fragment_offset() != 0) {
                // The packet does not match the previous, but it also is not the start of a new fragmented packet.
                // So we just ignore it and wait untill we do get a good packet.
                return false;
            }
            // We clear the stored data, and we can start reassembly again.
            output_data.clear();
        }
        return true;
    }

    /// \brief Adds the byte data to the output vector
    /// \param data pointer to start of byte data
    /// \param header ipv4 header information
    void AddDataToOutput(std::uint8_t const* data, Ipv4Header const& header) {
        // We only add the data, not the header
        output_data.insert(output_data.end(), data + header.header_size(), data + header.packet_size());
    }

    /// \brief Checks if the reassembly of a fragmented packet is completed
    /// \param header ipv4 header information
    /// \return [TRUE] if completed, [FALSE] if still fragmented
    bool CheckPacketCompleted(Ipv4Header const& header) {
        if (header.more_fragments()) {
            // Packet is not completed.
            // Set fragmented boolean
            has_fragmented_data = true;
            // Store the offset we expect the next packet to have
            expected_data_offset = header.fragment_offset() + header.data_size();
            return false;
        }
        // Packet is completed
        // We no longer will have fragmented data
        has_fragmented_data = false;
        // And the offset we expect is now 0 again
        expected_data_offset = 0;
        return true;
    }

    // Flag to indicate whether we do or do not have fragmented data
    bool has_fragmented_data;

    // Stores the packet id of the last fragmented packet
    uint last_packet_id;

    // Stores the data offset (in bytes) to the memory location where we expect to write data next.
    uint expected_data_offset;

    // Vector used to store (fragments of) the output data in bytes
    std::vector<std::uint8_t> output_data;
};

}  // namespace io

}  // namespace nie
