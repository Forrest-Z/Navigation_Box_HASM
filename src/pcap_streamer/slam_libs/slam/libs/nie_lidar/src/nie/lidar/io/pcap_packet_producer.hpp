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

#include "nie/lidar/io/lidar_streamer/eth_packet_handler.hpp"
#include "packet_producer.hpp"

namespace nie {

namespace io {

template <typename Packet>
class PcapFileProducer : public PacketProducer<Packet> {
public:
    explicit PcapFileProducer(
            std::vector<boost::filesystem::path> paths,
            boost::asio::ip::address source_address_filter = {},
            std::uint16_t source_port_filter = {})
        : pcap_paths_{std::move(paths)},
          pcap_path_index_{0},
          pcap_{nullptr},
          source_address_filter_{std::move(source_address_filter)},
          source_port_filter_{source_port_filter} {}

    void Start() {
        if (IsOpen()) return;

        OpenPcap();
    }

    void Stop() { ClosePcap(); };

    [[nodiscard]] bool IsOpen() const { return pcap_ != nullptr; }

    [[nodiscard]] bool Produce(std::unique_ptr<Packet>* packet) { return NextPacket(packet); }

    ~PcapFileProducer() { Stop(); }

private:
    void ClosePcap() {
        if (IsOpen()) {
            pcap_close(pcap_);
            pcap_ = nullptr;
        }
    }

    void OpenPcap() {
        auto pcap_path = pcap_paths_[pcap_path_index_];
        LOG(INFO) << "Reading pcap " << pcap_path;
        std::array<char, PCAP_ERRBUF_SIZE> error_buffer{};
        pcap_ = pcap_open_offline(pcap_path.c_str(), error_buffer.data());

        ApplyFilter();
    }

    [[nodiscard]] bool NextPcap() {
        ClosePcap();

        pcap_path_index_++;
        if (pcap_path_index_ == pcap_paths_.size()) {
            return false;
        }

        OpenPcap();
        return true;
    }

    void ApplyFilter() {
        std::ostringstream ss("udp ");
        if (!source_address_filter_.is_unspecified()) {
            ss << " and src port " << source_port_filter_ << " and src host " << source_address_filter_.to_string();
        }

        // PCAP_NETMASK_UNKNOWN should be 0xffffffff, but it is undefined in older PCAP versions
        bpf_program filter{};
        if (pcap_compile(pcap_, &filter, ss.str().c_str(), 0, 0xffffffff) == -1) {
            LOG(WARNING) << "Issue compiling filter: " << pcap_geterr(pcap_);
        } else if (pcap_setfilter(pcap_, &filter) == -1) {
            LOG(WARNING) << "Issue setting filter: " << pcap_geterr(pcap_);
        }
    }

    [[nodiscard]] bool NextPacket(std::unique_ptr<Packet>* packet) {
        pcap_pkthdr* header;
        std::uint8_t const* data;

        while (true) {
            if (pcap_next_ex(pcap_, &header, &data) < 1) {
                // There is no packet to read any more, read the next pcap file.
                if (NextPcap()) {
                    // There is another file, start over and try to read a packet again.
                    continue;
                }
                // There are no packets, nor other pcap files to read.
                break;
            }

            // We pass the ethernet frame data on to be generated into a packet
            // If it is completed, it returns true and we finished a packet.
            // If it is (still) fragmented, it returns false and we obtain the next packet.
            if (!eth_packet_handler_.GeneratePacket(data, packet)) {
                continue;
            }

            return true;
        }

        return false;
    }

    std::vector<boost::filesystem::path> pcap_paths_;
    std::size_t pcap_path_index_;
    pcap_t* pcap_;
    boost::asio::ip::address source_address_filter_;
    std::uint16_t source_port_filter_;
    EthPacketHandler eth_packet_handler_;
};

}  // namespace io

}  // namespace nie
