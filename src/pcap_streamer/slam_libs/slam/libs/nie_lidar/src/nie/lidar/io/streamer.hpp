/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <cstdint>

#include <atomic>
#include <memory>
#include <thread>

#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <nie/core/work_pool.hpp>

// Local includes
#include "packet_producer.hpp"

namespace nie {

namespace io {

// TODO (jbr): The consumer producer refactoring is not done yet. For the TODO, look at (for example)
// lidar_packet_producer.hpp

template <typename PacketProducer, typename PacketConsumer>
class Streamer {
public:
    Streamer(PacketProducer&& producer, PacketConsumer&& consumer)
        : packet_consumer{std::move(consumer)}, packet_producer_{std::move(producer)} {}

    ~Streamer() noexcept { Stop(); }

    Streamer(Streamer const&) = delete;
    Streamer& operator=(Streamer const&) = delete;

    // TODO: A different/better design might avoid having to implement this move functionality.
    // TODO: Likely the producer and consumer are actually not moved, but copied.
    Streamer(Streamer&& other)
        : packet_consumer{std::move(other.packet_consumer)}, packet_producer_{std::move(other.packet_producer_)} {
        CHECK(!IsRunning() && !other.IsRunning()) << "It is not allowed to move a streamer when it is running";
    }
    Streamer& operator=(Streamer&& other) {
        CHECK(!IsRunning() && !other.IsRunning()) << "It is not allowed to move a streamer when it is running";
        packet_producer_ = std::move(other.packet_producer_);
        packet_consumer = std::move(other.packet_consumer);
        return *this;
    }

    /// \brief Identical to Callback::AddCallback(), but enforces that the Streamer is not running
    template <typename PacketConsumer::CallbackTags index, typename Function>
    void AddCallback(Function function) {
        CHECK(!IsRunning()) << "It is not allowed to add a callback when the streamer is running";
        packet_consumer.template AddCallback<index, Function>(function);
    }

    /// Start reading and processing data packets, from the specified packet source
    void Start() {
        if (IsRunning()) return;

        // Reset stop condition
        read_packets_ = true;

        packet_producer_.Start();

        // Create thread that processes packets
        // NOTE: We let only one worker thread do packet processing, because we want to ensure that the order of packets
        //       as stored in a pcap file is also the order of processing. This ensures a reproducible output. The order
        //       of processing of packets received from a socket is not so important, since UDP packets are not
        //       guaranteed to be received in order anyway.
        packet_consumer_thread_ = std::make_unique<WorkPool<Packet>>(
                1, kPacketBufferSize, std::bind(&Streamer::ProcessPacket, this, std::placeholders::_1));

        // Create thread that produces packets
        packet_producer_thread_ = std::make_unique<std::thread>(&Streamer::ReadPackets, this);
    }

    /// Stop reading and processing data packets,
    void Stop() {
        // Set stop condition
        read_packets_ = false;

        packet_producer_.Stop();

        // Shut down packet producer, but wait until the packet producer has finished its current packet
        if (packet_producer_thread_ && packet_producer_thread_->joinable()) packet_producer_thread_->join();
        packet_producer_thread_.reset();

        // Shut down the packet processor; This implicitly wait until all packets remaining in its buffer are processed
        packet_consumer_thread_.reset();
    }

    /// Check if the streamer is still running
    [[nodiscard]] bool IsRunning() const {
        return read_packets_ || (packet_consumer_thread_ && !packet_consumer_thread_->queue().Empty());
    }

    PacketConsumer packet_consumer;

private:
    using Packet = typename PacketProducer::Packet;

    void ProcessPacket(std::unique_ptr<Packet> const& packet) { packet_consumer.ProcessPacket(packet); }

    void ReadPackets() {
        std::unique_ptr<Packet> packet;

        while (read_packets_ && packet_producer_.IsOpen()) {
            if (packet_producer_.Produce(&packet)) {
                // TODO(jbr): The network version used TryPushBack instead of blocking.
                // TODO(jbr): This is reasonable for a producer when the consumer isn't fast enough.
                // TODO(jbr): Normally the producer has his own buffer but this small bit of code/solution.
                // TODO(jbr): See packet_producer.hpp
                // Enqueue the packet for the processor
                packet_consumer_thread_->queue().BlockingPushBack(std::move(packet));
            }
        }

        // This is not the same as stop, read_packets_ is false because there simply aren't any packets anymore
        read_packets_ = false;
    }

    // Maximum number of packets that can be buffered (This also implies maximum buffering latency)
    constexpr static std::size_t kPacketBufferSize{512};

    std::atomic_bool read_packets_{false};
    PacketProducer packet_producer_;

    std::unique_ptr<std::thread> packet_producer_thread_{nullptr};
    // TODO(jbr) It only uses one thread and we can only consume with one thread. Make this simpler later just as a
    // single thread.
    std::unique_ptr<WorkPool<Packet>> packet_consumer_thread_{nullptr};
};

}  // namespace io

}  // namespace nie
