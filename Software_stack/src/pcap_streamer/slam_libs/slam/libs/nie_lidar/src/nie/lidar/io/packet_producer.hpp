/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <memory>

namespace nie {

namespace io {

// TODO(jbr): Initial refactoring of the packet producer. It needs to be refactored into a thread with a buffer. From
// which the lidar_streamer is a consumer (that in turn produces 3d points).
// TODO(jbr): We can probably consolidate all producers into a single class (see nie_hardware producers).

template <typename Packet_>
class PacketProducer {
public:
    using Packet = Packet_;

    virtual void Start() = 0;
    virtual void Stop() = 0;
    [[nodiscard]] virtual bool IsOpen() const = 0;
    [[nodiscard]] virtual bool Produce(std::unique_ptr<Packet_>* packet) = 0;

    virtual ~PacketProducer() = default;
};

}  // namespace io

}  // namespace nie
