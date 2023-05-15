/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <functional>

#include <nie/core/callbacks.hpp>
#include <nie/lidar/io/streamer.hpp>

#include "continental.hpp"
#include "radar_data_types.hpp"

namespace nie::io::radar {

using Packet = std::vector<std::uint8_t>;

enum class RadarCallbackTags : std::size_t { kObjects };

class PacketConsumer {
public:
    using CallbackTags = RadarCallbackTags;

    /// \brief Identical to Callback::AddCallback(), but enforces that the Streamer is not running
    template <CallbackTags index, typename Function>
    void AddCallback(Function function) {
        callbacks_.AddCallback<index, Function>(function);
    }

    void ProcessPacket(std::unique_ptr<Packet> const& packet);

protected:
    using CallbacksType = Callbacks<RadarCallbackTags, std::function<void(std::vector<Object>)>>;

    CallbacksType callbacks_;
};

}  // namespace nie::io::radar
