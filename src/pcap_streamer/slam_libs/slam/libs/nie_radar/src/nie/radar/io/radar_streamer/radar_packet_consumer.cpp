/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

// Local includes
#include "radar_packet_consumer.hpp"
#include "continental.hpp"

namespace nie::io::radar {

void PacketConsumer::ProcessPacket(std::unique_ptr<Packet> const& packet) {
    continental::SomeIpHeader const* header = reinterpret_cast<continental::SomeIpHeader const*>(packet->data());

    if (static_cast<continental::ServiceId>(continental::BigToLittle16(header->service_id)) ==
                continental::ServiceId::kObjects &&
        callbacks_.HasCallback<CallbackTags::kObjects>()) {
        continental::Objects const* raw_objects = reinterpret_cast<continental::Objects const*>(
                packet->data() + sizeof(continental::SomeIpHeader) + sizeof(continental::E2EP06Header));

        std::vector<Object> objects;
        objects.reserve(raw_objects->list.size());

        for (auto const& o : raw_objects->list) {
            // Same as: objects.push_back(Object(o));
            objects.emplace_back(o);
        }

        callbacks_.Callback<CallbackTags::kObjects>(std::move(objects));
    }
}

}  // namespace nie::io::radar