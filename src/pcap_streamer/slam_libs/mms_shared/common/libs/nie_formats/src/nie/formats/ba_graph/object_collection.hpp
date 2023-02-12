/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <vector>

#include <Eigen/Geometry>

#include "ba_graph_types.hpp"

// To expose the collection readers/writer when this collection is included
#include "collection_reader.hpp"
#include "collection_stream_reader.hpp"
#include "collection_stream_writer.hpp"
#include "collection_writer.hpp"

namespace nie {

namespace io {

constexpr VersionType kObjectCollectionVersionLatest = BaGraphVersion(1, 1);

struct ObjectCollection;  // Forward declaration

namespace graph {

template <>
inline std::string Extension<ObjectCollection>() {
    return ".objt";
}

}  // namespace graph

struct ObjectRecord {
    std::int32_t id;

    Eigen::Vector3d position;
    Eigen::Matrix3d information;  // Optional: when flag kHasInformationPerRecord in ObjectHeader is set
};

struct ObjectHeader {
    inline bool HasInformation() const { return (flags & kHasInformation) > 0; }

    inline bool HasInformationPerRecord() const { return (flags & kHasInformationPerRecord) > 0; }

    inline bool HasAnyInformation() const { return HasInformation() || HasInformationPerRecord(); }

    using Collection = ObjectCollection;

    using Flags = std::uint32_t;
    enum Flag : Flags { kHasInformation = 0x000F, kHasInformationPerRecord = 0x00F0 };

    [[nodiscard]] inline bool Has(Flag const& flag) const { return (flags & flag) > 0; }
    inline void Set(Flag const& flag) { flags |= flag; }
    inline void Unset(Flag const& flag) { flags &= ~flag; }

    VersionType version = kObjectCollectionVersionLatest;

    Flags flags;

    Eigen::Matrix3d information;  // Optional: when flag kHasInformation is set
};

struct ObjectCollection {
    using Header = ObjectHeader;

    static const detail::SignatureType Signature;

    ObjectHeader header;
    std::vector<ObjectRecord> objects;
};

using ObjectCollectionStreamWriter = detail::CollectionStreamWriter<ObjectCollection, ObjectHeader, ObjectRecord>;

using ObjectCollectionStreamReader = detail::CollectionStreamReader<ObjectCollection, ObjectHeader, ObjectRecord>;

}  // namespace io

}  // namespace nie

#include "object_collection.inl"
