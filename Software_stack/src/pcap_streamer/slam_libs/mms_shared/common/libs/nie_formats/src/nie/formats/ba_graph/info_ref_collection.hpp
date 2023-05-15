/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>
#include <vector>

#include "ba_graph_types.hpp"
#include "pose_collection.hpp"

// To expose the collection readers/writer when this collection is included
#include "collection_reader.hpp"
#include "collection_stream_reader.hpp"
#include "collection_stream_writer.hpp"
#include "collection_writer.hpp"

namespace nie {

namespace io {

constexpr VersionType kInfoRefCollectionVersionLatest = BaGraphVersion(1, 1);

struct InfoRefCollection;  // Forward declaration.

namespace graph {

template <>
inline std::string Extension<InfoRefCollection>() {
    return ".iref";
}

}  // namespace graph

struct InfoRefRecord {
    PoseId id;
    std::uint8_t frame_id;  // index of the coordinate frame in the intrinsics file
    std::string path;
};

struct InfoRefHeader {
    using Collection = InfoRefCollection;

    VersionType version = kInfoRefCollectionVersionLatest;
};

struct InfoRefCollection {
    using Header = InfoRefHeader;

    static const detail::SignatureType Signature;

    InfoRefHeader header;
    std::vector<InfoRefRecord> info_refs;
};

using InfoRefCollectionStreamWriter = detail::CollectionStreamWriter<InfoRefCollection, InfoRefHeader, InfoRefRecord>;

using InfoRefCollectionStreamReader = detail::CollectionStreamReader<InfoRefCollection, InfoRefHeader, InfoRefRecord>;

[[nodiscard]] inline bool Equals(InfoRefRecord const& l, InfoRefRecord const& r);

[[nodiscard]] inline bool Equals(InfoRefHeader const& l, InfoRefHeader const& r);

}  // namespace io

}  // namespace nie

#include "info_ref_collection.inl"
