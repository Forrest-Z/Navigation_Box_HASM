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

constexpr VersionType kBboxCollectionVersionLatest = BaGraphVersion(1, 0);

struct BboxCollection;  // Forward declaration.

namespace graph {

template <>
inline std::string Extension<BboxCollection>() {
    return ".bbox";
}

}  // namespace graph

struct BboxRecord {
    PoseId id;
    Eigen::Vector3d min;
    Eigen::Vector3d max;
};

struct BboxHeader {
    using Collection = BboxCollection;

    VersionType version = kBboxCollectionVersionLatest;
};

struct BboxCollection {
    using Header = BboxHeader;

    static const detail::SignatureType Signature;

    BboxHeader header;
    std::vector<BboxRecord> boxes;
};

using BboxCollectionStreamWriter = detail::CollectionStreamWriter<BboxCollection, BboxHeader, BboxRecord>;

using BboxCollectionStreamReader = detail::CollectionStreamReader<BboxCollection, BboxHeader, BboxRecord>;

[[nodiscard]] inline bool Equals(BboxRecord const& l, BboxRecord const& r);

[[nodiscard]] inline bool Equals(BboxHeader const& l, BboxHeader const& r);

}  // namespace io

}  // namespace nie

#include "bbox_collection.inl"
