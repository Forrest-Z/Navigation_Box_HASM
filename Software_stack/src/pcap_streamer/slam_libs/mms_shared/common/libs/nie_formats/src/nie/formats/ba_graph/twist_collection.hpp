/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <nie/core/time.hpp>

#include "ba_graph_types.hpp"

// To expose the collection readers/writer when this collection is included
#include "collection_reader.hpp"
#include "collection_stream_reader.hpp"
#include "collection_stream_writer.hpp"
#include "collection_writer.hpp"

namespace nie {

namespace io {

constexpr VersionType kTwistCollectionVersionLatest = BaGraphVersion(1, 1);

struct TwistCollection;  // Forward declaration.

namespace graph {

template <>
inline std::string Extension<TwistCollection>() {
    return ".twis";
}

}  // namespace graph

struct TwistRecord {
    Timestamp_ns timestamp;
    double velocity;
    double yaw_rate;

    friend inline bool operator<(nie::io::TwistRecord const& record, nie::Timestamp_ns const& other) {
        return record.timestamp < other;
    }

    friend inline bool operator<(nie::Timestamp_ns const& other, nie::io::TwistRecord const& record) {
        return other < record.timestamp;
    }
};

struct TwistHeader {
    using Collection = TwistCollection;

    VersionType version = kTwistCollectionVersionLatest;
};

struct TwistCollection {
    using Header = TwistHeader;

    static const detail::SignatureType Signature;

    TwistHeader header;
    std::vector<TwistRecord> twists;
};

using TwistCollectionStreamWriter = detail::CollectionStreamWriter<TwistCollection, TwistHeader, TwistRecord>;

using TwistCollectionStreamReader = detail::CollectionStreamReader<TwistCollection, TwistHeader, TwistRecord>;

[[nodiscard]] inline bool Equals(TwistRecord const& l, TwistRecord const& r);

[[nodiscard]] inline bool Equals(TwistHeader const& l, TwistHeader const& r);

}  // namespace io

}  // namespace nie

#include "twist_collection.inl"