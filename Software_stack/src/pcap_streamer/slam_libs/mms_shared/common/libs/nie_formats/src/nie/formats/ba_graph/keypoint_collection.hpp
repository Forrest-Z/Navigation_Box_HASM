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

constexpr VersionType kKeypointCollectionVersionLatest = BaGraphVersion(1, 1);

struct KeypointCollection;  // Forward declaration

namespace graph {

template <>
inline std::string Extension<KeypointCollection>() {
    return ".kpnt";
}

}  // namespace graph

struct KeypointRecord {
    PoseId pose_id;
    std::uint8_t frame_id;  // index of frame in rectified intrinsics file
    std::int32_t object_id;

    Eigen::Vector2f position;
    Eigen::Matrix2d information;  // Optional: when flag kHasInformationPerRecord in KeypointHeader is set
};

struct KeypointHeader {
    inline bool HasInformation() const { return (flags & kHasInformation) > 0; }

    inline bool HasInformationPerRecord() const { return (flags & kHasInformationPerRecord) > 0; }

    inline bool HasAnyInformation() const { return HasInformation() || HasInformationPerRecord(); }

    using Collection = KeypointCollection;

    using Flags = std::uint32_t;
    enum Flag : Flags {
        kHasInformation = 0x000F,
        kHasInformationPerRecord = 0x00F0,
    };

    [[nodiscard]] inline bool Has(Flag const& flag) const { return (flags & flag) > 0; }
    inline void Set(Flag const& flag) { flags |= flag; }
    inline void Unset(Flag const& flag) { flags &= ~flag; }

    VersionType version = kKeypointCollectionVersionLatest;

    Flags flags;

    Eigen::Matrix2d information;  // Optional: when flag kHasInformation is set
};

struct KeypointCollection {
    using Header = KeypointHeader;

    static const detail::SignatureType Signature;

    KeypointHeader header;
    std::vector<KeypointRecord> keypoints;
};

using KeypointCollectionStreamWriter =
    detail::CollectionStreamWriter<KeypointCollection, KeypointHeader, KeypointRecord>;

using KeypointCollectionStreamReader =
    detail::CollectionStreamReader<KeypointCollection, KeypointHeader, KeypointRecord>;

}  // namespace io

}  // namespace nie

#include "keypoint_collection.inl"
