/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Geometry>
#include <nie/core/geometry/interpolation.hpp>
#include <nie/core/time.hpp>
#include <vector>

#include "ba_graph_types.hpp"

// To expose the collection readers/writer when this collection is included
#include "collection_reader.hpp"
#include "collection_stream_reader.hpp"
#include "collection_stream_writer.hpp"
#include "collection_writer.hpp"

namespace nie {

namespace io {

constexpr VersionType kPoseCollectionVersionLatest = BaGraphVersion(1, 4);

struct PoseCollection;  // Forward declaration.

namespace graph {

template <>
inline std::string Extension<PoseCollection>() {
    return ".pose";
}

}  // namespace graph

using PoseId = std::int32_t;

struct PoseRecord {
    PoseId id;

    enum class Category : std::uint8_t { kGps, kOdom, kBbox };
    Category category;

    // Is used as an index to the intrinsics file of a specific device
    std::int32_t device_id;
    Timestamp_ns timestamp;
    Isometry3qd isometry;
    // Optional: when flag kHasPoseInformationPerRecord in PoseHeader is set
    Eigen::Matrix<double, 6, 6> information;

    friend inline bool operator<(nie::io::PoseRecord const& record, nie::Timestamp_ns const& other) {
        return record.timestamp < other;
    }

    friend inline bool operator<(nie::Timestamp_ns const& other, nie::io::PoseRecord const& record) {
        return other < record.timestamp;
    }
};

struct PoseEdgeRecord {
    PoseId id_begin;
    PoseId id_end;

    enum class Category : std::uint8_t {
        kRelToAbs,  // Edge does not strictly need to be in this order
        kOdom,
        kLoop
    };
    Category category;

    // T_begin_end (a transformation that brings us from the end to begin). Meaning: the origin is considered the
    // beginning.
    Isometry3qd isometry;
    // Optional: when flag kHasEdgeInformationPerRecord in PoseHeader is set
    Eigen::Matrix<double, 6, 6> information;
};

struct FixedPoseRecord {
    PoseId id;
};

struct PoseHeader {
    [[nodiscard]] inline bool HasCodeZ() const { return (flags & kHasCodeZ) > 0; }

    [[nodiscard]] inline bool HasPoseInformation() const { return (flags & kHasPoseInformation) > 0; }

    [[nodiscard]] inline bool HasPoseInformationPerRecord() const { return (flags & kHasPoseInformationPerRecord) > 0; }

    [[nodiscard]] inline bool HasAnyPoseInformation() const {
        return HasPoseInformation() || HasPoseInformationPerRecord();
    }

    [[nodiscard]] inline bool HasEdgeInformation() const { return (flags & kHasEdgeInformation) > 0; }

    [[nodiscard]] inline bool HasEdgeInformationPerRecord() const { return (flags & kHasEdgeInformationPerRecord) > 0; }

    [[nodiscard]] inline bool HasAnyEdgeInformation() const {
        return HasEdgeInformation() || HasEdgeInformationPerRecord();
    }

    [[nodiscard]] inline bool HasTimestampPerRecord() const {
        return (flags & PoseHeader::Flag::kHasTimestampPerRecord) > 0;
    }

    using Collection = PoseCollection;

    using Flags = std::uint32_t;
    enum Flag : Flags {
        kHasCodeZ = 0x0000000F,
        kHasPoseInformation = 0x000000F0,
        kHasPoseInformationPerRecord = 0x00000F00,
        kHasEdgeInformation = 0x0000F000,
        kHasEdgeInformationPerRecord = 0x000F0000,
        kHasTimestampPerRecord = 0x00F00000,
    };

    [[nodiscard]] inline bool Has(Flag const& flag) const { return (flags & flag) > 0; }
    inline void Set(Flag const& flag) { flags |= flag; }
    inline void Unset(Flag const& flag) { flags &= ~flag; }

    VersionType version = kPoseCollectionVersionLatest;

    Flags flags;

    // Default equals EPSG unless otherwise noted.
    std::string authority = "EPSG";
    // Coordinate system identifier. Either for a 2D x,y / lat,lon system or a 3D x,y,z / lat,lon,height one.
    std::int32_t code_xy_or_xyz;
    // Epsg code applicable for height/z/vertical in case previous field epsg_code is only applicable for the 2D system.
    std::int32_t code_z;  // Optional: when flag kHasCodeZ is set

    // Optional: when flag kHasPoseInformation is set
    Eigen::Matrix<double, 6, 6> pose_information;
    // Optional: when flag kHasEdgeInformation is set
    Eigen::Matrix<double, 6, 6> edge_information;
};

struct PoseCollection {
    using Header = PoseHeader;

    static const detail::SignatureType Signature;

    PoseHeader header;
    std::vector<PoseRecord> poses;
    std::vector<PoseEdgeRecord> edges;
    std::vector<FixedPoseRecord> fixes;
};

using PoseCollectionStreamWriter =
        detail::CollectionStreamWriter<PoseCollection, PoseHeader, PoseRecord, PoseEdgeRecord, FixedPoseRecord>;
using PoseCollectionStreamReader =
        detail::CollectionStreamReader<PoseCollection, PoseHeader, PoseRecord, PoseEdgeRecord, FixedPoseRecord>;

inline void SetNieAuthority(PoseHeader* header) {
    header->authority = "NIE";
    header->code_xy_or_xyz = 0;
    // If somehow kHasCodeZ was set, we unset it.
    header->Unset(PoseHeader::kHasCodeZ);
}

inline void CopyAuthority(PoseHeader const& header_a, PoseHeader* header_b) {
    header_b->authority = header_a.authority;
    header_b->code_xy_or_xyz = header_a.code_xy_or_xyz;
    if (header_a.Has(PoseHeader::kHasCodeZ)) {
        header_b->Set(PoseHeader::kHasCodeZ);
        header_b->code_z = header_a.code_z;
    } else {
        header_b->Unset(PoseHeader::kHasCodeZ);
    }
}

inline bool EqualAuthority(PoseHeader const& header_a, PoseHeader const& header_b) {
    bool result = header_a.authority == header_b.authority;
    result &= header_a.code_xy_or_xyz == header_b.code_xy_or_xyz;
    result &= header_a.Has(PoseHeader::kHasCodeZ) == header_b.Has(PoseHeader::kHasCodeZ);
    if (header_a.Has(PoseHeader::kHasCodeZ)) {
        result &= header_a.code_z == header_b.code_z;
    }
    return result;
}

[[nodiscard]] inline bool Equals(PoseRecord const& l, PoseRecord const& r, PoseHeader const& h);

[[nodiscard]] inline bool Equals(PoseEdgeRecord const& l, PoseEdgeRecord const& r, PoseHeader const& h);

[[nodiscard]] inline bool Equals(FixedPoseRecord const& l, FixedPoseRecord const& r);

[[nodiscard]] inline bool Equals(PoseHeader const& l, PoseHeader const& r);

namespace detail {

[[nodiscard]] inline double CalculateInterpolationRatio(
        nie::io::PoseRecord const& pose_before,
        nie::io::PoseRecord const& pose_after,
        nie::Timestamp_ns const& timestamp) {
    // Do the pose interpolation based on timestamps
    return nie::RepresentDurationAsDouble(timestamp - pose_before.timestamp) /
           nie::RepresentDurationAsDouble(pose_after.timestamp - pose_before.timestamp);
}

inline void InterpolateFromPoseRecords(
        nie::io::PoseRecord const& pose_before,
        nie::io::PoseRecord const& pose_after,
        nie::Timestamp_ns const& timestamp,
        nie::Isometry3qd* isometry) {
    *isometry = nie::Interpolate(
            pose_before.isometry, pose_after.isometry, CalculateInterpolationRatio(pose_before, pose_after, timestamp));
}

inline void InterpolateFromPoseRecords(
        nie::io::PoseRecord const& pose_before,
        nie::io::PoseRecord const& pose_after,
        nie::Timestamp_ns const& timestamp,
        nie::Isometry3qd* isometry,
        Eigen::Matrix<double, 6, 6>* information) {
    double const ratio = CalculateInterpolationRatio(pose_before, pose_after, timestamp);
    *isometry = nie::Interpolate(pose_before.isometry, pose_after.isometry, ratio);
    *information = nie::InfInterpolate(pose_before.information, pose_after.information, ratio);
}

}  // namespace detail

template <typename Iterator>
inline std::pair<Iterator const, Iterator const> InterpolateIsometry(
        Iterator const& begin, Iterator const& end, nie::Timestamp_ns const& timestamp, nie::Isometry3qd* isometry) {
    auto iterators = std::equal_range(begin, end, timestamp);

    // If they are equal we found a range, but both point 1 past the searched element.
    // Else ... we found an exact time stamp and the second iterator points one past the element (perhaps end())
    if (iterators.first == iterators.second) {
        iterators.first--;
        detail::InterpolateFromPoseRecords(*iterators.first, *iterators.second, timestamp, isometry);
    } else {
        iterators.second--;
        *isometry = iterators.first->isometry;
    }

    return iterators;
}

template <typename Iterator>
inline std::pair<Iterator const, Iterator const> InterpolateIsometry(
        Iterator const& begin,
        Iterator const& end,
        nie::Timestamp_ns const& timestamp,
        nie::Isometry3qd* isometry,
        Eigen::Matrix<double, 6, 6>* information) {
    auto iterators = std::equal_range(begin, end, timestamp);

    // If they are equal we found a range, but both point 1 past the searched element.
    // Else ... we found an exact time stamp and the second iterator points one past the element (perhaps end())
    if (iterators.first == iterators.second) {
        iterators.first--;
        detail::InterpolateFromPoseRecords(*iterators.first, *iterators.second, timestamp, isometry, information);
    } else {
        iterators.second--;
        *isometry = iterators.first->isometry;
        *information = iterators.first->information;
    }

    return iterators;
}

template <typename Container>
inline std::pair<typename Container::const_iterator, typename Container::const_iterator> InterpolateIsometry(
        Container poses, nie::Timestamp_ns const& timestamp, nie::Isometry3qd* isometry) {
    return InterpolateIsometry(poses.begin(), poses.end(), timestamp, isometry);
}

template <typename Container>
inline std::pair<typename Container::const_iterator, typename Container::const_iterator> InterpolateIsometry(
        Container poses,
        nie::Timestamp_ns const& timestamp,
        nie::Isometry3qd* isometry,
        Eigen::Matrix<double, 6, 6>* information) {
    return InterpolateIsometry(poses.begin(), poses.end(), timestamp, isometry, information);
}

}  // namespace io

}  // namespace nie

#include "pose_collection.inl"
