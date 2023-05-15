/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

/// Chunk/Signature functions
constexpr ChunkType kChunkTypePoseRecord{'P', 'O', 'S', 'E'};
constexpr ChunkType kChunkTypePoseEdgeRecord{'P', 'S', 'E', 'D'};
constexpr ChunkType kChunkTypeFixedPoseRecord{'P', 'S', 'F', 'X'};

// TODO(jbr): Maybe change to sums sizeof ?
// Isometry3qd (Eigen::Vector3d + Eigen::Quaterniond)
constexpr std::size_t kSe3Size = 3 * 8 + 4 * 8;
// Eigen::Matrix<double, 6, 6> upper triangle
constexpr std::size_t kSe3InformationSize = (6 * (6 + 1) / 2) * 8;

template <>
inline std::int32_t GetChunkLength(PoseHeader const& header) {
    std::int32_t size = 0;
    size += sizeof(PoseHeader::version);
    size += sizeof(PoseHeader::flags);
    size += SizeOfString(header.authority);
    size += sizeof(PoseHeader::code_xy_or_xyz);
    if (header.HasCodeZ()) {
        size += sizeof(PoseHeader::code_z);
    }
    if (header.HasPoseInformation()) {
        size += kSe3InformationSize;
    }
    if (header.HasEdgeInformation()) {
        size += kSe3InformationSize;
    }
    return size;
}
template <>
inline std::int32_t GetChunkLength(PoseHeader const& header, PoseRecord const&) {
    std::int32_t size = 0;
    size += sizeof(PoseRecord::id);
    size += sizeof(PoseRecord::category);
    size += sizeof(PoseRecord::device_id);
    if (header.HasTimestampPerRecord()) {
        size += sizeof(std::uint32_t);
        size += sizeof(std::uint64_t);
        static_assert(sizeof(nie::GPSWeekTime<std::chrono::nanoseconds>::week) <= sizeof(std::uint32_t));
        static_assert(sizeof(nie::GPSWeekTime<std::chrono::nanoseconds>::time_in_week) <= sizeof(std::uint64_t));
    }

    size += kSe3Size;
    if (header.HasPoseInformationPerRecord()) {
        size += kSe3InformationSize;
    }
    return size;
}
template <>
inline std::int32_t GetChunkLength(PoseHeader const& header, PoseEdgeRecord const&) {
    std::int32_t size = 0;
    size += sizeof(PoseEdgeRecord::id_begin);
    size += sizeof(PoseEdgeRecord::id_end);
    size += sizeof(PoseEdgeRecord::category);
    size += kSe3Size;
    if (header.HasEdgeInformationPerRecord()) {
        size += kSe3InformationSize;
    }
    return size;
}
template <>
inline std::int32_t GetChunkLength(FixedPoseRecord const&) {
    return sizeof(FixedPoseRecord::id);
}

/// Read/Write functions

template <>
inline void Read(PoseHeader const& header, IStreamWrapper* wrapper, PoseRecord* record) {
    wrapper->ReadValue(&record->id);
    wrapper->ReadValue(&record->category);
    wrapper->ReadValue(&record->device_id);
    if (header.HasTimestampPerRecord()) {
        wrapper->ReadValue(&record->timestamp);
    }
    wrapper->ReadValue(&record->isometry);
    if (header.HasPoseInformationPerRecord()) {
        auto r = MakeSymmetricMatrixRef(record->information);
        wrapper->ReadValue(&r);
    }
}
template <>
inline void Write(PoseHeader const& header, PoseRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header, record), kChunkTypePoseRecord, wrapper);
    wrapper->WriteValue(record.id);
    wrapper->WriteValue(record.category);
    wrapper->WriteValue(record.device_id);
    if (header.HasTimestampPerRecord()) {
        wrapper->WriteValue(record.timestamp);
    }
    wrapper->WriteValue(record.isometry);
    if (header.HasPoseInformationPerRecord()) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(record.information));
    }
}

template <>
inline void Read(PoseHeader const& header, IStreamWrapper* wrapper, PoseEdgeRecord* record) {
    wrapper->ReadValue(&record->id_begin);
    wrapper->ReadValue(&record->id_end);
    wrapper->ReadValue(&record->category);
    wrapper->ReadValue(&record->isometry);
    if (header.HasEdgeInformationPerRecord()) {
        auto r = MakeSymmetricMatrixRef(record->information);
        wrapper->ReadValue(&r);
    }
}
template <>
inline void Write(PoseHeader const& header, PoseEdgeRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header, record), kChunkTypePoseEdgeRecord, wrapper);
    wrapper->WriteValue(record.id_begin);
    wrapper->WriteValue(record.id_end);
    wrapper->WriteValue(record.category);
    wrapper->WriteValue(record.isometry);
    if (header.HasEdgeInformationPerRecord()) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(record.information));
    }
}

template <>
inline void Read(PoseHeader const&, IStreamWrapper* wrapper, FixedPoseRecord* record) {
    wrapper->ReadValue(&record->id);
}
template <>
inline void Write(PoseHeader const&, FixedPoseRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(record), kChunkTypeFixedPoseRecord, wrapper);
    wrapper->WriteValue(record.id);
}

template <>
inline void Read(IStreamWrapper* wrapper, PoseHeader* header) {
    ReadHeaderChunkHeader(wrapper);
    // We don't care about the version. Only the latest one.
    wrapper->ReadValue(&header->version);
    CHECK(header->version == kPoseCollectionVersionLatest) << "PoseCollection: Version mismatch.";
    wrapper->ReadValue(&header->flags);
    wrapper->ReadValue(&header->authority);
    wrapper->ReadValue(&header->code_xy_or_xyz);
    if (header->HasCodeZ()) {
        wrapper->ReadValue(&header->code_z);
    }
    if (header->HasPoseInformation()) {
        auto r = MakeSymmetricMatrixRef(header->pose_information);
        wrapper->ReadValue(&r);
    }
    if (header->HasEdgeInformation()) {
        auto r = MakeSymmetricMatrixRef(header->edge_information);
        wrapper->ReadValue(&r);
    }
}
template <>
inline void Write(PoseHeader const& header, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header), kChunkTypeHeader, wrapper);
    wrapper->WriteValue(header.version);
    wrapper->WriteValue(header.flags);
    wrapper->WriteValue(header.authority);
    wrapper->WriteValue(header.code_xy_or_xyz);
    if (header.HasCodeZ()) {
        wrapper->WriteValue(header.code_z);
    }
    if (header.HasPoseInformation()) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(header.pose_information));
    }
    if (header.HasEdgeInformation()) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(header.edge_information));
    }
}

template <>
inline void ReadRecords(
    IStreamWrapper* wrapper,
    PoseHeader const& header,
    std::function<void(PoseRecord)> const& callback_pose_record,
    std::function<void(PoseEdgeRecord)> const& callback_pose_edge_record,
    std::function<void(FixedPoseRecord)> const& callback_fixed_pose_record) {
    ChunkType chunk_type{};

    do {
        std::int32_t chunk_length;
        ReadChunkHeader(&chunk_length, &chunk_type, wrapper);

        if (chunk_type == kChunkTypePoseRecord) {
            PoseRecord record;
            Read(header, wrapper, &record);
            if (callback_pose_record) {
                callback_pose_record(std::move(record));
            }
        } else if (chunk_type == kChunkTypePoseEdgeRecord) {
            PoseEdgeRecord record;
            Read(header, wrapper, &record);
            if (callback_pose_edge_record) {
                callback_pose_edge_record(std::move(record));
            }
        } else if (chunk_type == kChunkTypeFixedPoseRecord) {
            FixedPoseRecord record{};
            Read(header, wrapper, &record);
            if (callback_fixed_pose_record) {
                callback_fixed_pose_record(std::move(record));
            }
        } else if (chunk_type != kChunkTypeEndOfFile) {
            LOG(FATAL) << "PoseCollection::ReadRecords(): Unexpected chunk type.";
        }
    } while (chunk_type != kChunkTypeEndOfFile);
}

template <>
inline void Read(IStreamWrapper* wrapper, PoseCollection* collection) {
    Read(wrapper, &collection->header);

    std::function<void(PoseRecord)> const f_add_pose = [&collection](PoseRecord r) {
        collection->poses.push_back(std::move(r));
    };
    std::function<void(PoseEdgeRecord)> const f_add_edge = [&collection](PoseEdgeRecord r) {
        collection->edges.push_back(std::move(r));
    };
    std::function<void(FixedPoseRecord)> const f_add_fix = [&collection](FixedPoseRecord r) {
        collection->fixes.push_back(std::move(r));
    };
    ReadRecords(wrapper, collection->header, f_add_pose, f_add_edge, f_add_fix);
}

template <>
inline void Write(PoseCollection const& collection, OStreamWrapper* wrapper) {
    Write(collection.header, wrapper);
    for (auto const& record : collection.poses) {
        Write(collection.header, record, wrapper);
    }
    for (auto const& record : collection.edges) {
        Write(collection.header, record, wrapper);
    }
    for (auto const& record : collection.fixes) {
        Write(collection.header, record, wrapper);
    }
}

}  // namespace detail

[[nodiscard]] inline bool Equals(PoseRecord const& l, PoseRecord const& r, PoseHeader const& h) {
    // clang-format off
    return
        l.id == r.id &&
        l.category == r.category &&
        l.device_id == r.device_id &&
        l.timestamp == r.timestamp &&
        l.isometry == r.isometry &&
        (!h.HasPoseInformationPerRecord() || (l.information == r.information));
    // clang-format on
}

[[nodiscard]] inline bool Equals(PoseEdgeRecord const& l, PoseEdgeRecord const& r, PoseHeader const& h) {
    // clang-format off
    return
        l.id_begin == r.id_begin &&
        l.id_end == r.id_end &&
        l.category == r.category &&
        l.isometry == r.isometry &&
        (!h.HasEdgeInformationPerRecord() || (l.information == r.information));
    // clang-format on
}

[[nodiscard]] inline bool Equals(FixedPoseRecord const& l, FixedPoseRecord const& r) { return l.id == r.id; }

[[nodiscard]] inline bool Equals(PoseHeader const& l, PoseHeader const& r) {
    // clang-format off
    return
        l.version == r.version &&
        l.flags == r.flags &&
        l.authority == r.authority &&
        l.code_xy_or_xyz == r.code_xy_or_xyz &&
        (!l.HasCodeZ() || (l.code_z == r.code_z)) &&
        (!l.HasPoseInformation() || (l.pose_information == r.pose_information)) &&
        (!l.HasEdgeInformation() || (l.edge_information == r.edge_information));
    // clang-format on
}

}  // namespace io

}  // namespace nie
