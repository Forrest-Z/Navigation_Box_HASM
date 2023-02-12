/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

/// Chunk/Signature functions

constexpr ChunkType kChunkTypeKeypointRecord{'K', 'P', 'N', 'T'};
// Eigen::Vector2f
constexpr std::size_t kR2Size = 2 * 4;
// Eigen::Matrix<double, 2, 2> upper triangle
constexpr std::size_t kR2InformationSize = (2 * (2 + 1) / 2) * 8;

template <>
inline std::int32_t GetChunkLength(KeypointHeader const& header) {
    std::int32_t size = 0;
    size += sizeof(KeypointHeader::version);
    size += sizeof(KeypointHeader::flags);
    if (header.flags & KeypointHeader::Flag::kHasInformation) {
        size += kR2InformationSize;
    }
    return size;
}
template <>
inline std::int32_t GetChunkLength(KeypointHeader const& header, nie::io::KeypointRecord const&) {
    std::int32_t size = 0;
    size += sizeof(KeypointRecord::pose_id);
    size += sizeof(KeypointRecord::frame_id);
    size += sizeof(KeypointRecord::object_id);
    size += kR2Size;
    if (header.flags & KeypointHeader::Flag::kHasInformationPerRecord) {
        size += kR2InformationSize;
    }
    return size;
}

/// Read/Write functions

template <>
inline void Read(KeypointHeader const& header, IStreamWrapper* wrapper, nie::io::KeypointRecord* record) {
    wrapper->ReadValue(&record->pose_id);
    wrapper->ReadValue(&record->frame_id);
    wrapper->ReadValue(&record->object_id);
    wrapper->ReadValue(&record->position);
    if (header.flags & KeypointHeader::Flag::kHasInformationPerRecord) {
        auto r = MakeSymmetricMatrixRef(record->information);
        wrapper->ReadValue(&r);
    }
}
template <>
inline void Write(KeypointHeader const& header, nie::io::KeypointRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header, record), kChunkTypeKeypointRecord, wrapper);
    wrapper->WriteValue(record.pose_id);
    wrapper->WriteValue(record.frame_id);
    wrapper->WriteValue(record.object_id);
    wrapper->WriteValue(record.position);
    if (header.flags & KeypointHeader::Flag::kHasInformationPerRecord) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(record.information));
    }
}

template <>
inline void Read(IStreamWrapper* wrapper, nie::io::KeypointHeader* header) {
    ReadHeaderChunkHeader(wrapper);
    // We don't care about the version. Only the latest one.
    VersionType version;
    wrapper->ReadValue(&version);
    wrapper->ReadValue(&header->flags);

    if (header->flags & KeypointHeader::Flag::kHasInformation) {
        auto r = MakeSymmetricMatrixRef(header->information);
        wrapper->ReadValue(&r);
    }
}
template <>
inline void Write(KeypointHeader const& header, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header), kChunkTypeHeader, wrapper);
    wrapper->WriteValue(header.version);
    wrapper->WriteValue(header.flags);

    if (header.flags & KeypointHeader::Flag::kHasInformation) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(header.information));
    }
}

template <>
inline void ReadRecords(
    IStreamWrapper* wrapper,
    KeypointHeader const& header,
    std::function<void(KeypointRecord)> const& callback_Keypoint_record) {
    ChunkType chunk_type{};
    do {
        std::int32_t chunk_length;
        ReadChunkHeader(&chunk_length, &chunk_type, wrapper);

        if (chunk_type == kChunkTypeKeypointRecord) {
            KeypointRecord record;
            Read(header, wrapper, &record);
            if (callback_Keypoint_record) {
                callback_Keypoint_record(std::move(record));
            }
        } else if (chunk_type != kChunkTypeEndOfFile) {
            LOG(FATAL) << "KeypointCollection::ReadRecords(): Unexpected chunk type.";
        }
    } while (chunk_type != kChunkTypeEndOfFile);
}

template <>
inline void Read(IStreamWrapper* wrapper, KeypointCollection* collection) {
    Read(wrapper, &collection->header);

    std::function<void(KeypointRecord)> const f_add = [&collection](KeypointRecord r) {
        collection->keypoints.push_back(std::move(r));
    };
    ReadRecords(wrapper, collection->header, f_add);
}
template <>
inline void Write(KeypointCollection const& collection, OStreamWrapper* wrapper) {
    Write(collection.header, wrapper);

    for (KeypointRecord const& record : collection.keypoints) {
        Write(collection.header, record, wrapper);
    }
}

}  // namespace detail

}  // namespace io

}  // namespace nie
