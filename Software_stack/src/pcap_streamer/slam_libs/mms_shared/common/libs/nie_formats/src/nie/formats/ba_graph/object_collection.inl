/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

/// Chunk/Signature functions

constexpr ChunkType kChunkTypeObjectRecord{'O', 'B', 'J', 'T'};
// TODO(jbr): Maybe change to sums sizeof ?
// Eigen::Vector3d
constexpr std::size_t kR3Size = 3 * 8;
// Eigen::Matrix<double, 3, 3> upper triangle
constexpr std::size_t kR3InformationSize = (3 * (3 + 1) / 2) * 8;

template <>
inline std::int32_t GetChunkLength(ObjectHeader const& header) {
    std::int32_t size = 0;
    size += sizeof(ObjectHeader::version);
    size += sizeof(ObjectHeader::flags);
    if (header.flags & ObjectHeader::Flag::kHasInformation) {
        size += kR3InformationSize;
    }
    return size;
}
template <>
inline std::int32_t GetChunkLength(ObjectHeader const& header, ObjectRecord const&) {
    std::int32_t size = 0;
    size += sizeof(ObjectRecord::id);
    size += kR3Size;
    if (header.flags & ObjectHeader::Flag::kHasInformationPerRecord) {
        size += kR3InformationSize;
    }
    return size;
}

/// Read/Write functions

template <>
inline void Read(ObjectHeader const& header, IStreamWrapper* wrapper, ObjectRecord* record) {
    wrapper->ReadValue(&record->id);
    wrapper->ReadValue(&record->position);
    if (header.flags & ObjectHeader::Flag::kHasInformationPerRecord) {
        auto r = MakeSymmetricMatrixRef(record->information);
        wrapper->ReadValue(&r);
    }
}
template <>
inline void Write(ObjectHeader const& header, ObjectRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header, record), kChunkTypeObjectRecord, wrapper);
    wrapper->WriteValue(record.id);
    wrapper->WriteValue(record.position);
    if (header.flags & ObjectHeader::Flag::kHasInformationPerRecord) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(record.information));
    }
}

template <>
inline void Read(IStreamWrapper* wrapper, ObjectHeader* header) {
    ReadHeaderChunkHeader(wrapper);
    // We don't care about the version. Only the latest one.
    VersionType version;
    wrapper->ReadValue(&version);
    wrapper->ReadValue(&header->flags);

    if (header->flags & ObjectHeader::Flag::kHasInformation) {
        auto r = MakeSymmetricMatrixRef(header->information);
        wrapper->ReadValue(&r);
    }
}
template <>
inline void Write(ObjectHeader const& header, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header), kChunkTypeHeader, wrapper);
    wrapper->WriteValue(header.version);
    wrapper->WriteValue(header.flags);

    if (header.flags & ObjectHeader::Flag::kHasInformation) {
        wrapper->WriteValue(MakeSymmetricMatrixRef(header.information));
    }
}

template <>
inline void ReadRecords(
    IStreamWrapper* wrapper,
    ObjectHeader const& header,
    std::function<void(ObjectRecord)> const& callback_object_record) {
    ChunkType chunk_type{};
    do {
        std::int32_t chunk_length;
        ReadChunkHeader(&chunk_length, &chunk_type, wrapper);

        if (chunk_type == kChunkTypeObjectRecord) {
            ObjectRecord record;
            Read(header, wrapper, &record);
            if (callback_object_record) {
                callback_object_record(std::move(record));
            }
        } else if (chunk_type != kChunkTypeEndOfFile) {
            LOG(FATAL) << "ObjectCollection::ReadRecords(): Unexpected chunk type.";
        }
    } while (chunk_type != kChunkTypeEndOfFile);
}

template <>
inline void Read(IStreamWrapper* wrapper, ObjectCollection* collection) {
    Read(wrapper, &collection->header);

    std::function<void(ObjectRecord)> const f_add = [&collection](ObjectRecord r) {
        collection->objects.push_back(std::move(r));
    };
    ReadRecords(wrapper, collection->header, f_add);
}
template <>
inline void Write(ObjectCollection const& collection, OStreamWrapper* wrapper) {
    Write(collection.header, wrapper);

    for (ObjectRecord const& record : collection.objects) {
        Write(collection.header, record, wrapper);
    }
}

}  // namespace detail

}  // namespace io

}  // namespace nie
