/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

/// Chunk/Signature functions

constexpr ChunkType kChunkTypeBboxRecord{'B', 'B', 'O', 'X'};  // BBOX

template <>
inline std::int32_t GetChunkLength(BboxRecord const&) {
    std::int32_t size = 0;
    size += sizeof(BboxRecord::id);
    size += sizeof(BboxRecord::min);
    size += sizeof(BboxRecord::max);
    return size;
}

template <>
inline std::int32_t GetChunkLength(BboxHeader const&) {
    return sizeof(BboxHeader::version);
}

/// Read/Write functions

template <>
inline void Read(BboxHeader const&, IStreamWrapper* wrapper, BboxRecord* record) {
    wrapper->ReadValue(&record->id);
    wrapper->ReadValue(&record->min);
    wrapper->ReadValue(&record->max);
}
template <>
inline void Write(BboxHeader const&, BboxRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(record), kChunkTypeBboxRecord, wrapper);
    wrapper->WriteValue(record.id);
    wrapper->WriteValue(record.min);
    wrapper->WriteValue(record.max);
}

template <>
inline void Read(IStreamWrapper* wrapper, BboxHeader*) {
    ReadHeaderChunkHeader(wrapper);
    // We don't care about the version. Only the latest one.
    VersionType version;
    wrapper->ReadValue(&version);
}
template <>
inline void Write(BboxHeader const& header, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header), kChunkTypeHeader, wrapper);
    wrapper->WriteValue(kBboxCollectionVersionLatest);
}

template <>
inline void ReadRecords(
    IStreamWrapper* wrapper, BboxHeader const& header, std::function<void(BboxRecord)> const& callback_record) {
    ChunkType chunk_type{};
    do {
        std::int32_t chunk_length{};
        ReadChunkHeader(&chunk_length, &chunk_type, wrapper);

        if (chunk_type == kChunkTypeBboxRecord) {
            BboxRecord record;
            Read(header, wrapper, &record);
            if (callback_record) {
                callback_record(std::move(record));
            }
        } else if (chunk_type != kChunkTypeEndOfFile) {
            LOG(FATAL) << "BboxCollection::ReadRecords(): Unexpected chunk type.";
        }
    } while (chunk_type != kChunkTypeEndOfFile);
}

template <>
inline void Read(IStreamWrapper* wrapper, BboxCollection* collection) {
    Read(wrapper, &collection->header);

    std::function<void(BboxRecord)> const f_add = [&collection](BboxRecord r) {
        collection->boxes.push_back(std::move(r));
    };
    ReadRecords(wrapper, collection->header, f_add);
}
template <>
inline void Write(BboxCollection const& collection, OStreamWrapper* wrapper) {
    Write(collection.header, wrapper);

    for (BboxRecord const& record : collection.boxes) {
        Write(collection.header, record, wrapper);
    }
}

}  // namespace detail

[[nodiscard]] inline bool Equals(BboxRecord const& l, BboxRecord const& r) {
    return l.id == r.id && l.min.isApprox(r.min) && l.max.isApprox(r.max);
}

[[nodiscard]] inline bool Equals(BboxHeader const& l, BboxHeader const& r) { return l.version == r.version; }

}  // namespace io

}  // namespace nie
