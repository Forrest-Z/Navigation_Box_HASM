/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

/// Chunk/Signature functions

constexpr ChunkType kChunkTypeInfoRefRecord{'I', 'R', 'E', 'F'};  // IREF

template <>
inline std::int32_t GetChunkLength(InfoRefRecord const& record) {
    std::int32_t size = 0;
    size += sizeof(InfoRefRecord::id);
    size += SizeOfString(record.path);
    return size;
}

template <>
inline std::int32_t GetChunkLength(InfoRefHeader const&) {
    return sizeof(InfoRefHeader::version);
}

/// Read/Write functions

template <>
inline void Read(InfoRefHeader const&, IStreamWrapper* wrapper, InfoRefRecord* record) {
    wrapper->ReadValue(&record->id);
    wrapper->ReadValue(&record->frame_id);
    wrapper->ReadValue(&record->path);
}
template <>
inline void Write(InfoRefHeader const&, InfoRefRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(record), kChunkTypeInfoRefRecord, wrapper);
    wrapper->WriteValue(record.id);
    wrapper->WriteValue(record.frame_id);
    wrapper->WriteValue(record.path);
}

template <>
inline void Read(IStreamWrapper* wrapper, InfoRefHeader*) {
    ReadHeaderChunkHeader(wrapper);
    // We don't care about the version. Only the latest one.
    VersionType version;
    wrapper->ReadValue(&version);
}
template <>
inline void Write(InfoRefHeader const& header, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header), kChunkTypeHeader, wrapper);
    wrapper->WriteValue(kInfoRefCollectionVersionLatest);
}

template <>
inline void ReadRecords(
        IStreamWrapper* wrapper,
        InfoRefHeader const& header,
        std::function<void(InfoRefRecord)> const& callback_record) {
    ChunkType chunk_type{};
    do {
        std::int32_t chunk_length{};
        ReadChunkHeader(&chunk_length, &chunk_type, wrapper);

        if (chunk_type == kChunkTypeInfoRefRecord) {
            InfoRefRecord record;
            Read(header, wrapper, &record);
            if (callback_record) {
                callback_record(std::move(record));
            }
        } else if (chunk_type != kChunkTypeEndOfFile) {
            LOG(FATAL) << "InfoRefCollection::ReadRecords(): Unexpected chunk type.";
        }
    } while (chunk_type != kChunkTypeEndOfFile);
}

template <>
inline void Read(IStreamWrapper* wrapper, InfoRefCollection* collection) {
    Read(wrapper, &collection->header);

    std::function<void(InfoRefRecord)> const f_add = [&collection](InfoRefRecord r) {
        collection->info_refs.push_back(std::move(r));
    };
    ReadRecords(wrapper, collection->header, f_add);
}
template <>
inline void Write(InfoRefCollection const& collection, OStreamWrapper* wrapper) {
    Write(collection.header, wrapper);

    for (InfoRefRecord const& record : collection.info_refs) {
        Write(collection.header, record, wrapper);
    }
}

}  // namespace detail

[[nodiscard]] inline bool Equals(InfoRefRecord const& l, InfoRefRecord const& r) {
    return l.id == r.id && l.frame_id == r.frame_id && l.path == r.path;
}

[[nodiscard]] inline bool Equals(InfoRefHeader const& l, InfoRefHeader const& r) { return l.version == r.version; }

}  // namespace io

}  // namespace nie
