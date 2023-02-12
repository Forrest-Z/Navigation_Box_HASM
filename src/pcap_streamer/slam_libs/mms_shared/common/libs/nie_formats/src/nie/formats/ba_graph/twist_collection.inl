/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

/// Chunk/Signature functions

constexpr ChunkType kChunkTypeTwistRecord{'T', 'W', 'I', 'S'};  // TWIS(t)

template <>
inline std::int32_t GetChunkLength(TwistRecord const&) {
    std::int32_t size = 0;
    size += sizeof(TwistRecord::timestamp);
    size += sizeof(TwistRecord::velocity);
    size += sizeof(TwistRecord::yaw_rate);
    return size;
}

template <>
inline std::int32_t GetChunkLength(TwistHeader const&) {
    return sizeof(TwistHeader::version);
}

/// Read/Write functions

template <>
inline void Read(TwistHeader const&, IStreamWrapper* wrapper, TwistRecord* record) {
    wrapper->ReadValue(&record->timestamp);
    wrapper->ReadValue(&record->velocity);
    wrapper->ReadValue(&record->yaw_rate);
}
template <>
inline void Write(TwistHeader const&, TwistRecord const& record, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(record), kChunkTypeTwistRecord, wrapper);
    wrapper->WriteValue(record.timestamp);
    wrapper->WriteValue(record.velocity);
    wrapper->WriteValue(record.yaw_rate);
}

template <>
inline void Read(IStreamWrapper* wrapper, TwistHeader*) {
    ReadHeaderChunkHeader(wrapper);
    // We don't care about the version. Only the latest one.
    VersionType version;
    wrapper->ReadValue(&version);
}
template <>
inline void Write(TwistHeader const& header, OStreamWrapper* wrapper) {
    WriteChunkHeader(GetChunkLength(header), kChunkTypeHeader, wrapper);
    wrapper->WriteValue(kTwistCollectionVersionLatest);
}

template <>
inline void ReadRecords(
    IStreamWrapper* wrapper, TwistHeader const& header, std::function<void(TwistRecord)> const& callback_record) {
    ChunkType chunk_type{};
    do {
        std::int32_t chunk_length{};
        ReadChunkHeader(&chunk_length, &chunk_type, wrapper);

        if (chunk_type == kChunkTypeTwistRecord) {
            TwistRecord record;
            Read(header, wrapper, &record);
            if (callback_record) {
                callback_record(std::move(record));
            }
        } else if (chunk_type != kChunkTypeEndOfFile) {
            LOG(FATAL) << "TwistCollection::ReadRecords(): Unexpected chunk type.";
        }
    } while (chunk_type != kChunkTypeEndOfFile);
}

template <>
inline void Read(IStreamWrapper* wrapper, TwistCollection* collection) {
    Read(wrapper, &collection->header);

    std::function<void(TwistRecord)> const f_add = [&collection](TwistRecord r) {
        collection->twists.push_back(std::move(r));
    };
    ReadRecords(wrapper, collection->header, f_add);
}
template <>
inline void Write(TwistCollection const& collection, OStreamWrapper* wrapper) {
    Write(collection.header, wrapper);

    for (TwistRecord const& record : collection.twists) {
        Write(collection.header, record, wrapper);
    }
}

}  // namespace detail

[[nodiscard]] inline bool Equals(TwistRecord const& l, TwistRecord const& r) {
    return l.timestamp == r.timestamp && l.velocity == r.velocity && l.yaw_rate == r.yaw_rate;
}

[[nodiscard]] inline bool Equals(TwistHeader const& l, TwistHeader const& r) { return l.version == r.version; }

}  // namespace io

}  // namespace nie