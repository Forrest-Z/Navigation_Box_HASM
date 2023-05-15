/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "helper_stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

void ReadChunkHeader(std::int32_t* length, ChunkType* type, IStreamWrapper* wrapper) {
    wrapper->ReadValue(length);
    wrapper->ReadValue(type);
}

void WriteChunkHeader(std::int32_t const& length, ChunkType const& type, OStreamWrapper* wrapper) {
    wrapper->WriteValue(length);
    wrapper->WriteValue(type);
}

SignatureType ReadSignature(IStreamWrapper* wrapper) {
    SignatureType type;
    wrapper->ReadValue(&type);
    return type;
}

void WriteSignature(SignatureType const& type, OStreamWrapper* wrapper) { wrapper->WriteValue(type); }

std::int32_t ReadHeaderChunkHeader(IStreamWrapper* wrapper) {
    std::int32_t length;
    ChunkType type;
    ReadChunkHeader(&length, &type, wrapper);
    assert(type == kChunkTypeHeader);
    return length;
}

void WriteFooter(OStreamWrapper* wrapper) { WriteChunkHeader(0, kChunkTypeEndOfFile, wrapper); }

}  // namespace detail

}  // namespace io

}  // namespace nie