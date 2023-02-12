/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/core/filesystem.hpp>

#include "nie/formats/ba_graph/helper_stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

template <typename Object>
inline void Read(std::istream* stream, typename detail::CollectionSupportTraits<Object>::Type* object) {
    IStreamWrapper wrapper(stream);
    // Read and check signature
    if (ReadSignature(&wrapper) != detail::CollectionSupportTraits<Object>::Signature) {
        LOG(FATAL) << "Unexpected signature type.";
    }
    detail::Read(&wrapper, object);
}

}  // namespace detail

template <typename Object>
inline void Read(std::istream* stream, Object* object) {
    // Note that detail::Read() already disallows any other objects to be written, but the compile error is unfriendly
    static_assert(
        detail::is_collection_v<Object> || detail::is_collection_header_v<Object>,
        "ONLY COLLECTIONS OR COLLECTION HEADERS CAN BE READ");
    detail::Read<Object>(stream, object);
}

template <typename Object>
inline void Read(std::string const& filename, Object* object) {
    std::fstream stream = OpenFile(filename, std::ios::in | std::ios::binary);
    Read(&stream, object);
}

template <typename Collection>
inline Collection ReadCollection(std::string const& filename) {
    Collection collection;
    nie::io::Read(filename, &collection);
    return collection;
}

}  // namespace io

}  // namespace nie

