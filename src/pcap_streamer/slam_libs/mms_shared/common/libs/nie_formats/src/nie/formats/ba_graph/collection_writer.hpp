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
inline void Write(typename detail::CollectionSupportTraits<Object>::Type const& object, std::ostream* stream) {
    OStreamWrapper wrapper(stream);
    WriteSignature(detail::CollectionSupportTraits<Object>::Signature, &wrapper);
    detail::Write(object, &wrapper);
    WriteFooter(&wrapper);
}

}  // namespace detail

template <typename Object>
inline void Write(Object const& object, std::ostream* stream) {
    // Note that detail::Write() already disallows any other objects to be written, but the compile error is unfriendly
    static_assert(
        detail::is_collection_v<Object> || detail::is_collection_header_v<Object>,
        "ONLY COLLECTIONS OR COLLECTION HEADERS CAN BE WRITTEN");
    detail::Write<Object>(object, stream);
}

template <typename Object>
inline void Write(Object const& object, std::string const& filename) {
    std::fstream stream = OpenFile(filename, std::ios::out | std::ios::binary);
    Write(object, &stream);
}

}  // namespace io

}  // namespace nie

