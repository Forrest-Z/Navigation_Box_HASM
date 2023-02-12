/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/core/filesystem.hpp>
#include <nie/core/type_traits.hpp>

#include "helper_stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

template <typename Collection, typename Header, typename... Records>
class CollectionStreamWriter {
public:
    CollectionStreamWriter(std::string const& filename, Header const& header)
        : stream_(OpenFile(filename, std::ios::out | std::ios::binary)), wrapper_(&stream_), header_(header) {
        detail::WriteSignature(Collection::Signature, &wrapper_);
        detail::Write(header_, &wrapper_);
    }
    ~CollectionStreamWriter() { detail::WriteFooter(&wrapper_); }

    template <typename Record>
    void Write(Record const& record) {
        static_assert(
            nie::any_v<std::is_same<Record, Records>::value...>,
            "RECORD TYPE NOT SUPPORTED BY THE COLLECTION STREAM. CHECK WHERE THE COLLECTION STREAM WAS INSTANTIATED "
            "TO SEE WHICH TYPES ARE SUPPORTED.");
        detail::Write(header_, record, &wrapper_);
    }

private:
    std::fstream stream_;
    OStreamWrapper wrapper_;

    Header const header_;
};

}  // namespace detail

}  // namespace io

}  // namespace nie

