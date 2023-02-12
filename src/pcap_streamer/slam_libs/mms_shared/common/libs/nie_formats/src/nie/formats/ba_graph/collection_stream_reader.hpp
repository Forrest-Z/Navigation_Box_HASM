/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/core/algorithm.hpp>
#include <nie/core/filesystem.hpp>
#include <nie/core/type_traits.hpp>

#include "helper_stream_wrapper.hpp"

namespace nie {

namespace io {

namespace detail {

template <typename Collection, typename Header, typename... Records>
class CollectionStreamReader {
public:
    explicit CollectionStreamReader(std::string const& filename)
        : stream_(OpenFile(filename, std::ios::in | std::ios::binary)), wrapper_(&stream_), header_() {
        CHECK(detail::ReadSignature(&wrapper_) == Collection::Signature) << "Wrong stream reader used.";
        nie::ForEach(callbacks_, [](auto& f) { f = nullptr; });
        detail::Read(&wrapper_, &header_);
    }

    Header const& GetHeader() const { return header_; }

    template <typename Record>
    void SetCallback(std::function<void(Record)> const& func) {
        static_assert(
                nie::any_v<std::is_same_v<Record, Records>...>,
                "RECORD TYPE NOT SUPPORTED BY THE COLLECTION STREAM. CHECK WHERE THE COLLECTION STREAM WAS "
                "INSTANTIATED TO SEE WHICH TYPES ARE SUPPORTED.");
        std::get<std::function<void(Record)>>(callbacks_) = func;
    }

    void ReadRecords() {
        bool any = false;
        nie::ForEach(callbacks_, [&any](auto f) { any |= (f != nullptr); });
        if (any) {
            std::apply([this](auto... x) { detail::ReadRecords(&wrapper_, header_, x...); }, callbacks_);
        }
    }

private:
    std::fstream stream_;
    IStreamWrapper wrapper_;

    std::tuple<std::function<void(Records)>...> callbacks_;

    Header header_;
};

}  // namespace detail

}  // namespace io

}  // namespace nie
