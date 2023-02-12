/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <mutex>
#include <vector>

namespace nie {

namespace mt {

// TODO(MVB) Not all stream can read and write, so using this class might lead to problems.
template <typename StreamT>
class MtStream {
public:
    MtStream(StreamT&& s) : stream_(std::move(s)) {}

    template <typename ValueT>
    friend MtStream& operator>>(MtStream& s, ValueT const& value) {
        std::lock_guard<std::mutex> lock(s.mutex_);
        s.stream_ >> value;
        return s;
    }

    template <typename ValueT>
    friend MtStream& operator<<(MtStream& s, ValueT const& value) {
        std::lock_guard<std::mutex> lock(s.mutex_);
        s.stream_ << value;
        return s;
    }

private:
    mutable std::mutex mutex_;
    StreamT stream_;
};

}  // namespace mt

}  // namespace nie
