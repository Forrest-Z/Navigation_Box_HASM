/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <mutex>
#include <vector>

namespace nie {

namespace mt {

template <typename T, typename Alloc = std::allocator<T>>
class MtVector {
public:
    MtVector() = default;
    virtual ~MtVector() = default;

    T operator[](std::size_t i) const;

    void PushBack(T const& t);
    void PushBack(T&& t);

    template <typename InputIterator>
    void Insert(InputIterator first, InputIterator last);

    bool empty() const;
    std::size_t size() const;

    /// Allows unguarded access
    std::vector<T, Alloc> const& vector() const { return vector_; }
    /// Allows unguarded access
    std::vector<T, Alloc>& vector() { return vector_; }

private:
    std::vector<T, Alloc> vector_;
    // Otherwise we have to discard various const qualifiers, which we don't want (really!)
    mutable std::mutex mutex_;
};

}  // namespace mt

}  // namespace nie

#include "mt_vector.inl"
