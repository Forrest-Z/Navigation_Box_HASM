/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Core>

namespace nie {

namespace detail {

// Based on https://stackoverflow.com/a/38140932 and boost hash_combine
inline std::size_t CombineHash(std::size_t const hash) { return hash; }

template <typename T, typename... Rest>
inline std::size_t CombineHash(std::size_t const hash, T const& value, Rest&&... rest) {
    return CombineHash(
            hash ^ (std::hash<T>{}(value) + 0x9e3779b9 + (hash << 6) + (hash >> 2)), std::forward<Rest>(rest)...);
}

}  // namespace detail

template <typename... Types>
inline std::size_t CreateHash(Types&&... values) {
    return detail::CombineHash(0, std::forward<Types>(values)...);
}

struct PairHash {
    template <typename T1, typename T2>
    std::size_t operator()(std::pair<T1, T2> const& pair) const {
        return CreateHash(pair.first, pair.second);
    }
};

struct EigenHash {
    template <typename Derived>
    std::size_t operator()(Eigen::PlainObjectBase<Derived> const& object) const {
        using Base = Eigen::PlainObjectBase<Derived>;
        static_assert(Base::RowsAtCompileTime != Eigen::Dynamic, "EigenHash not implemented for dynamic size types.");
        static_assert(Base::ColsAtCompileTime != Eigen::Dynamic, "EigenHash not implemented for dynamic size types.");

        Eigen::Map<Derived const> const map(object.data());
        std::size_t hash = 0;
        for (std::size_t i = 0; i < static_cast<std::size_t>(Base::RowsAtCompileTime * Base::ColsAtCompileTime); ++i) {
            hash = detail::CombineHash(hash, map[i]);
        }
        return hash;
    }
};

}  // namespace nie
