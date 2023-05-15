/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <vector>  // std::vector<T>::const_iterator;

#include <glog/logging.h>

namespace nie {

/// Functor that returns its argument.
struct Identity {
    template <typename T>
    T operator()(T t) {
        return t;
    }
};

/// Finds two iterators, left and right, so that the following holds for a given query q:
///     Compare(Adapter(*left), q) and Compare(q, Adapter(*right)).
///
/// Best performance when query values are in the same sorted order as the input data.
///
/// @precondition Input vector must be sorted.
///
/// \tparam T
/// \tparam Adapter Functor to convert type @tparam{T} to type U
/// \tparam Compare Binary operator so that Compare(Adapter(sorted_vector[i-1]), Adapter(sorted_vector[i])) == true
/// \tparam U  type of Adapter(sorted_vector[i])
template <
    typename T,
    template <typename> class Compare_ = std::less,
    typename Adapter = Identity,
    typename U = decltype(std::declval<Adapter>()(std::declval<T>()))>
class RangeFinder {
public:
    // TODO: Only have iterator so dependency on vector should be removed, find best way to do this.
    using Iterator = typename std::vector<T>::const_iterator;
    using Compare = Compare_<U>;

public:
    RangeFinder(std::vector<T> const& sorted_vector, Adapter adapter = Adapter{}, Compare compare = Compare{})
        : begin_{sorted_vector.cbegin()},
          end_{sorted_vector.cend()},
          adapter_{std::move(adapter)},
          compare_{std::move(compare)},
          it_before_{begin_},
          it_after_{end_} {
        CHECK(std::is_sorted(begin_, end_, [&](auto const& a, auto const& b) {
            return compare_(adapter(a), adapter(b));
        })) << "Input vector for RangeFinder must be sorted.";
    }
    RangeFinder(Iterator const first, Iterator const last, Adapter adapter = Adapter{}, Compare compare = Compare{})
        : begin_{first}, end_{last}, adapter_{adapter}, compare_{compare}, it_before_{begin_}, it_after_{end_} {
        CHECK(std::is_sorted(begin_, end_, [&](auto const& a, auto const& b) {
            return compare(adapter(a), adapter(b));
        })) << "Input vector for RangeFinder must be sorted.";
    }

    /// Find two iterators, left and right, so that the following holds:
    ///     Compare(Adapter(*left), @param{query_u}) and Compare(@param{query_u}, Adapter(*right)).
    /// If @param{query_u} comes before first_ or after _last then last_ iterators is returned.
    ///
    /// \param query_u query value for finding range.
    /// \return  tuple {left, right, success}
    std::tuple<Iterator, Iterator, bool> operator()(U const& query_u) {
        // Check if query is "before" begin
        if (!CompareOrEqual(adapter_(*begin_), query_u)) {
            return {end_, end_, false};
        }
        // Check if query is "after" end
        if (!CompareOrEqual(query_u, adapter_(*(end_ - 1)))) {
            return {end_, end_, false};
        }

        Iterator it_after;

        if (CompareOrEqual(adapter_(*it_before_), query_u)) {
            // it_before_ is before query so order is maintained
            // Find first pose after the query
            it_after = std::lower_bound(
                it_before_, end_, query_u, [&](T const& t, U const& u) { return CompareAndNotEqual(adapter_(t), u); });
        } else {
            // it_before_ is after query so order was broken
            // Find first pose after the query
            it_after = std::lower_bound(begin_, it_before_, query_u, [&](T const& t, U const& u) {
                return CompareAndNotEqual(adapter_(t), u);
            });
        }

        // Find last pose before the query
        auto it_before = it_after;
        while (it_before != begin_ && !CompareOrEqual(adapter_(*it_before), query_u)) {
            --it_before;
        }

        it_before_ = it_before;
        it_after_ = it_after;

        return {it_before, it_after, true};
    }

    Iterator const& it_before() const { return it_before_; }
    Iterator const& it_after() const { return it_after_; }

private:
    Iterator const begin_;
    Iterator const end_;
    Adapter adapter_;
    Compare compare_;
    Iterator it_before_;
    Iterator it_after_;

    template <template <class> class Comp, typename std::enable_if_t<!Comp<int>{}(0, 0), void*> = nullptr>
    constexpr bool CompareOrEqualImpl(U const& l, U const& r) {
        return compare_(l, r) || l == r;
    }
    template <template <class> class Comp, typename std::enable_if_t<Comp<int>{}(0, 0), void*> = nullptr>
    constexpr bool CompareOrEqualImpl(U const& l, U const& r) {
        return compare_(l, r);
    }
    template <template <class> class Comp, typename std::enable_if_t<!Comp<int>{}(0, 0), void*> = nullptr>
    constexpr bool CompareAndNotEqualImpl(U const& l, U const& r) {
        return compare_(l, r);
    }
    template <template <class> class Comp, typename std::enable_if_t<Comp<int>{}(0, 0), void*> = nullptr>
    constexpr bool CompareAndNotEqualImpl(U const& l, U const& r) {
        return compare_(l, r) && l != r;
    }

    constexpr bool CompareOrEqual(U const& l, U const& r) { return CompareOrEqualImpl<Compare_>(l, r); }
    constexpr bool CompareAndNotEqual(U const& l, U const& r) { return CompareAndNotEqualImpl<Compare_>(l, r); }
};

}  // namespace nie
