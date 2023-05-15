/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

// File named after #include <algorithm>

#include <vector>

#include <Eigen/Core>

namespace nie {

/// @brief Similar to std::copy_if but it allows to copy based on indices, not a value or predicate.
/// @details Container for result must be allocated already. Undefined behavior if indices aren't sorted
template <typename InputIt, typename OutputIt, typename ForwardIt>
OutputIt CopyIf(InputIt first_it, InputIt last_it, OutputIt result, ForwardIt index_it_first, ForwardIt index_it_last) {
    if (index_it_first == index_it_last) {
        return result;
    }

    std::size_t input_index = *index_it_first;
    std::advance(first_it, input_index);

    // Loop over the input container
    for (; first_it != last_it and index_it_first != index_it_last; ++input_index, ++first_it) {
        if (input_index == *index_it_first) {
            *result = *first_it;
            ++index_it_first;
            ++result;
        }
    }

    return result;
}

/// @brief Similar to std::remove_if but it allows to reorder based on indices, not a value or predicate.
/// @details Undefined behavior if indices aren't sorted
template <typename T, typename Alloc, typename ForwardIt>
typename std::vector<T, Alloc>::iterator RemoveIf(
        ForwardIt index_it_begin, ForwardIt index_it_end, std::vector<T, Alloc>* p_vector) {
    auto& vector = *p_vector;

    if (index_it_begin == index_it_end) return vector.end();

    auto first = vector.begin() + *index_it_begin;
    auto index = *index_it_begin;

    // Move 1 past the last index to be removed. This makes sure the next loop starts on this +1.
    for (; index != *(index_it_end - 1) + 1; ++index) {
        if (index != *index_it_begin) {
            *first = std::move(vector[index]);
            ++first;
        } else {
            ++index_it_begin;
        }
    }

    // Here we don't update the index iterator anymore since we moved past the last one and we don't know
    // what is in memory (may have bad luck that it actually points to a valid index).
    for (; index != vector.size(); ++index) {
        *first = std::move(vector[index]);
        ++first;
    }

    return first;
}

/// @brief Similar to std::remove_if but it allows to reorder based on a container, not a value or predicate.
/// @details Undefined behavior if indices aren't sorted
template <typename T, typename Alloc>
typename std::vector<T, Alloc>::iterator RemoveIf(std::vector<bool> filter, std::vector<T, Alloc>* p_vector) {
    if (filter.size() != p_vector->size()) {
        filter.resize(p_vector->size(), false);
    }

    // Two notes on the implementation below
    //  - remove_if does not guarantee sequential execution of the predicate, therefore stable_partition is used.
    //  - When the iterator is put inside the lambda captures '[]', then somehow the iterator is one ahead in the
    //    iteration.
    auto filter_iter = filter.cbegin();
    return std::stable_partition(  // puts 'true' before 'false', hence the not
            p_vector->begin(),
            p_vector->end(),
            [&filter_iter](auto) {
                bool const result = not(*filter_iter);
                ++filter_iter;
                return result;
            });
}

/// @brief Similar to std::for_each but can be used with a tuple
// By example of https://stackoverflow.com/q/1198260
// TODO: When switched to C++17, this can be simplified a lot (see link)
template <std::size_t I = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<I == sizeof...(Tp), void>::type ForEach(std::tuple<Tp...>&, FuncT) {}

template <std::size_t I = 0, typename FuncT, typename... Tp>
        inline typename std::enable_if < I<sizeof...(Tp), void>::type ForEach(std::tuple<Tp...>& t, FuncT f) {
    f(std::get<I>(t));
    ForEach<I + 1, FuncT, Tp...>(t, f);
}

struct EigenEqual {
    template <typename Derived>
    bool operator()(Eigen::PlainObjectBase<Derived> const& a, Eigen::PlainObjectBase<Derived> const& b) const {
        return (a == b).all();
    }
};

}  // namespace nie
