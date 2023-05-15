/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <type_traits>
#include <utility>

namespace nie {

// Implementation of all and any based on https://stackoverflow.com/q/24687026
template <bool... Bs>
using bool_sequence = std::integer_sequence<bool, Bs...>;

template <bool... Bs>
using all = std::is_same<bool_sequence<true, Bs...>, bool_sequence<Bs..., true>>;
template <bool... Bs>
constexpr bool all_v = all<Bs...>::value;

template <bool... Bs>
using any = std::integral_constant<bool, !all_v<!Bs...>>;
template <bool... Bs>
constexpr bool any_v = any<Bs...>::value;

// TODO: The following can be replaced by std::is_invocable when switched to C++17
namespace detail {

template <typename T>
struct is_invocable_impl {
    typedef char yes_type[1];
    typedef char no_type[2];
    template <typename Q>
    static yes_type& check(decltype(&Q::operator())*);
    template <typename Q>
    static no_type& check(...);
    static const bool value = sizeof(check<T>(0)) == sizeof(yes_type);
};
template <typename R, typename... Args>
struct is_invocable_impl<R (*)(Args...)> : std::true_type {};

template <typename R, typename... Args>
struct is_invocable_impl<R (&)(Args...)> : std::true_type {};

}  // namespace detail

template <typename T>
using is_invocable = std::integral_constant<bool, detail::is_invocable_impl<T>::value>;

}  // namespace nie
