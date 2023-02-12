/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include "nie/core/type_traits.hpp"

#include <tuple>
#include <vector>

namespace nie {

namespace detail {

template <bool Enabled, typename Handle_, typename... Functions>
class CallbacksBase {};

template <typename Handle_, typename... Functions>
class CallbacksBase<true, Handle_, Functions...> {
public:
    using Handle = Handle_;

    template <Handle index, typename Function>
    void AddCallback(Function const& function) {
        GetCallBacks<index>().push_back(function);
    }

    template <Handle index>
    bool HasCallback() const {
        return !GetCallBacks<index>().empty();
    }

    template <Handle index, typename... Arguments>
    void Callback(Arguments... arguments) const {
        auto const& fs = GetCallBacks<index>();
        for (auto const& f : fs) {
            if (f) {
                f(arguments...);
            }
        }
    }

protected:
    CallbacksBase() = default;

private:
    template <Handle index>
    auto const& GetCallBacks() const {
        return std::get<static_cast<std::size_t>(index)>(callbacks_);
    }
    template <Handle index>
    auto& GetCallBacks() {
        return std::get<static_cast<std::size_t>(index)>(callbacks_);
    }

    std::tuple<std::vector<Functions>...> callbacks_;
};

}  // namespace detail

template <typename Handle, typename... Functions>
class Callbacks : public detail::CallbacksBase<
                          std::is_enum<Handle>::value and nie::all<nie::is_invocable<Functions>::value...>::value,
                          Handle,
                          Functions...> {};

}  // namespace nie
