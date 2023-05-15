// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Developed by Apex.AI, Inc.
//
// The changes made in this file, of which a summary is listed below, are copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
// Information classification: Confidential
// This content is protected by international copyright laws.
// Reproduction and distribution is prohibited without written permission.
//
// List of changes:
// * Changed namespace from autoware to aiim
// * Adding aiim prefix to includes

#include <aiim_autoware_common/type_traits.hpp>
#include <aiim_autoware_common/types.hpp>

#include <gtest/gtest.h>

#include <tuple>

namespace {
/// @brief      A simple testing function to check if all types are arithmetic.
///
///             Trait "is_arithmetic" is picked at random and any other trait could have been used
///             instead.
template <typename... Ts>
bool all_are_arithmetic() {
    // This is just a random function that we use with conjunction.
    return aiim::common::type_traits::conjunction<std::is_arithmetic<Ts>...>::value;
}

using aiim::common::types::float32_t;
using aiim::common::types::float64_t;

}  // namespace

/// @test       Test that index of a type can be computed.
TEST(TestCommonTypeTraits, Index) {
    using T = std::tuple<std::int32_t, float64_t>;
    EXPECT_EQ(0, (aiim::common::type_traits::index<std::int32_t, T>::value));
    EXPECT_EQ(1, (aiim::common::type_traits::index<float64_t, T>::value));
}

TEST(TestCommonTypeTraits, Conjunction) {
    EXPECT_TRUE((all_are_arithmetic<std::int32_t, float32_t>()));
    EXPECT_FALSE((all_are_arithmetic<std::int32_t, float32_t, std::tuple<std::int32_t, float32_t>>()));
}

TEST(TestCommonTypeTraits, Visit) {
    const std::tuple<std::int32_t, float64_t> t;
    std::int32_t counter{};
    aiim::common::type_traits::visit(t, [&counter](const auto&) { counter++; });
    EXPECT_EQ(2, counter);
    float64_t sum{};
    aiim::common::type_traits::visit(
            std::make_tuple(2, 42.0F, 23.0), [&sum](const auto& element) { sum += static_cast<float64_t>(element); });
    EXPECT_DOUBLE_EQ(67.0, sum);
}
