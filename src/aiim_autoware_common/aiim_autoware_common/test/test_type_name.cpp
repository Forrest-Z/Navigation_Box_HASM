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

#include <aiim_autoware_common/types.hpp>
#include <aiim_helper_functions/type_name.hpp>

#include <gtest/gtest.h>

namespace {
using aiim::common::types::float32_t;
using aiim::common::types::float64_t;
using aiim::helper_functions::get_type_name;

struct SomeStruct {};
}  // namespace

/// @test       Test that type names can be demangled.
TEST(TestTypeDemangling, Demangle) {
    EXPECT_EQ(get_type_name<float32_t>(), "float");
    const float64_t val{42.0};
    EXPECT_EQ(get_type_name(val), "double");
    EXPECT_EQ(get_type_name<SomeStruct>(), "(anonymous namespace)::SomeStruct");
}
