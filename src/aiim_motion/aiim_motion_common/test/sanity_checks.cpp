// Copyright 2017-2021 the Autoware Foundation
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
// The changes made in this file, of which a summary is listed below, are
// copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights
// reserved Information classification: Confidential This content is protected
// by international copyright laws. Reproduction and distribution is prohibited
// without written permission.
//
// List of changes:
// * Changed namespace from autoware to aiim
// * Adding aiim prefix to includes
/// \file
/// \brief This file includes basic tests for utility functions in motion_common

#include <gtest/gtest.h>

#include <aiim_autoware_common/types.hpp>
#include <aiim_autoware_msgs/msg/complex32.hpp>
#include <aiim_motion_common/motion_common.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using aiim::common::types::float64_t;
using aiim::motion::motion_common::from_quat;
using aiim::motion::motion_common::to_quat;
using aiim_autoware_msgs::msg::Complex32;
using geometry_msgs::msg::Quaternion;

struct MyQuaternion {
  float64_t x{0.0};
  float64_t y{0.0};
  float64_t z{0.0};
  float64_t w{1.0};
};

TEST(HeadingFuncs, to_quat) {
  Complex32 complex_heading{};
  complex_heading.real = 0.5f;
  complex_heading.imag = 0.5f;

  Quaternion gm_quat{};
  MyQuaternion my_quat{};

  gm_quat = to_quat<Quaternion>(complex_heading);
  my_quat = to_quat<MyQuaternion>(complex_heading);

  EXPECT_FLOAT_EQ(gm_quat.z, complex_heading.imag);
  EXPECT_FLOAT_EQ(gm_quat.w, complex_heading.real);
  EXPECT_FLOAT_EQ(my_quat.z, complex_heading.imag);
  EXPECT_FLOAT_EQ(my_quat.w, complex_heading.real);
}

TEST(HeadingFuncs, from_quat) {
  Quaternion gm_quat{};
  gm_quat.z = 0.5f;
  gm_quat.w = 0.5f;

  Complex32 complex_heading{};
  complex_heading = from_quat<Quaternion>(gm_quat);

  EXPECT_FLOAT_EQ(complex_heading.imag, gm_quat.z);
  EXPECT_FLOAT_EQ(complex_heading.real, gm_quat.w);

  MyQuaternion my_quat{};
  my_quat.z = 0.75;
  my_quat.w = 0.25;

  complex_heading = from_quat<MyQuaternion>(my_quat);

  EXPECT_FLOAT_EQ(complex_heading.imag, my_quat.z);
  EXPECT_FLOAT_EQ(complex_heading.real, my_quat.w);
}
