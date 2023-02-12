// Copyright 2019 Christopher Ho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
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
// * Adding aiim prefix

#ifndef aiim_motion_testing__VISIBILITY_CONTROL_HPP_
#define aiim_motion_testing__VISIBILITY_CONTROL_HPP_

#if defined(__WIN32)
#if defined(aiim_motion_testing_BUILDING_DLL) ||                               \
    defined(aiim_motion_testing_EXPORTS)
#define aiim_motion_testing_PUBLIC __declspec(dllexport)
#define aiim_motion_testing_LOCAL
#else // defined(aiim_motion_testing_BUILDING_DLL) ||
      // defined(aiim_motion_testing_EXPORTS)
#define aiim_motion_testing_PUBLIC __declspec(dllimport)
#define aiim_motion_testing_LOCAL
#endif // defined(aiim_motion_testing_BUILDING_DLL) ||
       // defined(aiim_motion_testing_EXPORTS)
#elif defined(__linux__)
#define aiim_motion_testing_PUBLIC __attribute__((visibility("default")))
#define aiim_motion_testing_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define aiim_motion_testing_PUBLIC __attribute__((visibility("default")))
#define aiim_motion_testing_LOCAL __attribute__((visibility("hidden")))
#else // defined(_LINUX)
#error "Unsupported Build Configuration"
#endif // defined(_WINDOWS)

#endif // aiim_motion_testing__VISIBILITY_CONTROL_HPP_
