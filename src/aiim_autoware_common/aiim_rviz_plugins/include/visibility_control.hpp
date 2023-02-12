// Copyright 2019 the Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// The changes made in this file, of which a summary is listed below, are copyrighted:
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
// Information classification: Confidential
// This content is protected by international copyright laws.
// Reproduction and distribution is prohibited without written permission.
//
// List of changes:
// * Renamed definition name from AUTOWARE_RVIZ_PLUGINS_PUBLIC to AIIM_RVIZ_PLUGINS_PUBLIC
#pragma once

#if defined(__WIN32)
#if defined(AIIM_RVIZ_PLUGINS_BUILDING_DLL) || defined(AIIM_RVIZ_PLUGINS_EXPORTS)
#define AIIM_RVIZ_PLUGINS_PUBLIC __declspec(dllexport)
#define AIIM_RVIZ_PLUGINS_LOCAL
// defined(AIIM_RVIZ_PLUGINS_BUILDING_DLL) || defined(AIIM_RVIZ_PLUGINS_EXPORTS)
#else
#define AIIM_RVIZ_PLUGINS_PUBLIC __declspec(dllimport)
#define AIIM_RVIZ_PLUGINS_LOCAL
// defined(AIIM_RVIZ_PLUGINS_BUILDING_DLL) || defined(AIIM_RVIZ_PLUGINS_EXPORTS)
#endif
#elif defined(__linux__)
#define AIIM_RVIZ_PLUGINS_PUBLIC __attribute__((visibility("default")))
#define AIIM_RVIZ_PLUGINS_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
#define AIIM_RVIZ_PLUGINS_PUBLIC __attribute__((visibility("default")))
#define AIIM_RVIZ_PLUGINS_LOCAL __attribute__((visibility("hidden")))
#else  // defined(_LINUX)
#error "Unsupported Build Configuration"
#endif  // defined(_WINDOWS)
