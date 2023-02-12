/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <pcl/visualization/point_cloud_color_handlers.h>

#include "point_cloud_color_handler_intensity.hpp"

namespace nie {

// Type definitions

template <typename PointT>
using PointCloudColorHandler = pcl::visualization::PointCloudColorHandler<PointT>;
template <typename PointT>
using PointCloudColorHandlerPtr = typename PointCloudColorHandler<PointT>::Ptr;
template <typename PointT>
using PointCloudColorHandlerCustom = pcl::visualization::PointCloudColorHandlerCustom<PointT>;

// Single cloud color handler creator functions

template <typename PointT>
PointCloudColorHandlerPtr<PointT> GetColorHandlerReferenceCloud() {
    return PointCloudColorHandlerPtr<PointT>{new PointCloudColorHandlerCustom<PointT>(0, 0, 255)};
}
template <>
PointCloudColorHandlerPtr<pcl::PointXYZI> GetColorHandlerReferenceCloud<pcl::PointXYZI>() {
    return PointCloudColorHandlerPtr<pcl::PointXYZI>{new PointCloudColorHandlerIntensity<pcl::PointXYZI>(240.f, 1.f)};
}

template <typename PointT>
PointCloudColorHandlerPtr<PointT> GetColorHandlerRegisteredCloud() {
    return PointCloudColorHandlerPtr<PointT>{new PointCloudColorHandlerCustom<PointT>(255, 0, 0)};
}
template <>
PointCloudColorHandlerPtr<pcl::PointXYZI> GetColorHandlerRegisteredCloud<pcl::PointXYZI>() {
    return PointCloudColorHandlerPtr<pcl::PointXYZI>{new PointCloudColorHandlerIntensity<pcl::PointXYZI>(0.f, 1.f)};
}

template <typename PointT>
PointCloudColorHandlerPtr<PointT> GetColorHandlerUnRegisteredCloud() {
    return PointCloudColorHandlerPtr<PointT>{new PointCloudColorHandlerCustom<PointT>(153, 0, 0)};
}
template <>
PointCloudColorHandlerPtr<pcl::PointXYZI> GetColorHandlerUnRegisteredCloud<pcl::PointXYZI>() {
    return PointCloudColorHandlerPtr<pcl::PointXYZI>{new PointCloudColorHandlerIntensity<pcl::PointXYZI>(0.f, 0.6f)};
}

// Cloud color handler vector creator functions

template <typename PointT>
std::vector<PointCloudColorHandlerPtr<PointT>> GetColorHandlersRegisteredClouds1() {
    return {GetColorHandlerReferenceCloud<PointT>()};
}
template <typename PointT>
std::vector<PointCloudColorHandlerPtr<PointT>> GetColorHandlersRegisteredClouds2() {
    return {GetColorHandlerReferenceCloud<PointT>(), GetColorHandlerRegisteredCloud<PointT>()};
}
template <typename PointT>
std::vector<PointCloudColorHandlerPtr<PointT>> GetColorHandlersUnregisteredClouds2() {
    return {GetColorHandlerReferenceCloud<PointT>(), GetColorHandlerUnRegisteredCloud<PointT>()};
}
template <typename PointT>
std::vector<PointCloudColorHandlerPtr<PointT>> GetColorHandlersRegisteredClouds3() {
    return {GetColorHandlerReferenceCloud<PointT>(),
            GetColorHandlerRegisteredCloud<PointT>(),
            GetColorHandlerUnRegisteredCloud<PointT>()};
}

// Single trace color handler creator functions

PointCloudColorHandlerPtr<pcl::PointXYZ> GetColorHandlerReferenceTrace() {
    return PointCloudColorHandlerPtr<pcl::PointXYZ>{new PointCloudColorHandlerCustom<pcl::PointXYZ>(0, 164, 255)};
}
PointCloudColorHandlerPtr<pcl::PointXYZ> GetColorHandlerUnRegisteredTrace() {
    return PointCloudColorHandlerPtr<pcl::PointXYZ>{new PointCloudColorHandlerCustom<pcl::PointXYZ>(153, 98, 0)};
}
PointCloudColorHandlerPtr<pcl::PointXYZ> GetColorHandlerRegisteredTrace() {
    return PointCloudColorHandlerPtr<pcl::PointXYZ>{new PointCloudColorHandlerCustom<pcl::PointXYZ>(255, 164, 0)};
}

// Trace color handler vector creator functions

std::vector<PointCloudColorHandlerPtr<pcl::PointXYZ>> GetColorHandlersRegisteredTraces1() {
    return {GetColorHandlerReferenceTrace()};
}
std::vector<PointCloudColorHandlerPtr<pcl::PointXYZ>> GetColorHandlersRegisteredTraces2() {
    return {GetColorHandlerReferenceTrace(), GetColorHandlerRegisteredTrace()};
}
std::vector<PointCloudColorHandlerPtr<pcl::PointXYZ>> GetColorHandlersUnregisteredTraces2() {
    return {GetColorHandlerReferenceTrace(), GetColorHandlerUnRegisteredTrace()};
}
std::vector<PointCloudColorHandlerPtr<pcl::PointXYZ>> GetColorHandlersRegisteredTraces3() {
    return {GetColorHandlerReferenceTrace(), GetColorHandlerRegisteredTrace(), GetColorHandlerUnRegisteredTrace()};
}

}  // namespace nie
