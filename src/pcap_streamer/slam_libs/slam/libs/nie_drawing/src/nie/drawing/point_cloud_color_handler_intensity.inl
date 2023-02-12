/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "color_conversion.hpp"

namespace nie {

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void PointCloudColorHandlerIntensity<PointT>::setInputCloud(const PointCloudConstPtr& cloud) {
    pcl::visualization::PointCloudColorHandler<PointT>::setInputCloud(cloud);
    field_idx_ = pcl::getFieldIndex<PointT>("intensity", fields_);
    if (field_idx_ != -1)
        capable_ = true;
    else
        capable_ = false;

    if (capable_) {
        for (auto const& p : cloud->points) {
            min_intensity_ = std::min(p.intensity, min_intensity_);
            max_intensity_ = std::max(p.intensity, max_intensity_);
        }
    }
}

template <typename PointT>
float PointCloudColorHandlerIntensity<PointT>::HueFromIntensity(float intensity) const {
    return std::fmod(hue_offset_ + (intensity - min_intensity_) / (max_intensity_ - min_intensity_) * 180.0f, 360.0f);
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
bool PointCloudColorHandlerIntensity<PointT>::getColor(vtkSmartPointer<vtkDataArray>& scalars) const {
    if (!capable_ || !cloud_) return (false);

    if (!scalars) scalars = vtkSmartPointer<vtkUnsignedCharArray>::New();
    scalars->SetNumberOfComponents(4);

    vtkIdType nr_points = cloud_->points.size();
    reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->SetNumberOfTuples(nr_points);
    unsigned char* colors = reinterpret_cast<vtkUnsignedCharArray*>(&(*scalars))->GetPointer(0);

    int j = 0;
    // If XYZ present, check if the points are invalid
    int x_idx = -1;
    for (size_t d = 0; d < fields_.size(); ++d)
        if (fields_[d].name == "x") x_idx = static_cast<int>(d);

    if (x_idx != -1) {
        // Color every point
        for (vtkIdType cp = 0; cp < nr_points; ++cp) {
            // Copy the value at the specified field
            if (!pcl::isFinite(cloud_->points[cp])) continue;

            float h = HueFromIntensity(cloud_->points[cp].intensity);
            auto p = MakePointXYZRGB(h, value_);
            colors[j] = p.r;
            colors[j + 1] = p.g;
            colors[j + 2] = p.b;
            colors[j + 3] = 255;
            j += 4;
        }
    } else {
        // Color every point
        for (vtkIdType cp = 0; cp < nr_points; ++cp) {
            float h = HueFromIntensity(cloud_->points[cp].intensity);
            auto p = MakePointXYZRGB(h, value_);
            int idx = static_cast<int>(cp) * 4;
            colors[idx] = p.r;
            colors[idx + 1] = p.g;
            colors[idx + 2] = p.b;
            colors[idx + 3] = 255;
        }
    }
    return (true);
}

}  // namespace nie