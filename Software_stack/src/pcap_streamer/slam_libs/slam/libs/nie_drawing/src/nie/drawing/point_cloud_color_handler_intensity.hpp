/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/visualization/point_cloud_color_handlers.h>

namespace nie {

/// @brief Adapted version of the RGB one from PCL. Can map intensity and blend it a specific color.
/// @details pcl doesn't seem to support combinations of point cloud color handlers. So here a custom one.
template <typename PointT>
class PointCloudColorHandlerIntensity : public pcl::visualization::PointCloudColorHandler<PointT> {
    typedef typename pcl::visualization::PointCloudColorHandler<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

public:
    typedef boost::shared_ptr<PointCloudColorHandlerIntensity<PointT>> Ptr;
    typedef boost::shared_ptr<const PointCloudColorHandlerIntensity<PointT>> ConstPtr;

    /** \brief Constructor. */
    PointCloudColorHandlerIntensity(float hue_offet, float value)
        : hue_offset_(hue_offet),
          value_(value),
          min_intensity_(std::numeric_limits<float>::max()),
          max_intensity_(std::numeric_limits<float>::lowest()) {
        capable_ = false;
    }

    /** \brief Constructor. */
    PointCloudColorHandlerIntensity(float hue_offet, float value, const PointCloudConstPtr& cloud)
        : hue_offset_(hue_offet),
          value_(value),
          pcl::visualization::PointCloudColorHandler<PointT>(cloud),
          min_intensity_(std::numeric_limits<float>::max()),
          max_intensity_(std::numeric_limits<float>::lowest()) {
        setInputCloud(cloud);
    }

    /** \brief Destructor. */
    virtual ~PointCloudColorHandlerIntensity() {}

    /** \brief Get the name of the field used. */
    virtual std::string getFieldName() const { return ("intensity"); }

    /** \brief Obtain the actual color for the input dataset as vtk scalars.
     * \param[out] scalars the output scalars containing the color for the dataset
     * \return true if the operation was successful (the handler is capable and
     * the input cloud was given as a valid pointer), false otherwise
     */
    virtual bool getColor(vtkSmartPointer<vtkDataArray>& scalars) const;

    /** \brief Set the input cloud to be used.
     * \param[in] cloud the input cloud to be used by the handler
     */
    virtual void setInputCloud(const PointCloudConstPtr& cloud);

protected:
    float HueFromIntensity(float intensity) const;

    /** \brief Class getName method. */
    virtual std::string getName() const { return ("PointCloudColorHandlerIntensity"); }

    // Members derived from the base class
    using pcl::visualization::PointCloudColorHandler<PointT>::cloud_;
    using pcl::visualization::PointCloudColorHandler<PointT>::capable_;
    using pcl::visualization::PointCloudColorHandler<PointT>::field_idx_;
    using pcl::visualization::PointCloudColorHandler<PointT>::fields_;

    float hue_offset_;
    float value_;
    float min_intensity_;
    float max_intensity_;
};

}  // namespace nie

#include "point_cloud_color_handler_intensity.inl"
