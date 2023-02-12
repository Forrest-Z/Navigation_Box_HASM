/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/lidar/cloud_filter.hpp>
#include <nie/lidar/io/las_reader.hpp>

template <typename PointT>
class LasLoader {
public:
    LasLoader(double filter_grid_size) : 
        identity_(true), T_av_(nie::Isometry3qd::Identity()), filter_(typename nie::CloudFilter<PointT>::Parameters{filter_grid_size}) {}

    LasLoader(nie::Isometry3qd T_av, double filter_grid_size) : 
        identity_(T_av_ == nie::Isometry3qd::Identity()), T_av_(T_av), filter_(typename nie::CloudFilter<PointT>::Parameters{filter_grid_size}) {}

    nie::Cloud<PointT> operator()(std::string const& filename, std::chrono::weeks const& weeks) {
        if(!identity_) {
            return operator()(filename, weeks, nie::Isometry3qd::Identity());
        } else {
            return filter_.Filter(nie::io::ReadLas<PointT>(filename, weeks));
        }
    }

    // Even though it looks like we don't change the state, this operator function cannot be const because of filter_.Filter().
    nie::Cloud<PointT> operator()(std::string const& filename, std::chrono::weeks const& weeks, nie::Isometry3qd const& T) {
        nie::Cloud<PointT> cloud = nie::io::ReadLas<PointT>(filename, weeks);
        return filter_.Filter(nie::TransformCloud(cloud, T * T_av_, T_av_ * cloud.origin() * T_av_.Inversed() * T.Inversed()));
    }

private:
    bool identity_;
    // Vehicle to aircraft.
    nie::Isometry3qd T_av_;
    nie::CloudFilter<PointT> filter_;
};
