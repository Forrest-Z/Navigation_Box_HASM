/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef CORRESPONDENCE_VIEWER_HELPER_IO_HPP
#define CORRESPONDENCE_VIEWER_HELPER_IO_HPP

#include <pcl/io/ply_io.h>
#include <nie/core/filesystem.hpp>
#include <nie/core/string.hpp>
#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>
#include <nie/lidar/cloud.hpp>

namespace nie {

struct Correspondence {
    std::size_t index_a;
    std::size_t index_b;
    float distance;
};

std::vector<Correspondence> ReadCorrespondenceCsv(std::string const& path) {
    nie::CsvRecorderFast<std::size_t, std::size_t, float> csv_recorder{','};
    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>(
            {path}, "point_id_a,point_id_b,distance", &csv_recorder);
    auto csv_record = csv_recorder.ConvertToRecord();

    std::vector<Correspondence> result;
    result.reserve(csv_record.NumRows());
    for (std::size_t i = 0; i < csv_record.NumRows(); ++i) {
        auto row = csv_record.GetRow(i);
        result.push_back({std::get<0>(row), std::get<1>(row), std::get<2>(row)});
    }
    return result;
}

template <typename PointT>
nie::Cloud<PointT> ReadPlyFile(std::string const& file_name) {
    nie::Cloud<PointT> cloud(nie::PoseBbox(Eigen::Vector3d(0., 0., 0.), nie::Bboxf::MaxBoundingBox()));
    pcl::PLYReader().read(file_name, cloud.point_cloud());
    cloud.bounds().UpdateBbox(cloud.point_cloud());
    return cloud;
}

}  // namespace nie

#endif  // CORRESPONDENCE_VIEWER_HELPER_IO_HPP
