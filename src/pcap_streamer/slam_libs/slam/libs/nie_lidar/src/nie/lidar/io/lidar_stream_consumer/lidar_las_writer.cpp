/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "lidar_las_writer.hpp"

#include <numeric>

#include "nie/lidar/io/las.hpp"

namespace {

using LasPoint = nie::io::LasPoint<
        pdal::Dimension::Id::X,
        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Intensity,
        pdal::Dimension::Id::GpsTime>;

std::vector<LasPoint> ConvertPoints(
        pcl::PointCloud<pcl::PointXYZI> const& points,
        Eigen::Vector3d const& offset,
        std::vector<nie::Timestamp_ns> const& timestamps) {
    // Safety check for debugging
    DCHECK(points.size() == timestamps.size()) << "Returns object contains " << points.size() << " lidar points but "
                                               << timestamps.size() << " timestamps.";
    VLOG(6) << "Processing " << points.size() << " lidar points.";

    // Result to be returned
    std::vector<LasPoint> result(points.size());

    // Loop over all lidar points and timestamps
    for (std::size_t n = 0; n < points.size(); ++n) {
        Eigen::Vector3d const p = offset + points[n].getVector3fMap().cast<double>();
        auto const& intensity = points[n].intensity;
        nie::Timestamp_ns const& t = timestamps[n];
        // According to las, if bitfield bit 0 is set to 0, we use time_in_week
        // TODO: Header should be set accordingly
        double t_d = nie::RepresentDurationAsDouble(nie::ToGPSWeekTime(t).time_in_week);
        result[n] = {p.x(), p.y(), p.z(), intensity, t_d};
    }

    return result;
}

// The las format will store the point coordinates as, written value = (actual value - offset) / scale where the written
// value should be of type std::int32_t. The more the values spread across the int32 range, the better the precision.
double DetermineScale(nie::Bboxf const& bbox) {
    std::array<Eigen::Vector3f, 8> const corners = bbox.GetAllCorners();

    auto op = [](double const r, Eigen::Vector3f const& p) {
        double n = p.cast<double>().squaredNorm();
        return n > r ? n : r;
    };

    double radius_sqr = std::accumulate(corners.cbegin(), corners.cend(), 0.0, op);

    // The safety factor 1.001 is to ensure all points will fit in the range.
    return std::sqrt(radius_sqr) * 1.001 / static_cast<double>(std::numeric_limits<std::int32_t>::max());
}

}  // namespace

namespace nie {

void LidarLasWriter::ProcessSlice(
        pcl::PointCloud<pcl::PointXYZI> const& points,
        Eigen::Vector3d const& offset,
        std::vector<Timestamp_ns> const& timestamps) {
    auto const filepath_las = las_filenamer_.Peek();
    LOG(INFO) << "Writing las " << filepath_las;
    nie::io::WriteLas(filepath_las, ConvertPoints(points, offset, timestamps));
}

void LidarLasWriter::ProcessSliceWithBounds(
        pcl::PointCloud<pcl::PointXYZI> const& points,
        Eigen::Vector3d const& offset,
        std::vector<Timestamp_ns> const& timestamps,
        PoseBbox const& bounds) {
    auto const filepath_las = las_filenamer_.Peek();
    LOG(INFO) << "Writing las " << filepath_las;
    nie::io::WriteLasWithOffsetScale(
            filepath_las,
            ConvertPoints(points, offset, timestamps),
            bounds.origin().translation() + offset,
            DetermineScale(bounds.bbox()));
}

}  // namespace nie
