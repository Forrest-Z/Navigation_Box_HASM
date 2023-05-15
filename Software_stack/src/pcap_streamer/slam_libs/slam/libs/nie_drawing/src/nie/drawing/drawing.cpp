/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "drawing.hpp"
#include "drawing_color_handlers.hpp"

namespace nie {

std::vector<std::string> GenerateNames(std::string const& prefix, int range) {
    std::vector<std::string> names(range, prefix + "_");
    std::size_t counter = 0;
    std::for_each(names.begin(), names.end(), [&counter](std::string& s) { s += std::to_string(counter++); });
    return names;
}

void AddCoordinateSystem(
        Eigen::Vector3d const& origin,
        std::vector<PoseBbox> const& bounds_list,
        pcl::visualization::PCLVisualizer* viewer) {
    Eigen::Vector3d average = {0.0, 0.0, 0.0};
    for (auto const& bounds : bounds_list) {
        // We don't assume the center of the bounding box is in the same convention as the "world". So we transform it
        // into the world and then place the marker accordingly.
        average += (bounds.origin() * bounds.bbox().Center().cast<double>()) - origin;
    }
    average /= static_cast<double>(bounds_list.size());
    pcl::PointXYZ center = MakePointXYZ(average);

    float scale = bounds_list.front().bbox().Range().array().minCoeff() * 0.5f;

    viewer->addCoordinateSystem(scale, center.x, center.y, center.z);
}

void AddBboxesToMap(
        std::vector<PoseBbox> const& bounds,
        std::unordered_set<std::size_t> const& selection,
        std::unordered_set<std::size_t> const& highlighted,
        CoordinateConverter converter,
        cv::Mat* out) {
    CHECK(!bounds.empty()) << "AddBboxesToMap(): No bounds.";

    cv::Scalar const grey{128, 128, 128};
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        if (selection.count(i) == 0 && highlighted.count(i) == 0) {
            cv::polylines(*out, converter.GlobalToImage(bounds[i]), true, grey);
        }
    }

    cv::Scalar const white{255, 255, 255};
    for (std::size_t i : selection) {
        if (highlighted.count(i) == 0) {
            cv::polylines(*out, converter.GlobalToImage(bounds[i]), true, white);
        }
    }

    cv::Scalar const yellow{0, 255, 255};
    for (std::size_t i : highlighted) {
        cv::polylines(*out, converter.GlobalToImage(bounds[i]), true, yellow);
    }
}

void DrawOrientedBounds(
        Eigen::Vector3d const& origin,
        std::vector<PoseBbox> const& bounds,
        pcl::visualization::PCLVisualizer* viewer,
        std::string const& name_prefix) {
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        Eigen::Vector3d centre_d = bounds[i].origin() * bounds[i].bbox().Center().cast<double>();
        Eigen::Vector3f centre = (centre_d - origin).cast<float>();
        Eigen::Quaternionf const& q(bounds[i].origin().rotation().cast<float>());
        Eigen::Vector3f range = bounds[i].bbox().Range();
        std::string const name = name_prefix + std::to_string(i);

        // The cube is drawn with its center as the origin and the box around it
        viewer->addCube(centre, q, range.x(), range.y(), range.z(), name);
        viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                name);
    }
}

}  // namespace nie
