/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_SUBDIV_2D_HPP
#define NIE_CV_SUBDIV_2D_HPP

#include <opencv2/opencv.hpp>

namespace nie {

/*
 * Class that extents the opencv class Subdiv2D
 */
class Subdiv2D : public cv::Subdiv2D {
public:
    // Mimic interface of original class
    Subdiv2D() : cv::Subdiv2D() {}
    explicit Subdiv2D(cv::Rect rect) : cv::Subdiv2D(rect) {}

    /*
     * Function that returns all edges like
     *     void getEdgeList (std::vector< Vec4f >& edgeList) const
     * except that the vertex id's are returned instead of the vertex geometries.
     */
    void getConnectionList(std::vector<std::pair<int, int>>* p_connections) const;
};

}  // namespace nie

#endif  // NIE_CV_SUBDIV_2D_HPP
