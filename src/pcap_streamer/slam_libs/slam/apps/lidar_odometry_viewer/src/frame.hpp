/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef FRAME_HPP
#define FRAME_HPP

#include <array>
#include <string>

#include <Eigen/Eigen>

// Forward declarations
class vtkRenderer;

void ShowFrame(vtkRenderer& renderer, std::string const& name, Eigen::Isometry3d const& transform);

void ShowAxes(vtkRenderer& renderer, Eigen::Isometry3d const& frame, double scale = 1.0);

void ShowBillboardText(
    vtkRenderer& renderer,
    std::string const& name,
    Eigen::Vector3d const& position,
    double scale = 1.0,
    std::array<double, 3> color = {1, 1, 1});

#endif  // FRAME_HPP
