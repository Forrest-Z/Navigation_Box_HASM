/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef CIRCLE_GRID_HPP
#define CIRCLE_GRID_HPP

#include <array>

#include <vtkSmartPointer.h>

// Forward declarations
class vtkActor;

// NOTE: Returning vtkNew<> would require C++11 move semantics, but this is not yet available in VTK
vtkSmartPointer<vtkActor> CreateCircleGrid(
    unsigned num_circles,
    double scale,
    std::array<double, 3> origin,
    std::array<double, 3> normal,
    std::array<double, 3> color);

#endif  // CIRCLE_GRID_HPP
