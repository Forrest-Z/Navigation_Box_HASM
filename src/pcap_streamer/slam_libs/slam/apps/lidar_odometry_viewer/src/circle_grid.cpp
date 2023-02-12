/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "circle_grid.hpp"

#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkArcSource.h>
#include <vtkExtractEdges.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

vtkSmartPointer<vtkActor> CreateCircleGrid(
    unsigned num_circles,
    double scale,
    std::array<double, 3> origin,
    std::array<double, 3> normal,
    std::array<double, 3> color) {
    double const size = num_circles * scale;

    vtkNew<vtkPlaneSource> plane;
    plane->SetOrigin(-size, -size, 0.0);
    plane->SetPoint1(+size, -size, 0.0);
    plane->SetPoint2(-size, +size, 0.0);
    plane->SetCenter(origin.data());
    plane->SetNormal(normal.data());
    plane->SetResolution(num_circles * 2, num_circles * 2);

    vtkNew<vtkExtractEdges> edges;
    edges->SetInputConnection(plane->GetOutputPort());

    vtkNew<vtkAppendPolyData> append;
    append->AddInputConnection(edges->GetOutputPort());

    double arc_start[3];
    vtkMath::Perpendiculars(normal.data(), arc_start, nullptr, 0);

    for (unsigned i = 1; i <= num_circles; ++i) {
        double startPoint[3] = {arc_start[0] * i * scale, arc_start[1] * i * scale, arc_start[2] * i * scale};

        vtkNew<vtkArcSource> arc;
        arc->UseNormalAndAngleOn();
        arc->SetAngle(360);
        arc->SetResolution(360);
        arc->SetPolarVector(startPoint);
        arc->SetCenter(origin.data());
        arc->SetNormal(normal.data());

        append->AddInputConnection(arc->GetOutputPort());
    }
    append->Update();

    vtkNew<vtkPolyDataMapper> grid_mapper;
    grid_mapper->SetInputConnection(append->GetOutputPort());

    auto grid_actor = vtkSmartPointer<vtkActor>::New();
    grid_actor->SetMapper(grid_mapper.GetPointer());
    grid_actor->GetProperty()->SetColor(color.data());

    return grid_actor;
}
