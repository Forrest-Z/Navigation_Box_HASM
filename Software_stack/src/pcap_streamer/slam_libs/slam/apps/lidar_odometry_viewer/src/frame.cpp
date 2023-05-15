/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "frame.hpp"

#include <vtkAxesActor.h>
//#include <vtkBillboardTextActor3D.h>
#include <vtkFollower.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkVectorText.h>

void ShowFrame(vtkRenderer& renderer, std::string const& name, Eigen::Isometry3d const& transform) {
    double const scale = 0.3;
    ShowAxes(renderer, transform, scale);
    ShowBillboardText(renderer, name, transform.translation(), scale / 4);
}

void ShowAxes(vtkRenderer& renderer, Eigen::Isometry3d const& frame, double scale) {
    // Obtain translation / rotation part of the frame
    Eigen::Vector3d t(frame.translation());
    Eigen::AngleAxisd R(frame.linear());

    vtkNew<vtkTransform> transform;
    transform->Translate(t.data());
    transform->RotateWXYZ(vtkMath::DegreesFromRadians(R.angle()), R.axis().data());
    transform->Scale(scale, scale, scale);

    vtkNew<vtkAxesActor> axes;
    axes->AxisLabelsOff();
    axes->SetShaftTypeToCylinder();
    axes->SetCylinderResolution(6);
    axes->SetConeResolution(6);
    axes->SetCylinderRadius(0.05);
    axes->SetConeRadius(0.4);
    axes->SetUserTransform(transform.Get());

    renderer.AddActor(axes.Get());
}

void ShowBillboardText(
        vtkRenderer& renderer,
        std::string const& text,
        Eigen::Vector3d const& position,
        double scale,
        std::array<double, 3> color) {
#if 1
    vtkNew<vtkVectorText> text_source;
    text_source->SetText(text.c_str());
    text_source->Update();

    vtkNew<vtkPolyDataMapper> text_mapper;
    text_mapper->SetInputConnection(text_source->GetOutputPort());

    vtkNew<vtkFollower> text_actor;
    text_actor->SetMapper(text_mapper.Get());
    text_actor->SetPosition(const_cast<Eigen::Vector3d&>(position).data());
    text_actor->SetScale(scale);
    text_actor->GetProperty()->SetColor(color.data());
    text_actor->SetCamera(renderer.GetActiveCamera());
#else
    // Aternative implementation based on vtkBillboardTextActor3D; All text has a fixed size with respect to the screen
    vtkNew<vtkBillboardTextActor3D> text_actor;
    text_actor->SetInput(text.c_str());
    text_actor->SetPosition(const_cast<Eigen::Vector3d&>(position).data());
    text_actor->GetTextProperty()->SetFontSize(24);
    text_actor->GetTextProperty()->SetColor(color.data());
    text_actor->GetTextProperty()->SetJustificationToCentered();
    text_actor->GetTextProperty()->SetVerticalJustificationToTop();
#endif

    renderer.AddActor(text_actor.Get());
}
