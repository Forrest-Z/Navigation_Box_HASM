/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/drawing/drawing.hpp>
#include <nie/drawing/drawing_color_handlers.hpp>

template <typename PointT>
class Viewer {
    // A lot of code is commented out, as it is will be reused in the future when there is a cloud set to compare with.
    using CloudVector = std::vector<nie::Cloud<PointT>>;

    std::string const kPrefixTransformedCloud = "transformed";

public:
    explicit Viewer(std::string const& name)
        : pcl_viewer_(name),
          // clouds_untransformed_(),
          clouds_transformed_(),
          // clouds_untransformed_names_(),
          clouds_transformed_names_(),
          // visualize_untransformed_clouds_(false),
          visualize_transformed_clouds_(true) {
        // AddKeyboardListenerPairStep();
    }

    // bool& visualize_untransformed_clouds() { return visualize_untransformed_clouds_; }
    bool& visualize_transformed_clouds() { return visualize_transformed_clouds_; }

    pcl::visualization::PCLVisualizer& viewer() { return pcl_viewer_.viewer; }

    void View() { pcl_viewer_.View(); }

    void SetClouds(/*CloudVector&& clouds_untransformed, */ CloudVector&& clouds_transformed) {
        // clouds_untransformed_ = std::move(clouds_untransformed);
        clouds_transformed_ = std::move(clouds_transformed);

        // clouds_untransformed_names_= nie::GenerateNames("", clouds_untransformed_.size());
        clouds_transformed_names_ = nie::GenerateNames(kPrefixTransformedCloud, clouds_transformed_.size());

        pcl_viewer_.SetUpdate([this]() { UpdateClouds(); });
    }

    void SetVisibility() {
        // pcl_viewer_.SetVisibility(clouds_untransformed_names_, visualize_unregistered_cloud_);
        pcl_viewer_.SetVisibility(clouds_transformed_names_, visualize_transformed_clouds_);
    }

private:
    void UpdateClouds() {
        pcl_viewer_.Reset();

        // nie::AddClouds<PointT>(
        //    clouds_untransformed_,
        //    nie::GetColorHandlerUnRegisteredCloud<PointT>(),
        //    clouds_untransformed_names_,
        //    &pcl_viewer_);
        if (!clouds_transformed_.empty()) {
            nie::AddClouds<PointT>(
                    clouds_transformed_,
                    nie::GetColorHandlerRegisteredCloud<PointT>(),
                    clouds_transformed_names_,
                    &pcl_viewer_);

            nie::AddCoordinateSystem(clouds_transformed_, &pcl_viewer_);
        }

        SetVisibility();
    }

    // TODO(jbr): Prevent trigger-happy users from pressing keys while loading.
    void AddKeyboardListenerPairStep() {
        static constexpr auto kKeySymBracketLeft = "bracketleft";
        static constexpr auto kKeySymBracketRight = "bracketright";

        std::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_event =
                [this](const pcl::visualization::KeyboardEvent& event) -> void {
            // Don't want to do it twice (on up as well).
            if (event.keyDown()) {
                if (event.getKeySym() == kKeySymBracketRight) {
                    // visualize_untransformed_clouds() = !visualize_untransformed_clouds();
                    // Update();
                } else if (event.getKeySym() == kKeySymBracketLeft) {
                    visualize_transformed_clouds() = !visualize_transformed_clouds();
                    SetVisibility();
                }
            }
        };
        viewer().registerKeyboardCallback(keyboard_event);
    }

    nie::PclViewer pcl_viewer_;

    // CloudVector clouds_untransformed_;
    CloudVector clouds_transformed_;

    // std::vector<std::string> clouds_untransformed_names_;
    std::vector<std::string> clouds_transformed_names_;

    // bool visualize_untransformed_clouds_;
    bool visualize_transformed_clouds_;
};
