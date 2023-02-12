/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <thread>

#include <pcl/visualization/pcl_visualizer.h>

namespace nie {

class PclViewer {
public:
    explicit PclViewer(std::string const& name) : viewer(name), update_(false), update_function_() {
        viewer.setBackgroundColor(0.2, 0.2, 0.2, 0);
    }

    void View() {
        viewer.resetStoppedFlag();
        while (!viewer.wasStopped()) {
            if (update_) {
                update_function_();
                update_ = false;
                LOG(INFO) << "Viewer is updated";
            }
            viewer.spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        viewer.close();
    }

    // The SetUpdate "hack" is a solution to avoid running into multi threading issues.
    // Suppose we update point clouds via one thread while the viewer tries to render them in another thread.
    void SetUpdate(std::function<void(void)> const& update_function) {
        update_ = true;
        update_function_ = update_function;
    }

    void Reset() {
        viewer.removeAllCoordinateSystems();
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
    }

    void SetVisibility(std::string const& name, bool enable) {
        viewer.setPointCloudRenderingProperties(
                pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, (enable ? 1.0 : 0.0), name);
    }
    void SetVisibility(std::vector<std::string> const& names, bool enable) {
        for (std::string const& name : names) {
            SetVisibility(name, enable);
        }
    }

    ~PclViewer() { viewer.close(); }

    pcl::visualization::PCLVisualizer viewer;

private:
    std::atomic_bool update_;
    std::function<void(void)> update_function_;
};

}  // namespace nie
