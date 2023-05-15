/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

namespace nie {

// Sometimes useful for trying out filters in case indices are available and points
// shouldn't be removed.
class CorrespondenceRejectorBool final : public pcl::registration::CorrespondenceRejector {
public:
    using Ptr = boost::shared_ptr<CorrespondenceRejectorBool>;

    CorrespondenceRejectorBool() { rejection_name_ = "CorrespondenceRejectorBool"; };

    inline void setSourceRejections(std::vector<bool> source_rejections) {
        source_rejections_ = std::move(source_rejections);
    }

    inline void setTargetRejections(std::vector<bool> target_rejections) {
        target_rejections_ = std::move(target_rejections);
    }

    inline void getRemainingCorrespondences(
        const pcl::Correspondences& original_correspondences,
        pcl::Correspondences& remaining_correspondences) override {
        remaining_correspondences.clear();

        for (auto const& oc : original_correspondences) {
            if (!source_rejections_[oc.index_query] && !target_rejections_[oc.index_match]) {
                remaining_correspondences.push_back(oc);
            }
        }

        PCL_DEBUG(("[nie::CorrespondenceRejectorBool::getRemainingCorrespondences] Remaining " +
                   std::to_string(remaining_correspondences.size()) + " from " +
                   std::to_string(original_correspondences.size()) + "\n")
                      .c_str());
    }

protected:
    void applyRejection(pcl::Correspondences& correspondences) override {
        getRemainingCorrespondences(*input_correspondences_, correspondences);
    }

private:
    std::vector<bool> source_rejections_;
    std::vector<bool> target_rejections_;
};

}  // namespace nie
