/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "generate_edges.hpp"

#include <set>

#include <nie/cv/geometry/line.hpp>

namespace {

class EdgeCalculator {
public:
    explicit EdgeCalculator(nie::Isometry3qd T_corr) : T_corr_(std::move(T_corr)) {}

    // Calculate full transformation from local "end" to system "begin"
    void operator()(
            nie::io::PoseRecord const& pose_b,
            nie::io::PoseRecord const& pose_e,
            nie::Isometry3qd* T_b,
            nie::Isometry3qd* T_e_in_b,
            nie::Isometry3qd* T_edge = nullptr) const {
        *T_b = pose_b.isometry;
        *T_e_in_b = T_corr_ * pose_e.isometry;
        if (T_edge != nullptr) {
            *T_edge = T_b->Inversed() * *T_e_in_b;
        }
    }

private:
    nie::Isometry3qd const T_corr_;
};

using PointPair = std::pair<Eigen::Vector2d, Eigen::Vector2d>;

// Check if the given two line segments are intersecting each other.
// A line is defined by the starting and ending point.
bool AreLinesIntersecting(PointPair const& pts_a, PointPair const& pts_b) {
    // TODO(jbr): It's not the most beautiful, but for now it works.
    Eigen::Vector3d p0(pts_a.first.x(), pts_a.first.y(), 0.0);
    Eigen::Vector3d p1(pts_a.second.x(), pts_a.second.y(), 0.0);
    Eigen::Vector3d p2(pts_b.first.x(), pts_b.first.y(), 0.0);
    Eigen::Vector3d p3(pts_b.second.x(), pts_b.second.y(), 0.0);
    Eigen::Vector3d v0 = (p1 - p0).normalized();
    Eigen::Vector3d v1 = (p3 - p2).normalized();
    Eigen::Vector3d x;
    double s0, s1;

    return nie::LineLineIntersect(p0, v0, p2, v1, &x, &s0, &s1) && s0 >= 0.0 && s0 <= (p0 - p1).norm() && s1 >= 0.0 &&
           s1 <= (p2 - p3).norm();
}

void GetLineAndEdge(
        EdgeCalculator const& edge_calculator,
        nie::io::PoseRecord const& pose_b,
        nie::io::PoseRecord const& pose_e,
        PointPair* points,
        nie::Isometry3qd* T_edge = nullptr) {
    nie::Isometry3qd T_b, T_e_in_b;
    edge_calculator(pose_b, pose_e, &T_b, &T_e_in_b, T_edge);
    *points = {T_b.translation().head<2>(), T_e_in_b.translation().head<2>()};
}

// The order of forward / reverse iterator can be different. As such, 3 template arguments.
template <typename IteratorA, typename IteratorB, typename IteratorC>
void AddEdges(
        int const size,
        EdgeCalculator const& edge_calculator,
        nie::io::PoseEdgeRecord const& bbox_edge,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& trace_pose_map,
        IteratorA itf_b,
        IteratorB itf_e,
        IteratorC itb_e,
        std::vector<nie::io::PoseEdgeRecord>* trace_loops) {
    // All edges that will be created, are stored as edge records in the final pose collection using this function
    auto const create_edge_record =
            [&trace_loops, &bbox_edge](nie::io::PoseId id_b, nie::io::PoseId id_e, nie::Isometry3qd const& T_edge) {
                trace_loops->emplace_back(nie::io::PoseEdgeRecord{id_b, id_e, bbox_edge.category, T_edge, {}});
            };

    bool reverse = false;

    auto const get_pose_b = [&](std::size_t offset) -> nie::io::PoseRecord const& {
        return trace_pose_map.at(*(itf_b + offset)).get();
    };

    auto const get_pose_e = [&](std::size_t offset) -> nie::io::PoseRecord const& {
        if (!reverse) {
            return trace_pose_map.at(*(itf_e + offset)).get();
        } else {
            return trace_pose_map.at(*(itb_e + offset)).get();
        }
    };

    auto const advance_it_b = [&]() -> void { itf_b++; };

    auto const advance_it_e = [&]() -> void {
        if (!reverse) {
            itf_e++;
        } else {
            itb_e++;
        }
    };

    int i = 0, retries = 0;
    // Size minus 1, because we look ahead with 1 index.
    while (i < (size - 1)) {
        PointPair curr_line, next_line;
        nie::Isometry3qd T_edge;

        auto const& pose_b_c = get_pose_b(0);
        auto const& pose_e_c = get_pose_e(0);
        GetLineAndEdge(edge_calculator, pose_b_c, pose_e_c, &curr_line, &T_edge);

        auto const& pose_b_n = get_pose_b(1);
        auto const& pose_e_n = get_pose_e(1);
        GetLineAndEdge(edge_calculator, pose_b_n, pose_e_n, &next_line);

        // If the next and current line are crossing, then reverse and try again
        if (AreLinesIntersecting(curr_line, next_line)) {
            reverse = !reverse;
            if (retries++ >= 2) {
                DLOG(INFO) << "Canceling edge generation at Retry #" << retries << std::endl;
                // TODO(MMS-1549): Redesign the algorithm to support more complex edge crossing cases.
                return;  // nothing more to be done
            }
        } else {
            retries = 0;
            create_edge_record(pose_b_c.id, pose_e_c.id, T_edge);
            advance_it_b();
            advance_it_e();
            i++;
        }
    }

    // Last one, since we go to size - 1 to look one ahead, but we may also get here if the size is 1
    if (size > 0) {
        PointPair curr_line;
        nie::Isometry3qd T_edge;

        auto const& pose_b = get_pose_b(0);
        auto const& pose_e = get_pose_e(0);
        GetLineAndEdge(edge_calculator, pose_b, pose_e, &curr_line, &T_edge);
        create_edge_record(pose_b.id, pose_e.id, T_edge);
    }
}

}  // anonymous namespace

void GenerateEdgesForBboxes(
        nie::io::PoseEdgeRecord const& bbox_edge,
        std::vector<nie::io::PoseId> const& trace_b,
        std::vector<nie::io::PoseId> const& trace_e,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& aa_bbox_pose_map,
        std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord const>> const& trace_pose_map,
        std::vector<nie::io::PoseEdgeRecord>* trace_loops) {
    // If ICP did nothing, T_corr would be identity because bbox_edge would be identical to:
    //
    //      bbox_edge = aa_bbox_b.Inversed() * aa_bbox_e.
    //
    // So:
    //      I = aa_bbox_b * aa_bbox_b.Inversed() * aa_bbox_e * aa_bbox_e.Inversed()
    //
    // However, usually ICP does something:
    //
    //      bbox_edge = aa_bbox_b.Inversed() * T_corr * aa_bbox_e.
    //
    // Easiest interpretation is that it is a coordinate transformation from anything in world "end":
    //      All absolute coordinates tied to trace "end" or bounding box "end"
    // to anything in world "begin":
    //      All absolute coordinates tied to trace "begin" or bounding box "begin"
    auto const& aa_bbox_b = aa_bbox_pose_map.at(bbox_edge.id_begin).get().isometry;
    auto const& aa_bbox_e = aa_bbox_pose_map.at(bbox_edge.id_end).get().isometry;
    nie::Isometry3qd T_corr = aa_bbox_b * bbox_edge.isometry * aa_bbox_e.Inversed();
    T_corr.rotation().normalize();

    // Object that can be used to calculate the lidar odometry points and edges
    EdgeCalculator edge_calculator(T_corr);

    std::size_t size = std::min(trace_b.size(), trace_e.size());
    if (size == 0) {
        return;
    }

    // clang-format off
    // ******************************************************************************************************
    //
    // Both traces are split into two halves. Edges are connected between both traces starting from where
    // the split was made for both halves. This means that for the "positive" side we expect to move forward
    // and the "negative" side to move backward.
    //
    // It is also possible to simply do it in a single pass without splitting. However, this strategy results
    // in more "artifacts" around the outer edges of the traces. Also, it considered important to at least have
    // edges around the center of the traces, as those most closely resemble the output of ICP.
    //
    // Legacy single pass version:
    //
    //    std::size_t const offset_b = (trace_b.size() - size) / 2;
    //    std::size_t const offset_e = (trace_e.size() - size) / 2;
    //    auto fit_b = trace_b.begin() + offset_b;
    //    auto fit_e = trace_e.begin() + offset_e;
    //    auto rit_e = std::reverse_iterator(fit_e + size);
    //    AddEdges(size, edge_calculator, bbox_edge, trace_pose_map, fit_b, fit_e, rit_e, trace_loops);
    //
    // Double pass version:
    //
    // Below we first determine if the "first" two edges of both halves would intersect.
    // If the answer is no we assume both traces are pointing in the same direction:
    //
    //                                      negative      |       positive
    //                               >---------------rit_b|fit_b--------------->
    //                                                  | | |
    //                                                  | | |
    //                                                  | | |
    //                                                  | | |
    //                       n_fit_e >---------------rit_e|fit_e---------------> p_rit_e
    //                                      negative      |       positive
    //                               0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17
    //
    // If the answer is yes we assume we need to reverse one trace:
    //
    //
    //                                      negative      |       positive
    //                               >---------------rit_b|fit_b--------------->
    //                                                    |
    //                                                    |
    //                                                    |
    //                                                    |
    //                       p_fit_e <---------------rit_e|fit_e---------------< n_rit_e
    //                                      positive      |       negative
    //                               0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17
    //
    //
    //                                 fit / rit = forward and reverse iterator.
    //
    // If one trace needs reversing, it means that (for example) for the positive half we use a combined
    // forward and reverse iterator (for trace begin and end respectively) to move "forward". Otherwise
    // both would have used the forward iterator.
    //
    // NOTE:
    // 1) We treat each half independently. In theory the first edge may get "reversed" in it's respective
    //    half. But then this was a "problem" anyway.
    // 2) It is assumed that ordering from the middle makes sense. This is usually true as the driver is
    //    not likely to try and create bizarre traces/tracks.
    //
    // ******************************************************************************************************
    // clang-format on

    // The index pointing to the pose in the middle (or the one before it)
    std::size_t const mid_index_b = trace_b.size() / 2;
    std::size_t const mid_index_e = trace_e.size() / 2;
    std::size_t const size_p = size - size / 2;
    std::size_t const size_n = size - size_p;
    auto fit_b = trace_b.begin() + mid_index_b;
    // NOTE: For a reverse iterator r constructed from an iterator i,
    // the relationship &*r == &*(i-1) is always true (as long as r is dereferenceable);
    // See drawing above on which half it starts.
    auto rit_b = std::reverse_iterator(fit_b);
    auto fit_e = trace_e.begin() + mid_index_e;
    auto rit_e = std::reverse_iterator(fit_e);

    PointPair curr_line, next_line;
    nie::Isometry3qd T_edge;

    auto const& pose_b = trace_pose_map.at(*fit_b).get();
    auto const& pose_e = trace_pose_map.at(*fit_e).get();
    GetLineAndEdge(edge_calculator, pose_b, pose_e, &curr_line);

    auto const& pose_b_n = trace_pose_map.at(*rit_b).get();
    auto const& pose_e_n = trace_pose_map.at(*rit_e).get();
    GetLineAndEdge(edge_calculator, pose_b_n, pose_e_n, &next_line);

    if (AreLinesIntersecting(curr_line, next_line)) {
        // When we need to reverse the 2nd trace, we potentially have the wrong mid index for this trace.
        // The mid index is only the same for an odd amount of poses.
        bool even = (static_cast<int>(size) & int(1)) == 0;
        std::size_t const swapped_mid_index_e = (even) ? (mid_index_e - 1) : mid_index_e;
        fit_e = trace_e.begin() + swapped_mid_index_e + 1;
        rit_e = std::reverse_iterator(fit_e);

        auto p_fit_e = fit_e - size_p;
        auto n_rit_e = std::reverse_iterator(fit_e + size_n);

        AddEdges(size_p, edge_calculator, bbox_edge, trace_pose_map, fit_b, rit_e, p_fit_e, trace_loops);
        AddEdges(size_n, edge_calculator, bbox_edge, trace_pose_map, rit_b, fit_e, n_rit_e, trace_loops);
    } else {
        auto p_rit_e = std::reverse_iterator(fit_e + size_p);
        auto n_fit_e = fit_e - size_n;

        AddEdges(size_p, edge_calculator, bbox_edge, trace_pose_map, fit_b, fit_e, p_rit_e, trace_loops);
        AddEdges(size_n, edge_calculator, bbox_edge, trace_pose_map, rit_b, rit_e, n_fit_e, trace_loops);
    }
}
