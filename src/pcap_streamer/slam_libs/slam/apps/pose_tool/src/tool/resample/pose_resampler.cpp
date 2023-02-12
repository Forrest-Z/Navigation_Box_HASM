#include "pose_resampler.hpp"

#include <glog/logging.h>

#include <nie/core/geometry/interpolation.hpp>

nie::io::PoseRecord InterpolatePoseRecord(
        nie::io::PoseHeader const& header,
        nie::io::PoseRecord const& p0,
        nie::io::PoseRecord const& p1,
        nie::io::PoseId const id,
        double ratio) {
    // Only makes sense to interpolate when these are equal.
    DCHECK(p0.category == p1.category);
    DCHECK(p0.device_id == p1.device_id);

    auto isometry = nie::Interpolate(p0.isometry, p1.isometry, ratio);
    auto t0 = p0.timestamp;
    auto t1 = p1.timestamp;
    nie::Timestamp_ns timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>((t1 - t0) * ratio) + t0;
    Eigen::Matrix<double, 6, 6> information = (header.HasPoseInformationPerRecord())
                                                      ? nie::Interpolate(p0.information, p1.information, ratio)
                                                      : Eigen::Matrix<double, 6, 6>::Zero();

    return {id, p0.category, p0.device_id, timestamp, isometry, information};
}

nie::io::PoseCollection ResampleDistance(
        nie::io::PoseCollection const& c, double const interval, double const max_delta) {
    CHECK(c.edges.empty()) << "Edge resampling not supported.";

    nie::io::PoseCollection resampled;
    resampled.header = c.header;

    if (c.poses.empty()) {
        return resampled;
    }

    nie::io::PoseId id = 0;

    resampled.poses.push_back(c.poses.front());
    resampled.poses[0].id = id;
    ++id;

    double acc = 0.0;

    for (auto it = c.poses.cbegin() + 1; it != c.poses.cend(); ++it) {
        auto const& p0 = *(it - 1);
        auto const& p1 = *it;
        double delta = (p1.isometry.translation() - p0.isometry.translation()).norm();

        if (delta == 0.0) {
            continue;
        }

        if (delta > max_delta) {
            acc = 0.0;
            resampled.poses.emplace_back(p0);
            resampled.poses.back().id = id;
            id++;
            continue;
        }

        acc += delta;

        while (acc > interval) {
            double error = acc - interval;
            // If the error equals zero, we ended exactly on i1.
            double ratio = (delta - error) / delta;

            resampled.poses.emplace_back(InterpolatePoseRecord(c.header, p0, p1, id, ratio));
            acc -= interval;
            ++id;
        }
    }

    return resampled;
}
