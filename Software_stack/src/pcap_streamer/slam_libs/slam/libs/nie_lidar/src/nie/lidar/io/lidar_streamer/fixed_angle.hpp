/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <cassert>

namespace nie {

namespace io {

namespace velodyne {

/// Rotation angle representable as fixed number of steps per degree
template <typename T = std::uint16_t, T steps_per_degree = 100>
class FixedAngle {
public:
    // Make the amount of steps per degree accessible from outside
    constexpr static T kStepsPerDegree = steps_per_degree;

    // Define total number of steps required for a full rotation
    constexpr static T kTotalSteps = 360 * steps_per_degree;

    // Ensure that all steps are representable in the given type
    static_assert(kTotalSteps < std::numeric_limits<T>::max());

    FixedAngle() : steps_(kInvalidStep) {}
    explicit FixedAngle(T const steps) : steps_(steps) {}

    [[nodiscard]] bool Valid() const { return steps_ != kInvalidStep; }

    [[nodiscard]] T Steps() const { return steps_; }

    [[nodiscard]] double Degrees() const {
        assert(Valid());
        return static_cast<double>(steps_) / steps_per_degree;
    }

    [[nodiscard]] bool operator<(FixedAngle const other) const { return Valid() && steps_ < other.steps_; }

    [[nodiscard]] FixedAngle operator+(FixedAngle const other) const {
        assert(Valid() && other.Valid());
        return FixedAngle(static_cast<T>((steps_ + other.steps_) % kTotalSteps));
    }

    [[nodiscard]] FixedAngle operator-(FixedAngle const other) const {
        assert(Valid() && other.Valid());
        return FixedAngle(static_cast<T>((steps_ + (kTotalSteps - other.steps_)) % kTotalSteps));
    }

private:
    static_assert(std::is_integral<T>::value, "Integral required in FixedAngle");

    constexpr static T kInvalidStep = std::numeric_limits<T>::max();

    T steps_;
};

}  // namespace velodyne

}  // namespace io

}  // namespace nie
