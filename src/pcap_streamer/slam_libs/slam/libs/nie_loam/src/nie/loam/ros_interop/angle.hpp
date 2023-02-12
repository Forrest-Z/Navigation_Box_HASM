/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LOAM_ANGLE_H
#define LOAM_ANGLE_H

#include <cmath>

namespace loam {

/** \brief Class for holding an angle.
 *
 * This class provides buffered access to sine and cosine values to the
 * represented angular value.
 */
class Angle {
public:
    Angle() : _radian(0.0), _cos(1.0), _sin(0.0) {}

    Angle(double radValue) : _radian(radValue), _cos(std::cos(radValue)), _sin(std::sin(radValue)) {}

    Angle(const Angle& other) : _radian(other._radian), _cos(other._cos), _sin(other._sin) {}

    void operator=(const Angle& rhs) {
        _radian = (rhs._radian);
        _cos = (rhs._cos);
        _sin = (rhs._sin);
    }

    void operator+=(const double& radValue) { *this = (_radian + radValue); }

    void operator+=(const Angle& other) { *this = (_radian + other._radian); }

    void operator-=(const double& radValue) { *this = (_radian - radValue); }

    void operator-=(const Angle& other) { *this = (_radian - other._radian); }

    Angle operator-() const {
        Angle out;
        out._radian = -_radian;
        out._cos = _cos;
        out._sin = -(_sin);
        return out;
    }

    double rad() const { return _radian; }

    double deg() const { return _radian * 180 / M_PI; }

    double cos() const { return _cos; }

    double sin() const { return _sin; }

private:
    double _radian;  ///< angle value in radian
    double _cos;     ///< cosine of the angle
    double _sin;     ///< sine of the angle
};

}  // end namespace loam

#endif  // LOAM_ANGLE_H
