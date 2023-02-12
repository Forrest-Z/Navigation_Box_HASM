/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_DEVICE_PID_CONTROLLER_HPP
#define NIE_HARDWARE_DEVICE_PID_CONTROLLER_HPP

#include <queue>

namespace nie {

// Proportional Integral and Derivative controller.
//
// Controller for simple or unknown processes. In case of controlling a well known process a custom controller
// may be better.
//
// https://en.wikipedia.org/wiki/PID_controller
//
// https://jagger.berkeley.edu/~pack/me132/Section15.pdf
// https://en.wikipedia.org/wiki/Ziegler-Nichols_method
//
// TODO(jbr) Change i_min and i_max to be output_min, output_max and using i_deltas to bound error_i_
class PidController {
public:
    PidController(double gain_p, double gain_i, double gain_d, double i_min, double i_max);

    double ComputeDelta(double e, double delta_time);
    void Reset();

    double gain_p() const { return gain_p_; }
    double gain_i() const { return gain_i_; }
    double gain_d() const { return gain_d_; }

    double error_p() const { return error_p_; }
    double error_i() const { return error_i_; }
    double error_d() const { return error_d_; }

private:
    // Gains
    double gain_p_;
    double gain_i_;
    double gain_d_;

    // Bounds for the integral part of the error: windup guard.
    // This is typically most needed for startup conditions and situations for switching in and out of control.
    double i_min_;
    double i_max_;

    // Errors
    double error_p_;
    double error_i_;
    double error_d_;
};

}  // namespace nie

#endif  // NIE_HARDWARE_DEVICE_PID_CONTROLLER_HPP
