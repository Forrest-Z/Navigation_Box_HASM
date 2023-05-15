/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "pid_controller.hpp"

#include <cassert>

namespace nie {

PidController::PidController(double gain_p, double gain_i, double gain_d, double i_min, double i_max)
    : gain_p_(gain_p),
      gain_i_(gain_i),
      gain_d_(gain_d),
      i_min_(i_min),
      i_max_(i_max),
      error_p_(0.0),
      error_i_(0.0),
      error_d_(0.0) {}

double PidController::ComputeDelta(double e, double delta_time) {
    assert(delta_time != 0.0);

    error_d_ = e - error_p_;
    error_i_ += e * delta_time;
    error_p_ = e;

    // Windup guard
    // TODO: [EDD] Replace with std:clamp() when we moved to C++17
    // error_i_ = std::clamp(error_i_, i_min_, i_max_);
    if (error_i_ > i_max_)
        error_i_ = i_max_;
    else if (error_i_ < i_min_)
        error_i_ = i_min_;

    return gain_p_ * error_p_ + gain_i_ * error_i_ + gain_d_ * error_d_;
}

void PidController::Reset() {
    error_p_ = 0.0;
    error_i_ = 0.0;
    error_d_ = 0.0;
}

}  // namespace nie