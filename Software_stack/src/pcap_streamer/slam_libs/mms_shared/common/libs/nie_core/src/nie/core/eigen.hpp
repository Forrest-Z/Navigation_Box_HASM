/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Core>

namespace nie {

template <typename DerivedA, typename DerivedB>
static auto EigenModulo(Eigen::ArrayBase<DerivedA> const& a, Eigen::ArrayBase<DerivedB> const& b) {
    static_assert(std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar>);
    using S = typename DerivedA::Scalar;
    return a.binaryExpr(b, [](S const& a_, S const& b_) { return std::fmod(a_, b_); });
}

}  // namespace nie
