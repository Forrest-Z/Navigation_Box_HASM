/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Eigen>

namespace nie {

double CalcTranslationNorm(nie::Isometry3qd const& T) { return T.translation().norm(); }

double CalcRotationAngle(nie::Isometry3qd const& T) { return Eigen::AngleAxisd(T.rotation()).angle(); }

}  // namespace nie
