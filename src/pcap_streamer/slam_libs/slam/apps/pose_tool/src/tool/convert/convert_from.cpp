/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_from.hpp"

#include "convert_from_extended_mono_calibration.hpp"
#include "convert_from_g2o.hpp"
#include "convert_from_g2o_tue.hpp"
#include "convert_from_kinematica.hpp"
#include "convert_from_kitti.hpp"
#include "convert_from_post.hpp"
#include "flags.hpp"

void ConvertFrom() {
    switch (GetConversionFormatFrom()) {
        case ConversionFormatFrom::kExtendedMonoCalibration:
            LogConvertingFrom(kExtendedMonoCalibration);
            ConvertFromExtendedMonoCalibration();
            break;
        case ConversionFormatFrom::kG2o:
            LogConvertingFrom(kG2o);
            ConvertFromG2o();
            break;
        case ConversionFormatFrom::kG2oTue:
            LogConvertingFrom(kG2oTue);
            ConvertFromG2oTue();
            break;
        case ConversionFormatFrom::kPost:
            LogConvertingFrom(kPost);
            ConvertFromPost();
            break;
        case ConversionFormatFrom::kKinematica:
            LogConvertingFrom(kKinematica);
            ConvertFromKinematica();
            break;
        case ConversionFormatFrom::kKitti:
            LogConvertingFrom(kKitti);
            ConvertFromKitti();
            break;
    }
}
