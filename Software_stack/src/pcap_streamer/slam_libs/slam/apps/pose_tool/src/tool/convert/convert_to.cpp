/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_to.hpp"

#include "convert_to_csv.hpp"
#include "convert_to_g2o.hpp"
#include "convert_to_kitti.hpp"
#include "convert_to_kml.hpp"
#include "flags.hpp"

void ConvertTo() {
    switch (GetConversionFormatTo()) {
        case ConversionFormatTo::kCsv:
            LogConvertingTo(kCsv);
            ConvertToCsv();
            break;
        case ConversionFormatTo::kG2o:
            LogConvertingTo(kG2o);
            ConvertToG2o();
            break;
        case ConversionFormatTo::kKitti:
            LogConvertingTo(kKitti);
            ConvertToKitti();
            break;
        case ConversionFormatTo::kKml:
            LogConvertingTo(kKml);
            ConvertToKml();
            break;
    }
}
