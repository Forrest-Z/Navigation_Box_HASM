/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "flags.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(
        conversion_format,
        "",
        "Supported 'from' or 'to' formats. From: extended_mono_calibration, g2o, post, kinematica. To: csv, g2o, kml, "
        "kitti.");

std::string ConversionFormatEmpty() { return "-conversion_format empty."; }

std::string ConversionFormatNotSupported(std::string const& format_flag) {
    return "-conversion_format (" + format_flag + ") not supported.";
}

void LogConvertingFrom(std::string const& format) { LOG(INFO) << "Converting from: " + format; }

void LogConvertingTo(std::string const& format) { LOG(INFO) << "Converting to: " + format; }

ConversionFormatFrom GetConversionFormatFrom() {
    if (FLAGS_conversion_format.empty()) {
        LOG(FATAL) << ConversionFormatEmpty();
    }

    if (FLAGS_conversion_format == kExtendedMonoCalibration) {
        return ConversionFormatFrom::kExtendedMonoCalibration;
    } else if (FLAGS_conversion_format == kG2o) {
        return ConversionFormatFrom::kG2o;
    } else if (FLAGS_conversion_format == kG2oTue) {
        return ConversionFormatFrom::kG2oTue;
    } else if (FLAGS_conversion_format == kPost) {
        return ConversionFormatFrom::kPost;
    } else if (FLAGS_conversion_format == kKinematica) {
        return ConversionFormatFrom::kKinematica;
    } else if (FLAGS_conversion_format == kKitti) {
        return ConversionFormatFrom::kKitti;
    }

    LOG(FATAL) << ConversionFormatNotSupported(FLAGS_conversion_format);
}

ConversionFormatTo GetConversionFormatTo() {
    if (FLAGS_conversion_format.empty()) {
        LOG(FATAL) << ConversionFormatEmpty();
    }

    if (FLAGS_conversion_format == kCsv) {
        return ConversionFormatTo::kCsv;
    } else if (FLAGS_conversion_format == kG2o) {
        return ConversionFormatTo::kG2o;
    } else if (FLAGS_conversion_format == kKml) {
        return ConversionFormatTo::kKml;
    } else if (FLAGS_conversion_format == kKitti) {
        return ConversionFormatTo::kKitti;
    }

    LOG(FATAL) << ConversionFormatNotSupported(FLAGS_conversion_format);
}
