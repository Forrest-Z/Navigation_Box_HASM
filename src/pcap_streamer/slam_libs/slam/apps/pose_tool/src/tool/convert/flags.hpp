/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>

auto constexpr kCsv = "csv";
auto constexpr kExtendedMonoCalibration = "extended_mono_calibration";
auto constexpr kG2o = "g2o";
auto constexpr kG2oTue = "g2o_tue";
auto constexpr kKitti = "kitti";
auto constexpr kKml = "kml";
auto constexpr kPost = "post";
auto constexpr kKinematica = "kinematica";

void LogConvertingFrom(std::string const& format);
void LogConvertingTo(std::string const& format);

enum class ConversionFormatFrom { kExtendedMonoCalibration, kG2o, kG2oTue, kPost, kKinematica, kKitti };
enum class ConversionFormatTo { kCsv, kG2o, kKitti, kKml };

ConversionFormatFrom GetConversionFormatFrom();
ConversionFormatTo GetConversionFormatTo();
