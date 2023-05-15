/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>

namespace nie {

namespace kinematica {

/// Convert a Kinematica csv file to pose file
/// \param in_file input path of the Kinematica file
/// \param out_file output path of the pose file
void ConvertKinematicaToPoseFile(std::string const& in_file, std::string const& out_file);

}  // namespace kinematica

}  // namespace nie
