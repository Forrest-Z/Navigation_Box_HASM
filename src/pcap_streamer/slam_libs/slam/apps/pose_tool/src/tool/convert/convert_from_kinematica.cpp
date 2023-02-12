/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_from_kinematica.hpp"

#include "kinematica/write.hpp"
#include "tool/io.hpp"

void ConvertFromKinematica() {
    nie::kinematica::ConvertKinematicaToPoseFile(FLAGS_in_paths, FLAGS_out_paths);
}
