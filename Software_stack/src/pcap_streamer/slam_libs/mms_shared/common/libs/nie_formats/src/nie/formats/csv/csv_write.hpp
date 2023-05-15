/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <fstream>
#include <string>
#include <vector>

namespace nie {

static void WriteLines(std::string const& out_filepath, std::vector<std::string> const& lines_vector) {
    std::ofstream out_file{out_filepath};

    for (auto const& line : lines_vector) {
        out_file << line << "\n";
    }
}

static void WriteLines(
        std::string const& out_filepath,
        std::vector<std::string>::const_iterator const& begin,
        std::vector<std::string>::const_iterator const& end) {
    std::ofstream out_file{out_filepath};

    for (auto it = begin; it != end; ++it) {
        out_file << *it << "\n";
    }
}

}  // namespace nie
