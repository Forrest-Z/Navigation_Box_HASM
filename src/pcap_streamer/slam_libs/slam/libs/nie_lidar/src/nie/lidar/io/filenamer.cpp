/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "filenamer.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

namespace nie {

Filenamer::Filenamer(boost::filesystem::path root_dir, std::string base_filename, std::string extension)
    : root_dir_{std::move(root_dir)},
      base_filename_{std::move(base_filename)},
      extension_{std::move(extension)},
      full_basename_{(root_dir_ / base_filename_).string()},
      counter_{0},
      formatted_name_{FormattedName()} {}

std::string const& Filenamer::FullBasename() const { return full_basename_; }

boost::filesystem::path Filenamer::Peek() const { return formatted_name_; }

void Filenamer::Next() {
    ++counter_;
    formatted_name_ = FormattedName();
}

boost::filesystem::path Filenamer::FormattedName() const {
    std::stringstream ss;
    ss << base_filename_ << "_" << std::setw(6) << std::setfill('0') << counter_ << extension_;
    return root_dir_ / boost::filesystem::path{ss.str()};
}

}  // namespace nie
