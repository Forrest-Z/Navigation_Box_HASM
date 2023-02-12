/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>

#include <boost/filesystem.hpp>

namespace nie {

class Filenamer {
public:
    explicit Filenamer(boost::filesystem::path root_dir, std::string base_filename, std::string extension);

    std::string const& FullBasename() const;

    boost::filesystem::path Peek() const;

    void Next();

private:
    boost::filesystem::path FormattedName() const;

    boost::filesystem::path const root_dir_;
    std::string const base_filename_;
    std::string const extension_;
    std::string const full_basename_;

    size_t counter_;
    boost::filesystem::path formatted_name_;
};

}  // namespace nie
