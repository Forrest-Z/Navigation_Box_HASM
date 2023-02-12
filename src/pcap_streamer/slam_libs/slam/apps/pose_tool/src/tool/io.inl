/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
template <typename T>
bool ReadData(std::vector<std::pair<std::string, T>>* collections) {
    std::vector<boost::filesystem::path> paths;

    if (GetAndCheckInPathsForExtension(nie::io::graph::Extension<T>(), &paths)) {
        for (auto&& path : paths) {
            std::string const& filename = path.string();
            collections->emplace_back(filename, nie::io::ReadCollection<T>(filename));
            LOG(INFO) << "Read " << nie::io::graph::Extension<T>() << " file '" << filename << "'.";
        }

        return true;
    }

    return false;
}

template <typename T>
bool ReadData(std::vector<T>* collections) {
    std::vector<boost::filesystem::path> paths;

    if (GetAndCheckInPathsForExtension(nie::io::graph::Extension<T>(), &paths)) {
        for (auto&& path : paths) {
            std::string const& filename = path.string();
            collections->push_back(nie::io::ReadCollection<T>(filename));
            LOG(INFO) << "Read " << nie::io::graph::Extension<T>() << " file '" << filename << "'.";
        }

        return true;
    }

    return false;
}

template <typename T>
bool ReadData(T* collection) {
    std::vector<std::pair<std::string, T>> collections;

    if (ReadData(&collections)) {
        *collection = std::move(collections[0].second);
        return true;
    }

    return false;
}

template <typename T>
void WriteData(T const& collection) {
    auto extension = nie::io::graph::Extension<T>();
    std::string const out_file = GetAndCheckOutPathsForExtensionOrFatal(extension).string();
    nie::io::Write(collection, out_file);
    LOG(INFO) << "Wrote " << extension << " out_file '" << out_file << "'.";
}

template <typename T>
void CheckAllHeadersAreIdentical(std::vector<T> const& collections) {
    bool all_look_same = std::all_of(collections.begin() + 1, collections.end(), [&r = collections[0]](auto const& c) {
        return nie::io::Equals(r.header, c.header);
    });

    CHECK(all_look_same) << "The flags across all collection headers should be identical.";
}