/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "rectified_parameters.hpp"

namespace cv {

void write(cv::FileStorage& fs, std::string const&, nie::io::FrameData const& p) {
    fs << "{";
    fs << "id" << p.id;
    fs << "baseline" << p.baseline;
    fs << "}";
}

void read(
    cv::FileNode const& node, nie::io::FrameData& x, nie::io::FrameData const& default_value = nie::io::FrameData()) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["id"] >> x.id;
        node["baseline"] >> x.baseline;
    }
}

void write(cv::FileStorage& fs, std::string const&, nie::io::RectifiedCameraParameters const& p) {
    fs << "{";
    fs << "id" << p.id;
    fs << "image_size" << p.image_size;
    fs << "K" << p.K;
    fs << "frames" << p.frames;
    fs << "}";
}
void read(
    cv::FileNode const& node,
    nie::io::RectifiedCameraParameters& x,
    nie::io::RectifiedCameraParameters const& default_value = nie::io::RectifiedCameraParameters()) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["id"] >> x.id;
        node["image_size"] >> x.image_size;
        node["K"] >> x.K;
        node["frames"] >> x.frames;
    }
}
}  // namespace cv

namespace nie {

namespace io {

bool FrameData::operator==(FrameData const& other) const {
    return id == other.id && baseline.translation() == other.baseline.translation() &&
           baseline.rotation().matrix() == other.baseline.rotation().matrix();
}

bool RectifiedCameraParameters::operator==(RectifiedCameraParameters const& other) const {
    return id == other.id && image_size == other.image_size && K == other.K && frames.size() == other.frames.size() &&
           std::equal(frames.begin(), frames.end(), other.frames.begin());
}

RectifiedCameraParameters RectifiedCameraParameters::Read(std::string const& filename) {
    std::vector<RectifiedCameraParameters> x;
    nie::io::Read(filename, &x);
    return x[0];
}

void Read(std::string const& filename, std::vector<RectifiedCameraParameters>* ret) {
    cv::FileStorage storage(filename, cv::FileStorage::READ);
    CHECK(storage.isOpened()) << "Unable to read file: " + filename;

    storage["platforms"] >> *ret;

    CHECK(!ret->empty());
    for (auto const& p : *ret) {
        CHECK(!p.frames.empty()) << filename << ": A camera is expcted to have at least a single frame.";
    }
}

void RectifiedCameraParameters::Write(std::string const& filename) const {
    std::vector<RectifiedCameraParameters> x{};
    x.push_back(*this);
    nie::io::Write(filename, x);
}

void Write(std::string const& filename, std::vector<RectifiedCameraParameters> const& params) {
    cv::FileStorage storage(filename, cv::FileStorage::WRITE);
    CHECK(storage.isOpened()) << "Unable to write file: " + filename;

    storage << "platforms" << params;
}

}  // namespace io

}  // namespace nie
