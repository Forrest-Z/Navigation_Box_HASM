/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/formats/g2o_reader.hpp>

#include "g2o_tue.hpp"

namespace g2o_tue {

// Initialize the static maps that relate the g2o tag to the corresponding reader.
template <>
std::unordered_map<std::string, Entry<nie::io::PoseCollection>*> Entry<nie::io::PoseCollection>::entries_{
    {"VERTEX_CAM", new VertexCam}, {"EDGE_CAM", new EdgeCam}};
template <>
std::unordered_map<std::string, Entry<nie::io::ObjectCollection>*> Entry<nie::io::ObjectCollection>::entries_{
    {"VERTEX_XYZ", new VertexXyz}};
template <>
std::unordered_map<std::string, Entry<nie::io::KeypointCollection>*> Entry<nie::io::KeypointCollection>::entries_{
    {"EDGE_PROJECT_P2SC", new EdgeProjectP2SC}};

// All specific read functions
void VertexCam::Read(std::istream& input_stream, nie::io::PoseCollection* collection) const {
    collection->poses.emplace_back();
    auto& p = collection->poses.back();
    auto& t = p.isometry.translation();
    auto& r = p.isometry.rotation();
    input_stream >> p.id >> t.x() >> t.y() >> t.z() >> r.x() >> r.y() >> r.z() >> r.w();
    p.isometry = nie::ConvertFrame<nie::Frame::kCv, nie::Frame::kAircraft>(p.isometry);
}

void EdgeCam::Read(std::istream& input_stream, nie::io::PoseCollection* collection) const {
    collection->edges.emplace_back();
    auto& e = collection->edges.back();
    auto& t = e.isometry.translation();
    auto& r = e.isometry.rotation();
    input_stream >> e.id_begin >> e.id_end >> t.x() >> t.y() >> t.z() >> r.x() >> r.y() >> r.z() >> r.w();
    e.isometry = nie::ConvertFrame<nie::Frame::kCv, nie::Frame::kAircraft>(e.isometry);
    nie::io::ReadInformationMatrix(input_stream, &e.information);
    collection->header.flags |= nie::io::PoseCollection::Header::Flag::kHasEdgeInformationPerRecord;
}

void VertexXyz::Read(std::istream& input_stream, nie::io::ObjectCollection* collection) const {
    collection->objects.emplace_back();
    auto& p = collection->objects.back().position;
    input_stream >> collection->objects.back().id >> p.x() >> p.y() >> p.z();
    p = nie::detail::ConvertFrameVector<nie::Frame::kCv, nie::Frame::kAircraft>(p);
}

void EdgeProjectP2SC::Read(std::istream& input_stream, nie::io::KeypointCollection* collection) const {
    nie::io::KeypointRecord l{};
    l.frame_id = 0;
    input_stream >> l.object_id >> l.pose_id >> l.position.x() >> l.position.y();

    nie::io::KeypointRecord r{l};
    input_stream >> r.position.x();
    r.frame_id = 1;
    r.position.y() = l.position.y();

    collection->keypoints.push_back(std::move(l));
    collection->keypoints.push_back(std::move(r));
}

// The generic Tue g2o file read function
// When creating a generic g2o reader, then this could be reused, but after refactoring (not just trying to read to
// different containers)
void ReadG2oTue(
    std::string const& file_name,
    nie::io::PoseCollection* pose,
    nie::io::ObjectCollection* objt,
    nie::io::KeypointCollection* kpnt) {
    std::fstream input_stream = nie::OpenFile(file_name, std::ios::in);

    std::string line;
    while (std::getline(input_stream, line)) {
        std::stringstream ss(line);
        std::string tag;
        ss >> tag;
        Entry<nie::io::PoseCollection>::TryRead(tag, ss, pose);
        if (objt != nullptr) {
            Entry<nie::io::ObjectCollection>::TryRead(tag, ss, objt);
        }
        if (kpnt != nullptr) {
            Entry<nie::io::KeypointCollection>::TryRead(tag, ss, kpnt);
        }
    }
}

}  // namespace g2o_tue
