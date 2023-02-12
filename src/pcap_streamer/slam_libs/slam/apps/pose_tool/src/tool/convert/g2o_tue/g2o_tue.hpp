/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>
#include <unordered_map>

#include <nie/formats/ba_graph.hpp>

// A (partial) prototype pattern is applied to structure the reading of different tags in a g2o file and storing it in
// different containers. The main references used:
//   http://www.cs.sjsu.edu/faculty/pearce/modules/lectures/oop/types/reflection/prototype.htm
//   https://www.bogotobogo.com/DesignPatterns/prototype.php
//   https://github.com/RainerKuemmerle/g2o/blob/8564e1e365d4e719dceca6269a0ba203f7f43dec/g2o/types/sba/types_sba.h

namespace g2o_tue {

template <typename Storage>
class Entry {
public:
    // Prototype pattern interface
    virtual ~Entry() = default;

    static Entry* AddEntry(std::string const& tag, Entry* e) {
        entries_[tag] = e;
        return e;
    }

    // Specific interface
    static void TryRead(std::string const& tag, std::istream& input_stream, Storage* storage) {
        auto iter = entries_.find(tag);
        if (iter != entries_.cend()) {
            iter->second->Read(input_stream, storage);
        }
    }

private:
    virtual void Read(std::istream& input_stream, Storage* storage) const = 0;

    static std::unordered_map<std::string, Entry*> entries_;
};

class VertexCam : public Entry<nie::io::PoseCollection> {
public:
    ~VertexCam() override = default;

private:
    void Read(std::istream& input_stream, nie::io::PoseCollection* collection) const override;
};

class EdgeCam : public Entry<nie::io::PoseCollection> {
public:
    ~EdgeCam() override = default;

private:
    void Read(std::istream& input_stream, nie::io::PoseCollection* collection) const override;
};

class VertexXyz : public Entry<nie::io::ObjectCollection> {
public:
    ~VertexXyz() override = default;

private:
    void Read(std::istream& input_stream, nie::io::ObjectCollection* collection) const override;
};

class EdgeProjectP2SC : public Entry<nie::io::KeypointCollection> {
public:
    ~EdgeProjectP2SC() override = default;

private:
    void Read(std::istream& input_stream, nie::io::KeypointCollection* collection) const override;
};

void ReadG2oTue(
    std::string const& file_name,
    nie::io::PoseCollection* pose,
    nie::io::ObjectCollection* objt,
    nie::io::KeypointCollection* kpnt);

}  // namespace g2o_tue
