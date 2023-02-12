/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "write.hpp"

#include <nie/formats/ba_graph/pose_collection.hpp>      // for nie::io::PoseCollection
#include <nie/formats/kinematica/kinematica_reader.hpp>  // for nie::io::ReadKinematicaCsv

#include "conversion.hpp"

namespace nie {

namespace kinematica {

void ConvertKinematicaToPoseFile(std::string const& in_file, std::string const& out_file) {
    nie::io::KinematicaRecordCollection const kinematica_csv = nie::io::ReadKinematicaCsv(in_file);
    nie::io::PoseCollection const pose_collection = ConvertKinematicaCsvToPoseCollection(kinematica_csv);
    nie::io::Write(pose_collection, out_file);
}

}  // namespace kinematica

}  // namespace nie