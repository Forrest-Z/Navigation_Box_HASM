/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/formats/ba_graph/pose_collection.hpp>      // for nie::io::PoseCollection
#include <nie/formats/kinematica/kinematica_reader.hpp>  // for nie::io::KinematicaRecordCollection

namespace nie {

namespace kinematica {

/// Converts a KinematicaCsvRecord to PoseCollection
/// \param csv_row KinematicaCsvRecord to be converted
/// \return KinematicaCsvRecord converted to PoseCollection
nie::io::PoseCollection ConvertKinematicaCsvToPoseCollection(nie::io::KinematicaRecordCollection const& csv_row);

}  // namespace kinematica

}  // namespace nie