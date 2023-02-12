/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "nie/formats/ba_graph/bbox_collection.hpp"
#include "nie/formats/ba_graph/info_ref_collection.hpp"
#include "nie/formats/ba_graph/keypoint_collection.hpp"
#include "nie/formats/ba_graph/object_collection.hpp"
#include "nie/formats/ba_graph/pose_collection.hpp"
#include "nie/formats/ba_graph/twist_collection.hpp"

namespace nie::io {

detail::SignatureType const BboxCollection::Signature = detail::CreateSignature(detail::kChunkTypeBboxRecord);
detail::SignatureType const InfoRefCollection::Signature = detail::CreateSignature(detail::kChunkTypeInfoRefRecord);
detail::SignatureType const KeypointCollection::Signature = detail::CreateSignature(detail::kChunkTypeKeypointRecord);
detail::SignatureType const ObjectCollection::Signature = detail::CreateSignature(detail::kChunkTypeObjectRecord);
detail::SignatureType const PoseCollection::Signature = detail::CreateSignature(detail::kChunkTypePoseRecord);
detail::SignatureType const TwistCollection::Signature = detail::CreateSignature(detail::kChunkTypeTwistRecord);

}  // namespace nie::io
