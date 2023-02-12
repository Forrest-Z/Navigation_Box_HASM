/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_to_kml.hpp"

#include <fstream>
#include <iostream>

#include <kml/convenience/convenience.h>
#include <kml/dom.h>
#include <Eigen/Dense>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/object_collection.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "tool/io.hpp"

DEFINE_uint64(traject_time_gap_us, 1e6, "The time gap between trajectories in microseconds (1 second by default");
DEFINE_validator(traject_time_gap_us, nie::ValidateLargerThanZero);

DEFINE_bool(kml_edges, false, "If true, edges from the pose file are added to the output kml file.");
DEFINE_bool(
        kml_edges_relational,
        true,
        "If true, shows which vertices are related. If false, shows the geometric distance with respect to the first "
        "pose.");

namespace detail {

class KmlWrapper {
private:
    template <typename Collection>
    using AddDataFunction = std::function<void(Collection const&, kmldom::DocumentPtr*)>;

public:
    explicit KmlWrapper() : factory_(kmldom::KmlFactory::GetFactory()) {}

    void Write(
            nie::io::PoseCollection const& pose_coll,
            nie::io::ObjectCollection const& objt_coll,
            std::string const& path) {
        using namespace std::placeholders;
        WriteKml<nie::io::PoseCollection>(
                pose_coll, std::bind(&KmlWrapper::AddPoses, this, _1, _2), path + "/pose.kml");
        LOG(INFO) << "Pose collection kml file written.";

        if (!objt_coll.objects.empty()) {
            WriteKml<nie::io::ObjectCollection>(
                    objt_coll, std::bind(&KmlWrapper::AddObjects, _1, _2), path + "/objt.kml");
            LOG(INFO) << "Object collection kml file written.";
        }
    }

private:
    template <typename Collection>
    void WriteKml(Collection const& coll, AddDataFunction<Collection> const& add_data, std::string const& path) {
        kmldom::DocumentPtr document = factory_->CreateDocument();

        add_data(coll, &document);

        // Serialize to XML
        kmldom::KmlPtr kml = factory_->CreateKml();
        kml->set_feature(document);
        std::string xml = kmldom::SerializePretty(kml);

        // Print to stdout
        std::ofstream(path) << xml;
    }

    void AddPoses(nie::io::PoseCollection const& pose_coll, kmldom::DocumentPtr* p_doc) {
        CHECK(!pose_coll.poses.empty()) << "There are no poses in the supplied pose collection.";
        if (!FLAGS_kml_edges) {
            kmldom::CoordinatesPtr coords = factory_->CreateCoordinates();
            nie::Timestamp_ns prev_time = pose_coll.poses.front().timestamp;
            std::chrono::microseconds const time_gap{FLAGS_traject_time_gap_us};
            for (auto const& pose : pose_coll.poses) {
                if (pose.timestamp > prev_time + time_gap) {
                    AddLine(coords, p_doc);
                    coords.reset(factory_->CreateCoordinates());
                }
                auto const& t = pose.isometry.translation();
                coords->add_latlng(Lat(t), Lon(t));
                prev_time = pose.timestamp;
            }

            // Add remainder of the points
            AddLine(coords, p_doc);
        } else {
            std::unordered_map<std::uint32_t, std::size_t> index_by_id;

            for (std::size_t i = 0; i < pose_coll.poses.size(); ++i) {
                auto const& pose = pose_coll.poses[i];
                index_by_id[pose.id] = i;
                auto placemark = kmlconvenience::CreatePointPlacemark(
                        "Object " + std::to_string(pose.id),
                        Lat(pose.isometry.translation()),
                        Lon(pose.isometry.translation()));
                (*p_doc)->add_feature(placemark);
            }

            for (auto const& edge : pose_coll.edges) {
                kmldom::CoordinatesPtr coords = factory_->CreateCoordinates();
                auto const& pose0 = pose_coll.poses[index_by_id[edge.id_begin]];
                auto const& pose1 = pose_coll.poses[index_by_id[edge.id_end]];
                auto const& t0 = pose0.isometry.translation();
                // No reference here on purpose in case the flag is false. That result doesn't get caught by a ref.
                auto const t1 = FLAGS_kml_edges_relational ? (pose1.isometry.translation())
                                                           : ((pose0.isometry * edge.isometry).translation());

                coords->add_latlng(Lat(t0), Lon(t0));
                coords->add_latlng(Lat(t1), Lon(t1));
                AddLine(coords, p_doc);
            }
        }
    }

    static void AddObjects(nie::io::ObjectCollection const& objt_coll, kmldom::DocumentPtr* p_doc) {
        for (auto const& object : objt_coll.objects) {
            auto placemark = kmlconvenience::CreatePointPlacemark(
                    "Object " + std::to_string(object.id), Lat(object.position), Lon(object.position));
            (*p_doc)->add_feature(placemark);
        }
    }

    void AddLine(kmldom::CoordinatesPtr const& coords, kmldom::DocumentPtr* p_doc) const {
        if (coords->get_coordinates_array_size() == 0) {
            return;
        }

        kmldom::LineStringPtr line = factory_->CreateLineString();
        line->set_coordinates(coords);

        kmldom::PlacemarkPtr placemark = factory_->CreatePlacemark();
        placemark->set_geometry(line);

        (*p_doc)->add_feature(placemark);
    }

    inline static double Lat(Eigen::Vector3d const& t) { return t[1]; }
    inline static double Lon(Eigen::Vector3d const& t) { return t[0]; }

    kmldom::KmlFactory* const factory_;
};

}  // namespace detail

void ConvertToKml() {
    InPathExistsOrFatal(nie::io::graph::Extension<nie::io::PoseCollection>());
    bool has_objt = InPathExists(nie::io::graph::Extension<nie::io::ObjectCollection>());
    OutDirAvailableOrFatal();

    nie::io::PoseCollection pose_coll;
    ReadData(&pose_coll);
    nie::io::ObjectCollection objt_coll;
    if (has_objt) {
        ReadData(&objt_coll);
    }

    detail::KmlWrapper kml;
    kml.Write(pose_coll, objt_coll, FLAGS_out_paths);
}
