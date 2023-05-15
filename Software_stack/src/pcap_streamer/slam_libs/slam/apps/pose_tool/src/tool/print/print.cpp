/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "print.hpp"

#include <cstdint>
#include <iomanip>
#include <iostream>

#include <nie/formats/ba_graph/bbox_collection.hpp>
#include <nie/formats/ba_graph/info_ref_collection.hpp>
#include <nie/formats/ba_graph/keypoint_collection.hpp>
#include <nie/formats/ba_graph/object_collection.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "../io.hpp"

DEFINE_bool(to_file, false, "Set this flag to print the output to file.");

namespace detail {

// Formatting constants
std::int32_t const column_width = 24;
std::int32_t const small_field_width = 14;
std::int32_t const large_field_width = 32;
std::int32_t const decimal_precision = 8;

std::string const none = "N.A.";
std::string const file_divider = '\n' + std::string(75, '=') + '\n';
std::string const header_divider(35, '-');

// Common print function
// Set left and width
std::ostream& Side(std::ostream& out) { return out << std::right; }
std::ostream& SideAndFieldWidth(std::ostream& out) { return out << Side << std::setw(column_width); }
std::ostream& SideAndSmallFieldWidth(std::ostream& out) { return out << Side << std::setw(small_field_width); }
std::ostream& SideAndLargeFieldWidth(std::ostream& out) { return out << Side << std::setw(large_field_width); }
// Set left, width and fixed number precision
std::ostream& SideAndFieldWidthAndPrecision(std::ostream& out) {
    return out << SideAndFieldWidth << std::fixed << std::setprecision(decimal_precision);
}

std::string PrintSignature(nie::io::detail::SignatureType const& signature) {
    std::string result;
    for (std::size_t i = 1; i <= 4; ++i) {
        result.push_back(signature[i]);
    }
    return result;
}

template <int size>
void PrintInf(std::ostream& out, Eigen::Matrix<double, size, size> const& c) {
    for (std::size_t row = 0; row < size; ++row) {
        for (std::size_t col = row; col < size; ++col) {
            out << SideAndFieldWidth << c(row, col);
        }
    }
}

void PrintVersion(std::ostream& out, nie::io::VersionType version) {
    auto major_minor = nie::io::BaGraphMajorMinor(version);
    out << "version: " << major_minor.first << "." << major_minor.second << '\n';
}

/// InfoRefCollection

void Print(std::ostream& out, nie::io::InfoRefHeader const& header) { PrintVersion(out, header.version); }

void Print(std::ostream& out, nie::io::InfoRefRecord const& record) {
    out << SideAndFieldWidth << record.id << SideAndFieldWidth << static_cast<int>(record.frame_id) << " " << std::left
        << record.path << std::endl;
}

void Print(std::ostream& out, std::string const& path, nie::io::InfoRefCollection const& collection) {
    out << "InfoRefCollection (" << PrintSignature(nie::io::InfoRefCollection::Signature) << ")" << std::endl;
    out << path << std::endl;
    out << header_divider << std::endl;
    Print(out, collection.header);

    if (collection.info_refs.size() > 0) {
        out << std::endl;
        out << "info_refs" << '\n'
            << SideAndFieldWidth << "pose_id" << SideAndFieldWidth << "frame_id"
            << " " << std::left << "path" << std::endl;
        for (auto&& i : collection.info_refs) {
            Print(out, i);
        }
    }
}

/// PoseCollection

void PrintFlags(std::ostream& out, nie::io::PoseHeader const& header) {
    out << "flags" << '\n';
    out << " " << header.HasCodeZ() << " : has_code_z" << '\n';
    out << " " << header.HasPoseInformation() << " : has_pose_information" << '\n';
    out << " " << header.HasPoseInformationPerRecord() << " : has_pose_information_per_record" << '\n';
    out << " " << header.HasEdgeInformation() << " : has_edge_information" << '\n';
    out << " " << header.HasEdgeInformationPerRecord() << " : has_edge_information_per_record" << '\n';
    out << " " << header.HasTimestampPerRecord() << " : has_timestamp_per_record" << '\n';
}

void Print(std::ostream& out, nie::io::PoseRecord::Category const& category) {
    out << SideAndSmallFieldWidth;
    switch (category) {
        case nie::io::PoseRecord::Category::kGps:
            out << "gps";
            break;
        case nie::io::PoseRecord::Category::kOdom:
            out << "odom";
            break;
        case nie::io::PoseRecord::Category::kBbox:
            out << "bbox";
            break;
    }
}

void Print(std::ostream& out, nie::io::PoseEdgeRecord::Category const& category) {
    out << SideAndSmallFieldWidth;
    switch (category) {
        case nie::io::PoseEdgeRecord::Category::kOdom:
            out << "odom";
            break;
        case nie::io::PoseEdgeRecord::Category::kRelToAbs:
            out << "rel_to_abs";
            break;
        case nie::io::PoseEdgeRecord::Category::kLoop:
            out << "loop";
            break;
    }
}

void Print(std::ostream& out, nie::io::PoseHeader const& header) {
    PrintVersion(out, header.version);
    PrintFlags(out, header);
    out << "authority: " << header.authority << '\n' << "code_xy_or_xyz: " << header.code_xy_or_xyz << '\n';

    if (header.HasCodeZ()) {
        out << "code_z: " << header.code_z << '\n';
    }

    if (header.HasPoseInformation()) {
        out << "pose_information: " << '\n' << header.pose_information;
    }

    if (header.HasEdgeInformation()) {
        out << "edge_information: " << '\n' << header.edge_information;
    }
}

void PrintIsometry(std::ostream& out, nie::Isometry3qd const& isometry) {
    Eigen::Vector3d const& p = isometry.translation();
    out << SideAndFieldWidth << p[0] << SideAndFieldWidth << p[1] << SideAndFieldWidth << p[2];

    Eigen::Quaterniond const& q = isometry.rotation();
    out << SideAndFieldWidth << q.x() << SideAndFieldWidth << q.y() << SideAndFieldWidth << q.z() << SideAndFieldWidth
        << q.w();
}

void Print(std::ostream& out, nie::io::PoseHeader const& header, nie::io::PoseRecord const& record) {
    out << SideAndSmallFieldWidth << record.id;
    Print(out, record.category);
    out << SideAndSmallFieldWidth << record.device_id;
    if (header.HasTimestampPerRecord()) {
        // date tz has its own formatter that overrides width
        std::stringstream ss;
        ss << record.timestamp;
        out << SideAndLargeFieldWidth << ss.str();
    }

    PrintIsometry(out, record.isometry);

    if (header.Has(nie::io::PoseHeader::Flag::kHasPoseInformationPerRecord)) {
        PrintInf(out, record.information);
    }
    out << std::endl;
}

void Print(std::ostream& out, nie::io::PoseHeader const& header, nie::io::PoseEdgeRecord const& record) {
    out << SideAndSmallFieldWidth << record.id_begin << SideAndSmallFieldWidth << record.id_end;
    Print(out, record.category);

    PrintIsometry(out, record.isometry);

    if (header.HasEdgeInformationPerRecord()) {
        PrintInf(out, record.information);
    }
    out << std::endl;
}

void Print(std::ostream& out, nie::io::PoseHeader const&, nie::io::FixedPoseRecord const& record) {
    out << SideAndFieldWidth << record.id << std::endl;
}

void Print(std::ostream& out, std::string const& path, nie::io::PoseCollection const& collection) {
    out << "PoseCollection (" << PrintSignature(nie::io::PoseCollection::Signature) << ")" << std::endl;
    out << path << std::endl;
    out << header_divider << std::endl;

    Print(out, collection.header);

    if (collection.poses.size() > 0) {
        out << '\n' << "pose" << '\n';
        out << SideAndSmallFieldWidth << "id" << SideAndSmallFieldWidth << "category" << SideAndSmallFieldWidth
            << "device_id";
        if (collection.header.Has(nie::io::PoseHeader::Flag::kHasTimestampPerRecord)) {
            out << SideAndLargeFieldWidth << "timestamp";
        }
        out << SideAndFieldWidthAndPrecision << "p x" << SideAndFieldWidthAndPrecision << "p y"
            << SideAndFieldWidthAndPrecision << "p z" << SideAndFieldWidthAndPrecision << "q x"
            << SideAndFieldWidthAndPrecision << "q y" << SideAndFieldWidthAndPrecision << "q z"
            << SideAndFieldWidthAndPrecision << "q w";
        if (collection.header.Has(nie::io::PoseHeader::Flag::kHasPoseInformationPerRecord)) {
            out << SideAndFieldWidthAndPrecision << "i00" << SideAndFieldWidthAndPrecision << "i01"
                << SideAndFieldWidthAndPrecision << "i02" << SideAndFieldWidthAndPrecision << "i03"
                << SideAndFieldWidthAndPrecision << "i04" << SideAndFieldWidthAndPrecision << "i05"
                << SideAndFieldWidthAndPrecision << "i11" << SideAndFieldWidthAndPrecision << "i12"
                << SideAndFieldWidthAndPrecision << "i13" << SideAndFieldWidthAndPrecision << "i14"
                << SideAndFieldWidthAndPrecision << "i15" << SideAndFieldWidthAndPrecision << "i22"
                << SideAndFieldWidthAndPrecision << "i23" << SideAndFieldWidthAndPrecision << "i24"
                << SideAndFieldWidthAndPrecision << "i25" << SideAndFieldWidthAndPrecision << "i33"
                << SideAndFieldWidthAndPrecision << "i34" << SideAndFieldWidthAndPrecision << "i35"
                << SideAndFieldWidthAndPrecision << "i44" << SideAndFieldWidthAndPrecision << "i45"
                << SideAndFieldWidthAndPrecision << "i55";
        }
        out << std::endl;
        for (auto&& e : collection.poses) {
            Print(out, collection.header, e);
        }
    }

    if (collection.edges.size() > 0) {
        out << '\n' << "edge" << '\n';
        out << SideAndSmallFieldWidth << "id_begin" << SideAndSmallFieldWidth << "id_end" << SideAndSmallFieldWidth
            << "category";
        out << SideAndFieldWidthAndPrecision << "p x" << SideAndFieldWidthAndPrecision << "p y"
            << SideAndFieldWidthAndPrecision << "p z" << SideAndFieldWidthAndPrecision << "q x"
            << SideAndFieldWidthAndPrecision << "q y" << SideAndFieldWidthAndPrecision << "q z"
            << SideAndFieldWidthAndPrecision << "q w";
        if (collection.header.Has(nie::io::PoseHeader::Flag::kHasEdgeInformationPerRecord)) {
            out << SideAndFieldWidthAndPrecision << "i00" << SideAndFieldWidthAndPrecision << "i01"
                << SideAndFieldWidthAndPrecision << "i02" << SideAndFieldWidthAndPrecision << "i03"
                << SideAndFieldWidthAndPrecision << "i04" << SideAndFieldWidthAndPrecision << "i05"
                << SideAndFieldWidthAndPrecision << "i11" << SideAndFieldWidthAndPrecision << "i12"
                << SideAndFieldWidthAndPrecision << "i13" << SideAndFieldWidthAndPrecision << "i14"
                << SideAndFieldWidthAndPrecision << "i15" << SideAndFieldWidthAndPrecision << "i22"
                << SideAndFieldWidthAndPrecision << "i23" << SideAndFieldWidthAndPrecision << "i24"
                << SideAndFieldWidthAndPrecision << "i25" << SideAndFieldWidthAndPrecision << "i33"
                << SideAndFieldWidthAndPrecision << "i34" << SideAndFieldWidthAndPrecision << "i35"
                << SideAndFieldWidthAndPrecision << "i44" << SideAndFieldWidthAndPrecision << "i45"
                << SideAndFieldWidthAndPrecision << "i55";
        }
        out << std::endl;
        for (auto&& e : collection.edges) {
            Print(out, collection.header, e);
        }
    }

    if (collection.fixes.size() > 0) {
        out << std::endl;
        out << "fixes" << '\n' << SideAndFieldWidth << "id" << std::endl;
        for (auto&& e : collection.fixes) {
            Print(out, collection.header, e);
        }
    }
}

/// ObjectCollection

void PrintFlags(std::ostream& out, nie::io::ObjectHeader const& header) {
    out << "flags" << '\n';
    out << " " << ((header.flags & nie::io::ObjectHeader::Flag::kHasInformation) > 0) << " : has_information" << '\n';
    out << " " << ((header.flags & nie::io::ObjectHeader::Flag::kHasInformationPerRecord) > 0)
        << " : has_information_per_record" << '\n';
}

void Print(std::ostream& out, nie::io::ObjectHeader const& header) {
    PrintVersion(out, header.version);
    PrintFlags(out, header);

    if (header.flags & nie::io::ObjectHeader::Flag::kHasInformation) {
        out << "information: " << std::endl << header.information << std::endl;
    }
}

void Print(std::ostream& out, nie::io::ObjectHeader const& header, nie::io::ObjectRecord const& record) {
    out << SideAndFieldWidth << record.id << SideAndFieldWidth << record.position[0] << SideAndFieldWidth
        << record.position[1] << SideAndFieldWidth << record.position[2];
    if (header.Has(nie::io::ObjectHeader::Flag::kHasInformationPerRecord)) {
        PrintInf(out, record.information);
    }
    out << std::endl;
}

void Print(std::ostream& out, std::string const& path, nie::io::ObjectCollection const& collection) {
    out << "ObjectCollection (" << PrintSignature(nie::io::ObjectCollection::Signature) << ")" << std::endl;
    out << path << std::endl;
    out << header_divider << std::endl;

    Print(out, collection.header);

    if (collection.objects.size() > 0) {
        out << std::endl;
        out << "objects" << '\n'
            << SideAndFieldWidth << "id" << SideAndFieldWidthAndPrecision << "p x" << SideAndFieldWidthAndPrecision
            << "p y" << SideAndFieldWidthAndPrecision << "p z";
        if (collection.header.Has(nie::io::ObjectHeader::Flag::kHasInformationPerRecord)) {
            out << SideAndFieldWidthAndPrecision << "i00" << SideAndFieldWidthAndPrecision << "i01"
                << SideAndFieldWidthAndPrecision << "i02" << SideAndFieldWidthAndPrecision << "i11"
                << SideAndFieldWidthAndPrecision << "i12" << SideAndFieldWidthAndPrecision << "i22";
        }
        out << std::endl;
        for (auto&& i : collection.objects) {
            Print(out, collection.header, i);
        }
    }
}

/// KeypointCollection

void PrintFlags(std::ostream& out, nie::io::KeypointHeader const& header) {
    out << "flags" << '\n';
    out << " " << ((header.flags & nie::io::KeypointHeader::Flag::kHasInformation) > 0) << " : has_information" << '\n';
    out << " " << ((header.flags & nie::io::KeypointHeader::Flag::kHasInformationPerRecord) > 0)
        << " : has_information_per_record" << '\n';
}

void Print(std::ostream& out, nie::io::KeypointHeader const& header) {
    PrintVersion(out, header.version);
    PrintFlags(out, header);

    if (header.Has(nie::io::KeypointHeader::Flag::kHasInformation)) {
        out << "information: " << std::endl << header.information << std::endl;
    }
}

void Print(std::ostream& out, nie::io::KeypointHeader const& header, nie::io::KeypointRecord const& record) {
    out << SideAndFieldWidth << record.pose_id << SideAndFieldWidth << static_cast<int>(record.frame_id)
        << SideAndFieldWidth << record.object_id << SideAndFieldWidth << record.position[0] << SideAndFieldWidth
        << record.position[1];
    if (header.Has(nie::io::KeypointHeader::Flag::kHasInformationPerRecord)) {
        PrintInf(out, record.information);
    }
    out << std::endl;
}

void Print(std::ostream& out, std::string const& path, nie::io::KeypointCollection const& collection) {
    out << "KeypointCollection (" << PrintSignature(nie::io::KeypointCollection::Signature) << ")" << std::endl;
    out << path << std::endl;
    out << header_divider << std::endl;

    Print(out, collection.header);

    if (collection.keypoints.size() > 0) {
        out << std::endl;
        out << "keypoints" << '\n'
            << SideAndFieldWidth << "pose_id" << SideAndFieldWidth << "frame_id" << SideAndFieldWidth << "object_id"
            << SideAndFieldWidthAndPrecision << "p x" << SideAndFieldWidthAndPrecision << "p y";
        if (collection.header.Has(nie::io::KeypointHeader::Flag::kHasInformationPerRecord)) {
            out << SideAndFieldWidthAndPrecision << "i00" << SideAndFieldWidthAndPrecision << "i01"
                << SideAndFieldWidthAndPrecision << "i11";
        }
        out << std::endl;
        for (auto&& i : collection.keypoints) {
            Print(out, collection.header, i);
        }
    }
}

/// Bbox Collection

void Print(std::ostream& out, nie::io::BboxHeader const& header) { PrintVersion(out, header.version); }

void Print(std::ostream& out, nie::io::BboxRecord const& record) {
    out << SideAndFieldWidth << record.id << SideAndFieldWidthAndPrecision << record.min[0]
        << SideAndFieldWidthAndPrecision << record.min[1] << SideAndFieldWidthAndPrecision << record.min[2]
        << SideAndFieldWidthAndPrecision << record.max[0] << SideAndFieldWidthAndPrecision << record.max[1]
        << SideAndFieldWidthAndPrecision << record.max[2] << std::endl;
}

void Print(std::ostream& out, std::string const& path, nie::io::BboxCollection const& collection) {
    out << "BboxCollection (" << PrintSignature(nie::io::BboxCollection::Signature) << ")" << std::endl;
    out << path << std::endl;
    out << header_divider << std::endl;
    Print(out, collection.header);

    if (collection.boxes.size() > 0) {
        out << std::endl;
        out << "boxes" << '\n'
            << SideAndFieldWidth << "pose_id" << SideAndFieldWidthAndPrecision << "min x"
            << SideAndFieldWidthAndPrecision << "min y" << SideAndFieldWidthAndPrecision << "min z"
            << SideAndFieldWidthAndPrecision << "max x" << SideAndFieldWidthAndPrecision << "max y"
            << SideAndFieldWidthAndPrecision << "max z" << std::endl;

        for (auto&& i : collection.boxes) {
            Print(out, i);
        }
    }
}

/// General

template <typename T>
void Print(std::ostream& out) {
    std::vector<std::pair<std::string, T>> collections;
    ReadData(&collections);

    Print(out, collections[0].first, collections[0].second);
    for (std::size_t i = 1; i < collections.size(); ++i) {
        std::cout << file_divider << std::endl;
        Print(out, collections[i].first, collections[i].second);
    }
}

template <typename T>
void Print(std::ostream& out, bool printed) {
    if (printed) {
        std::cout << detail::file_divider << std::endl;
    }
    detail::Print<T>(out);
}

//! \p printed is both input and output
template <typename T>
void Print(std::ostream& out, bool print, bool* printed) {
    if (print) {
        detail::Print<T>(out, *printed);
        *printed = true;
    }
}

}  // namespace detail

void Print() {
    bool const iref = InPathExists(nie::io::graph::Extension<nie::io::InfoRefCollection>());
    bool const pose = InPathExists(nie::io::graph::Extension<nie::io::PoseCollection>());
    bool const objt = InPathExists(nie::io::graph::Extension<nie::io::ObjectCollection>());
    bool const kpnt = InPathExists(nie::io::graph::Extension<nie::io::KeypointCollection>());
    bool const bbox = InPathExists(nie::io::graph::Extension<nie::io::BboxCollection>());

    CHECK(iref || pose || objt || kpnt || bbox) << "No familiar input files found given the -in_paths argument.";

    std::string file_name;
    if (FLAGS_to_file) {
        CheckOutPathsLocationsOrFatal();
        file_name = GetAndCheckOutPathsForExtensionOrFatal(".txt").string();
    }
    std::ofstream f_out(file_name);
    std::ostream& out = FLAGS_to_file ? f_out : std::cout;

    bool printed = false;
    detail::Print<nie::io::InfoRefCollection>(out, iref, &printed);
    detail::Print<nie::io::PoseCollection>(out, pose, &printed);
    detail::Print<nie::io::ObjectCollection>(out, objt, &printed);
    detail::Print<nie::io::KeypointCollection>(out, kpnt, &printed);
    detail::Print<nie::io::BboxCollection>(out, bbox, &printed);

    if (FLAGS_to_file) {
        LOG(INFO) << "Successfully printed collections to file " << file_name;
    }
}
