/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <fstream>

#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <nie/core/geometry/isometry3.hpp>
#include <nie/lidar/geometry/pose_bbox.hpp>
#include <pdal/PointView.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>

// TODO(jbr): Remove all of the templating as it is not contributing to anything. Proof:
// TODO(jbr): * WriteLas: Hardcoded expectations of output and options.
// TODO(jbr): * WriteLas: No checks whether or not a correct tuple is used (e.g. missing a dimension [x,y,z] which is
//              not possible).

// TODO(jbr): Removing all templating results in:
// TODO(jbr): 60-70% reduction of code without loss of functionality.
// TODO(jbr): Code becomes clear for the general reader and more easy to maintain / optimize.
// TODO(jbr): Prevents the creation of many checks that are now required. Missing dimensions, correct format, etc.
// TODO(jbr): More compatible data types across this library.

namespace nie {

namespace io {

/// Type trait for PDAL dimension id. Each field in a LAS point has a unique dimension id.
template <pdal::Dimension::Id>
struct IdTrait {};
template <>
struct IdTrait<pdal::Dimension::Id::X> {
    using type = double;
};
template <>
struct IdTrait<pdal::Dimension::Id::Y> {
    using type = double;
};
template <>
struct IdTrait<pdal::Dimension::Id::Z> {
    using type = double;
};
template <>
struct IdTrait<pdal::Dimension::Id::Intensity> {
    using type = std::uint16_t;
};
template <>
struct IdTrait<pdal::Dimension::Id::GpsTime> {
    using type = double;
};

/// Wrapper around std::tuple<> for type safe usage.
/// Attributes of a LAS point are gettable by pdal::Dimension::Id.
template <pdal::Dimension::Id... Ids>
struct LasPoint : std::tuple<typename IdTrait<Ids>::type...> {
private:
    template <pdal::Dimension::Id Id, pdal::Dimension::Id...>
    struct IndexOf;  // forward decl
    template <pdal::Dimension::Id Id, pdal::Dimension::Id Id_, pdal::Dimension::Id... Ids_>
    struct IndexOf<Id, Id_, Ids_...> {
        static constexpr size_t value = 1 + IndexOf<Id, Ids_...>::value;
    };
    template <pdal::Dimension::Id Id, pdal::Dimension::Id... Ids_>
    struct IndexOf<Id, Id, Ids_...> {  // Found value so stop inheritance
        static constexpr size_t value = 0;
    };

    template <size_t Index, pdal::Dimension::Id...>
    struct GetElement;
    template <size_t Index, pdal::Dimension::Id Id, pdal::Dimension::Id... Ids_>
    struct GetElement<Index, Id, Ids_...> : GetElement<Index - 1, Ids_...> {};
    template <pdal::Dimension::Id Id, pdal::Dimension::Id... Ids_>
    struct GetElement<0, Id, Ids_...> {
        static constexpr pdal::Dimension::Id value = Id;
    };

public:
    template <typename... Ts>
    constexpr LasPoint(Ts&&... values) : std::tuple<typename IdTrait<Ids>::type...>{std::forward<Ts>(values)...} {}

    template <size_t Index>
    constexpr static pdal::Dimension::Id GetId() {
        return GetElement<Index, Ids...>::value;
    }
    template <pdal::Dimension::Id Id>
    constexpr static size_t GetIndex() {
        return IndexOf<Id, Ids...>::value;
    }

    template <pdal::Dimension::Id Id>
    constexpr auto Get() -> decltype(std::get<IndexOf<Id, Ids...>::value>(*this))& {
        return std::get<IndexOf<Id, Ids...>::value>(*this);
    }
    template <pdal::Dimension::Id Id>
    constexpr auto Get() const -> decltype(std::get<IndexOf<Id, Ids...>::value>(*this)) const& {
        return std::get<IndexOf<Id, Ids...>::value>(*this);
    }
};

namespace detail {

template <size_t Index = 0, pdal::Dimension::Id... Ids>
void PushLasPointImpl(pdal::PointViewPtr point_view_ptr, std::size_t const index, LasPoint<Ids...> const& las_point) {
    constexpr pdal::Dimension::Id kId = LasPoint<Ids...>::template GetId<Index>();
    point_view_ptr->setField(kId, index, las_point.template Get<kId>());
    if constexpr (Index + 1 < sizeof...(Ids)) {
        PushLasPointImpl<Index + 1>(point_view_ptr, index, las_point);
    }
}

}  // namespace detail

template <pdal::Dimension::Id... Ids>
void PushLasPoint(pdal::PointViewPtr point_view_ptr, LasPoint<Ids...> const& las_point) {
    detail::PushLasPointImpl(point_view_ptr, point_view_ptr->size(), las_point);
}

namespace detail {

pdal::Options DefaultWriteLasOptions(std::string const& file_path) {
    pdal::Options options;

    options.add("filename", file_path);
    options.add("dataformat_id", 1);  // times are stored

    options.add("offset_x", "auto");
    options.add("offset_y", "auto");
    options.add("offset_z", "auto");
    options.add("scale_x", "auto");
    options.add("scale_y", "auto");
    options.add("scale_z", "auto");

    return options;
}

template <pdal::Dimension::Id... Ids>
void WriteLasImpl(std::vector<LasPoint<Ids...>> const& las_data, pdal::Options options) {
    pdal::PointTable table;
    table.layout()->registerDims({Ids...});

    pdal::PointViewPtr point_view_ptr = std::make_shared<pdal::PointView>(table);
    for (auto const& las_point : las_data) {
        PushLasPoint(point_view_ptr, las_point);
    }

    pdal::BufferReader buffer_reader;
    buffer_reader.addView(point_view_ptr);

    pdal::LasWriter writer;
    writer.setOptions(std::move(options));
    writer.setInput(buffer_reader);
    writer.prepare(table);
    writer.execute(table);
}

}  // namespace detail

template <pdal::Dimension::Id... Ids>
void WriteLas(boost::filesystem::path const& filepath, std::vector<LasPoint<Ids...>> const& las_data) {
    detail::WriteLasImpl(las_data, detail::DefaultWriteLasOptions(filepath.string()));
}

template <pdal::Dimension::Id... Ids>
void WriteLasWithOffsetScale(
        boost::filesystem::path const& filepath,
        std::vector<LasPoint<Ids...>> const& las_data,
        Eigen::Vector3d const& offset,
        double const scale) {
    // Note that somehow the scaling can not be set to auto when a custom offset is supplied.
    // The offset is "manually" transformed to string which is done anyway internally.
    // As pdal uses the regular ostream to convert to string, the precision after 6 significant places is lost.
    // That is why to_string is used.
    pdal::Options options = detail::DefaultWriteLasOptions(filepath.string());
    options.replace("offset_x", std::to_string(offset.x()));
    options.replace("offset_y", std::to_string(offset.y()));
    options.replace("offset_z", std::to_string(offset.z()));
    options.replace("scale_x", scale);
    options.replace("scale_y", scale);
    options.replace("scale_z", scale);

    detail::WriteLasImpl(las_data, std::move(options));
}

namespace detail {

template <size_t Index = 0, pdal::Dimension::Id... Ids>
void GetLasPointImpl(pdal::PointViewPtr point_view_ptr, size_t const index, LasPoint<Ids...>* las_point) {
    constexpr pdal::Dimension::Id kId = LasPoint<Ids...>::template GetId<Index>();
    using Scalar = typename IdTrait<kId>::type;
    las_point->template Get<kId>() = point_view_ptr->getFieldAs<Scalar>(kId, index);
    if constexpr (Index + 1 < sizeof...(Ids)) {
        GetLasPointImpl<Index + 1>(point_view_ptr, index, las_point);
    }
}

}  // namespace detail

template <pdal::Dimension::Id... Ids>
void GetLasPoint(pdal::PointViewPtr point_view_ptr, size_t const index, LasPoint<Ids...>* las_point) {
    detail::GetLasPointImpl(point_view_ptr, index, las_point);
}

template <pdal::Dimension::Id... Ids>
void ReadLas(boost::filesystem::path const& filepath, std::vector<LasPoint<Ids...>>* las_data) {
    pdal::Options options;
    options.add("filename", filepath.string());
    pdal::LasReader reader;
    reader.setOptions(options);

    pdal::PointTable table;
    reader.prepare(table);

    pdal::PointViewSet point_view_set = reader.execute(table);

    // TODO: Investigate if this is required or just something that PDAL unit test does.
    CHECK(point_view_set.size() == 1) << "Expected pdal::PointViewSet size to be 1.";

    pdal::PointViewPtr point_view_ptr = *point_view_set.begin();

    las_data->resize(point_view_ptr->size());
    for (size_t i = 0; i < point_view_ptr->size(); ++i) {
        GetLasPoint(point_view_ptr, i, &(*las_data)[i]);
    }
}

}  // namespace io

}  // namespace nie
