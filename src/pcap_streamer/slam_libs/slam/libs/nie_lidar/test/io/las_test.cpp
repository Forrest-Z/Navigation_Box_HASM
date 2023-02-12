/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/lidar/io/las.hpp>
#include <nie/lidar/io/las_reader.hpp>

#include <pdal/PointView.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/io/LasWriter.hpp>

TEST(Las, PdalWrite) {
    std::string const kFilepath = nie::TemporaryFile("03-00001006190103-164144-00000.las").string();

    pdal::PointTable table;

    table.layout()->registerDims({pdal::Dimension::Id::X, pdal::Dimension::Id::Y, pdal::Dimension::Id::Z});

    pdal::BufferReader bufferReader;

    pdal::PointViewPtr view(new pdal::PointView(table));
    view->setField(pdal::Dimension::Id::X, 0, 125000.00);
    view->setField(pdal::Dimension::Id::X, 1, 74529.00);
    view->setField(pdal::Dimension::Id::X, 2, 1000000.02);
    view->setField(pdal::Dimension::Id::Y, 0, 0);
    view->setField(pdal::Dimension::Id::Y, 1, 1);
    view->setField(pdal::Dimension::Id::Y, 2, 2);
    view->setField(pdal::Dimension::Id::Z, 0, -123);
    view->setField(pdal::Dimension::Id::Z, 1, 456.78);
    view->setField(pdal::Dimension::Id::Z, 2, 945.23);
    bufferReader.addView(view);

    view.reset(new pdal::PointView(table));
    view->setField(pdal::Dimension::Id::X, 0, 25.00);
    view->setField(pdal::Dimension::Id::X, 1, 74529.00);
    view->setField(pdal::Dimension::Id::X, 2, 534252.35);
    view->setField(pdal::Dimension::Id::Y, 0, 3);
    view->setField(pdal::Dimension::Id::Y, 1, 4);
    view->setField(pdal::Dimension::Id::Y, 2, 5);
    view->setField(pdal::Dimension::Id::Z, 0, 1.5);
    view->setField(pdal::Dimension::Id::Z, 1, 2147483524);
    view->setField(pdal::Dimension::Id::Z, 2, 745.23);
    bufferReader.addView(view);

    pdal::Options writerOps;
    writerOps.add("filename", kFilepath);
    writerOps.add("offset_x", "auto");
    writerOps.add("scale_x", "auto");
    writerOps.add("offset_z", "auto");
    writerOps.add("scale_z", "auto");

    pdal::LasWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(bufferReader);

    writer.prepare(table);
    writer.execute(table);

    pdal::Options readerOps;
    readerOps.add("filename", kFilepath);

    pdal::PointTable readTable;

    pdal::LasReader reader;
    reader.setOptions(readerOps);

    reader.prepare(readTable);
    EXPECT_DOUBLE_EQ(25.00, reader.header().offsetX());
    EXPECT_DOUBLE_EQ(0, reader.header().offsetY());
    EXPECT_DOUBLE_EQ(-123, reader.header().offsetZ());

    EXPECT_DOUBLE_EQ(0.00046564965530561733, reader.header().scaleX());
    EXPECT_DOUBLE_EQ(.01, reader.header().scaleY());
    // (max - min) are chosen to yield std::numeric_limits<int>::max();
    EXPECT_DOUBLE_EQ(1.0, reader.header().scaleZ());

    pdal::PointViewSet viewSet = reader.execute(readTable);

    pdal::LasHeader las_header = reader.header();

    EXPECT_EQ(viewSet.size(), 1u);
    view = *viewSet.begin();
    EXPECT_EQ(view->size(), 6u);
    EXPECT_NEAR(125000.00, view->getFieldAs<double>(pdal::Dimension::Id::X, 0), .0001);
    EXPECT_NEAR(74529.00, view->getFieldAs<double>(pdal::Dimension::Id::X, 1), .001);
    EXPECT_NEAR(1000000.02, view->getFieldAs<double>(pdal::Dimension::Id::X, 2), .0001);
    EXPECT_NEAR(25.00, view->getFieldAs<double>(pdal::Dimension::Id::X, 3), .0001);
    EXPECT_NEAR(74529.00, view->getFieldAs<double>(pdal::Dimension::Id::X, 4), .001);
    EXPECT_NEAR(534252.35, view->getFieldAs<double>(pdal::Dimension::Id::X, 5), .0001);
}

TEST(LasIo, WriteRead) {
    constexpr float kPrecision = 1e-9;
    std::string const kFilepath = nie::TemporaryFile("03-00001006190103-164144-00000.las").string();

    using Point = nie::io::LasPoint<
        pdal::Dimension::Id::X,
        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Intensity,
        pdal::Dimension::Id::GpsTime>;

    std::vector<Point> points;
    for (std::uint16_t i = 0; i < 10; ++i) {
        double id = static_cast<double>(i + 1) / 100.0;
        points.emplace_back(id, id, id, i, id * 1000.0);
    }

    nie::io::WriteLas(kFilepath, points);

    //
    std::vector<Point> points_read;
    nie::io::ReadLas(kFilepath, &points_read);

    ASSERT_EQ(points.size(), points_read.size());

    for (size_t i = 0; i < points.size(); ++i) {
        ASSERT_NEAR(points[i].Get<pdal::Dimension::Id::X>(), points_read[i].Get<pdal::Dimension::Id::X>(), kPrecision);
        ASSERT_NEAR(points[i].Get<pdal::Dimension::Id::Y>(), points_read[i].Get<pdal::Dimension::Id::Y>(), kPrecision);
        ASSERT_NEAR(points[i].Get<pdal::Dimension::Id::Z>(), points_read[i].Get<pdal::Dimension::Id::Z>(), kPrecision);
        ASSERT_NEAR(
            points[i].Get<pdal::Dimension::Id::Intensity>(),
            points_read[i].Get<pdal::Dimension::Id::Intensity>(),
            kPrecision);
        ASSERT_NEAR(
            points[i].Get<pdal::Dimension::Id::GpsTime>(),
            points_read[i].Get<pdal::Dimension::Id::GpsTime>(),
            kPrecision);
    }
}

TEST(LasIoPoint, GetIndex) {
    using Point = nie::io::LasPoint<
        pdal::Dimension::Id::X,
        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Intensity,
        pdal::Dimension::Id::GpsTime>;

    ASSERT_EQ(Point::GetIndex<pdal::Dimension::Id::X>(), 0);
    ASSERT_EQ(Point::GetIndex<pdal::Dimension::Id::Y>(), 1);
    ASSERT_EQ(Point::GetIndex<pdal::Dimension::Id::Z>(), 2);
    ASSERT_EQ(Point::GetIndex<pdal::Dimension::Id::Intensity>(), 3);
    ASSERT_EQ(Point::GetIndex<pdal::Dimension::Id::GpsTime>(), 4);
}

TEST(LasIoPoint, GetId) {
    using Point = nie::io::LasPoint<
        pdal::Dimension::Id::X,
        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,
        pdal::Dimension::Id::Intensity,
        pdal::Dimension::Id::GpsTime>;

    ASSERT_EQ(Point::GetId<0>(), pdal::Dimension::Id::X);
    ASSERT_EQ(Point::GetId<1>(), pdal::Dimension::Id::Y);
    ASSERT_EQ(Point::GetId<2>(), pdal::Dimension::Id::Z);
    ASSERT_EQ(Point::GetId<3>(), pdal::Dimension::Id::Intensity);
    ASSERT_EQ(Point::GetId<4>(), pdal::Dimension::Id::GpsTime);
}