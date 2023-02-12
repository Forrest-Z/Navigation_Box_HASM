/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <nie/core/spatial_grid.hpp>

#include <gtest/gtest.h>

namespace {

using Ints = std::pair<int, int>;

class SpatialGridInts {
private:
    using Grid = nie::SpatialGrid<Ints, 2, int>;

public:
    explicit SpatialGridInts(double cell_size) : grid(cell_size) {}

    void Add(Ints const& ints) { grid.Add(ints, std::bind(&SpatialGridInts::Get, std::placeholders::_1)); }

    void Add(std::vector<Ints> const& ints_vector) {
        grid.Add(ints_vector, std::bind(&SpatialGridInts::Get, std::placeholders::_1));
    }

    Grid grid;

private:
    static Grid::CoordVector Get(Ints const& ints) { return {ints.first, ints.second}; }
};

}  // namespace

// Actual tests to be executed

TEST(SpatialGridTest, GetFromCellsOneCellGrid) {
    // One full square cell
    for (int size : {1, 3, 5, 8, 11, 16}) {
        std::vector<Ints> features;
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                features.emplace_back(i, j);
            }
        }

        SpatialGridInts spatial_grid(size);
        spatial_grid.Add(features);

        std::vector<Ints> indices;
        spatial_grid.grid.GetBox({size / 2., size / 2.}, 1., &indices);
        ASSERT_EQ(indices.size(), std::pow(size, 2));

        // Test the reduction functionality of no reduction
        spatial_grid.grid.Reduce(
                static_cast<int>(std::pow(size, 2)), nie::SpatialGridReductionStrategy::kRandom, &indices);
        ASSERT_EQ(indices.size(), std::pow(size, 2));
    }
}

TEST(SpatialGridTest, GetFromCellsMultipleCellGrid) {
    struct Test {
        int size;
        int bin;
        int max_amount;
    };
    std::vector<Test> tests = {// total size, bin size, reduction test amount
                               {6, 2, 3},
                               {25, 5, 15},
                               {8, 4, 10}};

    for (Test const& test : tests) {
        std::vector<Ints> features;
        for (int i = 0; i < test.size; ++i) {
            for (int j = 0; j < test.size; ++j) {
                features.emplace_back(i, j);
            }
        }

        SpatialGridInts spatial_grid(test.bin);

        spatial_grid.Add(features);

        // Test the filling of one bin
        std::vector<Ints> indices;
        spatial_grid.grid.GetBox({test.size / 2., test.size / 2.}, test.bin / 2., &indices);
        // Depending on the situation, the number of bins selected can be just
        // one (one by one), two (one by two and two by one) or four (two by two)
        ASSERT_TRUE(
                indices.size() == 1 * std::pow(test.bin, 2) || indices.size() == 2 * std::pow(test.bin, 2) ||
                indices.size() == 4 * std::pow(test.bin, 2));

        // Test the complete filling
        spatial_grid.grid.GetBox({test.size / 2., test.size / 2.}, test.size / 2., &indices);
        ASSERT_EQ(indices.size(), static_cast<int>(std::pow(test.size, 2)));

        // Test the reduction functionality
        spatial_grid.grid.Reduce(test.max_amount, nie::SpatialGridReductionStrategy::kRandom, &indices);
        ASSERT_EQ(indices.size(), static_cast<int>(std::pow(test.size / test.bin, 2) * test.max_amount));
    }
}
