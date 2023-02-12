/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <random>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>
#include <Eigen/Core>

#include "algorithm.hpp"
#include "eigen.hpp"
#include "hash.hpp"

namespace nie {

enum class SpatialGridReductionStrategy {
    kFirst,
    kRandom,
};

/// This class will create a fixed-size grid internally with cells having a certain size and add objects to the
/// appropriate one, which can be requested subsequently.
///
/// This class assumes the first number to correspond to the horizontal/row/x coordinate, the width and so on.
template <typename Object, int Dim = 2, typename Scalar = double>
class SpatialGrid {
    static_assert(Dim != Eigen::Dynamic, "Dynamic size not implemented.");

public:
    using CoordVector = Eigen::Array<Scalar, Dim, 1>;
    using Indices = Eigen::Array<int, Dim, 1>;

    /// Create an grid object.
    explicit SpatialGrid(Scalar const& cell_size)
        : cells_(), cell_size_(CoordVector::Constant(cell_size)), number_of_elements_(0) {}
    explicit SpatialGrid(CoordVector const& cell_size) : cells_(), cell_size_(cell_size), number_of_elements_(0) {}

    /// Clear all grid cells.
    void Clear() {
        if (number_of_elements_ > 0) {
            cells_.clear();
            number_of_elements_ = 0;
        }
    }

    /// Add an object to the grid.
    template <typename Input = Object, typename Getter>
    void Add(Input input, Getter const& retrieve_coordinates_function) {
        static_assert(
                std::is_invocable_r_v<CoordVector, Getter, Input>,
                "The coordinate retrieval function should take Input and return CoordVector.");
        CoordVector const p = retrieve_coordinates_function(input);

        Indices const indices = GetIndices(p);
        // Add object to the appropriate cell
        GetCell(indices).push_back(std::move(input));

        // Increase count
        number_of_elements_++;
    }

    /// Add all objects from the vector to the grid.
    template <typename Input = Object, typename InputAllocator, typename Getter>
    void Add(std::vector<Input, InputAllocator> inputs, Getter const& retrieve_coordinates_function) {
        // Add every object to the appropriate cell
        for (auto& input : inputs) {
            Add(std::move(input), retrieve_coordinates_function);
        }
    }

    /// Get bins with all objects. Column-major.
    [[nodiscard]] std::vector<std::vector<Object>> GetBinnedData() const {
        std::vector<std::vector<Object>> result;
        for (auto const& c : cells_) {
            result.push_back(c.second);
        }
        return result;
    }

    /// Returns a container of the positional indices of all objects that are stored in the requested cells. The cells
    /// requested are based on the central position and the distance from that point.
    ///
    /// Note that all objects from the requested cells are returned, not only the ones that are actually within that
    /// distance.
    void GetBox(CoordVector const& coord, Scalar const& distance, std::vector<Object>* result) const {
        return GetBox(coord, CoordVector::Constant(distance), result);
    }
    void GetBox(CoordVector const& coord, CoordVector const& distance, std::vector<Object>* result) const {
        CHECK((distance.cwiseAbs() == distance).any()) << "Distance vector can only contain positive values.";

        // When retrieving the cells, every cell should only be accessed once, therefore at this stage all cells are
        // identified
        std::vector<Indices> const key_list = GetIndices(coord - distance, coord + distance);
        result->clear();
        for (Indices const& key : key_list) {
            // Check if the cell exists
            auto const iter = cells_.find(key);
            if (iter == cells_.cend()) {
                continue;
            }
            std::vector<Object> const& cell_content = iter->second;
            if (not cell_content.empty()) {
                std::copy(cell_content.begin(), cell_content.end(), std::back_inserter(*result));
            }
        }
    }

    /// Returns a selection of all objects from all cells. The selection is based on the given maximum number of objects
    /// to be returned for every cell.
    void Reduce(
            std::size_t const& max_amount,
            SpatialGridReductionStrategy const& strategy,
            std::vector<Object>* result) const {
        // Make copy to potentially perform shuffle and move elements to result vector
        // TODO: Do more efficiently by shuffling a vector of iterators/indices and then only copy what is needed
        result->clear();
        for (auto cell : cells_) {
            if (cell.second.empty()) {
                continue;
            }

            std::size_t size = cell.second.size();
            if (size > max_amount) {
                if (strategy == SpatialGridReductionStrategy::kRandom) {
                    std::shuffle(
                            cell.second.begin(),
                            cell.second.end(),
                            std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
                }
            }

            // Items are copied already and so can be moved.
            std::move(
                    cell.second.begin(), cell.second.begin() + std::min(max_amount, size), std::back_inserter(*result));
        }
    }

private:
    // Class used to calculate all cell indices in the given range
    // If the range is [2, 3], then the resulting list of indices is:
    //     [0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2]
    class IndicesRange {
    private:
        using IndicesDouble = Eigen::Array<double, Dim, 1>;

    public:
        explicit IndicesRange(Indices const& range)
            : size_(range.prod()),
              rev_cum_prod_{RevCumProd(range).template cast<double>()},
              rev_cum_prod_shifted_{(Eigen::ArrayXd(Dim) << rev_cum_prod_.template tail<Dim - 1>(), 1.).finished()} {}

        typename Indices::Scalar Size() const { return size_; }

        // Asking for an index >= Size will just repeat the sequence due to the modulo calcuation
        Indices operator()(int i) const {
            IndicesDouble const mod =
                    nie::EigenModulo(IndicesDouble::Constant(i), rev_cum_prod_) / rev_cum_prod_shifted_;
            return mod.floor().template cast<typename Indices::Scalar>();
        }

    private:
        // Helper function to calculate the reverse cumulative product of an array.
        // The following is calculated: [x, y, z] -> [xyz, yz, z]
        static Indices RevCumProd(Indices array) {
            for (std::size_t i = 0; i < Dim - 1; ++i) {
                array[i] = array.tail(Dim - i).prod();
            }
            return array;
        }

        typename Indices::Scalar const size_;
        IndicesDouble const rev_cum_prod_;
        IndicesDouble const rev_cum_prod_shifted_;
    };

    // Function to calculate all cell indices in the range based on the supplied coordinates.
    // Also see the IndicesRange class. Function basically calculates min + IndicesRange(max - min).
    [[nodiscard]] std::vector<Indices> GetIndices(CoordVector const& min_coord, CoordVector const& max_coord) const {
        Indices const min = GetIndices(min_coord);
        IndicesRange const r{GetIndices(max_coord) - min + 1};

        std::vector<Indices> indices(r.Size());
        for (typename Indices::Scalar i = 0; i < r.Size(); ++i) {
            indices[i] = min + r(i);
        }
        return indices;
    }

    /// Functions to calculate the cell indices based on the supplied coordinates
    [[nodiscard]] Indices GetIndices(CoordVector const& coord) const {
        return (coord / cell_size_).round().template cast<int>();
    }

    /// The generic way to directly get a cells content based on the supplied indices in the order "column, row"
    std::vector<Object>& GetCell(Indices const& indices) {
        // Insert returns {iterator, bool}, iterator (first) points to map, which we want to have the value of (second)
        return (*cells_.insert({indices, {}}).first).second;
    }
    [[nodiscard]] std::vector<Object> const& GetCell(Indices const& indices) const {
        auto cell_it = cells_.find(indices);
        // Check and fatal, otherwise this function will crash. If cells_[key] does not exist, we cannot create it
        // because it is a const function.
        CHECK(cell_it != cells_.end()) << "Tried to access nonexistent cell";
        return cell_it->second;
    }

    /// A unordered_map container as storage of the spatial grid cells. A cell can be addressed using
    /// "cells_[Indices]" which will return the cells content, a vector of the given object.
    std::unordered_map<Indices, std::vector<Object>, nie::EigenHash, nie::EigenEqual> cells_;

    /// Some specifications and statistics about the grid
    CoordVector cell_size_;
    std::size_t number_of_elements_;
};

/// Convenience function that takes inputs from a container, but stores the indices
template <std::size_t Dim = 2, typename Scalar = double, typename Object, typename ObjectAllocator, typename Getter>
SpatialGrid<std::size_t, Dim, Scalar> CreateIndicesSpatialGrid(
        Scalar const& cell_size,
        std::vector<Object, ObjectAllocator> const& objects,
        Getter const& retrieve_coordinates_function,
        std::vector<bool> const& mask = {}) {
    bool const mask_empty = mask.empty();
    assert(mask_empty || objects.size() == mask.size());

    SpatialGrid<std::size_t, Dim, Scalar> grid(cell_size);
    auto const f = [&objects, &retrieve_coordinates_function](std::size_t const i) {
        return retrieve_coordinates_function(objects[i]);
    };
    for (std::size_t index = 0; index < objects.size(); ++index) {
        if (mask_empty || mask[index]) {
            grid.Add(index, f);
        }
    }
    return grid;
}

}  // namespace nie
