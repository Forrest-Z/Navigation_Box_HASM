/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <functional>  // std::reference_wrapper
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "csv_row_handler_tokenizer.hpp"
#include "csv_token_handler_parser.hpp"
#include "csv_tuple_handler.hpp"

namespace nie {

namespace detail {

/// Type trait function to convert tuple<Ts*...> into tuple<Ts&...>
template <typename... Ts, std::size_t... I>
constexpr auto MakeReferencesHelper(std::tuple<Ts...>& t, std::index_sequence<I...>) {
    return std::tie(*std::get<I>(t)...);
}
template <typename... Ts>
constexpr auto MakeReferences(std::tuple<Ts...>& t) {
    return MakeReferencesHelper<Ts...>(t, std::make_index_sequence<sizeof...(Ts)>{});
}

/// Trait to get element-type of vector.
template <typename...>
struct VectorElementType;
template <typename T>
struct VectorElementType<std::vector<T>> {
    using type = T;
};

template <typename Derived>
struct VectorFunctor {
    template <std::size_t Index = 0, typename T>
    void Evaluate(std::vector<T>& vec) {
        static_cast<Derived*>(this)->template Evaluate<Index>(vec);
    }
    template <std::size_t Index = 0, typename T>
    void Evaluate(std::vector<T> const& vec) {
        static_cast<Derived*>(this)->template Evaluate<Index>(vec);
    }
};

struct SizeVectorFunctor : VectorFunctor<SizeVectorFunctor> {
    template <std::size_t Index, typename T>
    void Evaluate(std::vector<T> const& vec) {
        vector_sizes.push_back(vec.size());
    }
    std::vector<std::size_t> vector_sizes;
};

struct ReserveVectorFunctor : VectorFunctor<ReserveVectorFunctor> {
    explicit ReserveVectorFunctor(std::size_t const& size) : size{size} {}
    template <std::size_t Index, typename T>
    void Evaluate(std::vector<T> const& vec) {
        vec.reserve(size);
    }
    std::size_t const size;
};

template <typename... Ts>
struct PushBackVectorFunctor : VectorFunctor<PushBackVectorFunctor<Ts...>> {
    explicit PushBackVectorFunctor(std::tuple<Ts...> const& row) : row{row} {}

    explicit PushBackVectorFunctor(std::tuple<Ts...>&& row) : row{std::move(row)} {}

    template <std::size_t Index, typename T>
    void Evaluate(std::vector<T>& vec) {
        vec.push_back(std::move(std::get<Index>(row)));
    }
    std::tuple<Ts...> row;
};

/// Trait for capturing type-pack.
template <typename...>
struct TypeList {};

/// Forward declaration.
/// Contains type-safe columns of csv data.
template <class>
struct CsvColumns_;

/// The implementation (bottom) of this function is a good example of how to use the csv parser.
/// \tparam Ts          Types per csv column
/// \param it_begin     Beginning of data (skip header)
/// \param it_end       End of data
/// \param delim        Delimiter by which values are separated in one line
/// \return             CsvColumns containing all data in csv file.
template <typename Tokenizer, typename... Ts>
CsvColumns_<TypeList<Ts...>> MakeCsvColumns(
        std::vector<std::string>::const_iterator it_begin,
        std::vector<std::string>::const_iterator it_end,
        char const& delim);

// TODO: method InsertRow.
// TODO: method DeleteRow.
// TODO: method InsertCol, this will create a new CsvColumns with one extra type. Move vectors, NO COPY!
// TODO: re-order columns.
// TODO: create row iterator.
/// Implement CsvTupleHandler interface so that it automatically appends incoming csv rows to itself.
template <template <class...> class TList, typename... Ts>
struct CsvColumns_<TList<Ts...>> : public CsvTupleHandler<Ts...> {
private:
    /// Tuple of vectors
    using VecTuple = std::tuple<std::vector<Ts>...>;

    /// Trait to get type of a column.
    template <std::size_t Index>
    struct VType_ {
        /// type of vector at column @tparam{Index}
        using v_type = std::tuple_element_t<Index, VecTuple>;
        /// type of element inside vector at column @tparam{Index}
        using type = typename VectorElementType<v_type>::type;
    };

    /// type of @tparam{Index}-th column.
    template <std::size_t Index>
    using ColType = typename VType_<Index>::type;

public:
    CsvColumns_() = default;

    /// Construct CsvColumns from a vector of strings. Each string is one line in csv file format with specific delim.
    template <typename Tokenizer = CsvRowHandlerTokenizerSafe>
    CsvColumns_(
            std::vector<std::string>::const_iterator it_begin,
            std::vector<std::string>::const_iterator it_end,
            char const& delim)
        : vec_tuple_{MakeCsvColumns<Tokenizer, Ts...>(it_begin, it_end, delim).vec_tuple_} {}

    /// Construct CsvColumns from vectors.
    explicit CsvColumns_(std::vector<Ts> const&... vectors) : vec_tuple_{vectors...} { AssertAllColsEqualSize(); }

    /// Construct CsvColumns from vectors.
    explicit CsvColumns_(std::vector<Ts>&&... vectors) : vec_tuple_{std::move(vectors)...} { AssertAllColsEqualSize(); }

    /// Copy constructor.
    CsvColumns_(CsvColumns_ const& other) : vec_tuple_{other.vec_tuple_} {}

    /// Copy assignment operator.
    CsvColumns_& operator=(CsvColumns_ const& other) {
        vec_tuple_ = other.vec_tuple_;
        return *this;
    }

    /// Move constructor.
    CsvColumns_(CsvColumns_&& other) noexcept : vec_tuple_{std::move(other.vec_tuple_)} {}

    /// Move assignment operator.
    CsvColumns_& operator=(CsvColumns_&& other) noexcept {
        vec_tuple_ = std::move(other.vec_tuple_);
        return *this;
    }

    /// Overridden virtual method from CsvTupleHandler.
    void operator()(std::tuple<Ts...>&& row) override { PushBack(std::move(row)); }

    /// Variadic push back.
    void PushBack(Ts&&... args) { PushBack(std::forward_as_tuple(args...)); }
    void PushBack(std::tuple<Ts...> const& row) {
        PushBackVectorFunctor<Ts...> push_back_vector_functor{row};
        ForEachCol(&push_back_vector_functor);
    }
    void PushBack(std::tuple<Ts...>&& row) {
        PushBackVectorFunctor<Ts...> push_back_vector_functor{std::move(row)};
        ForEachCol(&push_back_vector_functor);
    }

    /// Return vector containing all of the @tparam{Index}-th column.
    template <std::size_t Index>
    std::vector<ColType<Index>> const& GetCol() const {
        return std::get<Index>(vec_tuple_);
    }

    /// Return tuple of const references to objects inside one row.
    std::tuple<Ts const&...> GetRow(std::size_t const row_index) const {
        std::tuple<Ts const*...> row;
        CreateRowConst(row_index, &row);
        return MakeReferences(row);
    }
    /// Return tuple of references to objects inside one row.
    std::tuple<Ts&...> GetRow(std::size_t const row_index) {
        std::tuple<Ts*...> row;
        CreateRow(row_index, &row);
        return MakeReferences(row);
    }

    void Reserve(std::size_t const& size) {
        ReserveVectorFunctor vector_reserve_functor{size};
        ForEachCol(&vector_reserve_functor);
    }

    constexpr std::size_t NumCols() const { return sizeof...(Ts); }

    std::size_t NumRows() const { return GetCol<0>().size(); }

private:
    VecTuple vec_tuple_;

    /// Return vector containing all of the @tparam{Index}-th column.
    /// non-const method is hidden to enforce same size for all columns.
    template <std::size_t Index>
    std::vector<ColType<Index>>& GetColNonConst() {
        return std::get<Index>(vec_tuple_);
    }

    /// Assign values to tuple of pointers.
    template <std::size_t Index = 0>
    void CreateRowConst(
            std::size_t const row_index [[maybe_unused]], std::tuple<Ts const*...>* row [[maybe_unused]]) const {
        if constexpr (Index == sizeof...(Ts)) {
            return;
        } else {
            std::get<Index>(*row) = &GetCol<Index>().at(row_index);
            CreateRowConst<Index + 1>(row_index, row);
        }
    }

    /// Assign values to tuple of pointers.
    template <std::size_t Index = 0>
    void CreateRow(std::size_t const row_index [[maybe_unused]], std::tuple<Ts*...>* row [[maybe_unused]]) {
        if constexpr (Index == sizeof...(Ts)) {
            return;
        } else {
            std::get<Index>(*row) = &GetColNonConst<Index>().at(row_index);
            CreateRow<Index + 1>(row_index, row);
        }
    }

    /// Apply functor to each column
    template <std::size_t Index = 0, typename Functor>
    void ForEachCol(VectorFunctor<Functor>* functor [[maybe_unused]]) const {
        if constexpr (Index == sizeof...(Ts)) {
            return;
        } else {
            functor->template Evaluate<Index>(GetCol<Index>());
            ForEachCol<Index + 1>(functor);
        }
    }

    /// Apply functor to each column
    template <std::size_t Index = 0, typename Functor>
    void ForEachCol(VectorFunctor<Functor>* functor [[maybe_unused]]) {
        if constexpr (Index == sizeof...(Ts)) {
            return;
        } else {
            functor->template Evaluate<Index>(GetColNonConst<Index>());
            ForEachCol<Index + 1>(functor);
        }
    }

    /// Check if all columns have the same size
    /// Return true if all columns have equal size, else false.
    [[nodiscard]] bool AllColsEqualSize() const {
        SizeVectorFunctor vector_size_functor;
        ForEachCol(&vector_size_functor);
        for (std::size_t const& s : vector_size_functor.vector_sizes) {
            if (s != NumRows()) {
                return false;
            }
        }
        return true;
    }

    /// Throws logic_error if there are unequal size columns.
    void AssertAllColsEqualSize() const {
        if (!AllColsEqualSize()) {
            LOG(FATAL) << "Not allowed to create CsvColumn from unequally sized vectors.";
        }
    }
};

/// Typedef to omit typing TypeList<Ts...> everywhere
template <typename... Ts>
using CsvColumns = CsvColumns_<TypeList<Ts...>>;

/// Create a CsvColumns from strings.
/// Note:
///     Implementation of an almost complete csv-parsing pipeline.
///     It is not complete because the CsvHandler (first stage in pipeline) is not used here.
template <typename Tokenizer, typename... Ts>
CsvColumns_<detail::TypeList<Ts...>> MakeCsvColumns(
        std::vector<std::string>::const_iterator it_begin,
        std::vector<std::string>::const_iterator it_end,
        char const& delim) {
    // Create pipeline for constructing a CsvColumns.

    // Create CsvColumns which implements CsvTupleHandler.
    // It appends incoming csv rows (std::tuple) to itself.
    CsvColumns<Ts...> csv_columns{};

    // CsvTokenHandlerParser parses each string token to its data type.
    CsvTokenHandlerParser<Ts...> csv_token_handler_parser{&csv_columns};

    // CsvRowHandlerTokenizer splits a single row into tokens by delimiter.
    CsvRowHandlerTokenizer<Tokenizer> csv_row_handler_tokenizer{delim, &csv_token_handler_parser, sizeof...(Ts)};

    // Pass each row to the pipeline.
    for (auto it = it_begin; it != it_end; ++it) {
        csv_row_handler_tokenizer(*it);
    }

    return csv_columns;
}

}  // namespace detail

/// Typedef to expose CsvColumns
template <typename... Ts>
using CsvColumns = detail::CsvColumns<Ts...>;

}  // namespace nie
