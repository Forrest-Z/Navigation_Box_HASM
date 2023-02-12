/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <array>
#include <vector>

#include <Eigen/Geometry>
#include <nanoflann.hpp>
#include <opencv2/opencv.hpp>

namespace nie {

namespace detail {

template <typename T>
struct OpenCvTraits;
template <typename Scalar_>
struct OpenCvTraits<cv::Point_<Scalar_>> {
    using Scalar = Scalar_;
    static constexpr int Dims = 2;
    static inline Scalar const* Ptr(cv::Point_<Scalar_> const& p) { return &p.x; }
};
template <typename Scalar_>
struct OpenCvTraits<cv::Point3_<Scalar_>> {
    using Scalar = Scalar_;
    static constexpr int Dims = 3;
    static inline Scalar const* Ptr(cv::Point3_<Scalar_> const& p) { return &p.x; }
};
template <typename Scalar_, int Dims_>
struct OpenCvTraits<cv::Vec<Scalar_, Dims_>> {
    using Scalar = Scalar_;
    static constexpr int Dims = Dims_;
    static inline Scalar const* Ptr(cv::Vec<Scalar_, Dims_> const& p) { return p.val; }
};

}  // namespace detail

template <typename P>
class OpenCvKdTreeAdapter {
public:
    using Scalar = typename detail::OpenCvTraits<P>::Scalar;
    static constexpr int Dims = detail::OpenCvTraits<P>::Dims;
    explicit OpenCvKdTreeAdapter(std::vector<P> const& pts) : pts_{pts} {}
    inline std::size_t kdtree_get_point_count() const { return pts_.size(); }
    /// Returns the dim'th component of the idx'th point in the class.
    inline Scalar kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
        return detail::OpenCvTraits<P>::Ptr(pts_[idx])[dim];
    }
    template <typename U>
    inline Scalar const* Ptr(U const& p) const {
        return detail::OpenCvTraits<U>::Ptr(p);
    }
    /// Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
        return false;
    }

private:
    std::vector<P> const& pts_;
};

template <typename EigenMatrixType>
class EigenKdTreeAdapter {
public:
    using Scalar = typename EigenMatrixType::Scalar;

    explicit EigenKdTreeAdapter(EigenMatrixType const& pts) : pts_{pts} {}

    inline std::size_t kdtree_get_point_count() const {
        if constexpr (EigenMatrixType::IsRowMajor) {
            return pts_.rows();
        } else {
            return pts_.cols();
        }
    }

    /// Returns the dim'th component of the idx'th point in the class:
    /// Since this is inlined and the "dim" argument is typically an immediate value, the
    /// "if/else's" are actually solved at compile time.
    inline Scalar kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
        if constexpr (EigenMatrixType::IsRowMajor) {
            return pts_(idx, dim);
        } else {
            return pts_(dim, idx);
        }
    }

    template <typename U>
    inline Scalar const* Ptr(U const& p) const {
        return p.data();
    }

    /// Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
        return false;
    }

private:
    EigenMatrixType const& pts_;
};

///
/// typedefs to avoid a long line of code for just one declaration when instantiating a KDTree
///
template <typename AXX, typename BXX>
using KD_METRIC_L2 = typename nanoflann::L2_Simple_Adaptor<AXX, BXX>;

///
///
/// @tparam T  type of elements
/// @tparam D  dimension
/// @tparam M  nanoflann metric (e.g. nanoflann::L2_Simple_Adaptor<T, AdapterType>)
///
template <typename Scalar, typename AdapterType, size_t D, template <class, class> class M = KD_METRIC_L2>
class KdTree {
public:
    using MatchType = std::pair<size_t, Scalar>;

    ///
    /// @tparam U  type of input data, forwarded to FlannAdapterFactory.
    /// @param data  input data
    /// @param max_leaf_size  https://github.com/jlblancoc/nanoflann#2-any-help-choosing-the-kd-tree-parameters
    ///
    KdTree(AdapterType const& adapter, size_t max_leaf_size)
        : adapter_(adapter), index_tree_(D, adapter_, nanoflann::KDTreeSingleIndexAdaptorParams(max_leaf_size)) {
        index_tree_.buildIndex();
    }

    ///
    /// This is different from the nanoflann interface so that @tparam{MatchType}
    /// is private and we can use auto for the return value of this function.
    /// @param query  point
    /// @param radius  search distance from query point
    /// @return vector of matches (USE auto FOR return value!)
    /// @tparam{MatchType} is always std::pair<size_t, Scalar> (pair of index and distance)
    ///
    template <typename U>
    size_t RadiusSearch(U const& point, Scalar radius, std::vector<MatchType>* result) const {
        Scalar const* query = adapter_.Ptr(point);
        return index_tree_.radiusSearch(query, radius, *result, nanoflann::SearchParams());
    }

    ///
    /// This method computes the norm of a value.
    /// The object "distance" contains a method "accum_dist" which should be static but it is only inline!
    /// So when the nanoflann guys come to their senses we can just do the following:
    ///      value = M<T, AdapterType>::acumm_dist(value, T(0), size_t());
    /// @param value  input/output value
    ///
    Scalar NormalizeValue(Scalar value, Scalar other = Scalar{0}) const {
        return index_tree_.distance.accum_dist(value, other, size_t());
    }

private:
    using KdTreeType = typename nanoflann::KDTreeSingleIndexAdaptor<M<Scalar, AdapterType>, AdapterType, D>;
    AdapterType const& adapter_;
    KdTreeType index_tree_;
};

}  // namespace nie
