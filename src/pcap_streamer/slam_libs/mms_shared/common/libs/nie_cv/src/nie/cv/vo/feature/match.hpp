/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_CV_VO_FEATURE_MATCH_HPP
#define NIE_CV_VO_FEATURE_MATCH_HPP

#include <vector>

namespace nie {

class FeatureMatch {
public:
    explicit FeatureMatch(std::size_t a, std::size_t b) : index_a(a), index_b(b) {}

    bool operator<(FeatureMatch const& other) const {
        bool result = index_a < other.index_a;
        if (index_a == other.index_a) {
            result = index_b < other.index_b;
        }
        return result;
    }

    std::size_t index_a;
    std::size_t index_b;
};
using MatchVector = std::vector<FeatureMatch>;

///
/// This function will remove entries from the match vector based on the filling of the boolean vector WITHOUT
/// preserving the order. When a filter entry is true, then the corresponding match (at the same position in the vector)
/// will be removed. Note that the arguments are in- and outputs.
///
/// @param[in]     filter    The filtering that will be applied to the matches
/// @param[in,out] matches   The feature matches that will be filtered.
///
void FilterMatchVector(std::vector<bool> const& filter, MatchVector* p_matches);

///
/// Indices of matches in vectors a and b are returned when item in a has index_b the same as index_a of the item in b.
/// At least vector b must be sorted.
///
void FindLinkedMatches(
    MatchVector const& a,
    MatchVector const& b,
    std::vector<std::size_t>* a_indices,
    std::vector<std::size_t>* b_indices);

///
/// Combine the vectors a and b by linking an item in a with index_b being the same as index_a of the item in b. The
/// returned result is the combined match vector.
/// Note that at least vector b must be sorted. Order of vector a will be preserved.
///
MatchVector ConcatenateMatches(MatchVector const& a, MatchVector const& b);

///
/// Return a vector of features matches across frames, every entry contains indices of features in every frames, so
/// there will be 'vectors.size() + 1' indices per entry.
///
/// All MatchVectors must be sorted.
///
std::vector<std::vector<std::size_t>> ChainMatches(std::vector<MatchVector> const& vectors);

}  // namespace nie

#endif  // NIE_CV_VO_FEATURE_MATCH_HPP
