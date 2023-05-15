/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "match.hpp"

#include <algorithm>
#include <cassert>

#include <nie/core/algorithm.hpp>

namespace nie {

void FilterMatchVector(std::vector<bool> const& filter, MatchVector* p_matches) {
    assert(p_matches != nullptr);
    assert(p_matches->size() == filter.size());

    /*
     * Two notes on the implementation below
     *   - remove_if does not guarantee sequential execution of the predicate,
     *     therefore stable_partition is used.
     *   - When the iterator is put inside the lambda captures '[]', then
     *     somehow the iterator is one ahead in the iteration.
     */
    auto filter_iter = filter.begin();
    p_matches->erase(
        std::stable_partition(  // puts 'true' before 'false'
            p_matches->begin(),
            p_matches->end(),
            [&filter_iter](FeatureMatch const&) {
                bool result = not*filter_iter;
                ++filter_iter;
                return result;
            }),
        p_matches->end());
}

void FindLinkedMatches(
    MatchVector const& a,
    MatchVector const& b,
    std::vector<std::size_t>* a_indices,
    std::vector<std::size_t>* b_indices) {
    assert(a_indices != nullptr);
    assert(b_indices != nullptr);

    if (a.empty() or b.empty()) {
        return;
    }

    for (std::size_t index = 0; index < a.size(); ++index) {
        auto b_iter = std::find_if(b.begin(), b.end(), [a_item = a[index]](FeatureMatch const& b_item) {
            return a_item.index_b == b_item.index_a;
        });
        if (b_iter != b.end()) {
            a_indices->push_back(index);
            b_indices->push_back(std::distance(b.begin(), b_iter));
        }
    }
}

MatchVector ConcatenateMatches(MatchVector const& a, MatchVector const& b) {
    std::vector<std::size_t> a_indices, b_indices;
    FindLinkedMatches(a, b, &a_indices, &b_indices);

    if (a_indices.empty() or b_indices.empty()) {
        return {};
    }

    MatchVector result;
    result.reserve(a_indices.size());
    for (std::size_t i = 0; i < a_indices.size(); ++i) {
        result.emplace_back(a[a_indices[i]].index_a, b[b_indices[i]].index_b);
    }
    return result;
}

std::vector<std::vector<std::size_t>> ChainMatches(std::vector<MatchVector> const& vectors) {
    if (vectors.empty()) {
        return {};
    }

    MatchVector const& l = vectors.front();
    std::vector<std::vector<std::size_t>> result{l.size(), std::vector<std::size_t>{}};
    for (std::size_t i = 0; i < l.size(); ++i) {
        FeatureMatch const& m = l[i];
        result[i] = {m.index_a, m.index_b};
    }

    for (auto v = vectors.cbegin() + 1; v != vectors.cend(); ++v) {
        MatchVector const& r = *v;

        std::vector<std::size_t> l_indices, r_indices;
        FindLinkedMatches(l, r, &l_indices, &r_indices);
        assert(l_indices.size() == r_indices.size());

        std::vector<std::vector<std::size_t>> tmp_result{l_indices.size(), std::vector<std::size_t>{}};
        nie::CopyIf(result.begin(), result.end(), tmp_result.begin(), l_indices.begin(), l_indices.end());
        for (std::size_t i = 0; i < r_indices.size(); ++i) {
            tmp_result[i].push_back(r[r_indices[i]].index_b);
        }

        std::swap(result, tmp_result);
    }
    return result;
}

}  // namespace nie
