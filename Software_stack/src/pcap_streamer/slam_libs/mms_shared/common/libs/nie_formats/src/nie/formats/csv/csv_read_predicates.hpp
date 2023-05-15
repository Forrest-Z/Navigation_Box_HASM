/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <regex>
#include <string>
#include <tuple>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace nie {

/// CRTP base class for any predicate functor that conforms to bool operator()(std::string const&)
template <class Derived>
struct ReadLinePredicate {
    ReadLinePredicate() = default;
    virtual bool operator()(std::string const& line) { return static_cast<Derived&>(*this)(line); }
};

template <class... Predicates_>
class PredicateList : public ReadLinePredicate<PredicateList<Predicates_...>> {
public:
    explicit PredicateList(Predicates_... predicates) : predicates_{std::forward<Predicates_>(predicates)...} {}

    /// This method works as a foreach loop over all tuple elements.
    /// Calls operator() on each predicate sequentially.
    /// If one returns false then the entire PredicateList returns false.
    template <size_t Index = 0>
    bool operator()(std::string const& line) {
        if constexpr (Index == sizeof...(Predicates_)) {
            return true;
        } else {
            bool result = std::get<Index>(predicates_)(line);
            if (result) {
                // If the current predicate holds true, try the next one.
                result = operator()<Index + 1>(line);
            }
            return result;
        }
    }

    template <size_t Index>
    auto GetPredicate() -> std::tuple_element_t<Index, std::tuple<Predicates_...>>& {
        return std::get<Index>(predicates_);
    }

    template <size_t Index>
    auto GetPredicate() const -> std::tuple_element_t<Index, std::tuple<Predicates_...>> const& {
        return std::get<Index>(predicates_);
    }

private:
    std::tuple<Predicates_...> predicates_;
};

/// Predicate for ACCEPTING a string if it matches with a regex pattern.
struct RegexAcceptPredicate : ReadLinePredicate<RegexAcceptPredicate> {
    explicit RegexAcceptPredicate(std::string const& pattern_string) : pattern{pattern_string} {}
    /// Everything that matches is accepted.
    bool operator()(std::string const& line) {
        std::smatch what;
        return std::regex_match(line, what, pattern);
    }

    std::regex pattern;
};

/// Predicate for REJECTING a string if it matches with a regex pattern.
struct RegexRejectPredicate : ReadLinePredicate<RegexRejectPredicate> {
    explicit RegexRejectPredicate(std::string const& pattern_string) : pattern{pattern_string} {}
    /// Everything that matches is rejected.
    bool operator()(std::string const& line) {
        std::smatch what;
        return !std::regex_match(line, what, pattern);
    }

    std::regex pattern;
};

/// Predicate for accepting a string only if it was read after the header line.
struct ReadLinesAfterHeaderPredicate : ReadLinePredicate<ReadLinesAfterHeaderPredicate> {
    explicit ReadLinesAfterHeaderPredicate(std::string h_line, bool also_read_h = false)
        : header_line_{std::move(h_line)}, also_read_header_{also_read_h}, header_was_seen_{false} {}

    bool operator()(std::string const& line) {
        if (header_was_seen_) {
            // Already passed the header line.
            return true;
        } else if (line == header_line_) {
            // Header line seen, read all lines from now on.
            header_was_seen_ = true;
            return also_read_header_;
        } else {
            // This is not yet the header line.
            return false;
        }
    }

private:
    std::string const header_line_;
    bool also_read_header_;
    bool header_was_seen_ = false;
};

/// Predicate for accepting a string only if it was read after the n-th line.
struct ReadLinesAfterNthLinePredicate : ReadLinePredicate<ReadLinesAfterNthLinePredicate> {
    explicit ReadLinesAfterNthLinePredicate(int h_line_no, bool also_read_nth = false)
        : header_line_no_{h_line_no}, also_read_nth_{also_read_nth}, count_{0} {}

    bool operator()(std::string const& /*line*/) {
        if (count_ > header_line_no_) {
            // Already passed the n-th line.
            return true;
        } else if (count_ == header_line_no_) {
            ++count_;
            return also_read_nth_;
        } else {
            ++count_;
            return false;
        }
    }

private:
    int header_line_no_;
    bool also_read_nth_;
    int count_ = 0;
};

}  // namespace nie
