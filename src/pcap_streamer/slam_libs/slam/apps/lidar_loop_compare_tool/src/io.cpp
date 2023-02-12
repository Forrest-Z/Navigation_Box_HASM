/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "io.hpp"

#include <glog/logging.h>
#include <nie/core/gflags.hpp>
#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_read_predicates.hpp>
#include <nie/formats/csv/csv_record.hpp>

namespace {

bool ConvertStringToBool(std::string s) {
    nie::ToLower(&s);
    if (s.empty() || s == "false" || s == "f" || s == "no" || s == "n" || s == "0") {
        return false;
    } else if (s == "true" || s == "t" || s == "yes" || s == "y" || s == "1") {
        return true;
    }
    LOG(FATAL) << "Cannot identify boolean value from string '" << s << "'";
}

nie::io::PoseCollection ReadFile(std::string const& path) {
    LOG(INFO) << "Reading loops file: " << path;
    auto pose = nie::io::ReadCollection<nie::io::PoseCollection>(path);
    return pose;
}

std::vector<nie::io::PoseCollection> ReadFiles(std::string const& paths) {
    std::vector<nie::io::PoseCollection> result;
    if (!paths.empty()) {
        auto path_strings = nie::Split<std::string>(paths, ',');
        result.reserve(path_strings.size());
        std::transform(path_strings.begin(), path_strings.end(), std::back_inserter(result), [](auto const& s) {
            return ReadFile(nie::Trim(s));
        });
    }
    return result;
}

}  // anonymous namespace

std::vector<PoseSet> ReadPoseFiles(
        std::string const& unfiltered_paths, std::string const& filtered_paths, std::string const& closure_paths) {
    std::vector<nie::io::PoseCollection> unfiltered_data = ReadFiles(unfiltered_paths);
    std::vector<nie::io::PoseCollection> filtered_data = ReadFiles(filtered_paths);
    std::vector<nie::io::PoseCollection> closure_data = ReadFiles(closure_paths);

    CHECK(unfiltered_data.size() == filtered_data.size()) << "Expecting just as many unfiltered as filtered data.";
    CHECK(filtered_data.size() == closure_data.size()) << "Expecting just as many filtered as closure data.";

    std::vector<PoseSet> result;
    result.reserve(unfiltered_data.size());
    for (std::size_t i = 0; i < unfiltered_data.size(); i++) {
        result.push_back({std::move(unfiltered_data[i]), std::move(filtered_data[i]), std::move(closure_data[i])});
    }
    return result;
}

ClosureGroundTruthMap ReadGroundTruth(std::string const& path) {
    ClosureGroundTruthMap result;

    // In the ground truth file the last value in a row can be omitted, like:
    //   id_1, id_2, true
    // as compared to
    //   id_1, id_2, true, true
    // Therefore the "fast" recorder is used as it uses the nie::Split function internally.
    nie::CsvRecorderFast<nie::io::PoseId, nie::io::PoseId, std::string, std::string> recorder{','};
    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({path}, "id_a,id_b,success,improved", &recorder);
    auto const records = recorder.ConvertToRecord();

    for (std::size_t i = 0; i < records.NumRows(); ++i) {
        auto [id_a, id_b, success, improved] = records.GetRow(i);
        auto const key = std::make_pair(id_a, id_b);
        bool const success_bool = ConvertStringToBool(success);
        bool const improved_bool = ConvertStringToBool(improved);

        auto iter = result.find(key);
        if (iter == result.end()) {
            result[key] = {success_bool, improved_bool};
        } else {
            CHECK(iter->second.success == success_bool && (success_bool || iter->second.improved == improved_bool))
                    << "Duplicate pair of pose id's found in the ground truth with different results: " << id_a << ","
                    << id_b;
        }
    }

    return result;
}

namespace {

struct Spec {
    static std::size_t const w_n = 27;  // name width
    static std::size_t const w_c = 12;  // count width
    static std::size_t const w_p = 6;   // percentage width
    static std::size_t const p_p = 1;   // percentage precision

    // cell total width
    static std::size_t w_ct() { return w_c + w_p + 5; }

    // table total width
    static std::size_t w_tt(std::size_t n) { return w_n + w_c + w_ct() * (n - 1) + 3; }
};

void PrintCountsRow(std::ostream& out, std::string const& name, std::vector<std::size_t> const& set) {
    if (set.empty()) {
        return;
    }

    out << std::showpos;
    out << std::setw(Spec::w_n) << name;
    for (std::size_t i = 0; i < set.size(); ++i) {
        auto const& s = set[i];
        out << std::setw(Spec::w_c) << s;
        if (i > 0) {
            out << "  (" << std::internal << std::setw(Spec::w_p) << std::setprecision(Spec::p_p) << std::fixed;
            out << (float(s) - set.front()) / set.front() * 100 << "%)";
        }
    }
    out << "\n";
    out << std::noshowpos;
}

void PrintGroundTruthRow(std::ostream& out, std::string const& name, GroundTruthCompareResult const& result) {
    auto const print_sub_count = [&out](std::size_t const ref, std::size_t const tst) -> std::ostream& {
        out << std::setw(Spec::w_c) << tst;
        out << "   (" << std::setw(Spec::w_p - 1) << std::setprecision(Spec::p_p) << std::fixed;
        return out << float(tst) / ref * 100 << "%)";
    };

    // Line 1
    out << std::setw(Spec::w_n) << name + " - same" << std::setw(Spec::w_c) << result.ref_count;
    for (auto const& compare_counts : result.test_counts) {
        print_sub_count(result.ref_count, compare_counts.same);
    }
    out << "\n";

    // Line 2
    out << std::setw(Spec::w_n) << " - diff" << std::setw(Spec::w_c) << "";
    for (auto const& compare_counts : result.test_counts) {
        print_sub_count(result.ref_count, compare_counts.diff);
    }
    out << "\n";
}

}  // anonymous namespace

void Statistics::Print(bool print_ground_truth, std::ostream& out) const {
    // Cosmetics helper specifications
    std::size_t const n = original_matches.size();
    auto const ruler = std::string(Spec::w_tt(n), '-') + '\n';

    out << "  Statistics\n";
    out << "    Counts and percentile differences/fractions in brackets\n";
    out << std::setw(Spec::w_n) << "" << std::setw(Spec::w_c) << "Ref";
    for (std::size_t i = 1; i < n; ++i) {
        out << std::setw(Spec::w_ct()) << i;
    }
    out << "\n" << ruler;
    PrintCountsRow(out, "original matches", original_matches);
    PrintCountsRow(out, "filtered matches", filtered_matches);
    PrintCountsRow(out, "succeeded matches", succeeded_matches);
    if (print_ground_truth) {
        out << ruler;
        PrintGroundTruthRow(out, "confirmed matches", gt_confirmed_matches);
        PrintGroundTruthRow(out, "improved matches", gt_improved_matches);
        PrintGroundTruthRow(out, "failed matches", gt_failed_matches);
    }
    out << ruler;
}
