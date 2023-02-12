/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/collection_writer.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>

/// This app reads a csv file where each line is an edge which are added to a pose file.

DEFINE_string(in_out_file_pose, "", "Filepath to in- and output .pose file.");
DEFINE_string(in_file_csv_loops, "", "Filepath to input .csv file containing the loops detected.");
DEFINE_string(in_file_csv_merge_info, "", "Filepath to input .csv file containing the relative scaling factor.");

DEFINE_validator(in_out_file_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_csv_loops, nie::ValidateIsFile);

template <typename... Ts>
struct EdgeAppender : public nie::CsvTupleHandler<Ts...> {
    explicit EdgeAppender(double const factor, nie::io::PoseCollection* pose_collection)
        : factor{factor}, pose_collection{pose_collection} {}

    /// Overridden virtual method from CsvTupleHandler.
    void operator()(std::tuple<Ts...>&& row) override {
        std::int32_t id_begin = std::get<0>(row);
        std::int32_t id_end = std::get<1>(row);
        nie::Isometry3qd isometry{{std::get<2>(row), std::get<3>(row), std::get<4>(row)},
                                  {std::get<5>(row), std::get<6>(row), std::get<7>(row), std::get<8>(row)}};
        isometry = nie::ConvertFrame<nie::Frame::kCv, nie::Frame::kAircraft>(isometry);
        if (factor != 1.) {
            isometry.translation() *= factor;
        }
        // clang-format off
        Eigen::Matrix<double, 6, 6> information = (Eigen::MatrixXd(6, 6) <<
            std::get< 9>(row), std::get<10>(row), std::get<11>(row), std::get<12>(row), std::get<13>(row), std::get<14>(row),
                           0., std::get<15>(row), std::get<16>(row), std::get<17>(row), std::get<18>(row), std::get<19>(row),
                           0.,                0., std::get<20>(row), std::get<21>(row), std::get<22>(row), std::get<23>(row),
                           0.,                0.,                0., std::get<24>(row), std::get<25>(row), std::get<26>(row),
                           0.,                0.,                0.,                0., std::get<27>(row), std::get<28>(row),
                           0.,                0.,                0.,                0.,                0., std::get<29>(row)
            ).finished();
        // clang-format on
        information.triangularView<Eigen::StrictlyLower>() = information.transpose();

        DVLOG(6) << "Read edge " << id_begin << ", " << id_end;

        pose_collection->edges.push_back(
            {id_begin, id_end, nie::io::PoseEdgeRecord::Category::kLoop, isometry, information});
    }

    double const factor;
    nie::io::PoseCollection* pose_collection;
};

template <typename... Ts>
struct CsvToEdgeRecorder : public nie::CsvHandler {
    explicit CsvToEdgeRecorder(EdgeAppender<Ts...> edge_appender, char delim = ',')
        : edge_appender_{std::move(edge_appender)},
          csv_token_handler_parser_{&edge_appender_},
          csv_row_handler_tokenizer_{delim, &csv_token_handler_parser_, sizeof...(Ts)} {}

    /// Handle vector of csv rows.
    void operator()(
        std::vector<std::string>::const_iterator begin, std::vector<std::string>::const_iterator end) override {
        // Pass each row to the pipeline.
        for (auto it = begin; it != end; ++it) {
            csv_row_handler_tokenizer_(*it);
        }
    }

    void SetHeader(std::string const&) override {}

private:
    EdgeAppender<Ts...> edge_appender_;
    nie::CsvTokenHandlerParser<Ts...> csv_token_handler_parser_;
    nie::CsvRowHandlerTokenizer<nie::CsvRowHandlerTokenizerFast> csv_row_handler_tokenizer_;
};

double ReadFactor(std::string const& file_name) {
    std::ifstream ifs(file_name);
    std::string line;
    while (getline(ifs, line)) {
        std::vector<std::string> parts = nie::Split<std::string>(line, ',');
        if (parts.empty()) {
            // Empty line
            continue;
        }
        if (parts.front() == "scale") {
            return std::stod(parts[1]);
        }
    }
    LOG(ERROR) << "No scaling factor found in csv.";
    return -1.;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Reading pose file: " << FLAGS_in_out_file_pose;
    nie::io::PoseCollection pose_collection = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_out_file_pose);

    // clang-format off
    EdgeAppender<
        int, int, // pose_id
        double, double, double, // x y z
        double, double, double, double, // q0 q1 q2 q3
        double, double, double, double, double, double, // triangular information matrix
        double, double, double, double, double,
        double, double, double, double,
        double, double, double,
        double, double,
        double
    >
    edge_appender{
        (FLAGS_in_file_csv_merge_info.empty() ? 1. : ReadFactor(FLAGS_in_file_csv_merge_info)),
        &pose_collection};
    // clang-format on

    CsvToEdgeRecorder csv_to_edge_recorder{edge_appender};

    nie::ReadAllCsvToHandler<nie::ReadLinesAfterNthLinePredicate>({FLAGS_in_file_csv_loops}, -1, &csv_to_edge_recorder);

    LOG(INFO) << "Writing updated pose to file: " << FLAGS_in_out_file_pose;
    nie::io::Write(pose_collection, FLAGS_in_out_file_pose);
}
