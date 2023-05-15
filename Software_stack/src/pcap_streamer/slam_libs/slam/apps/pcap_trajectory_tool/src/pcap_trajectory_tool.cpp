/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <iomanip>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/container/mt_vector.hpp>
#include <nie/core/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/core/work_pool.hpp>
// TODO: Add support for ouster_streamer
// This can be done via a flag
#include <nie/lidar/io/streamer.hpp>
#include <nie/lidar/time_intervals/time_intervals.hpp>
#include <nie/lidar/time_intervals/time_intervals_reader.hpp>

#include "pcap_meta.hpp"

DEFINE_string(in_dir, "", "Input directory containing pcap files.");
DEFINE_string(out_dir, "", "Output directory for writing .txt files (track files and skipped file).");
DEFINE_string(
        pcap_pattern,
        ".*.pcap",
        "(Optional) Regex pattern for finding input Velodyne lidar recordings, in pcap format.");
DEFINE_string(in_lidar_intrinsics_file, "", "Filepath to input lidar intrinsics calibration parameters.");
DEFINE_string(in_lidar_extrinsics_file, "", "Filepath to input lidar exntrinsics calibration parameters.");
DEFINE_string(lidar_id, "", "Identifier of lidar to be used.");
DEFINE_string(
        in_stationary_intervals, "", "Filepath to csv file with list of intervals when the vehicle was not moving.");
DEFINE_uint64(max_pcap_time_diff_us, 1e6, "Maximum number of microseconds between two consecutive pcap files.");
DEFINE_uint64(
        max_packet_time_diff_us,
        1e5,
        "Maximum number of microseconds between two consecutive pcap packets within a file.");
DEFINE_uint64(nthreads, 0, "Number of threads to use for processing pcap files.");
DEFINE_bool(check_overlap, false, "Check if consecutive pcap files are overlapping in time.");
DEFINE_bool(report_info, false, "Print pcap meta info for each pcap file.");

DEFINE_validator(in_dir, nie::ValidateIsDirectory);
DEFINE_validator(out_dir, nie::ValidateIsDirectory);
DEFINE_validator(in_lidar_intrinsics_file, nie::ValidateIsFile);
DEFINE_validator(in_stationary_intervals, nie::ValidateIsFile);
DEFINE_validator(pcap_pattern, nie::ValidateStringNotEmpty);
DEFINE_validator(max_pcap_time_diff_us, nie::ValidateLargerThanZero);
DEFINE_validator(max_packet_time_diff_us, nie::ValidateLargerThanZero);

/// This app takes a set of pcap files as input and verifies which pcap files form a consecutive recording.
/// Output is multiple txt files, each containing a list of filepaths to pcap files. The filepaths in the
/// txt file are sorted by lidar timestamps in ascending order.

struct GapChecker {
    GapChecker(
            nie::Timestamp_ns::duration const& max_time_diff_between_packets_ns,
            nie::TimeIntervals* const stationary_intervals)
        : max_time_diff_between_packets_ns_(max_time_diff_between_packets_ns),
          stationary_intervals_(stationary_intervals) {}

    bool Check(nie::PcapMeta const& pcap_meta, nie::io::lidar::Returns const& lidar_points) const {
        nie::TimeIntervals::TimeRange const range_between_packets = {pcap_meta.end_time, lidar_points.timestamps[0]};
        auto const packet_time_delta = range_between_packets.second - range_between_packets.first;
        bool const is_moving = !stationary_intervals_->IsMostlyContained(range_between_packets);
        // Invalidate PCAP if it contains time gaps while the vehicle is moving.
        if (packet_time_delta > max_time_diff_between_packets_ns_) {
            if (is_moving) {
                LOG(WARNING) << "PCAP File [" << pcap_meta.filepath << "] has a " << packet_time_delta.count() / 1e9
                             << " seconds gap between subsequent packets in the interval: "
                             << "{" << range_between_packets.first << ", " << range_between_packets.second << "}";
                return true;
            }

            LOG(INFO) << "PCAP File [" << pcap_meta.filepath << "] has a " << packet_time_delta.count() / 1e9
                      << " seconds gap between subsequent packets, but the vehicle was stationary during the "
                         "interval: "
                      << "{" << range_between_packets.first << ", " << range_between_packets.second << "}";
        }

        return false;
    }

protected:
    nie::Timestamp_ns::duration const max_time_diff_between_packets_ns_;
    nie::TimeIntervals* const stationary_intervals_;
};

/// Get meta data from a single pcap file.
template <typename Streamer>
bool GetPcapMetaData(GapChecker const& gap_checker, Streamer* streamer, nie::PcapMeta* pcap_meta) {
    bool pcap_contains_gap = false;

    streamer->template AddCallback<nie::io::lidar::LidarCallbackTags::kPacket>(
            [pcap_meta, &pcap_contains_gap, &gap_checker](nie::io::lidar::Returns const& lidar_points) {
                if (pcap_contains_gap) {
                    // Pcap was checked as invalid, no more processing is needed
                    return;
                }
                if (lidar_points.timestamps.empty()) {
                    ++pcap_meta->packet_count;
                    ++pcap_meta->empty_packet_count;
                    return;
                }

                CHECK(lidar_points.timestamps.size() == lidar_points.points.size())
                        << "Returns has " << lidar_points.timestamps.size() << " timestamps and "
                        << lidar_points.points.size() << " points.";

                // Check if this is the first non-empty packet with a valid timestamp.
                if ((pcap_meta->packet_count - pcap_meta->empty_packet_count) == 0) {
                    pcap_meta->begin_time = lidar_points.timestamps[0];
                } else {
                    pcap_contains_gap = gap_checker.Check(*pcap_meta, lidar_points);
                }

                pcap_meta->end_time = lidar_points.timestamps.back();

                ++pcap_meta->packet_count;
                pcap_meta->point_count += lidar_points.timestamps.size();
            });

    streamer->Start();
    while (streamer->IsRunning()) {
        std::this_thread::sleep_for(std::chrono::microseconds{1});
        if (pcap_contains_gap) {
            // Skipping this pcap
            LOG(WARNING) << "Stopping processing of PCAP File [" << pcap_meta->filepath << "] that has gaps. Skipping.";
            break;
        }
    }
    streamer->Stop();
    return pcap_contains_gap;
}

bool GetPcapMetaData(
        nie::io::LidarType const& type,
        std::string const& intr_path,
        GapChecker const& gap_checker,
        nie::PcapMeta* pcap_meta) {
    switch (type) {
        case nie::io::LidarType::kVelodyneVLP16:
        case nie::io::LidarType::kVelodyneHDL32: {
            auto streamer = nie::io::velodyne::CreatePcapFileStreamer(type, intr_path, {pcap_meta->filepath});
            return GetPcapMetaData(gap_checker, &streamer, pcap_meta);
        }
        case nie::io::LidarType::kKitti: {
            auto streamer = nie::io::kitti::CreateTextFileStreamer({pcap_meta->filepath});
            return GetPcapMetaData(gap_checker, &streamer, pcap_meta);
        }
        case nie::io::LidarType::kOusterOS1_32:
        case nie::io::LidarType::kOusterOS1_128:
        case nie::io::LidarType::kOusterOS2_128: {
            auto streamer = nie::io::ouster::CreatePcapFileStreamer(intr_path, {pcap_meta->filepath});
            return GetPcapMetaData(gap_checker, &streamer, pcap_meta);
        }
    }
}

void WritePaths(std::string const& file, std::vector<boost::filesystem::path> const& paths) {
    boost::filesystem::path filepath = boost::filesystem::path{FLAGS_out_dir} / file;
    LOG(INFO) << "Writing file: " << filepath;
    nie::WriteLines(filepath, paths, [](boost::filesystem::path const& f) { return f.string(); });
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    std::vector<boost::filesystem::path> const filepaths_pcaps = nie::FindFiles(FLAGS_in_dir, FLAGS_pcap_pattern);
    CHECK(!filepaths_pcaps.empty()) << "No pcap files found for " << FLAGS_in_dir << "/" << FLAGS_pcap_pattern;

    LOG(INFO) << "Found " << filepaths_pcaps.size() << " pcap files.";

    // Loading stationary intervals
    std::vector<std::pair<nie::Timestamp_ns, nie::Timestamp_ns>> const intervals =
            nie::io::ReadIntervalsList(FLAGS_in_stationary_intervals);

    nie::Timestamp_ns::duration const max_packet_time_diff_ns{FLAGS_max_packet_time_diff_us * 1000};

    // Get lidar calibration intrinsics
    nie::io::LidarType lidar_type{};
    if (FLAGS_lidar_id.empty()) {
        lidar_type = nie::io::LidarType::kVelodyneHDL32;
    } else {
        CHECK(nie::ValidateIsFile("in_lidar_extrinsics_file", FLAGS_in_lidar_extrinsics_file));
        lidar_type = nie::io::ReadLidarParametersByIdentifier(FLAGS_in_lidar_extrinsics_file, FLAGS_lidar_id).type;
    }

    // Process pcaps
    nie::mt::MtVector<nie::PcapMeta> mt_metadata_pcaps;
    nie::mt::MtVector<boost::filesystem::path> mt_skipped_filepaths_pcaps;
    mt_metadata_pcaps.vector().reserve(filepaths_pcaps.size());
    {  // Dragons Ahead: MT Context
        auto extract_pcap_metadata =
                [&lidar_type,
                 &filepaths_pcaps,
                 &intervals,
                 &max_packet_time_diff_ns,
                 &mt_metadata_pcaps,
                 &mt_skipped_filepaths_pcaps](std::unique_ptr<std::size_t> const& filepath_index_ptr) {
                    std::size_t const filepath_index = *filepath_index_ptr;
                    boost::filesystem::path const filepath = filepaths_pcaps[filepath_index];
                    DLOG(INFO) << "Worker received " << filepath_index << ": " << filepath;

                    nie::TimeIntervals stationary_ranges(intervals);
                    GapChecker gap_checker{max_packet_time_diff_ns, &stationary_ranges};

                    nie::PcapMeta pcap_metadata{filepath};
                    bool const pcap_contains_gap =
                            GetPcapMetaData(lidar_type, FLAGS_in_lidar_intrinsics_file, gap_checker, &pcap_metadata);
                    if (pcap_contains_gap) {
                        mt_skipped_filepaths_pcaps.PushBack(filepath);
                    } else {
                        mt_metadata_pcaps.PushBack(std::move(pcap_metadata));
                    }
                };

        size_t const thread_count =
                FLAGS_nthreads == 0 ? std::ceil(std::thread::hardware_concurrency() / 2.0) : FLAGS_nthreads;
        LOG(INFO) << "Running with " << thread_count << " threads.";
        size_t const queue_size = 1;
        nie::WorkPool<size_t> work_pool(thread_count, queue_size, extract_pcap_metadata);
        // Get relevant information from each pcap file.
        for (size_t i = 0; i < filepaths_pcaps.size(); ++i) {
            work_pool.queue().BlockingPushBack(std::make_unique<size_t>(i));
        }
    }  // The destructors do all the cleanup for us.
    LOG(INFO) << "Done processing " << mt_metadata_pcaps.size() << " pcap files.";

    // Sort based on begin time.
    LOG(INFO) << "Sorting pcap meta data...";
    std::sort(
            mt_metadata_pcaps.vector().begin(),
            mt_metadata_pcaps.vector().end(),
            [](nie::PcapMeta const& meta1, nie::PcapMeta const& meta2) { return meta1.begin_time < meta2.begin_time; });

    if (FLAGS_report_info) {
        // Report info
        for (auto const& pcap_meta : mt_metadata_pcaps.vector()) {
            nie::PrintPcapMeta(pcap_meta);
        }
    }

    if (FLAGS_check_overlap) {
        // Check if pcap files are not overlapping
        LOG(INFO) << "Checking if pcap files have time overlap...";
        size_t overlap_count = 0;
        for (size_t i = 1; i < mt_metadata_pcaps.size(); ++i) {
            if (mt_metadata_pcaps[i - 1].end_time >= mt_metadata_pcaps[i].begin_time) {
                LOG(WARNING) << "Found overlap: " << mt_metadata_pcaps[i - 1].filepath.filename() << " and "
                             << mt_metadata_pcaps[i].filepath.filename();
                ++overlap_count;
            }
        }
        CHECK(overlap_count == 0) << "Found " << overlap_count << " overlapping pcap files.";
        LOG(INFO) << "No overlapping pcap files.";
    }

    // Compute time difference between consecutive pcap files
    LOG(INFO) << "Checking time difference between consecutive pcap files...";
    std::vector<boost::filesystem::path> pcap_list{};
    nie::Timestamp_ns::duration max_time_diff_ns{FLAGS_max_pcap_time_diff_us * 1000};
    std::size_t pcap_list_count = 0;
    auto write_pcap_list = [](std::vector<boost::filesystem::path> const& pcap_list, std::size_t pcap_list_count) {
        std::stringstream ss{};
        ss << "pcap_list_" << std::setw(6) << std::setfill('0') << pcap_list_count << ".txt";
        WritePaths(ss.str(), pcap_list);
    };

    if (!mt_metadata_pcaps.empty()) {
        nie::TimeIntervals stationary_ranges(intervals);
        pcap_list.push_back(mt_metadata_pcaps.vector().front().filepath);
        for (std::size_t i = 1; i < mt_metadata_pcaps.size(); ++i) {
            auto const& gap_begin = mt_metadata_pcaps[i - 1].end_time;
            auto const& gap_end = mt_metadata_pcaps[i].begin_time;
            auto time_delta = gap_end - gap_begin;

            VLOG(1) << "time_delta [ns] = " << time_delta.count()
                    << ". From file: " << mt_metadata_pcaps[i - 1].filepath.filename() << " (" << gap_begin << ") to "
                    << mt_metadata_pcaps[i].filepath.filename() << " (" << gap_end << ")";

            bool const is_stationary = stationary_ranges.IsMostlyContained({gap_begin, gap_end});
            VLOG(1) << "Is interval {" << gap_begin << ", " << gap_end << "} stationary? " << std::boolalpha
                    << is_stationary;
            if (time_delta > max_time_diff_ns && !is_stationary) {
                VLOG(1) << "Closed trajectory of " << pcap_list.size() << " pcap files.";
                write_pcap_list(pcap_list, pcap_list_count);
                pcap_list.clear();
                ++pcap_list_count;
            }
            pcap_list.push_back(mt_metadata_pcaps[i].filepath);
        }
        VLOG(1) << "Last trajectory not closed. Closing it.";
        write_pcap_list(pcap_list, pcap_list_count);
    }

    LOG(INFO) << "Skipped " << mt_skipped_filepaths_pcaps.size() << " pcaps.";
    // It gets written, even if the list is empty. Saves complexity on two sides (las creator is on the receiving side).
    WritePaths("skipped.txt", mt_skipped_filepaths_pcaps.vector());

    return 0;
}
