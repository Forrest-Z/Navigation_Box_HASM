#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/container/mt_vector.hpp>
#include <nie/core/gflags.hpp>
#include <nie/core/time.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
// TODO: Add support for ouster_streamer
// This can be done via a flag
#include <nie/lidar/io/lidar_streamer/velodyne_streamer.hpp>
#include <nie/lidar/time_intervals/time_intervals.hpp>
#include <nie/lidar/time_intervals/time_intervals_reader.hpp>

#include "pcap_meta.hpp"

// clang-format off
DEFINE_string(in_dir, "", "Input directory containing pcap files.");
DEFINE_string(lidar_intrinsics_file, "", "Filepath to input lidar intrinsics calibration parameters.");
DEFINE_string(in_stationary_intervals, "",
        "Filepath to csv file with list of intervals when the vehicle was not moving.");
DEFINE_string(out_dir, "", "Output directory for writing .txt files (track files and skipped file).");
DEFINE_string(pcap_pattern, ".*.pcap",
        "(Optional) Regex pattern for finding input Velodyne lidar recordings, in pcap format.");
DEFINE_uint64(max_pcap_time_diff_us, 1e6,
        "Maximum number of microseconds between two consecutive pcap files.");
DEFINE_uint64(max_packet_time_diff_us, 1e5,
        "Maximum number of microseconds between two consecutive pcap packets within a file.");
DEFINE_double(stop_interval_extension, 20.,
        "Pcap files will be selected when they overlap with a stationary intervals plus this time range extension.");
DEFINE_double(stop_interval_threshold, 2.,
        "Pcap files will be selected when they overlap with a stationary intervals plus this time range extension.");
DEFINE_uint64(nr_threads, 0, "Number of threads to use for processing pcap files.");
DEFINE_bool(output_time_file, false, "");
// clang-format on

DEFINE_validator(in_dir, nie::ValidateIsDirectory);
DEFINE_validator(lidar_intrinsics_file, nie::ValidateIsFile);
DEFINE_validator(in_stationary_intervals, nie::ValidateIsFile);
DEFINE_validator(out_dir, nie::ValidateIsDirectory);
DEFINE_validator(pcap_pattern, nie::ValidateStringNotEmpty);
DEFINE_validator(max_pcap_time_diff_us, nie::ValidateLargerThanZero);
DEFINE_validator(max_packet_time_diff_us, nie::ValidateLargerThanZero);
DEFINE_validator(stop_interval_extension, nie::ValidateLargerThanZero);
DEFINE_validator(stop_interval_threshold, nie::ValidateLargerThanZero);

struct PcapValidator {
    PcapValidator(
            nie::Timestamp_ns::duration const& max_time_diff_between_packets_ns,
            nie::TimeIntervals* const stationary_intervals)
        : max_time_diff_between_packets_ns_(max_time_diff_between_packets_ns),
          stationary_intervals_(stationary_intervals) {}

    bool operator()(nie::PcapMeta const& pcap_meta, nie::io::lidar::Returns const& lidar_points) const {
        nie::TimeIntervals::TimeRange const range_between_packets = {pcap_meta.end_time, lidar_points.timestamps[0]};
        auto const packet_time_delta = range_between_packets.second - range_between_packets.first;
        bool const is_moving = !stationary_intervals_->IsMostlyContained(range_between_packets);
        // Invalidate PCAP if it contains time gaps while the vehicle is moving.
        if (packet_time_delta > max_time_diff_between_packets_ns_) {
            if (is_moving) {
                LOG(WARNING) << "PCAP File [" << pcap_meta.filepath << "] is invalid because a "
                             << packet_time_delta.count() / 1e9
                             << " seconds gap was detected between subsequent packets in the interval: "
                             << "{" << range_between_packets.first << ", " << range_between_packets.second << "}";
                return false;
            }

            LOG(INFO) << "PCAP File [" << pcap_meta.filepath << "] has a " << packet_time_delta.count() / 1e9
                      << " seconds gap between subsequent packets, but the vehicle was stationary during the "
                         "interval: "
                      << "{" << range_between_packets.first << ", " << range_between_packets.second << "}";
        }

        return true;
    }

protected:
    nie::Timestamp_ns::duration const max_time_diff_between_packets_ns_;
    nie::TimeIntervals* const stationary_intervals_;
};

/// Get meta data from a single pcap file.
bool GetPcapMetaData(
        PcapValidator const& validator,
        nie::io::velodyne::LidarCalibration const& lidar_intrinsics,
        nie::PcapMeta* pcap_meta) {
    bool pcap_is_valid = true;

    auto streamer = nie::io::velodyne::CreatePcapFileStreamer(lidar_intrinsics, {pcap_meta->filepath});
    streamer.AddCallback<nie::io::lidar::LidarCallbackTags::kPacket>(
            [pcap_meta, &pcap_is_valid, &validator](nie::io::lidar::Returns const& lidar_points) {
                if (!pcap_is_valid) {
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
                    pcap_is_valid = pcap_is_valid && validator(*pcap_meta, lidar_points);
                }

                pcap_meta->end_time = *lidar_points.timestamps.crbegin();

                ++pcap_meta->packet_count;
                pcap_meta->point_count += lidar_points.timestamps.size();
            });

    streamer.Start();
    while (streamer.IsRunning()) {
        std::this_thread::sleep_for(std::chrono::microseconds{1});
        if (!pcap_is_valid) {
            // Skipping this pcap
            LOG(WARNING) << "Stopping processing of invalid PCAP File [" << pcap_meta->filepath << "] - Skipping";
            streamer.Stop();
            break;
        }
    }

    return pcap_is_valid;
}

void ReadMetadataPcaps(
        std::vector<boost::filesystem::path> const filepaths_pcaps,
        nie::io::velodyne::LidarCalibration const lidar_intrinsics,
        nie::TimeIntervals stationary_ranges,
        std::size_t const thread_count,
        nie::Timestamp_ns::duration const& max_packet_time_diff_ns,
        std::vector<nie::PcapMeta>* metadata_pcaps) {
    {
        // Dragons Ahead: MT Context
        PcapValidator const pcap_validator{max_packet_time_diff_ns, &stationary_ranges};

        // Prepare result vector
        nie::mt::MtVector<nie::PcapMeta> mt_metadata_pcaps;
        mt_metadata_pcaps.vector().reserve(filepaths_pcaps.size());

        // Prepare work function
        auto extract_pcap_metadata = [&filepaths_pcaps, &lidar_intrinsics, &mt_metadata_pcaps, &pcap_validator](
                                             std::unique_ptr<std::size_t> const& filepath_index_ptr) {
            nie::PcapMeta pcap_metadata{filepaths_pcaps[*filepath_index_ptr]};
            DLOG(INFO) << "Worker received " << *filepath_index_ptr << ": " << pcap_metadata.filepath;
            if (GetPcapMetaData(pcap_validator, lidar_intrinsics, &pcap_metadata)) {
                mt_metadata_pcaps.PushBack(std::move(pcap_metadata));
            }
        };

        // Start doing work
        {
            LOG(INFO) << "Running with " << thread_count << " threads.";
            size_t const queue_size = 1;
            nie::WorkPool<size_t> work_pool(thread_count, queue_size, extract_pcap_metadata);
            // Get relevant information from each pcap file.
            for (size_t i = 0; i < filepaths_pcaps.size(); ++i) {
                work_pool.queue().BlockingPushBack(std::make_unique<size_t>(i));
            }
        }

        // Return the result
        std::swap(*metadata_pcaps, mt_metadata_pcaps.vector());
    }  // The destructors do all the cleanup for us.
    LOG(INFO) << "Done processing " << metadata_pcaps->size() << " pcap files.";

    // Sort the result based on begin time
    std::sort(
            metadata_pcaps->begin(), metadata_pcaps->end(), [](nie::PcapMeta const& meta1, nie::PcapMeta const& meta2) {
                return meta1.begin_time < meta2.begin_time;
            });
}

void WritePaths(std::vector<boost::filesystem::path> const& paths, boost::filesystem::path const& filepath) {
    LOG(INFO) << "Writing file: " << filepath;
    nie::WriteLines(filepath, paths, [](boost::filesystem::path const& f) { return f.string(); });
}

void WritePaths(
        std::vector<boost::filesystem::path> const& paths,
        boost::filesystem::path const& out_dir,
        std::size_t const seq_nr) {
    std::stringstream ss{};
    ss << "pcap_list_" << std::setw(6) << std::setfill('0') << seq_nr << ".txt";

    WritePaths(paths, out_dir / ss.str());
}

void FlushPaths(
        std::string const& out_dir, std::size_t* pcap_list_seq_nr, std::vector<boost::filesystem::path>* pcap_list) {
    WritePaths(*pcap_list, out_dir, *pcap_list_seq_nr);
    pcap_list->clear();
    ++(*pcap_list_seq_nr);
}

using TimeRange = std::pair<nie::Timestamp_ns, nie::Timestamp_ns>;

bool Overlap(TimeRange const& a, TimeRange const& b) { return a.first <= b.second && b.first <= a.second; }

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    // Loading all inputs
    nie::io::velodyne::LidarCalibration const lidar_intrinsics =
            nie::io::velodyne::LoadCalibrationFromFile(nie::io::LidarType::kVelodyneHDL32, FLAGS_lidar_intrinsics_file);
    std::vector<TimeRange> stop_time_ranges = nie::io::ReadIntervalsList(FLAGS_in_stationary_intervals);
    CHECK(!stop_time_ranges.empty()) << "No intervals read from " << FLAGS_in_stationary_intervals;

    // It is assumed that the stationary time intervals are sorted and not overlapping
    for (auto interval_it = stop_time_ranges.cbegin() + 1; interval_it != stop_time_ranges.cend(); ++interval_it) {
        CHECK((interval_it - 1)->second < interval_it->first);
    }

    //    auto const tmp = nie::FindFiles(FLAGS_in_dir, FLAGS_pcap_pattern);
    //    std::vector<boost::filesystem::path> const filepaths_pcaps = {tmp.cbegin(), tmp.cbegin() + 20};
    std::vector<boost::filesystem::path> const filepaths_pcaps = nie::FindFiles(FLAGS_in_dir, FLAGS_pcap_pattern);
    CHECK(!filepaths_pcaps.empty()) << "No pcap files found for " << FLAGS_in_dir << "/" << FLAGS_pcap_pattern;
    LOG(INFO) << "Found " << filepaths_pcaps.size() << " pcap files.";

    // Read the metadata from all pcaps
    size_t const thread_count =
            FLAGS_nr_threads == 0 ? std::ceil(std::thread::hardware_concurrency() / 2.0) : FLAGS_nr_threads;
    nie::Timestamp_ns::duration const max_packet_time_diff_ns{FLAGS_max_packet_time_diff_us * 1000};

    std::vector<nie::PcapMeta> metadata_pcaps;
    ReadMetadataPcaps(
            filepaths_pcaps,
            lidar_intrinsics,
            nie::TimeIntervals{stop_time_ranges},
            thread_count,
            max_packet_time_diff_ns,
            &metadata_pcaps);

    // Remove stationary intervals when they are below the supplied threshold
    if (FLAGS_stop_interval_threshold > 0.) {
        std::chrono::nanoseconds const stop_threshold(std::llround(FLAGS_stop_interval_threshold * 1.e9));
        std::size_t const orig_size = stop_time_ranges.size();
        stop_time_ranges.erase(
                std::remove_if(
                        stop_time_ranges.begin(),
                        stop_time_ranges.end(),
                        [&stop_threshold](TimeRange const& t) { return t.second - t.first < stop_threshold; }),
                stop_time_ranges.end());
        LOG(INFO) << "Number of stationary intervals below threshold of " << FLAGS_stop_interval_threshold
                  << " seconds is " << (orig_size - stop_time_ranges.size());
    }

    // When requested, output the trajectory time intervals similar to aiim gps stop file
    if (FLAGS_output_time_file) {
        auto get_time = [](nie::Timestamp_ns const& t) -> std::string {
            auto const time_in_week_ns = nie::ToGPSWeekTime(t).time_in_week;
            auto const time_in_week_s = std::chrono::duration_cast<std::chrono::duration<float>>(time_in_week_ns);
            return std::to_string(time_in_week_s.count());
        };

        std::vector<std::string> lines(metadata_pcaps.size() + 2);
        lines[0] = std::to_string(nie::ToGPSWeekTime(metadata_pcaps.front().begin_time).week.count());
        lines[1] = "begin,end";
        for (std::size_t i = 0; i < metadata_pcaps.size(); ++i) {
            auto const& d = metadata_pcaps[i];
            lines[2 + i] = get_time(d.begin_time) + "," + get_time(d.end_time);
        }
        nie::WriteLines(FLAGS_out_dir + "/pcap.time", lines, [](std::string const& s) -> std::string { return s; });
    }

    // Identify trajectories based on time difference between consecutive pcap files
    LOG(INFO) << "Checking time differences between consecutive pcap files to identify trajectories...";
    LOG(INFO) << "Filtering the pcap files based on the stationary intervals...";
    nie::Timestamp_ns::duration const max_time_diff_ns{FLAGS_max_pcap_time_diff_us * 1000};
    std::chrono::nanoseconds const stop_extension(std::llround(FLAGS_stop_interval_extension * 1.e9));

    std::vector<boost::filesystem::path> pcap_list{};
    std::size_t pcap_list_seq_nr = 0;

    if (metadata_pcaps.empty()) {
        return 0;
    }

    auto pcap_it = metadata_pcaps.cbegin();
    auto stop_it = stop_time_ranges.cbegin();
    // Current stop time interval with extension
    TimeRange stop_range_ext{stop_it->first - stop_extension, stop_it->second + stop_extension};
    nie::TimeIntervals stationary_ranges(stop_time_ranges);

    while (pcap_it != metadata_pcaps.cend() && stop_it != stop_time_ranges.cend()) {
        TimeRange const pcap_interval{pcap_it->begin_time, pcap_it->end_time};

        if (Overlap(pcap_interval, stop_range_ext)) {
            pcap_list.push_back(pcap_it->filepath);
            ++pcap_it;
            continue;
        }

        if (pcap_it->end_time < stop_range_ext.first) {
            ++pcap_it;

            if (!pcap_list.empty()) {
                // If the new pcap file is much later than the previous one and the car was not stationary, then write
                // the pcap files found so far to file
                auto time_delta = pcap_it->begin_time - (pcap_it - 1)->end_time;
                bool const is_stationary =
                        stationary_ranges.IsMostlyContained({(pcap_it - 1)->end_time, pcap_it->begin_time});
                if (time_delta > max_time_diff_ns && !is_stationary) {
                    FlushPaths(FLAGS_out_dir, &pcap_list_seq_nr, &pcap_list);
                }
            }

            continue;

        } else if (stop_range_ext.second < pcap_it->begin_time) {
            ++stop_it;
            TimeRange new_stop_range_ext{stop_it->first - stop_extension, stop_it->second + stop_extension};

            if (!pcap_list.empty()) {
                // If the new stop range is not overlapping with the current pcap, then write the pcap files list
                bool const overlapping = Overlap(pcap_interval, new_stop_range_ext);
                if (!overlapping) {
                    FlushPaths(FLAGS_out_dir, &pcap_list_seq_nr, &pcap_list);
                }
            }

            stop_range_ext = new_stop_range_ext;
            continue;
        }
    }

    if (!pcap_list.empty()) {
        WritePaths(pcap_list, FLAGS_out_dir, pcap_list_seq_nr);
    }

    return 0;
}
