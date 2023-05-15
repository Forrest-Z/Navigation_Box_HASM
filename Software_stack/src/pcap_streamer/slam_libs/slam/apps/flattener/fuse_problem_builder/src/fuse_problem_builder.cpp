#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/gflags.hpp>
#include <nie/core/time.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>
#include <nie/lidar/time_intervals/time_intervals_reader.hpp>

DEFINE_string(in_file_lidar_extrinsics, "", "Filepath to input lidar extrinsics calibration parameters.");
DEFINE_string(in_file_jump_tracks, "", "Input .txt file that contains paths to loam .pose files.");
DEFINE_string(in_file_gps_pose, "", "Input gps .pose file.");
DEFINE_string(in_stationary_intervals, "", "Input .csv file containing stationary intervals.");
DEFINE_string(out_file_fuse_problem_pose, "", "Output gps .pose file containing the graph optimization problem.");

DEFINE_validator(in_file_lidar_extrinsics, nie::ValidateIsFile);
DEFINE_validator(in_file_jump_tracks, nie::ValidateIsFile);
DEFINE_validator(in_file_gps_pose, nie::ValidateIsFile);
DEFINE_validator(in_stationary_intervals, nie::ValidateIsFile);
DEFINE_validator(out_file_fuse_problem_pose, nie::ValidateParentDirExists);

constexpr double kWeightGnssImuScale = 0.01;
constexpr double kWeightOdomScale = 0.02;
// kTimeBuffer         kTimeMean        Stationary begin    Stationary end        -kTimeMean          kTimeBuffer
//     |------------------>|------------------>|------------------>|------------------>|------------------>|
//     |-----GPS/ODOM----->|-------ODOM------->|-----NO DATA------>|-------ODOM------->|-----GPS/ODOM----->|
//     |-----Gaussian----->|--------1.0------->|-----NO DATA------>|--------1.0------->|-----Gaussian----->|
//     |--------------20 seconds-------------->|----Gap seconds--->|--------------20 seconds-------------->|
constexpr double kTimeBuffer = 20.0;
constexpr double kTimeMean = 10.0;
// For non stops / places where the car was stationary, we can only add an edge if there wasn't a time gap.
// 5 seconds is a safe margin.
constexpr double kMaxDeltaTimeGpsEdge = 5.0;

class NormalDistribution1d {
public:
    NormalDistribution1d(double const mean, double const sigma) : mean_(mean), sigma_(sigma) {}

    inline double operator()(double const x) const {
        double md = (x - mean_) / sigma_;
        return std::exp(-0.5 * md * md);
    }

private:
    double const mean_;
    double const sigma_;
};

void Check(nie::io::PoseCollection const& collection, nie::Timestamp_ns const& timestamp) {
    CHECK(timestamp >= collection.poses.begin()->timestamp && timestamp <= collection.poses.rbegin()->timestamp)
            << "The timestamp of the first relative pose is not in timestamp range of the absolute poses.";
}

class Weighter {
public:
    Weighter(nie::Timestamp_ns const& begin, nie::Timestamp_ns const& end, double const mean)
        : mean_(mean), begin_(begin), end_(end), d_(mean_, mean_ / 3.0) {}

    double operator()(nie::Timestamp_ns const& x) const {
        if (std::chrono::duration<double>(x - begin_).count() < mean_) {
            return d_(std::chrono::duration<double>(x - begin_).count());
        } else if (std::chrono::duration<double>(end_ - x).count() < mean_) {
            return d_(std::chrono::duration<double>(end_ - x).count());
        } else {
            return 1.0;
        }
    }

private:
    double mean_;
    nie::Timestamp_ns begin_;
    nie::Timestamp_ns end_;
    NormalDistribution1d d_;
};

template <typename It, typename W>
void AddWeightedEdges(
        W const& weighter,
        double const delta_percentage,
        It begin,
        It end,
        std::vector<nie::io::PoseEdgeRecord>* edges) {
    for (auto it = begin + 1; it != end; ++it) {
        auto const& p0 = *(it - 1);
        auto const& p1 = *(it + 0);
        auto const ei = p0.isometry.TransformInverseLeft(p1.isometry);
        double const w = weighter((p1.timestamp - p0.timestamp) / 2 + p0.timestamp);

        edges->emplace_back(nie::io::MakeEdgeRecord(p0.id, p1.id, ei, w, delta_percentage));
    }
}

nie::io::PoseCollection AddProblemEdges(
        nie::Isometry3qd const& T_gps_lidar,
        nie::Isometry3qd const& T_lidar_gps,
        nie::io::PoseCollection const& collection_odom,
        nie::io::PoseCollection* p_collection_gps,
        std::vector<bool>* p_has_edge) {
    auto& collection_gps = *p_collection_gps;
    auto& has_edge = *p_has_edge;

    Check(collection_gps, collection_odom.poses.front().timestamp);
    nie::Isometry3qd T_world_gps;
    nie::io::InterpolateIsometry(
            collection_gps.poses.begin(),
            collection_gps.poses.end(),
            collection_odom.poses.front().timestamp,
            &T_world_gps);

    nie::io::PoseCollection rel_to_abs_out;
    rel_to_abs_out.header = collection_gps.header;

    auto its_pose_rel = std::make_pair(collection_odom.poses.begin(), collection_odom.poses.end());
    // Find the range of timestamps that allows interpolation of odometry poses.
    auto it_begin = std::lower_bound(
            collection_gps.poses.begin(), collection_gps.poses.end(), collection_odom.poses.front().timestamp);
    auto it_end = std::upper_bound(it_begin, collection_gps.poses.end(), collection_odom.poses.back().timestamp);

    if (it_end - it_begin < 2) {
        return rel_to_abs_out;
    }

    for (auto it = it_begin; it != it_end; ++it) {
        auto const& p = *it;
        Check(collection_odom, p.timestamp);
        nie::Isometry3qd T_odom_lidar;
        its_pose_rel = nie::io::InterpolateIsometry(
                its_pose_rel.first, collection_odom.poses.end(), p.timestamp, &T_odom_lidar);

        nie::Isometry3qd T_odom_gps = T_world_gps * T_gps_lidar * T_odom_lidar * T_lidar_gps;

        auto copy = *it;
        copy.isometry = T_odom_gps;
        rel_to_abs_out.poses.push_back(copy);
    }

    // These edges will be added to the collection by this function.
    for (auto it = it_begin; it != it_end - 1; ++it) {
        has_edge[it - collection_gps.poses.begin()] = true;
    }

    Weighter w(it_begin->timestamp, (it_end - 1)->timestamp, kTimeMean);
    auto p = [&w](nie::Timestamp_ns const& x) { return w(x); };
    auto n = [&w](nie::Timestamp_ns const& x) { return 1.0 - w(x); };

    // Add GNSS/IMU edges
    AddWeightedEdges(n, kWeightGnssImuScale, it_begin, it_end, &collection_gps.edges);
    // Add odometry edges
    AddWeightedEdges(
            p, kWeightOdomScale, rel_to_abs_out.poses.begin(), rel_to_abs_out.poses.end(), &collection_gps.edges);

    return rel_to_abs_out;
}

void AddGpsEdges(std::vector<bool> const& has_edge, nie::io::PoseCollection* p_collection_gps) {
    auto& collection_gps = *p_collection_gps;
    for (std::size_t i = 0; i < collection_gps.poses.size() - 1; ++i) {
        if (!has_edge[i]) {
            auto const& p0 = collection_gps.poses[i + 0];
            auto const& p1 = collection_gps.poses[i + 1];

            // Won't connect poses with too much time between them
            if (std::chrono::duration<double>(p1.timestamp - p0.timestamp).count() < kMaxDeltaTimeGpsEdge) {
                auto const ei = p0.isometry.TransformInverseLeft(p1.isometry);
                collection_gps.edges.emplace_back(nie::io::MakeEdgeRecord(p0.id, p1.id, ei, 1.0, kWeightGnssImuScale));
            }
        }
    }
}

void PrunePoses(
        std::vector<std::pair<nie::Timestamp_ns, nie::Timestamp_ns>> const& stops,
        nie::io::PoseCollection* collection_odom) {
    std::pair file_range =
            std::make_pair(collection_odom->poses.front().timestamp, collection_odom->poses.back().timestamp);
    std::size_t index = 0;
    std::size_t count = 0;
    for (std::size_t i = 0; i < stops.size(); ++i) {
        if (stops[i].second > file_range.first && stops[i].second < file_range.second) {
            count++;
            index = i;
        }
    }

    // Everything fails if this is not true. Too complex for the POC atm. Also unlikely.
    CHECK(count == 1) << "Gap inside trace count != 1: " << count;

    std::pair stop_range = stops[index];

    auto filter = [&stop_range](nie::io::PoseRecord const& r) -> bool {
        return r.timestamp < (stop_range.first - std::chrono::duration<double>(kTimeBuffer)) ||
               r.timestamp > (stop_range.second + std::chrono::duration<double>(kTimeBuffer));
    };

    collection_odom->poses.erase(
            std::remove_if(collection_odom->poses.begin(), collection_odom->poses.end(), filter),
            collection_odom->poses.end());
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    auto const T_gps_lidar = nie::ReadLidarExtrinsics(FLAGS_in_file_lidar_extrinsics);
    auto const T_lidar_gps = T_gps_lidar.Inversed();
    auto odom_pose_paths = nie::ReadLines(FLAGS_in_file_jump_tracks);
    auto collection_gps = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_gps_pose);
    std::vector<std::pair<nie::Timestamp_ns, nie::Timestamp_ns>> const intervals =
            nie::io::ReadIntervalsList(FLAGS_in_stationary_intervals);

    std::vector<bool> has_edge(collection_gps.poses.size() - 1, false);

    for (auto const& odom_pose_path : odom_pose_paths) {
        auto collection_odom = nie::io::ReadCollection<nie::io::PoseCollection>(odom_pose_path);

        CHECK(collection_odom.poses.size() > 1) << "Nothing to do.";

        PrunePoses(intervals, &collection_odom);

        std::size_t previous_size = collection_gps.edges.size();

        auto rel_to_abs_out = AddProblemEdges(T_gps_lidar, T_lidar_gps, collection_odom, &collection_gps, &has_edge);

        collection_gps.header.Set(nie::io::PoseHeader::kHasEdgeInformationPerRecord);
        // Because of classic post we set this as default.
        collection_gps.header.Set(nie::io::PoseHeader::kHasPoseInformation);
        collection_gps.header.pose_information = Eigen::Matrix<double, 6, 6>::Identity();

        CHECK_EQ(collection_gps.edges.size() - previous_size, (rel_to_abs_out.poses.size() - 1) * 2);

        LOG(INFO) << "Edge count problems: " << collection_gps.edges.size();
    }

    std::size_t previous_size = collection_gps.edges.size();

    AddGpsEdges(has_edge, &collection_gps);

    CHECK_LE(collection_gps.edges.size(), (collection_gps.poses.size() - 1) + previous_size);

    LOG(INFO) << "Edge count with gps: " << collection_gps.edges.size();

    nie::io::Write(collection_gps, FLAGS_out_file_fuse_problem_pose);

    return 0;
}