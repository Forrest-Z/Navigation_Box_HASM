/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "evaluate_odometry.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <sstream>
#include <vector>

#include <glog/logging.h>
#include <boost/filesystem/path.hpp>
#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/core/string.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "matrix.hpp"

auto constexpr kNoOuput = " >/dev/null 2>&1; ";

void Run(char* command) {
    // Suppress output (Unix only)
    std::strcat(command, kNoOuput);
    CHECK(!system(command)) << "Error during execution of command: " << command;
}

void Run(std::string const& string) {
    char command[1024];
    std::strcpy(command, string.c_str());
    Run(command);
}

float const kLengths[] = {100, 200, 300, 400, 500, 600, 700, 800};

struct Errors {
    int32_t first_frame;
    float r_err;
    float t_err;
    float len;
    float speed;
    Errors(int32_t first_frame, float r_err, float t_err, float len, float speed)
        : first_frame(first_frame), r_err(r_err), t_err(t_err), len(len), speed(speed) {}
};

inline float RotationError(Matrix const& pose_error) {
    float a = pose_error.val[0][0];
    float b = pose_error.val[1][1];
    float c = pose_error.val[2][2];
    float d = 0.5f * (a + b + c - 1.0f);
    return std::acos(std::max(std::min(d, 1.0f), -1.0f));
}

inline float TranslationError(Matrix const& pose_error) {
    float dx = pose_error.val[0][3];
    float dy = pose_error.val[1][3];
    float dz = pose_error.val[2][3];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Poses are in CV coordinate system
void LoadKittiPoses(std::string const& file_name, std::vector<Matrix>* p_poses) {
    std::vector<Matrix>& poses = *p_poses;
    FILE* fp = fopen(file_name.c_str(), "r");
    if (!fp) return;
    while (!feof(fp)) {
        Matrix P = Matrix::eye(4);
        if (fscanf(fp,
                   "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P.val[0][0],
                   &P.val[0][1],
                   &P.val[0][2],
                   &P.val[0][3],
                   &P.val[1][0],
                   &P.val[1][1],
                   &P.val[1][2],
                   &P.val[1][3],
                   &P.val[2][0],
                   &P.val[2][1],
                   &P.val[2][2],
                   &P.val[2][3]) == 12) {
            poses.push_back(P);
        }
    }
    fclose(fp);
}

// Poses are in aircraft coordinate system, but will be returned as CV as expected by tooling
// Also the kitti benchmark expects the trajectory to start at the identity
void LoadBinPoses(std::string const& file_name, std::vector<Matrix>* p_poses, std::vector<std::size_t>* p_frame_ids) {
    std::vector<Matrix>& poses = *p_poses;
    std::vector<std::size_t>& frame_ids = *p_frame_ids;

    nie::io::PoseCollection pose_coll;
    nie::io::Read(file_name, &pose_coll);

    poses.reserve(pose_coll.poses.size());
    frame_ids.reserve(pose_coll.poses.size());

    for (nie::io::PoseRecord const& pose : pose_coll.poses) {
        auto const isom = nie::ConvertFrame<nie::Frame::kAircraft, nie::Frame::kCv>(pose.isometry);
        Eigen::Matrix3f const R = isom.rotation().toRotationMatrix().cast<float>();
        Eigen::Vector3f const t = isom.translation().cast<float>();

        Matrix P = Matrix::eye(4);
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t col = 0; col < 3; ++col) {
                P.setVal(R(row, col), row, col, row, col);
            }
            P.setVal(t[row], row, 3, row, 3);
        }

        poses.push_back(std::move(P));
        frame_ids.push_back(static_cast<std::size_t>(pose.id));
    }
}

float DetermineScale(std::vector<Matrix> const& poses_ref, std::vector<Matrix> const& poses_tst) {
    auto max_distance_func = [](std::vector<Matrix> const& poses) -> float {
        float max_dist = 0.;
        for (Matrix const& p : poses) {
            float const d = TranslationError(p);
            if (d > max_dist) {
                max_dist = d;
            }
        }
        return max_dist;
    };
    float const max_distance_ref = max_distance_func(poses_ref);
    float const max_distance_tst = max_distance_func(poses_tst);

    float const scale = max_distance_ref / max_distance_tst;
    return scale;
}

std::vector<float> TrajectoryDistances(std::vector<Matrix> const& poses) {
    std::vector<float> dist;
    dist.push_back(0);
    for (std::size_t i = 1; i < poses.size(); i++) {
        Matrix P1 = poses[i - 1];
        Matrix P2 = poses[i];
        double dx = P1.val[0][3] - P2.val[0][3];
        double dy = P1.val[1][3] - P2.val[1][3];
        double dz = P1.val[2][3] - P2.val[2][3];
        dist.push_back(dist[i - 1] + sqrt(dx * dx + dy * dy + dz * dz));
    }
    return dist;
}

std::int32_t LastFrameFromSegmentLength(std::vector<float> const& dist, std::int32_t first_frame, float len) {
    for (std::size_t i = first_frame; i < dist.size(); i++)
        if (dist[i] > dist[first_frame] + len) return i;
    return -1;
}

std::vector<Errors> CalcSequenceErrors(
        std::vector<Matrix> const& poses_ref,
        std::vector<Matrix> const& poses_tst,
        std::vector<std::size_t> const& frame_ids) {
    // error vector
    std::vector<Errors> err;

    // pre-compute distances (from ground truth as reference)
    std::vector<float> dist = TrajectoryDistances(poses_ref);

    // for all start positions do
    for (std::size_t first_frame_tst = 0; first_frame_tst < poses_tst.size(); ++first_frame_tst) {
        std::size_t const first_frame_ref = frame_ids[first_frame_tst];

        // for all segment lengths do
        for (float const& len : kLengths) {
            // compute last frame based on length of reference trajectory
            std::int32_t last_frame = LastFrameFromSegmentLength(dist, first_frame_ref, len);

            // continue, if sequence is not long enough
            if (last_frame == -1) continue;

            // Find first reference pose index equal or larger than the found index
            auto const iter = std::lower_bound(frame_ids.cbegin() + first_frame_tst, frame_ids.cend(), last_frame);
            std::size_t const& last_frame_ref = *iter;
            std::size_t const last_frame_tst = std::distance(frame_ids.cbegin(), iter);

            // compute rotational and translational errors
            Matrix pose_delta_ref = Matrix::inv(poses_ref[first_frame_ref]) * poses_ref[last_frame_ref];
            Matrix pose_delta_tst = Matrix::inv(poses_tst[first_frame_tst]) * poses_tst[last_frame_tst];
            Matrix pose_error = Matrix::inv(pose_delta_tst) * pose_delta_ref;
            float r_err = RotationError(pose_error);
            float t_err = TranslationError(pose_error);

            // compute speed
            float num_frames = 1.f + last_frame_ref - first_frame_ref;
            float speed = len / (0.1f * num_frames);

            // write to file
            err.push_back(Errors(first_frame_ref, r_err / len, t_err / len, len, speed));
        }
    }

    // return error vector
    return err;
}

void SaveSequenceErrors(std::vector<Errors> const& err, std::string const& file_name) {
    // open file
    FILE* fp;
    fp = fopen(file_name.c_str(), "w");

    // write to file
    for (Errors const& it : err) {
        fprintf(fp, "%d %f %f %f %f\n", it.first_frame, it.r_err, it.t_err, it.len, it.speed);
    }

    // close file
    fclose(fp);
}

void SavePathPlot(
        std::vector<Matrix> const& poses_ref,
        std::vector<Matrix> const& poses_tst,
        std::vector<std::size_t> const& frame_ids,
        std::string const& file_name) {
    // open file
    FILE* fp = fopen(file_name.c_str(), "w");

    // save x/z coordinates of all frames to file
    for (std::size_t i = 0; i < poses_tst.size(); ++i) {
        fprintf(fp,
                "%f %f %f %f\n",
                poses_ref[frame_ids[i]].val[0][3],
                poses_ref[frame_ids[i]].val[2][3],
                poses_tst[i].val[0][3],
                poses_tst[i].val[2][3]);
    }

    // close file
    fclose(fp);
}

std::vector<std::int32_t> ComputeRoi(std::vector<Matrix> const& poses_gt, std::vector<Matrix> const& poses_result) {
    double x_min = std::numeric_limits<std::int32_t>::max();
    double x_max = std::numeric_limits<std::int32_t>::min();
    double z_min = std::numeric_limits<std::int32_t>::max();
    double z_max = std::numeric_limits<std::int32_t>::min();

    for (Matrix const& it : poses_gt) {
        float x = it.val[0][3];
        float z = it.val[2][3];
        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
    }

    for (Matrix const& it : poses_result) {
        float x = it.val[0][3];
        float z = it.val[2][3];
        if (x < x_min) x_min = x;
        if (x > x_max) x_max = x;
        if (z < z_min) z_min = z;
        if (z > z_max) z_max = z;
    }

    double dx = 1.1 * (x_max - x_min);
    double dz = 1.1 * (z_max - z_min);
    double mx = 0.5 * (x_max + x_min);
    double mz = 0.5 * (z_max + z_min);
    double r = 0.5 * std::max(dx, dz);

    std::vector<std::int32_t> roi;
    roi.push_back(static_cast<int32_t>(mx - r));
    roi.push_back(static_cast<int32_t>(mx + r));
    roi.push_back(static_cast<int32_t>(mz - r));
    roi.push_back(static_cast<int32_t>(mz + r));
    return roi;
}

void ConvertEpsToPdf(std::string const& dir, std::string const& file_name) {
    std::string base_command = "cd \"" + dir + "\"; ";

    // build command to create pdf and crop
    std::ostringstream oss;
    oss << "cd \"" << dir << "\"; ";
    oss << "ps2pdf \"" << file_name << ".eps\" \"" << file_name << "_large.pdf\"" << kNoOuput;
    oss << "pdfcrop \"" << file_name << "_large.pdf\" \"" << file_name << ".pdf\"" << kNoOuput;
    oss << "rm \"" << file_name << "_large.pdf\"";

    Run(oss.str());
}

void PlotPathPlot(std::string const& dir, std::vector<std::int32_t>& roi, std::string const& id) {
    // gnuplot file name
    char command[1024];
    std::string full_name = dir + "/" + id + ".gp";

    // create png + eps
    for (int32_t i = 0; i < 2; i++) {
        // open file
        FILE* fp = fopen(full_name.c_str(), "w");

        // save gnuplot instructions
        if (i == 0) {
            fprintf(fp, "set term png size 900,900\n");
            fprintf(fp, "set output \"%s.png\"\n", id.c_str());
        } else {
            fprintf(fp, "set term postscript eps enhanced color\n");
            fprintf(fp, "set output \"%s.eps\"\n", id.c_str());
        }

        fprintf(fp, "set size ratio -1\n");
        fprintf(fp, "set xrange [%d:%d]\n", roi[0], roi[1]);
        fprintf(fp, "set yrange [%d:%d]\n", roi[2], roi[3]);
        fprintf(fp, "set xlabel \"x [m]\"\n");
        fprintf(fp, "set ylabel \"z [m]\"\n");
        fprintf(fp, "plot \"%s.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground Truth' w lines,", id.c_str());
        fprintf(fp, "\"%s.txt\" using 3:4 lc rgb \"#0000FF\" title 'Test Odometry' w lines,", id.c_str());
        fprintf(fp,
                "\"< head -1 %s.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",
                id.c_str());

        // close file
        fclose(fp);

        // run gnuplot => create png + eps
        sprintf(command, "cd \"%s\"; gnuplot \"%s.gp\"", dir.c_str(), id.c_str());
        Run(command);

        // Clean up gp file
        sprintf(command, "cd \"%s\"; rm \"%s.gp\"", dir.c_str(), id.c_str());
        Run(command);
    }

    ConvertEpsToPdf(dir, id);
}

void SaveErrorPlots(std::vector<Errors> const& seq_err, std::string const& plot_error_dir, std::string const& prefix) {
    // file names
    std::string const file_name_tl = plot_error_dir + "/" + prefix + "_tl.txt";
    std::string const file_name_rl = plot_error_dir + "/" + prefix + "_rl.txt";
    std::string const file_name_ts = plot_error_dir + "/" + prefix + "_ts.txt";
    std::string const file_name_rs = plot_error_dir + "/" + prefix + "_rs.txt";

    // open files
    FILE* fp_tl = fopen(file_name_tl.c_str(), "w");
    FILE* fp_rl = fopen(file_name_rl.c_str(), "w");
    FILE* fp_ts = fopen(file_name_ts.c_str(), "w");
    FILE* fp_rs = fopen(file_name_rs.c_str(), "w");

    // for each segment length do
    for (float const& len : kLengths) {
        float t_err = 0;
        float r_err = 0;
        float num = 0;

        // for all errors do
        for (Errors const& it : seq_err) {
            if (std::fabs(it.len - len) < 1.0) {
                t_err += it.t_err;
                r_err += it.r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_tl, "%f %f\n", len, t_err / num);
            fprintf(fp_rl, "%f %f\n", len, r_err / num);
        }
    }

    // for each driving speed do (in m/s)
    for (float speed = 2; speed < 25; speed += 2) {
        float t_err = 0;
        float r_err = 0;
        float num = 0;

        // for all errors do
        for (Errors const& it : seq_err) {
            if (std::fabs(it.speed - speed) < 2.0) {
                t_err += it.t_err;
                r_err += it.r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_ts, "%f %f\n", speed, t_err / num);
            fprintf(fp_rs, "%f %f\n", speed, r_err / num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

void PlotErrorPlots(std::string const& dir, std::string const& prefix) {
    char command[1024];

    // for all four error plots do
    for (int32_t i = 0; i < 4; i++) {
        // create suffix
        std::string suffix;
        switch (i) {
            case 0:
                suffix = "tl";
                break;
            case 1:
                suffix = "rl";
                break;
            case 2:
                suffix = "ts";
                break;
            case 3:
                suffix = "rs";
                break;
        }

        // gnuplot file name
        std::string const presuffix{prefix + "_" + suffix};
        std::string const file_name{presuffix + ".gp"};
        std::string const full_name{dir + "/" + file_name};

        // create png + eps
        for (int32_t j = 0; j < 2; j++) {
            // open file
            FILE* fp = fopen(full_name.c_str(), "w");

            // save gnuplot instructions
            if (j == 0) {
                fprintf(fp, "set term png size 500,250 font \"Helvetica\" 11\n");
                fprintf(fp, "set output \"%s.png\"\n", presuffix.c_str());
            } else {
                fprintf(fp, "set term postscript eps enhanced color\n");
                fprintf(fp, "set output \"%s.eps\"\n", presuffix.c_str());
            }

            // start plot at 0
            fprintf(fp, "set size ratio 0.5\n");
            fprintf(fp, "set yrange [0:*]\n");

            // x label
            if (i <= 1)
                fprintf(fp, "set xlabel \"Path Length [m]\"\n");
            else
                fprintf(fp, "set xlabel \"Speed [km/h]\"\n");

            // y label
            if (i == 0 || i == 2)
                fprintf(fp, "set ylabel \"Translation Error [%%]\"\n");
            else
                fprintf(fp, "set ylabel \"Rotation Error [deg/m]\"\n");

            // plot error curve
            fprintf(fp, "plot \"%s.txt\" using ", presuffix.c_str());
            switch (i) {
                case 0:
                    fprintf(fp, "1:($2*100) title 'Translation Error'");
                    break;
                case 1:
                    fprintf(fp, "1:($2*57.3) title 'Rotation Error'");
                    break;
                case 2:
                    fprintf(fp, "($1*3.6):($2*100) title 'Translation Error'");
                    break;
                case 3:
                    fprintf(fp, "($1*3.6):($2*57.3) title 'Rotation Error'");
                    break;
            }
            fprintf(fp, " lc rgb \"#0000FF\" pt 4 w linespoints\n");

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command, "cd \"%s\"; gnuplot \"%s\"", dir.c_str(), file_name.c_str());
            Run(command);

            // Clean up gp file
            sprintf(command, "cd \"%s\"; rm \"%s\"", dir.c_str(), file_name.c_str());
            Run(command);
        }

        ConvertEpsToPdf(dir, presuffix);
    }
}

void SaveStats(std::vector<Errors> const& err, std::string const& dir) {
    float t_err = 0;
    float r_err = 0;

    // for all errors do => compute sum of t_err, r_err
    for (Errors const& it : err) {
        t_err += it.t_err;
        r_err += it.r_err;
    }

    // open file
    FILE* fp = fopen((dir + "/stats.txt").c_str(), "w");

    // save errors
    float num = err.size();
    fprintf(fp, "%f %f\n", t_err / num, r_err / num);

    // close file
    fclose(fp);
}

bool EvaluateFile(
        std::string const& ref_file,
        std::string const& tst_file,
        bool const correct_scale,
        std::string const& result_dir,
        std::string file_name) {
    // read ground truth poses
    std::vector<Matrix> poses_ref;
    LoadKittiPoses(ref_file, &poses_ref);
    CHECK(!poses_ref.empty()) << "Could not read any poses from: " << ref_file;

    // read result poses
    std::vector<Matrix> poses_tst;
    std::vector<std::size_t> frame_ids;
    if (nie::EndsWith(tst_file, ".pose")) {
        LoadBinPoses(tst_file, &poses_tst, &frame_ids);
    } else {
        LoadKittiPoses(tst_file, &poses_tst);
        frame_ids.resize(poses_tst.size());
        std::iota(frame_ids.begin(), frame_ids.end(), 0);
    }
    CHECK(!poses_tst.empty()) << "Could not read any poses from: " << tst_file;

    // Apply filtering to the poses and make sure the poses are of matching size
    LOG(INFO) << "Number of poses read: " << poses_ref.size() << " (reference) versus " << poses_tst.size() << " (test)"
              << std::endl;
    if (frame_ids.back() + 1 < poses_ref.size()) {
        LOG(INFO) << "Number of reference poses that will be used: " << frame_ids.back() + 1;
        poses_ref.resize(frame_ids.back() + 1);
    } else if (frame_ids.back() + 1 > poses_ref.size()) {
        LOG(WARNING) << "Number of test poses that will be used: " << poses_ref.size();
        poses_tst.resize(poses_ref.size());
        frame_ids.resize(poses_ref.size());
    }

    // ground truth and result directories
    std::string const error_dir = result_dir + "/errors";
    std::string const plot_path_dir = result_dir + "/plot_path";
    std::string const plot_error_dir = result_dir + "/plot_error";

    // create output directories when not present
    for (std::string const& dir : {error_dir, plot_path_dir, plot_error_dir}) {
        std::ostringstream command;
        command << "mkdir -p \"" << dir << "\"";
        Run(command.str());
    }

    if (file_name.empty()) {
        file_name = boost::filesystem::path(ref_file).stem().string() + "-" +
                    boost::filesystem::path(tst_file).stem().string();
    }
    std::string const file_name_txt = file_name + ".txt";

    // apply scaling to the test poses
    if (correct_scale) {
        float scale = DetermineScale(poses_ref, poses_tst);
        std::for_each(poses_tst.begin(), poses_tst.end(), [&scale](Matrix& m) {
            m.val[0][3] *= scale;
            m.val[1][3] *= scale;
            m.val[2][3] *= scale;
        });
    }

    // compute sequence errors
    std::vector<Errors> const seq_err = CalcSequenceErrors(poses_ref, poses_tst, frame_ids);
    SaveSequenceErrors(seq_err, error_dir + "/" + file_name_txt);

    // save + plot bird's eye view trajectories
    SavePathPlot(poses_ref, poses_tst, frame_ids, plot_path_dir + "/" + file_name_txt);
    std::vector<int32_t> roi = ComputeRoi(poses_ref, poses_tst);
    PlotPathPlot(plot_path_dir, roi, file_name);

    // save + plot individual errors
    SaveErrorPlots(seq_err, plot_error_dir, file_name);
    PlotErrorPlots(plot_error_dir, file_name);

    // success
    return true;
}
