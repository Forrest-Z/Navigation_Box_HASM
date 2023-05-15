/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/core/scoped_timer.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>
#include <nie/vo/create_visual_odometry.hpp>
#include <opencv2/opencv.hpp>

#include "demo.hpp"

DEFINE_string(in_file_intrinsics, "", "The input file containing the camera intrinsics.");
DEFINE_string(in_dir_images, "", "The input directory containing the images.");
DEFINE_string(out_dir, "", "The output directory containing all results.");

DEFINE_bool(with_demo, false, "Enable demo functionality.");
DEFINE_bool(with_timing, false, "Enable timing functionality.");

DEFINE_validator(in_file_intrinsics, nie::ValidateIsFile);
DEFINE_validator(in_dir_images, nie::ValidateIsDirectory);
DEFINE_validator(out_dir, nie::ValidateIsDirectory);

constexpr int kLoadImageType = cv::IMREAD_GRAYSCALE;
constexpr float kFactor = 1;

// General scaling of the demo displays (change in kFactor is already incorporated)
constexpr float kDrawingScale = 0.45;

std::vector<std::string> GetFiles(std::string const& path) {
    std::vector<boost::filesystem::path> files = nie::FindFiles(path);
    std::vector<std::string> result(files.size());
    for (std::int32_t image_id = 0; image_id < static_cast<std::int32_t>(files.size()); ++image_id) {
        boost::filesystem::path const& image_file = files[image_id];
        result[image_id] = image_file.string();
    }
    return result;
}

void WriteObjectAndKeypoints(
        std::uint8_t const frame_id,
        Eigen::Vector3d const& object,
        std::vector<std::pair<std::int32_t, nie::Keypoint>> const& keypoints,
        std::int32_t* p_object_id,
        nie::io::ObjectCollectionStreamWriter* p_object_stream,
        nie::io::KeypointCollectionStreamWriter* p_keypoint_stream) {
    assert(p_object_id != nullptr);
    assert(p_object_stream != nullptr);
    assert(p_keypoint_stream != nullptr);
    assert(!keypoints.empty());

    std::int32_t& object_id = *p_object_id;
    p_object_stream->Write(nie::io::ObjectRecord{object_id, object, {}});

    for (std::pair<std::int32_t, nie::Keypoint> const& kp : keypoints) {
        nie::Keypoint const& p = kp.second;
        p_keypoint_stream->Write(nie::io::KeypointRecord{kp.first, frame_id, object_id, {p.x, p.y}, {}});
    }

    ++object_id;
}

int main(int argc, char* argv[]) {
    {  // scope of timer
        nie::ScopedTimer timer("main");
        nie::ScopedTimer::SetEnabled(FLAGS_with_timing);

        google::InitGoogleLogging(argv[0]);
        gflags::ParseCommandLineFlags(&argc, &argv, false);

        // Get list of image files
        LOG(INFO) << "Search for images in: " << FLAGS_in_dir_images;
        std::vector<std::string> const image_files = GetFiles(FLAGS_in_dir_images);
        if (image_files.empty()) {
            LOG(WARNING) << "An empty list of images is supplied.";
        }

        // Read intrinsics
        std::int32_t const camera_id = 0;
        std::uint8_t const frame_id = 0;
        auto camera_param = nie::io::RectifiedCameraParameters::Read(FLAGS_in_file_intrinsics);
        if (kFactor != 1.) {
            camera_param.K(0, 2) /= kFactor;
            camera_param.K(1, 2) /= kFactor;
        }

        // Create the visual odometry object
        nie::VisualOdometryMono::Parameters params(camera_param.K);
        nie::VisualOdometryMonoPtr visual_odometry_mono = nie::CreateVisualOdometryMono(
                camera_param.image_size(0) / kFactor, camera_param.image_size(1) / kFactor, params, FLAGS_with_demo);

        // Add the drawing callbacks
        if (FLAGS_with_demo) {
            AddDrawingCallbacks(image_files, kDrawingScale * kFactor, kLoadImageType, &visual_odometry_mono);
        }

        // Set up writing of the information reference collection
        nie::io::InfoRefCollectionStreamWriter info_ref_stream(FLAGS_out_dir + "/info_refs.iref", {});
        visual_odometry_mono->AddCallback<nie::VisualOdometryMono::Handle::kWriteInfoRef>(
                [&image_files, &info_ref_stream](std::int32_t const& image_id) {
                    info_ref_stream.Write(nie::io::InfoRefRecord{image_id, frame_id, image_files[image_id]});
                });

        // Set up writing of the pose collection
        nie::io::PoseHeader pose_header{};
        pose_header.authority = "local";
        nie::io::PoseCollectionStreamWriter pose_stream(FLAGS_out_dir + "/poses.pose", pose_header);
        visual_odometry_mono->AddCallback<nie::VisualOdometryMono::Handle::kWritePose>(
                [&pose_stream](std::int32_t const& image_id, nie::Isometry3md const& pose) {
                    pose_stream.Write(nie::io::PoseRecord{
                            image_id, nie::io::PoseRecord::Category::kOdom, camera_id, {}, nie::Isometry3qd{pose}, {}});
                });

        // Set up writing of the object and keypoint collections
        nie::io::ObjectCollectionStreamWriter object_stream(FLAGS_out_dir + "/objects.objt", {});
        nie::io::KeypointCollectionStreamWriter keypoint_stream(FLAGS_out_dir + "/keypoints.kpnt", {});
        std::int32_t object_id = 0;
        visual_odometry_mono->AddCallback<nie::VisualOdometryMono::Handle::kWriteObjectAndKeypoints>(std::bind(
                WriteObjectAndKeypoints,
                frame_id,
                std::placeholders::_1,
                std::placeholders::_2,
                &object_id,
                &object_stream,
                &keypoint_stream));

        // Loop over all images and feed them into the visual odometry
        for (std::int32_t image_id = 0; image_id < static_cast<std::int32_t>(image_files.size()); ++image_id) {
            std::string const& image_name = image_files[image_id];

            // Read the image
            VLOG(google::INFO) << "Processing frame '" << image_name << "' with id " << image_id;
            cv::Mat image = cv::imread(image_name, kLoadImageType);
            DCHECK(image.cols == camera_param.image_size(0))
                    << "Image width does not correspond to given calibration value.";
            DCHECK(image.rows == camera_param.image_size(1))
                    << "Image height does not correspond to given calibration value.";

            // Process the image
            visual_odometry_mono->Process(image_id, image);
        }
        LOG(INFO) << "End of program";
    }  // scope of main timer

    if (FLAGS_with_timing) {
        LOG(INFO) << nie::ScopedTimer::FormattedStatistics();
    }

    return 0;
}
