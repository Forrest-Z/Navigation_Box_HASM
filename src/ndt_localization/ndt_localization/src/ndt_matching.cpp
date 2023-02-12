/**
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------------------------
 * This is a modified version of the original ndt_matching.cpp code from Autoware.
 * The changes made to this code, of which a summary is listed below, are copyrighted:
 * ------------------------------------------------------------------------------------
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 * ------------------------------------------------------------------------------------
 *
 * List of changes:
 * * A lot of work has been done to refactor this file, with an focus on readability and maintainability.
 * * The code style has been adjusted in many places to conform to the AIIM Code Style.
 * * Some topics that were previously published were removed as we did not see any use for them.
 * * Configuration callback and initialpose callback have been removed / refactored into main initialization.
 * * The input/tuning parameters are generalized and the defaults have been moved to the accompanying launch file.
 * * These parameters are now also all checked whether they are present, so the launch file is considered mandatory to
 * be used.
 * * Most of the global variables have been refactored to be used local only.
 * * Duplicate pieces of code have been refactored to their own function which is called instead.
 * * Big functions have been split up into smaller atomic functions.
 * * The 'pose' struct has been reworked into nie::ndt::NdtPose and operators are used to reduce verbosity.
 * * Velocity and acceleration variables have been moved to a nie::ndt::NdtRateOfChange struct.
 * * The pose prediction now actually is functional and bugs were fixed.
 * * Velocity and yaw rate are being smoothed to improve pose prediction.
 * * Docstrings have been added to functions, where also is indicated whether those functions were already present in
 * the original version.
 * * Renamed exe_time to processing_time
 * * Removed the ndt reliability as it was not a reliable metric
 * * Added a publisher of the NDT Transformation Probability
 * * Upgrade to ROS2
 * * Removal of IMU/Odometry support
 */
#include <pthread.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

// ROS
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

// Tf
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// NDT
#include <ndt_anh_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#ifdef USE_CUDA
#include <ndt_anh_gpu/NormalDistributionsTransform.h>
#endif
#ifdef USE_PCL_OPENMP
#include <ndt_pcl_openmp/ndt.h>
#endif

// Messages
#include <aiim_autoware_msgs/msg/vehicle_kinematic_state.hpp>
// #include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <aiim_autoware_msgs/msg/vehicle_kinematic_state_autoware.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <ndt_msgs/msg/ndt_stat.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

// AIIM NDT Classes
#include "ndt_matching_pose_prediction.hpp"
#include "ndt_matching_trajectory_estimator.hpp"
#include "ndt_matching_types.hpp"

// Node
rclcpp::Node::SharedPtr node;

// Node Parameters
// Main parameters
static nie::ndt::MethodType method_type;
static double gnss_reinit_fitness;

static bool enable_gnss;
static bool smooth_velocity;

static std::string offset_type;  // linear, zero, quadratic
static std::string gnss_topic;
static std::string points_topic;

// Ndt Tunable parameters
static int max_iterations;  // Maximum iterations
static float resolution;    // Resolution
static double step_size;    // Step size
static double tf_epsilon;   // Transformation epsilon

// Node Variables
// Publishers and msgs
static rclcpp::Publisher<aiim_autoware_msgs::msg::VehicleKinematicState>::SharedPtr vehicle_state_pub;
// static rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>::SharedPtr autoware_vehicle_state_pub;
static rclcpp::Publisher<aiim_autoware_msgs::msg::VehicleKinematicStateAutoware>::SharedPtr autoware_vehicle_state_pub;
static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub;
static rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr estimate_twist_pub;
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr estimated_vel_kmph_pub;
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr estimated_yaw_rads_pub;
static rclcpp::Publisher<ndt_msgs::msg::NdtStat>::SharedPtr ndt_stat_pub;
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_ndt_matching_pub;
static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr tf_probability_pub;

// Other variables
// Poses
static nie::ndt::Pose previous_pose;
static nie::ndt::Pose previous_gnss_pose;

// Ndt Versions
// PCL Generic
static pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
// ANH CPU
static nie::ndt::cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> anh_ndt;
// ANH GPU
#ifdef USE_CUDA
static std::shared_ptr<nie::ndt::gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr =
        std::make_shared<nie::ndt::gpu::GNormalDistributionsTransform>();
#endif
// PCL OpenMP
#ifdef USE_PCL_OPENMP
static pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> omp_ndt;
#endif

// Other
static bool map_loaded = false;
static bool init_pos_set = false;
static double map_time = 0.0;
static double fitness_score = 0.0;

// Rate of changes [m/s]
// Twist (Velocities)
static nie::ndt::RateOfChange current_velocities;
static nie::ndt::RateOfChange previous_velocities;
static nie::ndt::RateOfChange previous_previous_velocities;
// Accel(eration)
static nie::ndt::RateOfChange current_acceleration;

// Time tracking
static rclcpp::Time previous_scan_time;
static rclcpp::Time previous_gnss_time;

static Eigen::Matrix4f tf_baselink_to_ndt;
pthread_mutex_t mutex;

// Trajectory estimation and recovery
static nie::ndt::TrajectoryEstimator trajectory_estimator;

// =============================================
// Generic helper functions
// =============================================

static std::pair<nie::ndt::Pose, double> GetBestEstimatedTrajectory(double tf_probability) {
    std::vector<nie::ndt::Pose> potential_poses(8);
    for (unsigned int i = 8; i >= 1; --i) {
        potential_poses.emplace_back(trajectory_estimator.EstimateWithLag(i));
        Eigen::Matrix4f tentative_T = potential_poses.back().ToEigenMatrix4f();

        double tentative_probability;

        /// TODO: Remove this duplication?
        // Run NDT for estimated poses.
        if (method_type == nie::ndt::MethodType::PCL_GENERIC) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            ndt.align(*output_cloud, tentative_T);
            tentative_probability = ndt.getTransformationProbability();
        } else if (method_type == nie::ndt::MethodType::PCL_ANH) {
            anh_ndt.align(tentative_T);
            tentative_probability = anh_ndt.getTransformationProbability();
        }
#ifdef USE_CUDA
        else if (method_type == nie::ndt::MethodType::PCL_ANH_GPU) {
            anh_gpu_ndt_ptr->align(tentative_T);
            tentative_probability = anh_gpu_ndt_ptr->getTransformationProbability();
        }
#endif
#ifdef USE_PCL_OPENMP
        else if (method_type == nie::ndt::MethodType::PCL_OPENMP) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            omp_ndt.align(*output_cloud, tentative_T);
            tentative_probability = omp_ndt.getTransformationProbability();
        }
#endif

        // Let's be a bit greedy and stop at the first "improvement". Data shows
        // a correlation between looking further into the past and greater
        // negative deltas, that's why we started at 8.
        double probability_delta = tentative_probability - tf_probability;
        if (probability_delta > 0) {
            return {potential_poses.back(), probability_delta};
        }
    }

    // If we didn't find an improvement, return identity.
    return {nie::ndt::Pose{}, 0.0};
}

/** Tries to obtain a node parameter from the rosparam server and fatals if not found
 *
 * @param node_handle: Node handle (private recommended)
 * @param param_name: Parameter name (e.g. "/example_param")
 * @param val: Reference to value where to save the obtained parameter
 */
template <typename T>
static void getParamOrFail(const std::string& param_name, T& val) {
    node->declare_parameter(param_name);
    if (node->get_parameter(param_name, val) == false) {
        RCLCPP_FATAL_STREAM(node->get_logger(), param_name << " is not set.");
        rclcpp::shutdown();
    }
}

/** Wraps upperbound of given value around 0
 *
 * @warning: Only subtracts 2* a_max. Higher values might still be above a_max.
 * @warning: Does not wrap values from -a_max to >0
 * @note: Legacy autoware code.
 */
static double WrapDown(double a_num, const double a_max) {
    if (a_num >= a_max) {
        a_num -= 2.0 * a_max;
    }
    return a_num;
}

/** Wraps radians above pi to below 0
 *
 * @warning: Only subtracts 2* pi! Higher values might still be above pi!
 * @note: Legacy autoware code.
 */
static double WrapDownPi(const double a_angle_rad) { return WrapDown(a_angle_rad, M_PI); }

/** Predicts pose based on selected method
 *
 * @param previous: Previous (reference) pose
 * @param dt: Time since last pose in seconds
 *
 * @return Estimated pose
 *
 * @note: uses the static parameters: offset_type
 */
static nie::ndt::Pose PredictPose(nie::ndt::Pose const& previous, double dt) {
    if (offset_type == "linear") {
        return nie::ndt::PredictPoseLinear(previous, current_velocities, dt);
    } else if (offset_type == "quadratic") {
        return nie::ndt::PredictPoseQuadratic(previous, current_velocities, current_acceleration, dt);
    } else if (offset_type == "zero") {
        return previous;
    }
    RCLCPP_WARN(node->get_logger(), "Offset type not recognized! Defaulting to zero.");
    return previous;
}

// =============================================
// NDT helper functions
// =============================================

/** Transforms an (Eigen::Matrix4f) transformation matrix to nie::ndt::NdtPose
 *
 * @param tf_matrix: input transformation matrix formatted as [3x3 rotation, 3x1 translation; 1x3 null; 1x1 1]
 *
 * @return nie::ndt::NdtPose struct
 */
static nie::ndt::Pose CalculatePoseFromTfMatrix(Eigen::Matrix4f const& tf_matrix) {
    tf2::Matrix3x3 mat_rot;  // localizer
    mat_rot.setValue(
            static_cast<double>(tf_matrix(0, 0)),
            static_cast<double>(tf_matrix(0, 1)),
            static_cast<double>(tf_matrix(0, 2)),
            static_cast<double>(tf_matrix(1, 0)),
            static_cast<double>(tf_matrix(1, 1)),
            static_cast<double>(tf_matrix(1, 2)),
            static_cast<double>(tf_matrix(2, 0)),
            static_cast<double>(tf_matrix(2, 1)),
            static_cast<double>(tf_matrix(2, 2)));

    // Set localizer_pose
    nie::ndt::Pose result{};
    result.x = tf_matrix(0, 3);
    result.y = tf_matrix(1, 3);
    result.z = tf_matrix(2, 3);
    mat_rot.getRPY(result.roll, result.pitch, result.yaw, 1);
    return result;
}

/** Smooths the input with the previous two values using moving window average
 *
 * @param current: current velocities
 * @return nie::ndt::NdtRateOfChange struct
 *
 * @warning: Only the absolute and yaw are placed into a deadzone
 */
static nie::ndt::RateOfChange SmoothVelocities(nie::ndt::RateOfChange const& current) {
    nie::ndt::RateOfChange result = (current + previous_velocities + previous_previous_velocities) / 3.0;

    // If value is too small, we assume it just to be noise
    double constexpr velocity_threshold = 0.2;
    double constexpr yaw_rate_threshold = 0.01;
    result.absolute = std::abs(result.absolute) < velocity_threshold ? 0 : result.absolute;
    result.linear_x = std::abs(result.linear_x) < velocity_threshold ? 0 : result.linear_x;
    result.linear_y = std::abs(result.linear_y) < velocity_threshold ? 0 : result.linear_y;
    result.linear_z = std::abs(result.linear_z) < velocity_threshold ? 0 : result.linear_z;
    result.angular_z = std::abs(result.angular_z) < yaw_rate_threshold ? 0 : result.angular_z;

    return result;
}

// =============================================
// Callbacks
// =============================================

/** Callback for map update
 *
 * @param input: pc2 map msg
 */
static void map_callback(sensor_msgs::msg::PointCloud2::SharedPtr input) {
    RCLCPP_INFO(node->get_logger(), "Update points_map.");

    std::chrono::time_point<std::chrono::system_clock> map_start;

    // Convert the data type(from sensor_msgs to pcl).
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *map_ptr);

    // Here we are switching to the new map, this is done by first creating a new instance of the ndt node.
    // This is then filled with the correct parameters and with the new map.
    // If this is complete, we access the main ndt mutex and we replace the old ndt with the updated one.
    // This is done to allow the ndt to continue to localize itself while the new one is initializing, which might
    // take some time (>1 second).
    if (method_type == nie::ndt::MethodType::PCL_GENERIC) {
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_ndt;
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        new_ndt.setResolution(resolution);
        new_ndt.setInputTarget(map_ptr);
        new_ndt.setMaximumIterations(max_iterations);
        new_ndt.setStepSize(step_size);
        new_ndt.setTransformationEpsilon(tf_epsilon);

        new_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

        pthread_mutex_lock(&mutex);
        map_start = std::chrono::system_clock::now();
        ndt = new_ndt;
        pthread_mutex_unlock(&mutex);
    } else if (method_type == nie::ndt::MethodType::PCL_ANH) {
        nie::ndt::cpu::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_anh_ndt;
        new_anh_ndt.setResolution(resolution);
        new_anh_ndt.setInputTarget(map_ptr);
        new_anh_ndt.setMaximumIterations(max_iterations);
        new_anh_ndt.setStepSize(step_size);
        new_anh_ndt.setTransformationEpsilon(tf_epsilon);

        pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointXYZ dummy_point;
        dummy_scan_ptr->push_back(dummy_point);
        new_anh_ndt.setInputSource(dummy_scan_ptr);

        new_anh_ndt.align(Eigen::Matrix4f::Identity());

        pthread_mutex_lock(&mutex);
        map_start = std::chrono::system_clock::now();
        anh_ndt = new_anh_ndt;
        pthread_mutex_unlock(&mutex);
    }
#ifdef USE_CUDA
    else if (method_type == nie::ndt::MethodType::PCL_ANH_GPU) {
        std::shared_ptr<nie::ndt::gpu::GNormalDistributionsTransform> new_anh_gpu_ndt_ptr =
                std::make_shared<nie::ndt::gpu::GNormalDistributionsTransform>();
        new_anh_gpu_ndt_ptr->setResolution(resolution);
        new_anh_gpu_ndt_ptr->setInputTarget(map_ptr);
        new_anh_gpu_ndt_ptr->setMaximumIterations(max_iterations);
        new_anh_gpu_ndt_ptr->setStepSize(step_size);
        new_anh_gpu_ndt_ptr->setTransformationEpsilon(tf_epsilon);

        pcl::PointCloud<pcl::PointXYZ>::Ptr dummy_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointXYZ dummy_point;
        dummy_scan_ptr->push_back(dummy_point);
        new_anh_gpu_ndt_ptr->setInputSource(dummy_scan_ptr);

        new_anh_gpu_ndt_ptr->align(Eigen::Matrix4f::Identity());

        pthread_mutex_lock(&mutex);
        map_start = std::chrono::system_clock::now();
        anh_gpu_ndt_ptr = new_anh_gpu_ndt_ptr;
        pthread_mutex_unlock(&mutex);
    }
#endif
#ifdef USE_PCL_OPENMP
    else if (method_type == nie::ndt::MethodType::PCL_OPENMP) {
        pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> new_omp_ndt;
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        new_omp_ndt.setResolution(resolution);
        new_omp_ndt.setInputTarget(map_ptr);
        new_omp_ndt.setMaximumIterations(max_iterations);
        new_omp_ndt.setStepSize(step_size);
        new_omp_ndt.setTransformationEpsilon(tf_epsilon);

        new_omp_ndt.align(*output_cloud, Eigen::Matrix4f::Identity());

        pthread_mutex_lock(&mutex);
        map_start = std::chrono::system_clock::now();
        omp_ndt = new_omp_ndt;
        pthread_mutex_unlock(&mutex);
    }
#endif
    std::chrono::time_point<std::chrono::system_clock> map_end = std::chrono::system_clock::now();
    map_time = std::chrono::duration_cast<std::chrono::microseconds>(map_end - map_start).count() / 1000.0;
    map_loaded = true;
}

/** Callback for gnss update
 *
 * @param input: gnss msg
 *
 * @note: legacy autoware code
 */
static void gnss_callback(geometry_msgs::msg::PoseStamped::SharedPtr input) {
    tf2::Quaternion gnss_q(
            input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z, input->pose.orientation.w);
    tf2::Matrix3x3 gnss_m(gnss_q);

    nie::ndt::Pose current_gnss_pose{};
    current_gnss_pose.x = input->pose.position.x;
    current_gnss_pose.y = input->pose.position.y;
    current_gnss_pose.z = input->pose.position.z;
    gnss_m.getRPY(current_gnss_pose.roll, current_gnss_pose.pitch, current_gnss_pose.yaw);

    previous_gnss_pose = current_gnss_pose;
    rclcpp::Time current_gnss_time = input->header.stamp;
    const double diff_time = (current_gnss_time - previous_gnss_time).seconds();

    if ((enable_gnss && !init_pos_set) || (fitness_score >= gnss_reinit_fitness)) {
        previous_pose = previous_gnss_pose;

        current_velocities = nie::ndt::RateOfChange::FromPoseDifference(current_gnss_pose, previous_pose, diff_time);
        current_acceleration = nie::ndt::RateOfChange();

        init_pos_set = true;
    }

    previous_gnss_pose = current_gnss_pose;
    previous_gnss_time = current_gnss_time;
}




/** Callback for input points callback
 *
 * @param input: filtered point cloud coming from voxel grid filter
 *
 */
static void points_callback(sensor_msgs::msg::PointCloud2::SharedPtr input) {
    if (!map_loaded) {
        RCLCPP_WARN(node->get_logger(), "Receiving lidar points but no map loaded yet");
        return;
    }
    if (!init_pos_set) {
        if (enable_gnss) {
            RCLCPP_WARN(node->get_logger(), "Receiving lidar points but no gnss is received yet");
        } else {
            RCLCPP_WARN(node->get_logger(), "Receiving lidar points but no initial position is set yet");
        }
        return;
    }
    std::chrono::time_point<std::chrono::system_clock> matching_start = std::chrono::system_clock::now();

    // Convert msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *filtered_scan_ptr);

    // Guess the initial gross estimation of the transformation
    rclcpp::Time current_scan_time = input->header.stamp;
    double diff_time = (current_scan_time - previous_scan_time).seconds();
    previous_scan_time = current_scan_time;
    RCLCPP_INFO(node->get_logger(), "DELTA T - %lf", diff_time);
    nie::ndt::Pose predict_pose_for_ndt = PredictPose(previous_pose, diff_time);

    // Calculate Initial guess
    Eigen::Matrix4f init_guess = predict_pose_for_ndt.ToEigenMatrix4f() * tf_baselink_to_ndt;

    // NDT Status variables
    double tf_probability;
    int iteration;
    bool has_converged;
    std::chrono::time_point<std::chrono::system_clock> align_start;
    std::chrono::time_point<std::chrono::system_clock> align_end;
    std::chrono::time_point<std::chrono::system_clock> fitness_start;
    std::chrono::time_point<std::chrono::system_clock> fitness_stop;
    Eigen::Matrix4f tf_ndt(Eigen::Matrix4f::Identity());  // base_link

    pthread_mutex_lock(&mutex);
    // Run NDT
    if (method_type == nie::ndt::MethodType::PCL_GENERIC) {
        ndt.setInputSource(filtered_scan_ptr);

        align_start = std::chrono::system_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        ndt.align(*output_cloud, init_guess);
        align_end = std::chrono::system_clock::now();

        has_converged = ndt.hasConverged();

        tf_ndt = ndt.getFinalTransformation();
        iteration = ndt.getFinalNumIteration();

        fitness_start = std::chrono::system_clock::now();
        fitness_score = ndt.getFitnessScore();
        fitness_stop = std::chrono::system_clock::now();

        tf_probability = ndt.getTransformationProbability();
    } else if (method_type == nie::ndt::MethodType::PCL_ANH) {
        anh_ndt.setInputSource(filtered_scan_ptr);

        align_start = std::chrono::system_clock::now();
        anh_ndt.align(init_guess);
        align_end = std::chrono::system_clock::now();

        has_converged = anh_ndt.hasConverged();

        tf_ndt = anh_ndt.getFinalTransformation();
        iteration = anh_ndt.getFinalNumIteration();

        fitness_start = std::chrono::system_clock::now();
        fitness_score = anh_ndt.getFitnessScore();
        fitness_stop = std::chrono::system_clock::now();

        tf_probability = anh_ndt.getTransformationProbability();
    }
#ifdef USE_CUDA
    else if (method_type == nie::ndt::MethodType::PCL_ANH_GPU) {
        anh_gpu_ndt_ptr->setInputSource(filtered_scan_ptr);

        align_start = std::chrono::system_clock::now();
        anh_gpu_ndt_ptr->align(init_guess);
        align_end = std::chrono::system_clock::now();

        has_converged = anh_gpu_ndt_ptr->hasConverged();

        tf_ndt = anh_gpu_ndt_ptr->getFinalTransformation();
        iteration = anh_gpu_ndt_ptr->getFinalNumIteration();

        fitness_start = std::chrono::system_clock::now();
        fitness_score = anh_gpu_ndt_ptr->getFitnessScore();
        fitness_stop = std::chrono::system_clock::now();

        tf_probability = anh_gpu_ndt_ptr->getTransformationProbability();
    }
#endif
#ifdef USE_PCL_OPENMP
    else if (method_type == nie::ndt::MethodType::PCL_OPENMP) {
        omp_ndt.setInputSource(filtered_scan_ptr);

        align_start = std::chrono::system_clock::now();
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        omp_ndt.align(*output_cloud, init_guess);
        align_end = std::chrono::system_clock::now();

        has_converged = omp_ndt.hasConverged();

        tf_ndt = omp_ndt.getFinalTransformation();
        iteration = omp_ndt.getFinalNumIteration();

        fitness_start = std::chrono::system_clock::now();
        fitness_score = omp_ndt.getFitnessScore();
        fitness_stop = std::chrono::system_clock::now();

        tf_probability = omp_ndt.getTransformationProbability();
    }
#endif
    // Calculate poses from transformation matrices
    nie::ndt::Pose localizer_pose = CalculatePoseFromTfMatrix(tf_ndt);
    nie::ndt::Pose baselink_pose = CalculatePoseFromTfMatrix(tf_ndt * tf_baselink_to_ndt.inverse());

    // TODO: replace the following hacky usage of the trajectory estimation with
    // an actual "reaction" based on the status report.
    // trajectory_estimator.Update(baselink_pose, diff_time);

    // if (tf_probability < 0.5) {
    //     RCLCPP_INFO(node->get_logger(), "NDT Yielded a Low-probability TF. Attempting to find a better
    //     alternative..."); nie::ndt::Pose tentative_baselink_pose{}; double probability_delta{};
    //     std::tie(tentative_baselink_pose, probability_delta) = GetBestEstimatedTrajectory(tf_probability);
    //     if (tentative_baselink_pose != nie::ndt::Pose{}) {
    //         RCLCPP_INFO(node->get_logger(), "Alternative found! %lf better probability.", probability_delta);
    //         // We found something better!
    //         baselink_pose = tentative_baselink_pose;
    //         trajectory_estimator.ReplaceLast(baselink_pose, diff_time);
    //     }
    // }

    pthread_mutex_unlock(&mutex);

    // Publish poses
    std::string const pose_frame_id = "map";
    ndt_pose_pub->publish(baselink_pose.ToPoseStamped(current_scan_time, pose_frame_id));
    // Compute the velocity and acceleration
    current_velocities = nie::ndt::RateOfChange::FromPoseDifference(baselink_pose, previous_pose, diff_time);
    current_acceleration = nie::ndt::RateOfChange::FromDifference(current_velocities, previous_velocities, diff_time);

    // Update previous pose and velocities
    previous_pose = baselink_pose;
    previous_previous_velocities = previous_velocities;
    previous_velocities = current_velocities;

    // Smooth velocity to reduce shaky behaviour
    if (smooth_velocity) {
        current_velocities = SmoothVelocities(current_velocities);
    }

    // Publish estimated velocity and acceleration
    std_msgs::msg::Float32 estimated_vel_kmph;
    std_msgs::msg::Float32 estimated_yaw_rads;
    double constexpr kMpsToKmph = 3.6;
    estimated_vel_kmph.data = current_velocities.absolute * kMpsToKmph;
    estimated_yaw_rads.data = current_velocities.angular_z;
    estimated_vel_kmph_pub->publish(estimated_vel_kmph);
    estimated_yaw_rads_pub->publish(estimated_yaw_rads);

    // Send TF "/base_link" to "/map"
    static tf2_ros::TransformBroadcaster tf_broadcaster(node);
    geometry_msgs::msg::TransformStamped baselink_to_map;
    tf2::Quaternion baselink_q;

    baselink_to_map.header.stamp = current_scan_time;
    baselink_to_map.header.frame_id = pose_frame_id;
    baselink_to_map.child_frame_id = "/base_link";
    baselink_to_map.transform.translation.x = baselink_pose.x;
    baselink_to_map.transform.translation.y = baselink_pose.y;
    baselink_to_map.transform.translation.z = baselink_pose.z;
    baselink_q.setRPY(baselink_pose.roll, baselink_pose.pitch, baselink_pose.yaw);
    baselink_to_map.transform.rotation = tf2::toMsg(baselink_q);

    tf_broadcaster.sendTransform(baselink_to_map);

    // Send vehicle kinematic state
    aiim_autoware_msgs::msg::VehicleKinematicState msg_vehicle_state;
    msg_vehicle_state.header = baselink_to_map.header;
    msg_vehicle_state.state.x = baselink_pose.x;
    msg_vehicle_state.state.y = baselink_pose.y;
    msg_vehicle_state.state.heading.real = cos(baselink_pose.yaw / 2);
    msg_vehicle_state.state.heading.imag = sin(baselink_pose.yaw / 2);
    msg_vehicle_state.state.longitudinal_velocity_mps = current_velocities.absolute;
    msg_vehicle_state.state.acceleration_mps2 = current_acceleration.absolute;
    msg_vehicle_state.state.heading_rate_rps = current_velocities.angular_z;
    msg_vehicle_state.delta = baselink_to_map.transform;
    vehicle_state_pub->publish(msg_vehicle_state);

    // Send vehicle kinematic state to autoware
    aiim_autoware_msgs::msg::VehicleKinematicStateAutoware autoware_msg_vehicle_state;
    autoware_msg_vehicle_state.header = baselink_to_map.header;
    autoware_msg_vehicle_state.state.pose = baselink_pose.ToPose();
    // autoware_msg_vehicle_state.state.pose.position.x = baselink_pose.x;
    // autoware_msg_vehicle_state.state.pose.position.y = baselink_pose.y;
    // autoware_msg_vehicle_state.state.pose.orientation.w = cos(baselink_pose.yaw / 2);
    // autoware_msg_vehicle_state.state.pose.orientation.z = sin(baselink_pose.yaw / 2);
    autoware_msg_vehicle_state.state.acceleration_mps2 = current_acceleration.absolute;
    autoware_msg_vehicle_state.state.longitudinal_velocity_mps = current_velocities.absolute;
    autoware_msg_vehicle_state.state.heading_rate_rps = current_velocities.angular_z;
    autoware_msg_vehicle_state.state.lateral_velocity_mps = current_velocities.linear_x;
    autoware_msg_vehicle_state.delta = baselink_to_map.transform;
    autoware_vehicle_state_pub->publish(autoware_msg_vehicle_state);
    


    // Calculate ndt time(s)
    std::chrono::time_point<std::chrono::system_clock> matching_end = std::chrono::system_clock::now();
    double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
    double fitness_time =
            std::chrono::duration_cast<std::chrono::microseconds>(fitness_stop - fitness_start).count() / 1000.0;
    double processing_time =
            std::chrono::duration_cast<std::chrono::microseconds>(matching_end - matching_start).count() / 1000.0;
    double response_time = (node->get_clock()->now() - current_scan_time).seconds() * 1000;
    double latency = response_time - processing_time;

    // Publish ndt time
    std_msgs::msg::Float32 time_ndt_matching;
    time_ndt_matching.data = processing_time;
    time_ndt_matching_pub->publish(time_ndt_matching);

    // Publish estimated twist
    estimate_twist_pub->publish(current_velocities.ToTwistStamped(current_scan_time, "/base_link"));

    // Publish /ndt_stat
    ndt_msgs::msg::NdtStat ndt_stat_msg;
    ndt_stat_msg.header.stamp = current_scan_time;
    ndt_stat_msg.converged = has_converged;
    ndt_stat_msg.response_time = response_time;
    ndt_stat_msg.map_time = map_time;
    ndt_stat_msg.latency = latency;
    ndt_stat_msg.processing_time = processing_time;
    ndt_stat_msg.align_time = align_time;
    ndt_stat_msg.fitness_time = fitness_time;
    ndt_stat_msg.iteration = iteration;
    ndt_stat_msg.tf_probability = tf_probability;
    ndt_stat_msg.fitness_score = fitness_score;
    ndt_stat_msg.velocity = current_velocities.absolute;
    ndt_stat_msg.yaw_rate = current_velocities.angular_z;
    ndt_stat_msg.acceleration = current_acceleration.absolute;
    ndt_stat_pub->publish(ndt_stat_msg);

    // Publish NDT Transformation Probability
    std_msgs::msg::Float32 tf_probability_msg;
    tf_probability_msg.data = tf_probability;
    tf_probability_pub->publish(tf_probability_msg);

    // Log ndt information
    RCLCPP_INFO_STREAM(node->get_logger(), "-----------------------------------------------------------------");
    RCLCPP_INFO_STREAM(node->get_logger(), "Timestamp: " << input->header.stamp.nanosec);
    RCLCPP_INFO_STREAM(node->get_logger(), "Frame ID: " << input->header.frame_id);
    RCLCPP_INFO_STREAM(
            node->get_logger(), "Number of Filtered Scan Points: " << filtered_scan_ptr->size() << " points.");
    RCLCPP_INFO_STREAM(node->get_logger(), "NDT has converged: " << has_converged);
    RCLCPP_INFO_STREAM(node->get_logger(), "Fitness Score: " << fitness_score);
    RCLCPP_INFO_STREAM(node->get_logger(), "Transformation Probability: " << tf_probability);
    RCLCPP_INFO_STREAM(node->get_logger(), "Processing Time: " << processing_time << " ms.");
    RCLCPP_INFO_STREAM(node->get_logger(), "Number of Iterations: " << iteration);
    RCLCPP_INFO_STREAM(node->get_logger(), "(x,y,z,roll,pitch,yaw): ");
    RCLCPP_INFO_STREAM(
            node->get_logger(),
            "(" << baselink_pose.x << ", " << baselink_pose.y << ", " << baselink_pose.z << ", " << baselink_pose.roll
                << ", " << baselink_pose.pitch << ", " << baselink_pose.yaw << ")");
    RCLCPP_INFO_STREAM(node->get_logger(), "Align time: " << align_time);
    RCLCPP_INFO_STREAM(node->get_logger(), "Get fitness score time: " << fitness_time);
    RCLCPP_INFO_STREAM(node->get_logger(), "Last map update time: " << map_time);
    RCLCPP_INFO_STREAM(node->get_logger(), "Total Reponse Time: " << response_time);
    RCLCPP_INFO_STREAM(node->get_logger(), "-----------------------------------------------------------------");
}

// =============================================
// Initialization helper functions
// =============================================

/** Obtains transform from localizer to base link tf frames
 *
 * @note: stores the result in the static global variable 'tf_baselink_to_ndt'
 */
static void GetTransformToBaseLink(std::string const& localizer) {
    // Get transform from localizer to base_link.
    RCLCPP_INFO(node->get_logger(), "-----------------------------------------------------------------");
    RCLCPP_INFO(node->get_logger(), "Getting transform from localizer to base_link");

    // We need to wait some time before the frame will become known to tf2.
    // We cannot simply wait for transform as it will then give an error that the frame is unknown.
    // Thus, we need to wait for the static_tf_publisher to publish it for the first time.
    // So, we sleep for 100 ms here as that is the maximum time between publishes of the static_tf_publisher.
    // TODO(tijs.vandersmagt): This still is based on debugging and trial and error. Possibly find a better solution
    // for this, or the exact reason.
    usleep(1000000);

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    rclcpp::Duration const transform_timeout(1.0);
    std::string const base_link_frame = "base_link";
    geometry_msgs::msg::TransformStamped tf_local_to_baselink;
    try {
        tf_local_to_baselink = tf_buffer.lookupTransform(base_link_frame, localizer, node->now(), transform_timeout);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_STREAM(
                node->get_logger(),
                "Could not get transform from " << localizer << " to base_link, defaulting to identity.\n"
                                                << ex.what());
    }
    // Print information
    RCLCPP_INFO_STREAM(
            node->get_logger(),
            "(tf_t_x,tf_t_y,tf_t_z,tf_q_x,tf_q_y,tf_q_z,tf_q_w): ("
                    << tf_local_to_baselink.transform.translation.x << ", "
                    << tf_local_to_baselink.transform.translation.y << ", "
                    << tf_local_to_baselink.transform.translation.z << ", " << tf_local_to_baselink.transform.rotation.x
                    << ", " << tf_local_to_baselink.transform.rotation.y << ", "
                    << tf_local_to_baselink.transform.rotation.z << ", " << tf_local_to_baselink.transform.rotation.w
                    << ")");
    RCLCPP_INFO(node->get_logger(), "-----------------------------------------------------------------");

    // Calculate 4x4 transformation matrix from obtained tf2 transform.
    // The transform gives Eigen::Affine3d (Set of transformations), from which a Eigen::Matrix4d can be extracted.
    // But, the tf_baselink_to_ndt is Matrix4f, so a conversion to float is required.
    tf_baselink_to_ndt = tf2::transformToEigen(tf_local_to_baselink).matrix().cast<float>();
}

/** Sets initial position of the ndt
 *
 * @param initial: The pose to set the initial position to
 * @note: also sets the previous times
 */
static void SetInitialPosition(nie::ndt::Pose const& initial) {
    // Setting position and posture for the first time.
    previous_pose = initial;
    previous_scan_time = node->now();
    previous_gnss_time = node->now();
    init_pos_set = true;
}

/** Callback for localization initialization through rviz pose estimate.
 *  The NDT localizer subscribes to the /initialpose topic so that it would be possible to give the initial pose through rviz instead of yaml file.
 * @param input: rviz initial pose estimate
 *
 */
static void rviz_initialpose_callback(const typename geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg_ptr)
{
    tf2::Quaternion q(
        msg_ptr->pose.pose.orientation.x,
        msg_ptr->pose.pose.orientation.y,
        msg_ptr->pose.pose.orientation.z,
        msg_ptr->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    static nie::ndt::Pose initial_pose;
    initial_pose.x = msg_ptr->pose.pose.position.x;
    initial_pose.y = msg_ptr->pose.pose.position.y;
    // initial_pose.z = msg_ptr->pose.pose.position.z;
    // @ TODO: The z retrieved from rviz is 0 whereas for AIIM's lidar configuration it be 9. Make it smarter to it finds it itself instead of hardcoding
    initial_pose.z = 9.0;
    initial_pose.roll = roll;
    initial_pose.pitch = pitch;
    initial_pose.yaw = yaw;
    
    SetInitialPosition(initial_pose);

    RCLCPP_INFO_STREAM(node->get_logger(), "initial_x: " << initial_pose.x);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_y: " << initial_pose.y);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_z: " << initial_pose.z);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_roll: " << initial_pose.roll);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_pitch: " << initial_pose.pitch);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_yaw: " << initial_pose.yaw);
  }

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    node = rclcpp::Node::make_shared("ndt_matching");

    // Main configuration parameters
    int method_type_tmp;
    std::string localizer;
    getParamOrFail("method_type", method_type_tmp);
    method_type = static_cast<nie::ndt::MethodType>(method_type_tmp);
    getParamOrFail("localizer", localizer);
    getParamOrFail("points_topic", points_topic);

    // Pose Prediction
    getParamOrFail("offset_type", offset_type);
    getParamOrFail("smooth_velocity", smooth_velocity);

    // NDT Tunables
    getParamOrFail("max_iterations", max_iterations);
    getParamOrFail("resolution", resolution);
    getParamOrFail("step_size", step_size);
    getParamOrFail("trans_epsilon", tf_epsilon);

    // Pose initialization
    auto qos_subscribers = rclcpp::SystemDefaultsQoS();
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rviz_init_pose_sub;
    rviz_init_pose_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", qos_subscribers, rviz_initialpose_callback);
    static nie::ndt::Pose initial_pose;
    getParamOrFail("enable_gnss", enable_gnss);
    if (enable_gnss) {
        getParamOrFail("gnss_reinit_fitness", gnss_reinit_fitness);
        getParamOrFail("gnss_topic", gnss_topic);
    } else {
        getParamOrFail("initial_x", initial_pose.x);
        getParamOrFail("initial_y", initial_pose.y);
        getParamOrFail("initial_z", initial_pose.z);
        getParamOrFail("initial_roll", initial_pose.roll);
        getParamOrFail("initial_pitch", initial_pose.pitch);
        getParamOrFail("initial_yaw", initial_pose.yaw);
    }

    // Output imported parameters
    RCLCPP_INFO_STREAM(node->get_logger(), "--------------------------Node-Parameters------------------------");
    RCLCPP_INFO_STREAM(node->get_logger(), "method_type: " << static_cast<int>(method_type));
    RCLCPP_INFO_STREAM(node->get_logger(), "offset_type: " << offset_type);
    RCLCPP_INFO_STREAM(node->get_logger(), "localizer: " << localizer);
    RCLCPP_INFO_STREAM(node->get_logger(), "points_topic: " << points_topic);
    RCLCPP_INFO_STREAM(node->get_logger(), "-----------------------------------------------------------------");
    RCLCPP_INFO_STREAM(node->get_logger(), "---------------------------NDT-Tunables--------------------------");
    RCLCPP_INFO_STREAM(node->get_logger(), "max_iterations: " << max_iterations);
    RCLCPP_INFO_STREAM(node->get_logger(), "resolution: " << resolution);
    RCLCPP_INFO_STREAM(node->get_logger(), "step_size: " << step_size);
    RCLCPP_INFO_STREAM(node->get_logger(), "trans_epsilon: " << tf_epsilon);
    RCLCPP_INFO_STREAM(node->get_logger(), "-----------------------------------------------------------------");
    RCLCPP_INFO_STREAM(node->get_logger(), "-------------------------Initial-Position------------------------");
    RCLCPP_INFO_STREAM(node->get_logger(), "enable_gnss: " << enable_gnss);
    RCLCPP_INFO_STREAM(node->get_logger(), "gnss_reinit_fitness: " << gnss_reinit_fitness);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_x: " << initial_pose.x);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_y: " << initial_pose.y);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_z: " << initial_pose.z);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_roll: " << initial_pose.roll);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_pitch: " << initial_pose.pitch);
    RCLCPP_INFO_STREAM(node->get_logger(), "initial_yaw: " << initial_pose.yaw);
    RCLCPP_INFO_STREAM(node->get_logger(), "-----------------------------------------------------------------");

#ifndef USE_CUDA
    if (method_type == nie::ndt::MethodType::PCL_ANH_GPU) {
        RCLCPP_WARN(node->get_logger(), "**************************************************************");
        RCLCPP_WARN(node->get_logger(), "[ERROR]PCL_ANH_GPU is not built. Please use other method type.");
        RCLCPP_WARN(node->get_logger(), "**************************************************************");
        exit(1);
    }
#endif
#ifndef USE_PCL_OPENMP
    if (method_type == nie::ndt::MethodType::PCL_OPENMP) {
        RCLCPP_WARN(node->get_logger(), "**************************************************************");
        RCLCPP_WARN(node->get_logger(), "[ERROR]PCL_OPENMP is not built. Please use other method type.");
        RCLCPP_WARN(node->get_logger(), "**************************************************************");
        exit(1);
    }
#endif

    // Set initial position if we are not using gnss
    if (!enable_gnss) {
        SetInitialPosition(initial_pose);
    }
    GetTransformToBaseLink(localizer);

    // Publishers
    auto qos_publishers = rclcpp::SystemDefaultsQoS();
    // auto qos_subscribers = rclcpp::SystemDefaultsQoS();
    autoware_vehicle_state_pub =
            // node->create_publisher<autoware_auto_vehicle_msgs::msg::VehicleKinematicState>("/vehicle/vehicle_kinematic_state", qos_publishers);
            node->create_publisher<aiim_autoware_msgs::msg::VehicleKinematicStateAutoware>("/vehicle/vehicle_kinematic_state", qos_publishers);


    vehicle_state_pub =
            node->create_publisher<aiim_autoware_msgs::msg::VehicleKinematicState>("/vehicle_state", qos_publishers);
    ndt_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/ndt_pose", qos_publishers);
    estimate_twist_pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/estimate_twist", qos_publishers);
    estimated_vel_kmph_pub = node->create_publisher<std_msgs::msg::Float32>("/estimated_vel_kmph", qos_publishers);
    estimated_yaw_rads_pub = node->create_publisher<std_msgs::msg::Float32>("/estimated_yaw_rads", qos_publishers);
    time_ndt_matching_pub = node->create_publisher<std_msgs::msg::Float32>("/time_ndt_matching", qos_publishers);
    ndt_stat_pub = node->create_publisher<ndt_msgs::msg::NdtStat>("/ndt_stat", qos_publishers);
    tf_probability_pub = node->create_publisher<std_msgs::msg::Float32>("/ndt_pose_probability", qos_publishers);

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gnss_sub;

    if (enable_gnss) {
        gnss_sub =
                node->create_subscription<geometry_msgs::msg::PoseStamped>(gnss_topic, qos_subscribers, gnss_callback);
    }

    // Points map
    // Has to be the same as publisher. It is set to be latching (transient_local) to allow late subscription.
    rclcpp::QoS qos_map_sub(rclcpp::KeepLast(1));
    qos_map_sub.transient_local();
    // Map callback will be in its own group to avoid blocking input cloud callback.
    rclcpp::CallbackGroup::SharedPtr map_callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    auto map_sub_opt = rclcpp::SubscriptionOptions();
    map_sub_opt.callback_group = map_callback_group;
    auto map_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("points_map",
                                                                             qos_map_sub,
                                                                             map_callback,
                                                                             map_sub_opt);

    // Subscribing to input point clouds
    // The callback will be in its own to avoid being blocked when a new map is loaded.
    rclcpp::CallbackGroup::SharedPtr input_callback_group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    auto input_sub_opt = rclcpp::SubscriptionOptions();
    input_sub_opt.callback_group = input_callback_group;
    auto points_sub =
            node->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic,
                                                                    qos_subscribers,
                                                                    points_callback,
                                                                    input_sub_opt);

    // Using MultiThreadedExecutor to have the map and input cloud callbacks being executed in parallel.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
