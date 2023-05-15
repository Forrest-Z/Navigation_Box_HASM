/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#pragma once

#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <aiim_roscpp/logging.hpp>

#include "ouster/client.h"
#include "ouster/compat.h"
#include "ouster/types.h"


class OusterInitializerNode : public rclcpp::Node {
public:
    OusterInitializerNode() : rclcpp::Node("ouster_initializer") {
        std::string lidar_mode;
        std::string timestamp_mode;
        GetParameter("hostname", &hostname_);
        GetParameter("udp_dest_host", &udp_dest_host_);
        GetParameter("lidar_mode", &lidar_mode);
        GetParameter("timestamp_mode", &timestamp_mode);
        GetParameter("lidar_port", &lidar_port_);
        GetParameter("imu_port", &imu_port_);
        GetParameter("timeout_sec", &timeout_sec_);

        lidar_mode_ = ouster::sensor::lidar_mode_of_string(lidar_mode);
        if (!lidar_mode_) {
            RCLCPP_FATAL(this->get_logger(), "%s is invalid lidar mode", lidar_mode.c_str());
            throw EXIT_FAILURE;
        }
        timestamp_mode_ = ouster::sensor::timestamp_mode_of_string(timestamp_mode);
        if (!timestamp_mode_) {
            RCLCPP_FATAL(this->get_logger(), "%s is invalid timestamp mode", timestamp_mode.c_str());
            throw EXIT_FAILURE;
        }

        socket_init();
        auto cli = ouster::sensor::init_client(
                hostname_, udp_dest_host_, lidar_mode_, timestamp_mode_, lidar_port_, imu_port_, timeout_sec_);
        if (!cli) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to client %s", hostname_.c_str());
            throw EXIT_FAILURE;
        }
        RCLCPP_INFO(this->get_logger(), "Ouster lidar %s initialized successfully", hostname_.c_str());

        // write metadata file containing lidar settings and instrinsic calibration (?)
        // TODO(davide.congiu): discuss where metadata file has to go to
        meta_file_ = hostname_ + ".json";
        auto metadata = ouster::sensor::get_metadata(*cli);
        write_metadata(meta_file_, metadata);

        socket_quit();
    }

private:
    template <typename T>
    void GetParameter(std::string const& key, T* value) {
        declare_parameter(key);
        RCHECK(get_logger(), "Parameter " << key << " not set.", get_parameter(key, *value));
    }

    // write metadata file
    // TODO(davide.congiu): specify metadata file path?
    void write_metadata(const std::string& meta_file, const std::string& metadata) {
        std::ofstream ofs;
        ofs.open(meta_file);
        ofs << metadata << std::endl;
        ofs.close();
        if (ofs) {
            RCLCPP_INFO(this->get_logger(), "Wrote metadata to %s", meta_file.c_str());
        } else {
            RCLCPP_WARN(
                    this->get_logger(),
                    "Failed to write metadata to %s; check that the path is valid",
                    meta_file.c_str());
        }
    }

    std::string hostname_;
    std::string udp_dest_host_;
    ouster::sensor::lidar_mode lidar_mode_;
    ouster::sensor::timestamp_mode timestamp_mode_;
    unsigned int lidar_port_;
    unsigned int imu_port_;
    unsigned int timeout_sec_;
    std::string meta_file_;
};
