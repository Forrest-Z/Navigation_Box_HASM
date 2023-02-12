/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_POSITIONING_XSENS_GPS_IMU_HPP
#define NIE_HARDWARE_POSITIONING_XSENS_GPS_IMU_HPP

#include <chrono>
#include <iostream>

#include <glog/logging.h>
#include <xsensdeviceapi.h>
#include <nie/core/container/mt_circular_buffer.hpp>

#include "helper_xsens_gps_imu.hpp"
#include "nie/hardware/positioning/gps_imu_measurements.hpp"

namespace nie {

namespace detail {

using MeasurementBuffer = mt::MtCircularBuffer<GpsMeasurement>;

class PacketCallback : public XsCallback {
public:
    PacketCallback(MeasurementBuffer* measurements) : measurements_(measurements) {}

protected:
    virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override {
        if (!packet->containsPositionLLA() || !packet->containsOrientation()) {
            VLOG(8) << "Missing packet information (lla, o): " << packet->containsPositionLLA() << ", "
                    << packet->containsOrientation();
            return;
        }

        XsVector position = packet->positionLLA();
        VLOG(7) << "Position LLA: " << position[0] << ", " << position[1] << ", " << position[2];

        XsEuler orientation = packet->orientationEuler();
        // FIXME(JBr): How to use it?
        VLOG(7) << "Orienation Euler: " << orientation.yaw() << "," << orientation.pitch() << ", "
                << orientation.roll();

        auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
        // FIXME(JBr):
        VLOG(7) << "Timestamp: " << timestamp;

        // FIXME(JBr): Use synchronized time.
        measurements_->BlockingPushBack({position[0], position[1], timestamp});
        // measurements_->BlockingPushBack({ position[0], position[1], static_cast<std::int64_t>(packet->sampleTime64())
        // });

        // DEBUG
        // nie::PrintInfoXsDataPacket(*packet);
    }

private:
    MeasurementBuffer* measurements_;
};

}  // namespace detail

class XsensGpsImu {
public:
    using MeasurementBuffer = detail::MeasurementBuffer;

    XsensGpsImu(std::size_t const& buffer_size, std::size_t const& sample_frequency);
    ~XsensGpsImu();

    MeasurementBuffer& measurements();

private:
    XsOutputConfigurationArray CreateOutputConfiguration(std::size_t const& sample_frequency);

    MeasurementBuffer measurements_;
    XsControl* control_;
    XsDevice* device_;
    detail::PacketCallback callback_;
};

}  // namespace nie

#endif  // NIE_HARDWARE_POSITIONING_XSENS_GPS_IMU_HPP
