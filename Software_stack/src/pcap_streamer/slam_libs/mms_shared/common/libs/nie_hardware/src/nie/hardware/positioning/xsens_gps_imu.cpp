/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "xsens_gps_imu.hpp"

#include <cassert>

#include <xstypes.h>

namespace nie {

// TODO(Jbr): See if we can check for privileges
XsensGpsImu::XsensGpsImu(std::size_t const& buffer_size, std::size_t const& sample_frequency)
    : measurements_(buffer_size), callback_(&measurements_) {
    control_ = XsControl::construct();
    // FIXME
    assert(control_ != nullptr);

    XsPortInfoArray port_info_array = XsScanner::scanPorts();
    XsPortInfoArray::const_iterator port = port_info_array.begin();

    while (port != port_info_array.end() && !port->deviceId().isMtMk5_710()) ++port;

    if (port == port_info_array.end()) {
        throw std::runtime_error("XsensGpsImu::XsensGpsImu(): No MtMk5 710 device found (try with sudo).");
    }

    if (!control_->openPort(port->portName(), port->baudrate())) {
        throw std::runtime_error(
            "XsensGpsImu::XsensGpsImu(): Could not open port. " + std::string(port->portName().c_str()));
    }

    device_ = control_->device(port->deviceId());
    // FIXME
    assert(device_ != 0);

    // Put the device into configuration mode before configuring the device
    if (!device_->gotoConfig()) {
        throw std::runtime_error(
            "XsensGpsImu::XsensGpsImu(): Could not put device into configuration mode: " +
            std::string(port->deviceId().toString().c_str()));
    }

    XsOutputConfigurationArray configuration = CreateOutputConfiguration(sample_frequency);
    device_->setOutputConfiguration(configuration);
    device_->addCallbackHandler(&callback_);

    if (!device_->gotoMeasurement()) {
        throw std::runtime_error(
            "XsensGpsImu::XsensGpsImu(): Could not put device into measurement mode: " +
            std::string(port->deviceId().toString().c_str()));
    }
}

XsensGpsImu::~XsensGpsImu() {
    if (control_ != nullptr) {
        try {
            control_->closePort(device_);
        } catch (std::exception const& e) {
        }

        control_->destruct();
    }
}

XsOutputConfigurationArray XsensGpsImu::CreateOutputConfiguration(std::size_t const& sample_frequency) {
    if (sample_frequency > 400) {
        throw std::runtime_error(
            "XsensGpsImu::CreateOutputConfiguration(): The device output frequency is set too high: " +
            std::to_string(sample_frequency));
    }

    XsOutputConfigurationArray array;

    // The group contains the flags below it: every message
    // array.push_back(XsOutputConfiguration(XDI_TimestampGroup, sample_frequency));
    // array.push_back(XsOutputConfiguration(XDI_PacketCounter, sample_frequency));
    // This is the combination of tine and coarse:
    // See manual low level communication protocol. Note: Doesn't work stand alone =D Might as well not use it.
    // array.push_back(XsOutputConfiguration(XDI_SampleTime64, sample_frequency));
    array.push_back(XsOutputConfiguration(XDI_SampleTimeFine, sample_frequency));
    array.push_back(XsOutputConfiguration(XDI_SampleTimeCoarse, sample_frequency));

    // The group contains the flags below it: max 400 hz
    // array.push_back(XsOutputConfiguration(XDI_OrientationGroup, sample_frequency));
    // array.push_back(XsOutputConfiguration(XDI_Quaternion, sample_frequency));
    // array.push_back(XsOutputConfiguration(XDI_RotationMatrix, sample_frequency));
    array.push_back(XsOutputConfiguration(XDI_EulerAngles, sample_frequency));

    // The group contains the flags below it: max 400 hz
    // array.push_back(XsOutputConfiguration(XDI_PositionGroup, sample_frequency));
    // The combination of XDI_LatLon + XDI_AltitudeEllipsoid results in the positionLLA
    // package. Otherwise XDI_LatLon equals the package latitudeLongitude
    array.push_back(XsOutputConfiguration(XDI_LatLon, sample_frequency));
    // Earth centered earth fixed might be useful later?
    // array.push_back(XsOutputConfiguration(XDI_PositionEcef, sample_frequency));
    array.push_back(XsOutputConfiguration(XDI_AltitudeEllipsoid, sample_frequency));

    // The group contains the flags below it: max 4 hz (otherwise interp.?)
    // array.push_back(XsOutputConfiguration(XDI_GnssGroup, sample_frequency));
    // Below is the raw gnss data.
    // array.push_back(XsOutputConfiguration(XDI_GnssPvtData, sample_frequency));
    // array.push_back(XsOutputConfiguration(XDI_GnssSatInfo, sample_frequency));

    return array;
}

XsensGpsImu::MeasurementBuffer& XsensGpsImu::measurements() { return measurements_; }

}  // namespace nie
