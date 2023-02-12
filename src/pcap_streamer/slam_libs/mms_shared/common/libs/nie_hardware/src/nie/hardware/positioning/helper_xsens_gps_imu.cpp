/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "helper_xsens_gps_imu.hpp"

#include <iostream>

namespace nie {

void PrintInfoXsDevices() {
    XsControl* control = XsControl::construct();
    XsPortInfoArray port_info_array = XsScanner::scanPorts();

    for (XsPortInfo& port_info : port_info_array) {
        if (!control->openPort(port_info.portName(), port_info.baudrate())) {
            std::cout << "Unable to open port: " << port_info.portName() << std::endl;
        }

        XsDevice* device = control->device(port_info.deviceId());

        PrintInfoXsDevice(*device);

        control->closePort(port_info.portName());
    }
}

void PrintInfoXsDevice(const XsDevice& device, bool recurse) {
    std::cout << std::string(80, '*') << std::endl;
    std::cout << "id: " << device.deviceId().toString() << std::endl;
    std::cout << "typeName: " << device.deviceId().typeName() << std::endl;
    std::cout << "childCount: " << device.childCount() << std::endl;

    if (device.isMotionTracker()) {
        std::cout << "isMotionTracker" << std::endl;
    }

    if (device.deviceId().isMtiX()) {
        std::cout << "isMtiX" << std::endl;
    }

    if (device.deviceId().isMtiX00()) {
        std::cout << "isMtiX00" << std::endl;
    }

    if (device.deviceId().isMtigX00()) {
        std::cout << "isMtigX00" << std::endl;
    }

    if (device.deviceId().isMtigX10()) {
        std::cout << "isMtigX10" << std::endl;
    }

    if (device.deviceId().isMtw()) {
        std::cout << "isMtw" << std::endl;
    }

    if (device.deviceId().isMtw2()) {
        std::cout << "isMtw2" << std::endl;
    }

    if (device.deviceId().isMtx()) {
        std::cout << "isMtx" << std::endl;
    }

    if (device.deviceId().isMtx2()) {
        std::cout << "isMtx2" << std::endl;
    }

    if (device.deviceId().isBodyPack()) {
        std::cout << "isBodyPack" << std::endl;
    }

    if (device.deviceId().isWirelessMaster()) {
        std::cout << "isWirelessMaster" << std::endl;
    }

    if (device.deviceId().isAwinda()) {
        std::cout << "isAwinda" << std::endl;
    }

    if (device.deviceId().isAwindaStation()) {
        std::cout << "isAwindaStation" << std::endl;
    }

    if (device.deviceId().isAwindaDongle()) {
        std::cout << "isAwindaDongle" << std::endl;
    }

    if (device.deviceId().isAwindaOem()) {
        std::cout << "isAwindaOem" << std::endl;
    }

    if (device.deviceId().isAwinda2()) {
        std::cout << "isAwinda2" << std::endl;
    }

    if (device.deviceId().isAwinda2Station()) {
        std::cout << "isAwinda2Station" << std::endl;
    }

    if (device.deviceId().isAwinda2Dongle()) {
        std::cout << "isAwinda2Dongle" << std::endl;
    }

    if (device.deviceId().isAwinda2Oem()) {
        std::cout << "isAwinda2Oem" << std::endl;
    }

    if (device.deviceId().isSyncStation()) {
        std::cout << "isSyncStation" << std::endl;
    }

    if (device.deviceId().isSyncStation2()) {
        std::cout << "isSyncStation2" << std::endl;
    }

    if (device.deviceId().isImu()) {
        std::cout << "isImu" << std::endl;
    }

    if (device.deviceId().isVru()) {
        std::cout << "isVru" << std::endl;
    }

    if (device.deviceId().isAhrs()) {
        std::cout << "isAhrs" << std::endl;
    }

    if (device.deviceId().isGnss()) {
        std::cout << "isGnss" << std::endl;
    }

    if (device.deviceId().isContainerDevice()) {
        std::cout << "isContainerDevice" << std::endl;
    }

    if (device.deviceId().isMt()) {
        std::cout << "isMt" << std::endl;
    }

    if (device.deviceId().isMti()) {
        std::cout << "isMti" << std::endl;
    }

    if (device.deviceId().isMtig()) {
        std::cout << "isMtig" << std::endl;
    }

    if (device.deviceId().isMtMark4()) {
        std::cout << "isMtMark4" << std::endl;
    }

    if (device.deviceId().isMtMark5()) {
        std::cout << "isMtMark5" << std::endl;
    }

    if (device.deviceId().isType()) {
        std::cout << "isType" << std::endl;
    }

    if (device.deviceId().isMtMk4()) {
        std::cout << "isMtMk4" << std::endl;
    }

    if (device.deviceId().isMtMk4_X()) {
        std::cout << "isMtMk4_X" << std::endl;
    }

    if (device.deviceId().isMtMk4_1()) {
        std::cout << "isMtMk4_1" << std::endl;
    }

    if (device.deviceId().isMtMk4_2()) {
        std::cout << "isMtMk4_2" << std::endl;
    }

    if (device.deviceId().isMtMk4_3()) {
        std::cout << "isMtMk4_3" << std::endl;
    }

    if (device.deviceId().isMtMk4_7()) {
        std::cout << "isMtMk4_7" << std::endl;
    }

    if (device.deviceId().isMtMk4_X0()) {
        std::cout << "isMtMk4_X0" << std::endl;
    }

    if (device.deviceId().isMtMk4_10()) {
        std::cout << "isMtMk4_10" << std::endl;
    }

    if (device.deviceId().isMtMk4_20()) {
        std::cout << "isMtMk4_20" << std::endl;
    }

    if (device.deviceId().isMtMk4_30()) {
        std::cout << "isMtMk4_30" << std::endl;
    }

    if (device.deviceId().isMtMk4_X00()) {
        std::cout << "isMtMk4_X00" << std::endl;
    }

    if (device.deviceId().isMtMk4_100()) {
        std::cout << "isMtMk4_100" << std::endl;
    }

    if (device.deviceId().isMtMk4_200()) {
        std::cout << "isMtMk4_200" << std::endl;
    }

    if (device.deviceId().isMtMk4_300()) {
        std::cout << "isMtMk4_300" << std::endl;
    }

    if (device.deviceId().isMtMk4_400()) {
        std::cout << "isMtMk4_400" << std::endl;
    }

    if (device.deviceId().isMtMk4_500()) {
        std::cout << "isMtMk4_500" << std::endl;
    }

    if (device.deviceId().isMtMk4_600()) {
        std::cout << "isMtMk4_600" << std::endl;
    }

    if (device.deviceId().isMtMk4_700()) {
        std::cout << "isMtMk4_700" << std::endl;
    }

    if (device.deviceId().isMtMk4_710()) {
        std::cout << "isMtMk4_710" << std::endl;
    }

    if (device.deviceId().isMtMk4_800()) {
        std::cout << "isMtMk4_800" << std::endl;
    }

    if (device.deviceId().isMtMk4_900()) {
        std::cout << "isMtMk4_900" << std::endl;
    }

    if (device.deviceId().isMtMk5()) {
        std::cout << "isMtMk5" << std::endl;
    }

    if (device.deviceId().isMtMk5_X0()) {
        std::cout << "isMtMk5_X0" << std::endl;
    }

    if (device.deviceId().isMtMk5_10()) {
        std::cout << "isMtMk5_10" << std::endl;
    }

    if (device.deviceId().isMtMk5_20()) {
        std::cout << "isMtMk5_20" << std::endl;
    }

    if (device.deviceId().isMtMk5_30()) {
        std::cout << "isMtMk5_30" << std::endl;
    }

    if (device.deviceId().isMtMk5_X00()) {
        std::cout << "isMtMk5_X00" << std::endl;
    }

    if (device.deviceId().isMtMk5_100()) {
        std::cout << "isMtMk5_100" << std::endl;
    }

    if (device.deviceId().isMtMk5_200()) {
        std::cout << "isMtMk5_200" << std::endl;
    }

    if (device.deviceId().isMtMk5_300()) {
        std::cout << "isMtMk5_300" << std::endl;
    }

    if (device.deviceId().isMtMk5_710()) {
        std::cout << "isMtMk5_710" << std::endl;
    }

    if (device.isContainerDevice()) {
        std::cout << "isContainerDevice" << std::endl;
    }

    for (const XsDevice* child : device.children()) {
        PrintInfoXsDevice(*child);
    }
}

void PrintInfoXsDataPacket(const XsDataPacket& packet) {
    if (packet.containsAccelerationHR()) {
        std::cout << "containsAccelerationHR" << std::endl;
    }

    if (packet.containsAltitude()) {
        std::cout << "containsAltitude" << std::endl;
    }

    if (packet.containsAltitudeMsl()) {
        std::cout << "containsAltitudeMsl" << std::endl;
    }

    if (packet.containsAnalogIn1Data()) {
        std::cout << "containsAnalogIn1Data" << std::endl;
    }

    if (packet.containsAnalogIn2Data()) {
        std::cout << "containsAnalogIn2Data" << std::endl;
    }

    if (packet.containsAwindaSnapshot()) {
        std::cout << "containsAwindaSnapshot" << std::endl;
    }

    if (packet.containsCalibratedAcceleration()) {
        std::cout << "containsCalibratedAcceleration" << std::endl;
    }

    if (packet.containsCalibratedData()) {
        std::cout << "containsCalibratedData" << std::endl;
    }

    if (packet.containsCalibratedGyroscopeData()) {
        std::cout << "containsCalibratedGyroscopeData" << std::endl;
    }

    if (packet.containsCalibratedMagneticField()) {
        std::cout << "containsCalibratedMagneticField" << std::endl;
    }

    if (packet.containsDetailedStatus()) {
        std::cout << "containsDetailedStatus" << std::endl;
    }

    if (packet.containsFrameRange()) {
        std::cout << "containsFrameRange" << std::endl;
    }

    if (packet.containsFreeAcceleration()) {
        std::cout << "containsFreeAcceleration" << std::endl;
    }

    if (packet.containsFullSnapshot()) {
        std::cout << "containsFullSnapshot" << std::endl;
    }

    if (packet.containsGnssAge()) {
        std::cout << "containsGnssAge" << std::endl;
    }

    if (packet.containsLatitudeLongitude()) {
        std::cout << "containsLatitudeLongitude" << std::endl;
    }

    if (packet.containsOrientation()) {
        std::cout << "containsOrientation" << std::endl;
    }

    if (packet.containsOrientationIncrement()) {
        std::cout << "containsOrientationIncrement" << std::endl;
    }

    if (packet.containsPacketCounter()) {
        std::cout << "containsPacketCounter" << std::endl;
    }

    if (packet.containsPacketCounter8()) {
        std::cout << "containsPacketCounter8" << std::endl;
    }

    if (packet.containsPositionLLA()) {
        std::cout << "containsPositionLLA" << std::endl;
    }

    if (packet.containsPressure()) {
        std::cout << "containsPressure" << std::endl;
    }

    if (packet.containsPressureAge()) {
        std::cout << "containsPressureAge" << std::endl;
    }

    if (packet.containsRateOfTurnHR()) {
        std::cout << "containsRateOfTurnHR" << std::endl;
    }

    if (packet.containsRawAcceleration()) {
        std::cout << "containsRawAcceleration" << std::endl;
    }

    if (packet.containsRawBlob()) {
        std::cout << "containsRawBlob" << std::endl;
    }

    if (packet.containsRawData()) {
        std::cout << "containsRawData" << std::endl;
    }

    if (packet.containsRawGnssPvtData()) {
        std::cout << "containsRawGnssPvtData" << std::endl;
    }

    if (packet.containsRawGnssSatInfo()) {
        std::cout << "containsRawGnssSatInfo" << std::endl;
    }

    if (packet.containsRawGyroscopeData()) {
        std::cout << "containsRawGyroscopeData" << std::endl;
    }

    if (packet.containsRawGyroscopeTemperatureData()) {
        std::cout << "containsRawGyroscopeTemperatureData" << std::endl;
    }

    if (packet.containsRawMagneticField()) {
        std::cout << "containsRawMagneticField" << std::endl;
    }

    if (packet.containsRawTemperature()) {
        std::cout << "containsRawTemperature" << std::endl;
    }

    if (packet.containsRssi()) {
        std::cout << "containsRssi" << std::endl;
    }

    if (packet.containsSampleTime64()) {
        std::cout << "containsSampleTime64" << std::endl;
    }

    if (packet.containsSampleTimeCoarse()) {
        std::cout << "containsSampleTimeCoarse" << std::endl;
    }

    if (packet.containsSampleTimeFine()) {
        std::cout << "containsSampleTimeFine" << std::endl;
    }

    if (packet.containsSdiData()) {
        std::cout << "containsSdiData" << std::endl;
    }

    if (packet.containsStatus()) {
        std::cout << "containsStatus" << std::endl;
    }

    if (packet.containsStoredDeviceId()) {
        std::cout << "containsSdiData" << std::endl;
    }

    if (packet.containsTemperature()) {
        std::cout << "containsTemperature" << std::endl;
    }

    //        if (packet.containsTriggerIndication())
    //        {
    //            std::cout << "containsTriggerIndication" << std::endl;
    //        }

    if (packet.containsUtcTime()) {
        std::cout << "containsUtcTime" << std::endl;
    }

    if (packet.containsVelocity()) {
        std::cout << "containsVelocity" << std::endl;
    }

    if (packet.containsVelocityIncrement()) {
        std::cout << "containsVelocityIncrement" << std::endl;
    }
}

// From:
// XsRawGnssPvtData data = packet->rawGnssPvtData();
// PrintXsRawGnssPvtDataFixType(data.m_fixType);
void PrintXsRawGnssPvtDataFixType(std::uint8_t fix_type) {
    //    0x00 = No Fix
    //    0x01 = Dead Reckoning only
    //    0x02 = 2D-Fix
    //    0x03 = 3D-Fix
    //    0x04 = GNSS + dead reckoning combined
    //    0x05 = Time only fix
    //    0x06 = Reserved ... 0xFF

    std::cout << "gnss fix type: ";

    switch (fix_type) {
        case 0:
            std::cout << "No Fix" << std::endl;
            break;
        case 1:
            std::cout << "Dead Reckoning only" << std::endl;
            break;
        case 2:
            std::cout << "2D-Fix" << std::endl;
            break;
        case 3:
            std::cout << "3D-Fix" << std::endl;
            break;
        case 4:
            std::cout << "GNSS + dead reckoning combined" << std::endl;
            break;
        case 5:
            std::cout << "Time only fix" << std::endl;
            break;
        default:
            std::cout << "Reserved" << std::endl;
    }
}

}  // namespace nie
