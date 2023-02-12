/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_POSITIONING_HELPER_XSENS_GPS_IMU_HPP
#define NIE_HARDWARE_POSITIONING_HELPER_XSENS_GPS_IMU_HPP

#include <xsensdeviceapi.h>

namespace nie {

void PrintInfoXsDevices();
void PrintInfoXsDevice(const XsDevice& device, bool recurse = true);
void PrintInfoXsDataPacket(const XsDataPacket& packet);
// From:
// XsRawGnssPvtData data = packet->rawGnssPvtData();
// PrintXsRawGnssPvtDataFixType(data.m_fixType);
void PrintXsRawGnssPvtDataFixType(std::uint8_t fix_type);

class PrintInfoXsDataPacketCallback : public XsCallback {
protected:
    virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override {
        nie::PrintInfoXsDataPacket(*packet);
    }
};

}  // namespace nie

#endif  // NIE_HARDWARE_POSITIONING_HELPER_XSENS_GPS_IMU_HPP
