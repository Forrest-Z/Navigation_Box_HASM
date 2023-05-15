/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_POSITIONING_GPS_IMU_MEASUREMENTS_HPP
#define NIE_HARDWARE_POSITIONING_GPS_IMU_MEASUREMENTS_HPP

#include <cstdint>

namespace nie {

class GpsMeasurement {
public:
    GpsMeasurement() : latitude(0.0), longitude(0.0), system_time(0.0) {}

    GpsMeasurement(double latitude, double longitude, std::int64_t system_time)
        : latitude(latitude), longitude(longitude), system_time(system_time) {}

    double latitude;
    double longitude;
    std::int64_t system_time;
};

}  // namespace nie

#endif  // NIE_HARDWARE_POSITIONING_GPS_IMU_MEASUREMENTS_HPP
