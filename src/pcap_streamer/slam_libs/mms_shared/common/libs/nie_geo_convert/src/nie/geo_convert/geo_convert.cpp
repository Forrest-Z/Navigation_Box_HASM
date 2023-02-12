/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "geo_convert.hpp"

#include <nie/core/geometry/rotation.hpp> // for nie::Deg2Rad
#include <GeographicLib/UTMUPS.hpp>

namespace nie {

namespace geo_convert {

void Wgs84ToUtm(
    double const latitude,
    double const longitude,
    double* easting,
    double* northing,
    std::string* zone,
    double* gamma,
    double* k) {
    int zone_num;
    bool north_pole;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone_num, north_pole, *easting, *northing, *gamma, *k);
    // libgeographic outputs gamma in degrees
    *gamma = Deg2Rad(*gamma);
    *zone = GeographicLib::UTMUPS::EncodeZone(zone_num, north_pole);
}

void UtmToWgs84(
    double const easting,
    double const northing,
    std::string const& zone,
    double* latitude,
    double* longitude,
    double* gamma,
    double* k) {
    int zone_num;
    bool north_pole;
    GeographicLib::UTMUPS::DecodeZone(zone, zone_num, north_pole);
    GeographicLib::UTMUPS::Reverse(zone_num, north_pole, easting, northing, *latitude, *longitude, *gamma, *k);
    // libgeographic outputs gamma in degrees
    *gamma = Deg2Rad(*gamma);
}

}  // namespace geo_convert

}  // namespace nie
