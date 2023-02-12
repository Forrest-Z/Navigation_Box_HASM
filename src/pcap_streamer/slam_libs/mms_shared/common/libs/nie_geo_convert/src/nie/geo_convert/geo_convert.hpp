/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CARTESIANTRANSFORMATION_HPP
#define NIE_CV_CARTESIANTRANSFORMATION_HPP

#include <string>

namespace nie {

namespace geo_convert {
/*
 *  Converts between geographic coordinates in the wgs84 geodetic to UTM.
 *
 * @param[in] latitude latitude value in degrees.
 * @param[in] longitude longitude value in degrees.
 * @param[out] zone string representing UTM zone number and letter for hemisphere in the format of i. e. "30n"
 * @param[out] easting easting of point in meters inside the calculate UTM zone.
 * @param[out] northing northing of point in meters inside the calculate UTM zone.
 * @param[out] gamma meridian convergence in radians.
 * @param[out] k map scaling.
 *
 */
void Wgs84ToUtm(
    double const latitude,
    double const longitude,
    double* easting,
    double* northing,
    std::string* zone,
    double* gamma,
    double* k);
/*
 *  Converts between UTM to geographic coordinates in the wgs84 geodetic.
 *
 * @param[in] easting easting of point in meters inside the calculate UTM zone.
 * @param[in] northing northing of point in meters inside the calculate UTM zone.
 * @param[in] zone string representing UTM zone number and letter for hemisphere in the format of i. e. "30n"
 * @param[out] latitude value in degrees.
 * @param[out] longitude value in degrees.
 * @param[out] gamma meridian convergence in radians.
 * @param[out] k map scaling.
 *
 */
void UtmToWgs84(
    double const easting,
    double const northing,
    std::string const& zone,
    double* latitude,
    double* longitude,
    double* gamma,
    double* k);

}  // namespace geo_convert

}  // namespace nie

#endif  // NIE_CV_CARTESIANTRANSFORMATION_HPP
