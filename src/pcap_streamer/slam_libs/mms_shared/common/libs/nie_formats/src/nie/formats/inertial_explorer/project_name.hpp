/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_TYPE_A_PROJECT_NAME_HPP
#define NIE_FORMATS_TYPE_A_PROJECT_NAME_HPP

#include <nie/core/string.hpp>
#include <nie/core/time.hpp>

namespace nie {
namespace io {
namespace inertial_explorer {

// According to one reference in China, the date in the project name must always be interpreted to be in
// CST / Beijing time (UTC+08), even when a recording was made in another time zone.
// However, for several data sets we get a mis match of the day. Now assume it to be +0.
static auto const kTimezoneOffset = +std::chrono::hours(0);

/**
 * @brief Obtain the collection epoch from a project name used by data Collection vehicles from China.
 *
 * @details Prefer not to use this method! It is only here for backwards compatibility. I.e. to support old PosT
 * files that did not provide gps week and time in week values.
 *
 * Without an authoritative source, we derived that project names are generated as follows:
 *
 *     'cccc-n-svv-yymmdd'
 *
 *  Where all symbols are numeric, and have the following meaning:
 *
 *      cccc    = Internal city identification code; Refers to major cities in China. (0000 is undefined)
 *      n       = Flag indicating night or day time recording. (1 means day, 0 means night)
 *      s       = Sequential collection round this day. (0 means first recording)
 *      vv      = Collection vehicle identification number.
 *      yy      = UTC year  of collection epoch. (2000-2099)
 *      mm      = UTC month of collection epoch. (00-12)
 *      dd      = UTC day   of collection epoch. (01-31)
 *
 * See also: https://confluence.navinfo.eu/pages/viewpage.action?spaceKey=MMS&title=Output+images+format
 *
 * NOTE: The project name may occur with all, some, or no dashes; This function accepts all forms. Composite strings
 *       may use the project name with additional pre- or postfixes. Such strings are NOT parsed by this function.
 *
 * WARNING: The collection epoch is generated as the date from the system clock of the data collection PC. This date is
 *          is always expressed in timezone UTC+08:00 (CST - Chinese Standard Time), no matter where the collection
 *          was performed. However, there is currently no procedure to synchronize the system clock of the collection
 *          PC (e.g. using NTP or manually), and it is also unknown how much clock drift can be expected, so the date
 *          should be regarded as unreliable.
 *
 * @param project_name
 *
 * @return date of the collection epoch
 */
std::chrono::year_month_day ParseDateFromProjectName(std::string const& project_name);

}  // namespace inertial_explorer
}  // namespace io
}  // namespace nie

#endif  // NIE_FORMATS_TYPE_A_PROJECT_NAME_HPP
