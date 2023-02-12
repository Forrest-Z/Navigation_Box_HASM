/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "kinematica_reader.hpp"

#include <tuple>

#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>

namespace nie {

namespace io {

KinematicaRecordCollection ReadKinematicaCsv(std::string const& kinematica_csv_path) {
    nie::CsvRecorderSafe<
            std::string,  // 0  : Human Timestamp
            long,         // 1  : Unix Time
            long,         // 2  : Microseconds
            int,          // 3  : Fix Type
            double,       // 4  : Latitude
            double,       // 5  : Longitude
            double,       // 6  : Height
            double,       // 7  : Latitude SD
            double,       // 8  : Longitude SD
            double,       // 9  : Height SD
            double,       // 10 : Velocity North
            double,       // 11 : Velocity East
            double,       // 12 : Velocity Down
            double,       // 13 : Velocity North SD
            double,       // 14 : Velocity East SD
            double,       // 15 : Velocity Down SD
            double,       // 16 : Roll
            double,       // 17 : Pitch
            double,       // 18 : Heading
            double,       // 19 : Roll SD
            double,       // 20 : Pitch SD
            double,       // 21 : Heading SD
            double,       // 22 : Accelerometer Bias X
            double,       // 23 : Accelerometer Bias Y
            double,       // 24 : Accelerometer Bias Z
            double,       // 25 : Accelerometer Bias X SD
            double,       // 26 : Accelerometer Bias Y SD
            double,       // 27 : Accelerometer Bias Z SD
            double,       // 28 : Gyroscope Bias X
            double,       // 29 : Gyroscope Bias Y
            double,       // 30 : Gyroscope Bias Z
            double,       // 31 : Gyroscope Bias X SD
            double,       // 32 : Gyroscope Bias Y SD
            double,       // 33 : Gyroscope Bias Z SD
            int,          // 34 : GPS Satellites
            int,          // 35 : GLONASS Satellites
            int,          // 36 : BeiDou Satellites
            int,          // 37 : Galileo Satellites
            int,          // 38 : Differential GPS Satellites
            int,          // 39 : Differential Glonass Satellites
            int,          // 40 : Differential BeiDou Satellites
            int,          // 41 : Differential Galileo Satellites
            int,          // 42 : Dual Antenna Fix
            double,       // 43 : Horizontal Separation
            double,       // 44 : Vertical Separation
            int,          // 45 : SBAS Satellites
            int,          // 46 : Differential SBAS Satellites
            int,          // 47 : Zero Velocity Update
            double,       // 48 : Base to Rover North
            double,       // 49 : Base to Rover East
            double,       // 50 : Base to Rover Down
            double,       // 51 : Base to Rover North SD
            double,       // 52 : Base to Rover East SD
            double,       // 53 : Base to Rover Down SD
            int,          // 54 : Moving Base Fix Type
            int,          // 55 : Event 1 Flag
            int>          // 56 : Event 2 Flag
            csv_recorder{','};

    // Due to template deduction rules, if nie::ReadLinesAfterNthLinePredicate is used, then the type of the second
    // argument (header_l) is int. Setting it to 0 means that the file will be read since the first line.
    nie::ReadAllCsvToHandler<nie::ReadLinesAfterNthLinePredicate>({kinematica_csv_path}, 0, &csv_recorder);

    // Template deduction lets us omit the long list of arguments
    nie::CsvRecord const csv_record = csv_recorder.ConvertToRecord();

    size_t const num_rows = csv_record.NumRows();
    KinematicaRecordCollection kinematica_record_collection(num_rows);
    for (size_t i = 0; i < num_rows; i++) {
        std::tuple const csv_row = csv_record.GetRow(i);
        KinematicaCsvRecord& kinematica_row = kinematica_record_collection[i];
        kinematica_row.unix_time = std::get<1>(csv_row);
        kinematica_row.microseconds = std::get<2>(csv_row);
        kinematica_row.latitude = std::get<4>(csv_row);
        kinematica_row.longitude = std::get<5>(csv_row);
        kinematica_row.height = std::get<6>(csv_row);
        kinematica_row.latitude_sd = std::get<7>(csv_row);
        kinematica_row.longitude_sd = std::get<8>(csv_row);
        kinematica_row.height_sd = std::get<9>(csv_row);
        kinematica_row.velocity_north = std::get<10>(csv_row);
        kinematica_row.velocity_east = std::get<11>(csv_row);
        kinematica_row.roll = std::get<16>(csv_row);
        kinematica_row.pitch = std::get<17>(csv_row);
        kinematica_row.heading = std::get<18>(csv_row);
        kinematica_row.roll_sd = std::get<19>(csv_row);
        kinematica_row.pitch_sd = std::get<20>(csv_row);
        kinematica_row.heading_sd = std::get<21>(csv_row);
    }

    return kinematica_record_collection;
}

}  // namespace io

}  // namespace nie