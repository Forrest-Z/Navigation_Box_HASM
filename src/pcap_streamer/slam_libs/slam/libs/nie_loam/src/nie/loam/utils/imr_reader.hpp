/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_LOAM_IMR_READER_HPP
#define NIE_LOAM_IMR_READER_HPP

namespace loam {

#pragma pack(push, 1)
// This header tells you almost everything you need to interpret the raw IMU
// measurements (you still have to know the coordinate system for your app, ha!)
// This reader itself will assume nothing and just give you the data.
// For details, please refer to:
// https://docs.novatel.com/Waypoint/Content/Data_Formats/IMR_File.htm
// Of key importance are the bDeltaTheta and bDeltaVelocity flags, which tell
// you what to do with the gyro/accel measurements of the records. iUtcOrGpsTime
// tells you which clock the provided timestamps are in.
struct ImrHeader {
    char szHeader[8];
    std::int8_t bIsIntelOrMotorola;
    double dVersionNumber;
    std::int32_t bDeltaTheta;
    std::int32_t bDeltaVelocity;
    double dDataRateHz;
    double dGyroScaleFactor;
    double dAccelScaleFactor;
    std::int32_t iUtcOrGpsTime;
    std::int32_t iRcvTimeOrCorrTime;
    double dTimeTagBias;
    char szImuName[32];
    std::uint8_t reserved1[4];
    char szProgramName[32];
    char tCreate[12];
    bool bLeverArmValid;
    std::int32_t lXoffset;
    std::int32_t lYoffset;
    std::int32_t lZoffset;
    std::int8_t Reserved[354];
};

// These are scaled (you need the factor from the header) and possibly a delta.
// Depending on what they are you have to do something different to get rad/s
// and m/s^2
struct ImrRecord {
    // Either GPS *OR* UTC seconds in week (depends on header).
    double Time;
    // Gyroscope
    std::int32_t gx;
    std::int32_t gy;
    std::int32_t gz;
    // Accelerometer
    std::int32_t ax;
    std::int32_t ay;
    std::int32_t az;
};
#pragma pack(pop)

struct ImrData {
    ImrHeader header{};
    std::vector<ImrRecord> records;
};

ImrData ReadImrCollection(std::string const& path_imr_file) {
    std::ifstream imr_ifstream(path_imr_file);

    if (!imr_ifstream) {
        LOG(FATAL) << "Could not read input IMR file: " << path_imr_file;
    }

    ImrData collection;
    imr_ifstream.read(reinterpret_cast<char*>(&collection.header), sizeof(collection.header));

    while (!imr_ifstream.eof()) {
        ImrRecord aux{};
        imr_ifstream.read(reinterpret_cast<char*>(&aux), sizeof(ImrRecord));
        collection.records.push_back(aux);
    }

    return collection;
}

}  // namespace loam

#endif  // NIE_LOAM_IMR_READER_HPP
