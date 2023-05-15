/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
//
// Created by robert.vaneerdewijk on 10/30/19.
//

#include <gtest/gtest.h>
#include <string>

#include <nie/core/string.hpp>

#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>
#include <nie/formats/csv/csv_record_io.hpp>

class CsvRecordTest : public testing::Test {
protected:
    // CSV data with complex string values in "STRING" column. delimiter is comma.
    std::string const kFilePathSafe = "/data/aiim/unit_tests_data/csv/change_detection_csv_format_safe.txt";
    std::string const kHeaderLineSafe = "GUID,FILE_URL,STRING,LONGITUDE,LATITUDE,ALTITUDE,DIRECTION,DEVICE_NUM,CREATE";

    // CSV data without complex string values. delimiter is semicolon.
    std::string const kFilePathFast = "/data/aiim/unit_tests_data/csv/change_detection_csv_format_fast.txt";
    std::string const kHeaderLineFast = "GUID;FILE_URL;STRING;LONGITUDE;LATITUDE;ALTITUDE;DIRECTION;DEVICE_NUM;CREATE";

    // clang-format off
    std::vector<std::string> kCol0{
        "a6c9a8ab-4537-4d08-abec-dc0917bab6e2",
        "abc7ac04-499b-4efc-a20b-ba56515cf3a7",
        "1540e01d-30ec-4e71-9bae-5dad46304302",
        "8ca10f2d-ba11-4833-befc-181e09936ed9",
        "56f5be4a-2bcb-4288-9cf4-cdd0b638a03e",
        "4b1436dc-66ab-4968-937e-b5a5ff3b7cd7",
        "c833ab4b-0a97-4f2d-af8d-605dde6a74e4",
        "0d4d9021-ad21-40db-ac04-17a3ecee9b88"
    };
    std::vector<std::string> kCol1{
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000869_7.42839914_51.49802624.jpg",
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000870_7.42838850_51.49802634.jpg",
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000871_7.42837786_51.49802645.jpg",
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000872_7.42836722_51.49802655.jpg",
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000873_7.42835658_51.49802665.jpg",
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000874_7.42834593_51.49802675.jpg",
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000875_7.42833529_51.49802685.jpg",
        "/volumes2/decay/restructured/0101-2019-08-261116/2019-08-261116_GPS_Photo/08/26/11/XX_20190826-111602-NF_00000876_7.42832465_51.49802695.jpg"
    };
    std::vector<std::string> kCol2{
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5",
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5",
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5",
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5",
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5",
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5",
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5",
        "$GPGGA,101633.00,5129.88157,N,00725.70444,E,1,06,2.67,108.3,M,46.5,M,,*5"
    };
    std::vector<double> kCol3{
        7.4283991396441715,
        7.42838849848942,
        7.428377857334674,
        7.428367216179923,
        7.428356575025171,
        7.42834593387042,
        7.428335292715674,
        7.4283246515609225
    };
    std::vector<double> kCol4{
        51.4980262442095,
        51.49802634491443,
        51.49802644561937,
        51.4980265463243,
        51.49802664702923,
        51.49802674773416,
        51.4980268484391,
        51.49802694914403
    };
    std::vector<double> kCol5{
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    };
    std::vector<double> kCol6{
        271.7823430681437,
        271.7464249748237,
        271.7105068815038,
        271.674588788184,
        271.638670694864,
        271.60275260154407,
        271.5668345082242,
        271.5309164149043
    };
    std::vector<int> kCol7{
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    };
    std::vector<std::string> kCol8{
        "2019-08-2610163325666",
        "2019-08-2610163359000",
        "2019-08-2610163392333",
        "2019-08-26101633125666",
        "2019-08-26101633159000",
        "2019-08-26101633192333",
        "2019-08-26101633225666",
        "2019-08-26101633259000"
    };
    // clang-format on
};

TEST_F(CsvRecordTest, ParseFileSafe) {
    nie::CsvRecorderSafe<std::string, std::string, std::string, double, double, double, double, int, std::string>
        csv_recorder{','};

    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({kFilePathSafe}, kHeaderLineSafe, &csv_recorder);

    auto csv_record = csv_recorder.ConvertToRecord();

    ASSERT_EQ(csv_record.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_record.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_record.GetCol<2>(), kCol2);
    ASSERT_EQ(csv_record.GetCol<3>(), kCol3);
    ASSERT_EQ(csv_record.GetCol<4>(), kCol4);
    ASSERT_EQ(csv_record.GetCol<5>(), kCol5);
    ASSERT_EQ(csv_record.GetCol<6>(), kCol6);
    ASSERT_EQ(csv_record.GetCol<7>(), kCol7);
    ASSERT_EQ(csv_record.GetCol<8>(), kCol8);
}

TEST_F(CsvRecordTest, ParseFileFast) {
    nie::CsvRecorderFast<std::string, std::string, std::string, double, double, double, double, int, std::string>
        csv_recorder{';'};

    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({kFilePathFast}, kHeaderLineFast, &csv_recorder);

    auto csv_record = csv_recorder.ConvertToRecord();

    ASSERT_EQ(csv_record.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_record.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_record.GetCol<2>(), kCol2);
    ASSERT_EQ(csv_record.GetCol<3>(), kCol3);
    ASSERT_EQ(csv_record.GetCol<4>(), kCol4);
    ASSERT_EQ(csv_record.GetCol<5>(), kCol5);
    ASSERT_EQ(csv_record.GetCol<6>(), kCol6);
    ASSERT_EQ(csv_record.GetCol<7>(), kCol7);
    ASSERT_EQ(csv_record.GetCol<8>(), kCol8);
}

TEST_F(CsvRecordTest, ParseFileWithVoid) {
    nie::CsvRecorderSafe<std::string, std::string, std::string, void*, double, double, double, int, std::string>
        csv_recorder{','};

    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({kFilePathSafe}, kHeaderLineSafe, &csv_recorder);

    auto csv_record = csv_recorder.ConvertToRecord();

    ASSERT_EQ(csv_record.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_record.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_record.GetCol<2>(), kCol2);
    ASSERT_EQ(csv_record.GetCol<3>().size(), kCol3.size());
    for (auto const& val : csv_record.GetCol<3>()) {
        ASSERT_EQ(val, nullptr);
    }
    ASSERT_EQ(csv_record.GetCol<4>(), kCol4);
    ASSERT_EQ(csv_record.GetCol<5>(), kCol5);
    ASSERT_EQ(csv_record.GetCol<6>(), kCol6);
    ASSERT_EQ(csv_record.GetCol<7>(), kCol7);
    ASSERT_EQ(csv_record.GetCol<8>(), kCol8);
}

TEST_F(CsvRecordTest, WriteToFileSafe) {
    nie::CsvRecorderSafe<std::string, std::string, std::string, double, double, double, double, int, std::string>
        csv_recorder{','};

    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({kFilePathSafe}, kHeaderLineSafe, &csv_recorder);

    auto csv_record = csv_recorder.ConvertToRecord();

    std::string tmp_file = nie::TemporaryFile("CsvRecordTest_WriteToFileSafe.csv").string();

    ASSERT_EQ(csv_record.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_record.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_record.GetCol<2>(), kCol2);
    ASSERT_EQ(csv_record.GetCol<3>(), kCol3);
    ASSERT_EQ(csv_record.GetCol<4>(), kCol4);
    ASSERT_EQ(csv_record.GetCol<5>(), kCol5);
    ASSERT_EQ(csv_record.GetCol<6>(), kCol6);
    ASSERT_EQ(csv_record.GetCol<7>(), kCol7);
    ASSERT_EQ(csv_record.GetCol<8>(), kCol8);

    {
        std::ofstream outfile{tmp_file};
        nie::CsvOutStream<char> csv_out_stream{outfile, ',', 21};
        csv_out_stream << csv_record;
    }

    nie::CsvRecorderSafe<std::string, std::string, std::string, double, double, double, double, int, std::string>
        csv_recorder2{','};

    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({tmp_file}, kHeaderLineSafe, &csv_recorder2);

    auto csv_record2 = csv_recorder2.ConvertToRecord();

    ASSERT_EQ(csv_record2.header(), kHeaderLineSafe);

    ASSERT_EQ(csv_record2.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_record2.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_record2.GetCol<2>(), kCol2);
    ASSERT_EQ(csv_record2.GetCol<3>(), kCol3);
    ASSERT_EQ(csv_record2.GetCol<4>(), kCol4);
    ASSERT_EQ(csv_record2.GetCol<5>(), kCol5);
    ASSERT_EQ(csv_record2.GetCol<6>(), kCol6);
    ASSERT_EQ(csv_record2.GetCol<7>(), kCol7);
    ASSERT_EQ(csv_record2.GetCol<8>(), kCol8);
}