/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <vector>

#include <helper_registered_pair.hpp>

class HelperRegisteredPairF : public ::testing::Test {
protected:
    constexpr static double kPrecision = 1e-12;

    std::string const kFilepathPose = "/data/aiim/unit_tests_data/lidar/alignment_viewer/overpass.pose";
    // TODO: Create iref file
    std::string const kFilepathInfoRef = "";

    static nie::io::PoseCollection CreateCollection() {
        // clang-format off
        std::vector<nie::io::PoseEdgeRecord> const edges{
                {1, 2, nie::io::PoseEdgeRecord::Category::kLoop,
                        {{4., 4., 4.}, {4., 4., 4., 4.}}, Eigen::Matrix<double, 6, 6>::Identity()}};
        // clang-format on
        std::vector<nie::io::FixedPoseRecord> const fixes{{1}};

        nie::io::PoseHeader header{};
        nie::io::SetNieAuthority(&header);
        header.Set(nie::io::PoseHeader::Flag::kHasEdgeInformationPerRecord);

        return {header, {}, edges, fixes};
    }

    //clang-format off
    std::vector<boost::filesystem::path> kLasFiles{
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121225-00115.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111159-00027.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-134335-00165.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-114150-00073.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110750-00019.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113940-00069.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143306-00214.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120445-00106.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141431-00179.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122125-00127.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112749-00049.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142531-00199.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120104-00099.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-114404-00075.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112957-00052.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113652-00063.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112312-00038.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115826-00094.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115509-00090.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141612-00182.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122500-00136.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120204-00102.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115130-00080.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-130823-00139.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111636-00036.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142812-00205.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-105912-00002.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-132045-00159.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143102-00210.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110509-00012.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143022-00208.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111406-00031.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141752-00187.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131905-00154.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-140830-00168.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122105-00126.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131845-00153.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110206-00006.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113148-00053.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131925-00155.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113228-00055.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110939-00020.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112937-00051.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142311-00192.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142752-00204.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143002-00207.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120044-00098.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122045-00125.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143042-00209.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-140810-00167.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110730-00018.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121005-00108.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115409-00087.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122220-00128.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122340-00132.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121025-00109.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115944-00095.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112509-00041.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115030-00077.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142108-00190.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110959-00021.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131805-00151.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122300-00130.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131725-00149.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-134235-00162.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142251-00191.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111516-00032.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142942-00206.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113800-00064.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115449-00089.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115229-00082.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113208-00054.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121145-00113.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110610-00014.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110550-00013.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131945-00156.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122320-00131.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112917-00050.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143206-00211.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115349-00086.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111556-00034.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122240-00129.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110349-00008.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122025-00124.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122005-00123.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141632-00183.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141134-00174.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-130903-00141.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143246-00213.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115309-00084.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115329-00085.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111059-00024.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141014-00170.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141652-00184.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115429-00088.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115806-00093.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131605-00145.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131825-00152.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122400-00133.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122420-00134.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121745-00116.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113552-00060.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-105852-00001.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111019-00022.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-130943-00143.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131625-00146.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-143226-00212.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110409-00009.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110630-00015.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120345-00103.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142331-00193.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111306-00028.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120425-00105.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110710-00017.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110449-00011.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131745-00150.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121945-00122.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113820-00065.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112449-00040.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120024-00097.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115209-00081.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112649-00046.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141552-00181.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121125-00112.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112729-00048.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113632-00062.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131545-00144.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121925-00121.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-140750-00166.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-130803-00138.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111346-00030.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121845-00119.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121825-00118.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121905-00120.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111039-00023.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112429-00039.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-130743-00137.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113532-00059.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131705-00148.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-131645-00147.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120945-00107.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-122440-00135.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142632-00200.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-132005-00157.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142048-00189.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111139-00026.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113920-00068.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115249-00083.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121105-00111.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111536-00033.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141351-00177.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-140954-00169.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-134315-00164.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-132025-00158.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112252-00037.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115010-00076.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-134255-00163.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121205-00114.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-114110-00071.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-114050-00070.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111616-00035.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112709-00047.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110329-00007.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115746-00092.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-130843-00140.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142511-00198.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110106-00003.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113432-00056.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141311-00175.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115050-00078.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-105832-00000.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113512-00058.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112529-00042.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142351-00194.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111119-00025.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142451-00197.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110650-00016.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110429-00010.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-134215-00161.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141331-00176.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-111326-00029.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115726-00091.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110126-00004.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-134155-00160.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113900-00067.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120124-00100.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141712-00185.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120004-00096.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120405-00104.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112609-00044.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141054-00172.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-121045-00110.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142712-00202.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113612-00061.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-115110-00079.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142431-00196.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141732-00186.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142732-00203.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142652-00201.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112629-00045.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-110146-00005.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141411-00178.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141532-00180.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-130923-00142.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142411-00195.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-120144-00101.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-114344-00074.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141034-00171.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-114130-00072.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-141114-00173.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113840-00066.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-142028-00188.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-113452-00057.las",
            "CHINA_20190716/Shanghai-overpass/las/40-10021005180120-112549-00043.las"};

    std::vector<nie::RegisteredPair> const kRegisteredPairs{

    };
    //clang-format on
};

TEST_F(HelperRegisteredPairF, DISABLED_ReadFilterPoseEdgeRecords) {
    auto const pose_collection = nie::io::ReadCollection<nie::io::PoseCollection>(kFilepathPose);
    auto const iref_collection = nie::io::ReadCollection<nie::io::InfoRefCollection>(kFilepathInfoRef);

    std::vector<nie::RegisteredPair> pairs = nie::GetLoopClosureCandidates({}, iref_collection, pose_collection);

    ASSERT_EQ(pairs.size(), 19);
}

TEST_F(HelperRegisteredPairF, ReadFilterPoseEdgeRecordsEmpty) {
    nie::io::PoseCollection pose_collection = CreateCollection();
    EXPECT_DEATH(
            nie::GetLoopClosureCandidates({}, {}, pose_collection),
            "Check failed:.*Mismatch between pose file and LAS directory.");
}
