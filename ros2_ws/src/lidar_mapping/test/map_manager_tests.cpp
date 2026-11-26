#include "lidar_mapping/core/map_manager.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <limits>
#include <string>
#include <system_error>

namespace {

using namespace lidar_core;

PointCloud cloud_of(std::initializer_list<Point> points) {
    PointCloud cloud;
    for (const auto& point : points) {
        cloud.push_back(point);
    }
    return cloud;
}

Pose pose(double x, double y, double z, double rotation_deg = 0) {
    constexpr double radians_per_degree = 0.017453292519943295;
    Pose transform = Pose::Identity();
    transform.translation() = Eigen::Vector3d(x, y, z);
    transform.linear() = Eigen::AngleAxisd(rotation_deg * radians_per_degree,
                                          Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return transform;
}

void expect_point(const Point& point, float x, float y, float z) {
    EXPECT_NEAR(point.x, x, 1e-6f);
    EXPECT_NEAR(point.y, y, 1e-6f);
    EXPECT_NEAR(point.z, z, 1e-6f);
}

class MapFiles : public testing::Test {
protected:
    void SetUp() override {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto temp = std::filesystem::temp_directory_path();
        for (int suffix = 0; suffix < 100; ++suffix) {
            auto candidate = temp / ("lidar_map_test_" + std::to_string(stamp) + "_" +
                                     std::to_string(suffix));
            std::error_code error;
            if (std::filesystem::create_directory(candidate, error)) {
                directory_ = candidate;
                config_.directory = directory_ / "maps";
                return;
            }
            ASSERT_FALSE(error) << error.message();
        }
        FAIL() << "Could not create a unique temporary directory";
    }

    void TearDown() override {
        if (!directory_.empty()) {
            std::error_code error;
            std::filesystem::remove_all(directory_, error);
            EXPECT_FALSE(error) << error.message();
        }
    }

    MapConfig config_;

private:
    std::filesystem::path directory_;
};

TEST(MapManager, FirstValidScanInitializesTheMap) {
    MapManager map;
    EXPECT_TRUE(map.cloud().empty());
    EXPECT_FALSE(map.add_cloud({}, Pose::Identity()));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(map.add_cloud(cloud_of({{0, 0, 0}, {nan, 0, 0}}), Pose::Identity()));
    Pose reflected = Pose::Identity();
    reflected.linear()(0, 0) = -1;
    EXPECT_FALSE(map.add_cloud(cloud_of({{0, 0, 0}}), reflected));
    EXPECT_TRUE(map.cloud().empty());
    EXPECT_TRUE(map.should_add_keyframe(Pose::Identity()));
    ASSERT_TRUE(map.add_cloud(cloud_of({{1, 2, 3}}), pose(0, 0, 2)));
    ASSERT_EQ(map.cloud().size(), 1u);
    expect_point(map.cloud().front(), 1, 2, 5);
}

TEST(MapManager, AccumulatesTransformedScansInMapCoordinates) {
    MapManager map;
    ASSERT_TRUE(map.add_cloud(cloud_of({{1, 2, 3}}), pose(2, 0, 1)));
    ASSERT_TRUE(map.add_cloud(cloud_of({{1, 2, 3}, {0, 0, 0}}), pose(5, -3, 2, 90)));
    ASSERT_EQ(map.cloud().size(), 3u);
    expect_point(map.cloud()[0], 3, 2, 4);
    expect_point(map.cloud()[1], 3, -2, 5);
    expect_point(map.cloud()[2], 5, -3, 2);
    EXPECT_EQ(map.cloud().height, 1u);
    EXPECT_EQ(map.cloud().width, 3u);
}

TEST(MapManager, SelectsASphericalLocalMapAroundTheQueryInMapCoordinates) {
    MapConfig config;
    config.local_radius_m = 2;
    MapManager map(config);
    ASSERT_TRUE(map.add_cloud(cloud_of({{0, 0, 0}, {1, 0, 1}, {0, 0, 3}, {1.5f, 1.5f, 0}}),
                              pose(4, -2, 1)));
    const PointCloud local = map.local_map(Eigen::Vector3d(4, -2, 1));
    ASSERT_EQ(local.size(), 2u);
    expect_point(local[0], 4, -2, 1);
    expect_point(local[1], 5, -2, 2);
    EXPECT_TRUE(map.local_map(Eigen::Vector3d::Constant(100)).empty());
    EXPECT_EQ(map.cloud().size(), 4u);
}

TEST(MapManager, DownsamplesTheAccumulatedMapAcrossScans) {
    MapConfig config;
    config.voxel_size_m = 0.5f;
    MapManager map(config);
    ASSERT_TRUE(map.add_cloud(cloud_of({{0.10f, 0.10f, 0.10f}}), Pose::Identity()));
    ASSERT_TRUE(map.add_cloud(cloud_of({{0.30f, 0.10f, 0.10f}, {0.75f, 0.1f, 0.1f}}),
                              Pose::Identity()));
    ASSERT_TRUE(map.downsample());
    ASSERT_EQ(map.cloud().size(), 2u);
    for (const auto& point : map.cloud()) {
        expect_point(point, point.x < 0.5f ? 0.20f : 0.75f, 0.10f, 0.10f);
    }
    EXPECT_NE(map.cloud()[0].x < 0.5f, map.cloud()[1].x < 0.5f);
}

TEST(MapManager, UsesEitherKeyframeThresholdRelativeToTheLastAcceptedPose) {
    MapManager map;
    const auto scan = cloud_of({{0, 0, 0}});
    ASSERT_TRUE(map.add_cloud(scan, pose(1, 2, 3, 175)));
    EXPECT_FALSE(map.should_add_keyframe(pose(1.19, 2, 3, 175)));
    EXPECT_TRUE(map.should_add_keyframe(pose(1.21, 2, 3, 175)));
    EXPECT_FALSE(map.should_add_keyframe(pose(1, 2, 3, -176)));
    EXPECT_TRUE(map.should_add_keyframe(pose(1, 2, 3, -174)));
    EXPECT_FALSE(map.add_cloud({}, pose(1.5, 2, 3)));
    EXPECT_FALSE(map.should_add_keyframe(pose(1, 2, 3, 175)));

    ASSERT_TRUE(map.add_cloud(scan, pose(1.25, 2, 3, 175)));
    EXPECT_FALSE(map.should_add_keyframe(pose(1.25, 2, 3, 175)));
    EXPECT_TRUE(map.should_add_keyframe(pose(1, 2, 3, 175)));
}

TEST_F(MapFiles, BinaryPcdRoundTripPreservesGlobalCoordinates) {
    MapManager mapping(config_);
    EXPECT_FALSE(mapping.save_pcd("empty.pcd"));
    ASSERT_TRUE(mapping.add_cloud(cloud_of({{1, 2, 3}, {-1, 0, 4}}), pose(2.5, 0, 1)));
    EXPECT_FALSE(mapping.save_pcd("../outside.pcd"));
    ASSERT_TRUE(mapping.save_pcd("baseline.pcd"));
    ASSERT_TRUE(std::filesystem::is_regular_file(config_.directory / "baseline.pcd"));

    MapManager loaded(config_);
    ASSERT_TRUE(loaded.load_pcd("baseline.pcd"));
    ASSERT_EQ(loaded.cloud().size(), 2u);
    expect_point(loaded.cloud()[0], 3.5f, 2, 4);
    expect_point(loaded.cloud()[1], 1.5f, 0, 5);
    EXPECT_TRUE(loaded.should_add_keyframe(Pose::Identity()));
}

TEST_F(MapFiles, FailedPcdLoadPreservesTheCurrentMap) {
    MapManager mapping(config_);
    ASSERT_TRUE(mapping.add_cloud(cloud_of({{1, 2, 3}}), Pose::Identity()));
    ASSERT_TRUE(mapping.save_pcd("valid.pcd"));
    {
        std::ofstream file(config_.directory / "no_xyz.pcd");
        ASSERT_TRUE(file);
        file << "VERSION .7\nFIELDS intensity\nSIZE 4\nTYPE F\nCOUNT 1\n"
                "WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n42\n";
    }
    EXPECT_FALSE(mapping.load_pcd("no_xyz.pcd"));
    EXPECT_FALSE(mapping.load_pcd("missing.pcd"));
    ASSERT_EQ(mapping.cloud().size(), 1u);
    expect_point(mapping.cloud().front(), 1, 2, 3);
    EXPECT_FALSE(mapping.should_add_keyframe(Pose::Identity()));
}

TEST_F(MapFiles, LocalizationKeepsTheFirstLoadedMapFixed) {
    const auto original = cloud_of({{0.01f, 0, 0}, {0.02f, 0, 0}});
    MapManager mapping(config_);
    ASSERT_TRUE(mapping.add_cloud(original, Pose::Identity()));
    ASSERT_TRUE(mapping.save_pcd("baseline.pcd"));
    ASSERT_TRUE(mapping.add_cloud(original, pose(20, 0, 0)));
    ASSERT_TRUE(mapping.save_pcd("other.pcd"));

    MapManager localization(config_, MapMode::localization);
    EXPECT_FALSE(localization.add_cloud(original, Pose::Identity()));
    EXPECT_FALSE(localization.load_pcd("missing.pcd"));
    ASSERT_TRUE(localization.load_pcd("baseline.pcd"));
    EXPECT_FALSE(localization.should_add_keyframe(pose(20, 0, 0)));
    EXPECT_FALSE(localization.add_cloud(original, pose(20, 0, 0)));
    EXPECT_FALSE(localization.downsample());
    EXPECT_FALSE(localization.load_pcd("other.pcd"));
    auto local = localization.local_map(Eigen::Vector3d::Zero());
    ASSERT_EQ(local.size(), 2u);
    local[0].x = 100;
    EXPECT_TRUE(localization.local_map(Eigen::Vector3d(20, 0, 0)).empty());
    ASSERT_EQ(localization.cloud().size(), 2u);
    expect_point(localization.cloud()[0], 0.01f, 0, 0);
    expect_point(localization.cloud()[1], 0.02f, 0, 0);
}

}  // namespace
