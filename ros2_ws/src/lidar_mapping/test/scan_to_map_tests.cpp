#include "lidar_mapping/core/scan_to_map.hpp"

#include <gtest/gtest.h>

namespace {
using namespace lidar_core;

Pose translated(double x) {
    Pose pose = Pose::Identity();
    pose.translation().x() = x;
    return pose;
}

ScanToMapConfig config() {
    return {{0, 30, 0}, {0.15, 0.10, 30, 4}, {},
            {6, 0.02, 0.75, 0.3, 0.2, 2}, 10, 20, 10};
}

PointCloud scan() {
    PointCloud cloud;
    for (int i = 0; i < 8; ++i) {
        cloud.emplace_back(1.0f + 0.2f * i, 0.13f * (i % 3), 0.17f * (i % 4));
    }
    return cloud;
}

TEST(ScanToMap, FallbackConsumesEachOdometryDeltaExactlyOnce) {
    ScanToMap pipeline(config(), MapMode::mapping);
    EXPECT_EQ(pipeline.process({}, 0, translated(5)).status, ScanStatus::insufficient_source);
    const auto initialized = pipeline.process(scan(), 1, translated(5));
    ASSERT_EQ(initialized.status, ScanStatus::initialized);
    ASSERT_TRUE(initialized.map_updated);
    ASSERT_TRUE(initialized.final_pose);
    EXPECT_DOUBLE_EQ(initialized.final_pose->translation().x(), 0);

    const auto empty = pipeline.process({}, 2, translated(6));
    EXPECT_EQ(empty.status, ScanStatus::insufficient_source);
    EXPECT_EQ(empty.tracking, TrackingStatus::degraded);
    ASSERT_TRUE(empty.final_pose);
    EXPECT_DOUBLE_EQ(empty.final_pose->translation().x(), 1);
    EXPECT_TRUE(empty.aligned_cloud.empty());
    EXPECT_FALSE(empty.map_updated);

    PointCloud sparse;
    sparse.emplace_back(0, 0, 0);
    const auto second = pipeline.process(sparse, 3, translated(7));
    EXPECT_EQ(second.status, ScanStatus::insufficient_source);
    EXPECT_EQ(second.tracking, TrackingStatus::lost);
    EXPECT_EQ(second.consecutive_failures, 2u);
    ASSERT_TRUE(second.predicted_pose);
    ASSERT_TRUE(second.final_pose);
    EXPECT_DOUBLE_EQ(second.predicted_pose->translation().x(), 2);
    EXPECT_DOUBLE_EQ(second.final_pose->translation().x(), 2);
    ASSERT_EQ(second.aligned_cloud.size(), 1u);
    EXPECT_FLOAT_EQ(second.aligned_cloud.front().x, 2);
    EXPECT_EQ(pipeline.map().size(), scan().size());
}

TEST(ScanToMap, RejectsGicpWithoutCorrespondencesAndKeepsPrediction) {
    ScanToMap pipeline(config(), MapMode::mapping);
    ASSERT_EQ(pipeline.process(scan(), 0, translated(0)).status, ScanStatus::initialized);
    auto disjoint = scan();
    for (auto& point : disjoint) {
        point.y += 3;
    }
    const auto result = pipeline.process(disjoint, 1, translated(0.4));
    EXPECT_EQ(result.status, ScanStatus::registration_failed);
    ASSERT_TRUE(result.registration);
    EXPECT_FALSE(result.registration->converged);
    ASSERT_TRUE(result.final_pose);
    EXPECT_DOUBLE_EQ(result.final_pose->translation().x(), 0.4);
    EXPECT_FALSE(result.map_updated);
    EXPECT_EQ(pipeline.map().size(), scan().size());
}

TEST(ScanToMap, SessionResetClearsTimestampFailuresAndOdometryBuffer) {
    ScanToMap pipeline(config(), MapMode::mapping);
    ASSERT_TRUE(pipeline.push_odometry(90, translated(0)));
    ASSERT_TRUE(pipeline.push_odometry(110, translated(2)));
    ASSERT_EQ(pipeline.process(scan(), 100).status, ScanStatus::initialized);
    const auto fallback = pipeline.process({}, 110);
    ASSERT_TRUE(fallback.final_pose);
    EXPECT_DOUBLE_EQ(fallback.final_pose->translation().x(), 1);
    EXPECT_EQ(pipeline.process(scan(), 100).status, ScanStatus::stale_scan);

    ASSERT_TRUE(pipeline.start_session(translated(10)));
    const auto missing = pipeline.process(scan(), 100);
    EXPECT_EQ(missing.status, ScanStatus::missing_odometry);
    EXPECT_EQ(missing.tracking, TrackingStatus::waiting);
    EXPECT_EQ(missing.consecutive_failures, 0u);
    const auto fresh = pipeline.process({}, 0, translated(500));
    ASSERT_TRUE(fresh.final_pose);
    EXPECT_DOUBLE_EQ(fresh.final_pose->translation().x(), 10);
    EXPECT_EQ(pipeline.map().size(), scan().size());
}
}  // namespace
