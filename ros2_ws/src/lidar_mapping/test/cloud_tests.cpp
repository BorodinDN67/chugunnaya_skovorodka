#include "lidar_mapping/core/cloud_preprocessor.hpp"
#include "lidar_mapping/core/gicp_registration.hpp"

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

#include <cmath>
#include <limits>

namespace {

using namespace lidar_core;

TEST(CloudPreprocessor, RemovesNonFinitePointsAndAppliesInclusiveRadialRange) {
    PointCloud input;
    input.is_dense = true;  // Deliberately inaccurate metadata.
    input.emplace_back(std::numeric_limits<float>::quiet_NaN(), 0, 0);
    input.emplace_back(0, std::numeric_limits<float>::infinity(), 0);
    input.emplace_back(0, 0, -std::numeric_limits<float>::infinity());
    input.emplace_back(0.5f, 0, 0);
    input.emplace_back(1, 0, 0);
    input.emplace_back(0, 0, 2);
    input.emplace_back(1.5f, 1.5f, 0);

    const auto result = CloudPreprocessor({1, 2, 0}).process(input);
    EXPECT_EQ(result.diagnostics.input_points, 7u);
    EXPECT_EQ(result.diagnostics.finite_points, 4u);
    EXPECT_EQ(result.diagnostics.in_range_points, 2u);
    EXPECT_EQ(result.diagnostics.output_points, 2u);
    ASSERT_EQ(result.cloud.size(), 2u);
    EXPECT_FLOAT_EQ(result.cloud[0].x, 1);
    EXPECT_FLOAT_EQ(result.cloud[1].z, 2);
    EXPECT_TRUE(result.cloud.is_dense);
}

TEST(CloudPreprocessor, VoxelGridProducesCentroids) {
    PointCloud input;
    input.emplace_back(0.10f, 0.2f, 0.3f);
    input.emplace_back(0.30f, 0.2f, 0.3f);
    input.emplace_back(1.25f, 0.2f, 0.3f);
    const auto result = CloudPreprocessor({0, 3, 1}).process(input);

    EXPECT_EQ(result.diagnostics.in_range_points, 3u);
    ASSERT_EQ(result.diagnostics.output_points, 2u);
    ASSERT_EQ(result.cloud.size(), 2u);
    for (const Point& point : result.cloud) {
        EXPECT_NEAR(point.x, point.x < 1 ? 0.20f : 1.25f, 1e-6f);
        EXPECT_FLOAT_EQ(point.y, 0.2f);
        EXPECT_FLOAT_EQ(point.z, 0.3f);
    }
    EXPECT_NE(result.cloud[0].x < 1, result.cloud[1].x < 1);
}

PointCloud::Ptr asymmetric_cloud() {
    PointCloud::Ptr cloud(new PointCloud);
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 8; ++j) {
            if (i + j < 14) {
                cloud->emplace_back(0.16f * i, 0.18f * j, 0.002f * i * j);
            }
        }
    }
    for (int i = 0; i < 8; ++i) {
        for (int j = 1; j < 7; ++j) {
            cloud->emplace_back(0.17f * i, -0.25f + 0.008f * j, 0.19f * j);
        }
    }
    for (int i = 0; i < 6; ++i) {
        for (int j = 1; j < 5; ++j) {
            cloud->emplace_back(-0.25f + 0.006f * i * j, 0.18f * i, 0.20f * j);
        }
    }
    return cloud;
}

Eigen::Matrix4f reference_transform() {
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::AngleAxisf(0.34f, Eigen::Vector3f(1, -2, 3).normalized())
                        .toRotationMatrix();
    pose.translation() = Eigen::Vector3f(0.42f, -0.31f, 0.27f);
    return pose.matrix();
}

Eigen::Matrix4f perturbed_guess(const Eigen::Matrix4f& reference) {
    Eigen::Isometry3f error = Eigen::Isometry3f::Identity();
    error.linear() = Eigen::AngleAxisf(0.025f, Eigen::Vector3f::UnitY()).toRotationMatrix();
    error.translation() = Eigen::Vector3f(0.025f, -0.020f, 0.015f);
    return error.matrix() * reference;
}

const GICPRegistration registration({0.18, 0.05, 60, 12});

TEST(GICPRegistration, RecoversFullSourceToTargetTransform) {
    const auto source = asymmetric_cloud();
    PointCloud::Ptr target(new PointCloud);
    const Eigen::Matrix4f expected = reference_transform();
    pcl::transformPointCloud(*source, *target, expected);

    const auto result = registration.align(source, target, perturbed_guess(expected));
    ASSERT_EQ(result.status, RegistrationStatus::success);
    ASSERT_TRUE(result.converged);
    ASSERT_TRUE(result.source_to_target);
    ASSERT_TRUE(result.fitness_m2);
    ASSERT_TRUE(result.correspondence_fraction);
    const Eigen::Matrix4f actual = *result.source_to_target;
    EXPECT_LT((actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm(), 0.005f);
    EXPECT_LT((actual.block<3, 3>(0, 0) - expected.block<3, 3>(0, 0)).norm(), 0.01f);
    const Eigen::Vector4f point = source->back().getVector4fMap();
    EXPECT_LT(((actual * point).head<3>() - target->back().getVector3fMap()).norm(), 0.01f);
    EXPECT_LT(*result.fitness_m2, 1e-4);
    EXPECT_DOUBLE_EQ(*result.correspondence_fraction, 1.0);
    EXPECT_GE(result.elapsed_ms, 0.0);
}

TEST(GICPRegistration, HandlesEmptyCloudsAndMissingPointers) {
    const PointCloud::Ptr empty(new PointCloud);
    const auto points = asymmetric_cloud();
    for (const auto& result : {
             registration.align(empty, points, Eigen::Matrix4f::Identity()),
             registration.align(points, empty, Eigen::Matrix4f::Identity()),
             registration.align({}, points, Eigen::Matrix4f::Identity())}) {
        EXPECT_EQ(result.status, RegistrationStatus::empty_cloud);
        EXPECT_FALSE(result.converged);
        EXPECT_FALSE(result.source_to_target.has_value());
        EXPECT_FALSE(result.fitness_m2.has_value());
        EXPECT_FALSE(result.correspondence_fraction.has_value());
    }
}

TEST(GICPRegistration, RejectsInvalidInputsAndInsufficientPoints) {
    const auto cloud = asymmetric_cloud();
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
    guess(2, 2) = -1;
    EXPECT_EQ(registration.align(cloud, cloud, guess).status, RegistrationStatus::invalid_input);

    PointCloud::Ptr small(new PointCloud);
    small->emplace_back(0, 0, 0);
    EXPECT_EQ(registration.align(small, cloud, Eigen::Matrix4f::Identity()).status,
              RegistrationStatus::insufficient_points);

    cloud->front().z = std::numeric_limits<float>::quiet_NaN();
    EXPECT_EQ(registration.align(cloud, cloud, Eigen::Matrix4f::Identity()).status,
              RegistrationStatus::invalid_input);
}

TEST(GICPRegistration, RejectsCloudsWithoutOverlapAtTheInitialGuess) {
    const auto source = asymmetric_cloud();
    PointCloud::Ptr target(new PointCloud);
    Eigen::Matrix4f far = Eigen::Matrix4f::Identity();
    far(0, 3) = 100;
    pcl::transformPointCloud(*source, *target, far);

    const auto result = registration.align(source, target, Eigen::Matrix4f::Identity());
    EXPECT_EQ(result.status, RegistrationStatus::not_converged);
    EXPECT_FALSE(result.converged);
    EXPECT_FALSE(result.source_to_target.has_value());
}

TEST(GICPRegistration, RecoversPoseWithNoiseAndPartialOverlap) {
    const auto source = asymmetric_cloud();
    const auto shared_points = source->size();
    PointCloud::Ptr target(new PointCloud);
    const Eigen::Matrix4f expected = reference_transform();
    pcl::transformPointCloud(*source, *target, expected);
    for (std::size_t i = 0; i < shared_points; ++i) {
        const float index = static_cast<float>(i);
        (*target)[i].x += 0.004f * std::sin(index * 1.7f);
        (*target)[i].y += 0.004f * std::cos(index * 2.1f);
        (*target)[i].z += 0.003f * std::sin(index * 0.9f);
    }
    // This source-only patch is outside the optimization and overlap distance gates.
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 6; ++j) {
            source->emplace_back(3.0f + 0.18f * i, 1.8f + 0.15f * j, 0.6f + 0.01f * i * j);
        }
    }

    const auto result = registration.align(source, target, perturbed_guess(expected));
    ASSERT_EQ(result.status, RegistrationStatus::success);
    ASSERT_TRUE(result.converged);
    ASSERT_TRUE(result.source_to_target);
    ASSERT_TRUE(result.correspondence_fraction);
    ASSERT_TRUE(result.fitness_m2);
    const Eigen::Matrix4f actual = *result.source_to_target;
    EXPECT_LT((actual.block<3, 1>(0, 3) - expected.block<3, 1>(0, 3)).norm(), 0.02f);
    EXPECT_LT((actual.block<3, 3>(0, 0) - expected.block<3, 3>(0, 0)).norm(), 0.03f);
    EXPECT_NEAR(*result.correspondence_fraction,
                static_cast<double>(shared_points) / source->size(), 0.02);
    EXPECT_GT(*result.fitness_m2, 0.01);
}

}  // namespace
