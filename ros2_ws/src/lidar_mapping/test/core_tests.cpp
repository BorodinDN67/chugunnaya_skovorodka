#include "lidar_mapping/core/odometry_buffer.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

namespace {

using lidar_core::OdometryBuffer;
using lidar_core::Pose;
using lidar_core::TimestampNs;
using Eigen::Vector3d;

constexpr double pi = 3.14159265358979323846;

Pose make_pose(const Vector3d& translation, const Vector3d& axis, double angle) {
    Pose result = Pose::Identity();
    result.translation() = translation;
    result.linear() = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    return result;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

TEST(Pose, RelativeMotionAndOdometryPrediction) {
    const Pose previous = make_pose({3, 4, 5}, Vector3d::UnitZ(), pi / 2);
    const Pose current = make_pose({3, 6, 6}, Vector3d::UnitZ(), pi);
    const Pose expected_delta = make_pose({2, 0, 1}, Vector3d::UnitZ(), pi / 2);
    require(lidar_core::relative_motion(previous, current).isApprox(expected_delta),
            "relative motion must be expressed in the previous body frame");

    const Pose map_previous = make_pose({10, -2, 7}, Vector3d::UnitX(), pi / 2);
    const Pose prediction = lidar_core::predict_pose(map_previous, previous, current);
    require(prediction.translation().isApprox(Vector3d(12, -3, 7), 1e-12),
            "prediction must rotate the odometry displacement into the map");
    require((prediction.linear() * Vector3d::UnitX()).isApprox(Vector3d::UnitZ(), 1e-12),
            "prediction must compose rotations in map-then-odometry order");
}

TEST(OdometryBuffer, Interpolation) {
    constexpr TimestampNs start_time = 1700000000000000000LL;
    const Pose start = make_pose({0, 2, 4}, Vector3d::UnitZ(), 170 * pi / 180);
    const Pose end = make_pose({4, 6, 8}, Vector3d::UnitZ(), -170 * pi / 180);
    OdometryBuffer buffer(2, 20);
    require(!buffer.pose_at(start_time), "empty buffer must report a missing pose");
    buffer.insert(start_time + 20, end);
    require(!buffer.pose_at(start_time + 10), "one sample cannot bracket a query");
    buffer.insert(start_time, start);
    const auto middle = buffer.pose_at(start_time + 10);
    require(middle.has_value(), "a bracket at the gap limit must be accepted");
    require(middle->translation().isApprox(Vector3d(2, 4, 6), 1e-12),
            "translation must be interpolated linearly in the parent frame");
    require((middle->linear() * Vector3d::UnitX()).isApprox(-Vector3d::UnitX(), 1e-12),
            "SLERP must cross the shortest arc through 180 degrees");
    require(!buffer.pose_at(start_time - 1) && !buffer.pose_at(start_time + 21),
            "queries outside the sample range must not extrapolate");
}

TEST(OdometryBuffer, BufferLimits) {
    const Pose identity = Pose::Identity();
    const Pose replacement = make_pose({1, 0, 0}, Vector3d::UnitY(), pi / 2);
    OdometryBuffer buffer(2, 10);
    buffer.insert(-20, identity);
    buffer.insert(0, identity);
    buffer.insert(0, replacement);
    const auto exact = buffer.pose_at(0);
    require(exact && exact->isApprox(replacement), "duplicate must replace the exact pose");
    require(buffer.pose_at(-20).has_value(), "duplicate must not consume capacity");
    require(!buffer.pose_at(-10), "gap above the limit must reject interpolation");
    buffer.insert(10, identity);
    buffer.insert(-30, identity);
    require(!buffer.pose_at(-20) && !buffer.pose_at(-30) &&
                buffer.pose_at(0) && buffer.pose_at(10),
            "capacity must retain the latest samples by timestamp");
    buffer.clear();
    require(!buffer.pose_at(10), "clear must discard previously stored samples");

    OdometryBuffer wide(2, std::numeric_limits<std::uint64_t>::max());
    wide.insert(std::numeric_limits<TimestampNs>::min(), identity);
    wide.insert(std::numeric_limits<TimestampNs>::max(), replacement);
    const auto middle = wide.pose_at(0);
    require(middle && middle->translation().isApprox(Vector3d(0.5, 0, 0), 1e-12),
            "full signed timestamp range must interpolate without integer overflow");
}

}  // namespace
