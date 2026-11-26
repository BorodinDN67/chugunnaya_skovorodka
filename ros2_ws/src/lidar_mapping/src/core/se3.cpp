#include "lidar_mapping/core/se3.hpp"

#include <cmath>

namespace lidar_core {

bool is_rigid_transform(const Eigen::Matrix4d& matrix) {
    constexpr double tolerance = 1e-4;
    const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
    return matrix.allFinite() &&
           matrix.row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), tolerance) &&
           (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), tolerance) &&
           std::abs(rotation.determinant() - 1.0) <= tolerance;
}

Pose relative_motion(const Pose& previous, const Pose& current) {
    return previous.inverse() * current;
}

Pose predict_pose(const Pose& map_previous, const Pose& odom_previous,
                  const Pose& odom_current) {
    return map_previous * relative_motion(odom_previous, odom_current);
}

Pose interpolate_pose(const Pose& start, const Pose& end, double fraction) {
    const Eigen::Quaterniond start_rotation(start.linear());
    const Eigen::Quaterniond end_rotation(end.linear());
    Pose result = Pose::Identity();
    result.translation() = (1.0 - fraction) * start.translation() +
                           fraction * end.translation();
    result.linear() = start_rotation.normalized()
                          .slerp(fraction, end_rotation.normalized())
                          .normalized()
                          .toRotationMatrix();
    return result;
}

}  // namespace lidar_core
