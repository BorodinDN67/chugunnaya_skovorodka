#pragma once

#include <Eigen/Geometry>

namespace lidar_core {

// T_A_B maps points expressed in B into A. Inputs must be valid rigid transforms.
using Pose = Eigen::Isometry3d;

bool is_rigid_transform(const Eigen::Matrix4d& matrix);

// Given T_parent_previous and T_parent_current, return T_previous_current.
Pose relative_motion(const Pose& previous, const Pose& current);

Pose predict_pose(const Pose& map_previous, const Pose& odom_previous,
                  const Pose& odom_current);

// Interpolate translation in the common parent frame and rotation by shortest-path SLERP.
// fraction must be in [0, 1].
Pose interpolate_pose(const Pose& start, const Pose& end, double fraction);

}  // namespace lidar_core
