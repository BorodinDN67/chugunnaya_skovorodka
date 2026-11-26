#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_core {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

}  // namespace lidar_core
