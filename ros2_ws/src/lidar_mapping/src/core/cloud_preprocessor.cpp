#include "lidar_mapping/core/cloud_preprocessor.hpp"

#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace lidar_core {

CloudPreprocessor::CloudPreprocessor(CloudFilterConfig config) : config_(config) {
    if (!std::isfinite(config.min_range_m) || !std::isfinite(config.max_range_m) ||
        !std::isfinite(config.voxel_size_m) || config.min_range_m < 0 ||
        config.max_range_m < config.min_range_m || config.voxel_size_m < 0) {
        throw std::invalid_argument("Invalid point cloud filter configuration");
    }
}

CloudFilterResult CloudPreprocessor::process(const PointCloud& input) const {
    CloudFilterResult result{};
    result.diagnostics.input_points = input.size();
    PointCloud::Ptr in_range(new PointCloud);
    in_range->header = input.header;
    in_range->sensor_origin_ = input.sensor_origin_;
    in_range->sensor_orientation_ = input.sensor_orientation_;
    in_range->reserve(input.size());

    for (const Point& point : input) {
        if (!pcl::isFinite(point)) {
            continue;
        }
        ++result.diagnostics.finite_points;
        const double distance = point.getVector3fMap().cast<double>().norm();
        if (distance >= config_.min_range_m && distance <= config_.max_range_m) {
            in_range->push_back(point);
        }
    }
    result.diagnostics.in_range_points = in_range->size();

    if (config_.voxel_size_m > 0 && !in_range->empty()) {
        pcl::VoxelGrid<Point> voxel;
        const float leaf = config_.voxel_size_m;
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(in_range);
        voxel.filter(result.cloud);
    } else {
        result.cloud = std::move(*in_range);
    }
    result.diagnostics.output_points = result.cloud.size();
    return result;
}

}  // namespace lidar_core
