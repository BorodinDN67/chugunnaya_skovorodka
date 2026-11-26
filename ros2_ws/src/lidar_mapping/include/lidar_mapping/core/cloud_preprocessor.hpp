#pragma once

#include "lidar_mapping/core/point_cloud.hpp"

#include <cstddef>

namespace lidar_core {

struct CloudFilterConfig {
    double min_range_m;
    double max_range_m;
    float voxel_size_m;  // Zero disables voxelization.
};

struct CloudFilterDiagnostics {
    std::size_t input_points;
    std::size_t finite_points;
    std::size_t in_range_points;
    std::size_t output_points;
};

struct CloudFilterResult {
    PointCloud cloud;
    CloudFilterDiagnostics diagnostics;
};

class CloudPreprocessor {
public:
    explicit CloudPreprocessor(CloudFilterConfig config);

    // Applies an inclusive Euclidean range relative to the cloud's origin.
    CloudFilterResult process(const PointCloud& input) const;

private:
    CloudFilterConfig config_;
};

}  // namespace lidar_core
