#pragma once

#include "lidar_mapping/core/point_cloud.hpp"

#include <Eigen/Core>

#include <optional>

namespace lidar_core {

struct GICPConfig {
    double max_correspondence_distance_m;
    double overlap_distance_m;
    int max_iterations;
    int covariance_neighbors;
};

enum class RegistrationStatus {
    empty_cloud,
    insufficient_points,
    invalid_input,
    not_converged,
    invalid_result,
    success
};

struct RegistrationResult {
    RegistrationStatus status;
    bool converged = false;  // Raw PCL convergence flag, including the iteration limit.
    std::optional<Eigen::Matrix4f> source_to_target;
    std::optional<double> fitness_m2;  // Mean squared nearest-target distance over all source points.
    std::optional<double> correspondence_fraction;  // Fraction within overlap_distance_m.
    double elapsed_ms = 0;  // Wall time including validation and diagnostics.
};

class GICPRegistration {
public:
    explicit GICPRegistration(GICPConfig config);

    // Inputs must contain finite XYZ points. A successful result includes the initial guess.
    RegistrationResult align(const PointCloud::ConstPtr& source,
                             const PointCloud::ConstPtr& target,
                             const Eigen::Matrix4f& initial_source_to_target) const;

private:
    GICPConfig config_;
};

}  // namespace lidar_core
