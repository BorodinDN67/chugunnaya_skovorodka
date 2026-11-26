#include "lidar_mapping/core/gicp_registration.hpp"

#include "lidar_mapping/core/se3.hpp"
#include <pcl/common/point_tests.h>
#include <pcl/exceptions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace lidar_core {
namespace {

bool is_finite_cloud(const PointCloud& cloud) {
    return std::all_of(cloud.begin(), cloud.end(), [](const Point& point) {
        return pcl::isFinite(point);
    });
}

}  // namespace

GICPRegistration::GICPRegistration(GICPConfig config) : config_(config) {
    if (!std::isfinite(config.max_correspondence_distance_m) ||
        !std::isfinite(config.overlap_distance_m) ||
        config.max_correspondence_distance_m <= 0 || config.overlap_distance_m <= 0 ||
        config.max_iterations <= 0 || config.covariance_neighbors < 3) {
        throw std::invalid_argument("Invalid GICP configuration");
    }
}

RegistrationResult GICPRegistration::align(const PointCloud::ConstPtr& source,
                                           const PointCloud::ConstPtr& target,
                                           const Eigen::Matrix4f& initial_source_to_target) const {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    RegistrationResult result{};
    const auto finish = [&](RegistrationStatus status) {
        result.status = status;
        result.elapsed_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return result;
    };

    if (!source || !target || source->empty() || target->empty()) {
        return finish(RegistrationStatus::empty_cloud);
    }
    // PCL's rigid optimizer requires four correspondences.
    constexpr std::size_t min_correspondences = 4;
    const auto neighbors = static_cast<std::size_t>(config_.covariance_neighbors);
    const auto min_points = std::max(min_correspondences, neighbors);
    if (source->size() < min_points || target->size() < min_points) {
        return finish(RegistrationStatus::insufficient_points);
    }
    if (!is_rigid_transform(initial_source_to_target.cast<double>()) ||
        !is_finite_cloud(*source) || !is_finite_cloud(*target)) {
        return finish(RegistrationStatus::invalid_input);
    }

    try {
        pcl::GeneralizedIterativeClosestPoint<Point, Point> registration;
        registration.setInputSource(source);
        registration.setInputTarget(target);
        registration.setMaxCorrespondenceDistance(config_.max_correspondence_distance_m);
        registration.setMaximumIterations(config_.max_iterations);
        registration.setCorrespondenceRandomness(config_.covariance_neighbors);
        PointCloud aligned;
        registration.align(aligned, initial_source_to_target);
        result.converged = registration.hasConverged();
        if (!result.converged) {
            return finish(RegistrationStatus::not_converged);
        }

        const Eigen::Matrix4f transform = registration.getFinalTransformation();
        if (!is_rigid_transform(transform.cast<double>()) || aligned.size() != source->size() ||
            !is_finite_cloud(aligned)) {
            return finish(RegistrationStatus::invalid_result);
        }

        pcl::KdTreeFLANN<Point> target_tree;
        target_tree.setInputCloud(target);
        pcl::Indices indices(1);
        std::vector<float> squared_distances(1);
        const double overlap_squared = config_.overlap_distance_m * config_.overlap_distance_m;
        const double max_squared =
            config_.max_correspondence_distance_m * config_.max_correspondence_distance_m;
        std::size_t overlap_count = 0;
        std::size_t optimizer_count = 0;
        double sum_squared = 0;
        for (const Point& point : aligned) {
            if (target_tree.nearestKSearch(point, 1, indices, squared_distances) != 1 ||
                !std::isfinite(squared_distances.front())) {
                return finish(RegistrationStatus::invalid_result);
            }
            const double squared_distance = squared_distances.front();
            sum_squared += squared_distance;
            overlap_count += squared_distance <= overlap_squared;
            optimizer_count += squared_distance < max_squared;
        }
        if (optimizer_count < min_correspondences) {
            return finish(RegistrationStatus::invalid_result);
        }

        result.source_to_target = transform;
        result.fitness_m2 = sum_squared / source->size();
        result.correspondence_fraction = static_cast<double>(overlap_count) / source->size();
        return finish(RegistrationStatus::success);
    } catch (const pcl::PCLException&) {
        return finish(RegistrationStatus::invalid_result);
    }
}

}  // namespace lidar_core
