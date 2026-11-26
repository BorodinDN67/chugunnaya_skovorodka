#include "lidar_mapping/core/scan_to_map.hpp"

#include <pcl/common/transforms.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lidar_core {
namespace {

using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

bool valid_quality(const RegistrationQuality& quality) {
    return quality.min_points >= 4 && quality.failures_until_lost > 0 &&
           std::isfinite(quality.max_fitness_m2) && quality.max_fitness_m2 >= 0 &&
           std::isfinite(quality.min_correspondence_fraction) &&
           quality.min_correspondence_fraction >= 0 && quality.min_correspondence_fraction <= 1 &&
           std::isfinite(quality.max_correction_translation_m) &&
           quality.max_correction_translation_m >= 0 &&
           std::isfinite(quality.max_correction_rotation_rad) &&
           quality.max_correction_rotation_rad >= 0 &&
           std::isfinite(quality.max_odometry_drift_m) && quality.max_odometry_drift_m >= 0 &&
           std::isfinite(quality.max_odometry_drift_rad) && quality.max_odometry_drift_rad >= 0;
}

bool outside_odometry_anchor(const Pose& pose, const Pose& anchor, const RegistrationQuality& quality) {
    const Pose delta = relative_motion(anchor, pose);
    const double angle = Eigen::AngleAxisd(Eigen::Quaterniond(delta.linear()).normalized()).angle();
    return (quality.max_odometry_drift_m > 0 && delta.translation().norm() > quality.max_odometry_drift_m) ||
           (quality.max_odometry_drift_rad > 0 && angle > quality.max_odometry_drift_rad);
}

}  // namespace

ScanToMap::ScanToMap(ScanToMapConfig config, MapMode mode)
    : config_(std::move(config)), mode_(mode),
      odometry_(config_.odometry_capacity, config_.max_odometry_gap_ns),
      preprocessor_(config_.cloud), registration_(config_.gicp), map_(config_.map, mode) {
    if (!valid_quality(config_.quality) || config_.keyframes_per_downsample == 0 ||
        (mode_ == MapMode::localization && !config_.enable_registration)) {
        throw std::invalid_argument("Invalid scan-to-map configuration");
    }
    if (mode_ == MapMode::mapping) {
        pose_ = Pose::Identity();
    }
}

bool ScanToMap::start_session(const Pose& initial_map_from_robot) {
    if (!is_rigid_transform(initial_map_from_robot.matrix())) {
        return false;
    }
    pose_ = initial_map_from_robot;
    odometry_.clear();
    previous_odometry_.reset();
    anchored_map_from_odom_.reset();
    previous_stamp_.reset();
    consecutive_failures_ = 0;
    tracking_ = TrackingStatus::waiting;
    return true;
}

bool ScanToMap::push_odometry(TimestampNs stamp, const Pose& odom_from_robot) {
    if (!is_rigid_transform(odom_from_robot.matrix())) {
        return false;
    }
    odometry_.insert(stamp, odom_from_robot);
    return true;
}

ScanResult ScanToMap::process(const PointCloud& scan, TimestampNs stamp) {
    return process_scan(scan, stamp, nullptr);
}

ScanResult ScanToMap::process(const PointCloud& scan, TimestampNs stamp,
                             const Pose& odom_from_robot) {
    return process_scan(scan, stamp, &odom_from_robot);
}

ScanResult ScanToMap::process_scan(const PointCloud& scan, TimestampNs stamp,
                                  const Pose* synchronized_odometry) {
    const auto start = Clock::now();
    auto stage_start = start;
    const auto lap = [&] {
        const auto now = Clock::now();
        return elapsed_ms(std::exchange(stage_start, now), now);
    };
    ScanResult result;
    const auto finish = [&] {
        result.tracking = tracking_;
        result.consecutive_failures = consecutive_failures_;
        result.times.total_ms = elapsed_ms(start, Clock::now());
        return std::move(result);
    };

    if (!pose_) {
        return finish();
    }
    if (mode_ == MapMode::localization && map_.cloud().empty()) {
        result.status = ScanStatus::awaiting_map;
        return finish();
    }
    if (previous_stamp_ && stamp <= *previous_stamp_) {
        result.status = ScanStatus::stale_scan;
        return finish();
    }
    const auto current_odometry = synchronized_odometry
        ? std::optional<Pose>(*synchronized_odometry) : odometry_.pose_at(stamp);
    result.times.synchronization_ms = lap();
    if (!current_odometry) {
        result.status = ScanStatus::missing_odometry;
        return finish();
    }
    if (!is_rigid_transform(current_odometry->matrix())) {
        result.status = ScanStatus::invalid_odometry;
        return finish();
    }

    auto filtered = preprocessor_.process(scan);
    result.filter = filtered.diagnostics;
    result.times.preprocessing_ms = lap();
    const auto min_points = std::max(config_.quality.min_points,
                                     static_cast<std::size_t>(config_.gicp.covariance_neighbors));
    const bool sufficient_source = filtered.cloud.size() >= min_points;
    const bool first_mapping_scan = mode_ == MapMode::mapping && map_.cloud().empty();
    if (first_mapping_scan && !sufficient_source) {
        result.status = ScanStatus::insufficient_source;
        return finish();
    }

    Pose prediction = previous_odometry_
        ? predict_pose(*pose_, *previous_odometry_, *current_odometry) : *pose_;
    std::optional<Pose> map_from_odom;
    Pose anchored_pose = Pose::Identity();
    if (mode_ == MapMode::mapping && config_.enable_registration &&
        (config_.quality.max_odometry_drift_m > 0 || config_.quality.max_odometry_drift_rad > 0)) {
        map_from_odom = anchored_map_from_odom_.value_or(*pose_ * current_odometry->inverse());
        anchored_pose = *map_from_odom * *current_odometry;
        if (outside_odometry_anchor(prediction, anchored_pose, config_.quality)) {
            prediction = anchored_pose;
        }
    }
    result.times.prediction_ms = lap();
    if (!is_rigid_transform(prediction.matrix())) {
        result.status = ScanStatus::invalid_prediction;
        return finish();
    }

    Pose estimate = prediction;
    result.predicted_pose = prediction;
    PointCloud::Ptr source(new PointCloud(std::move(filtered.cloud)));
    if (first_mapping_scan) {
        result.status = ScanStatus::initialized;
    } else if (!sufficient_source) {
        result.status = ScanStatus::insufficient_source;
    } else if (!config_.enable_registration) {
        result.status = ScanStatus::odometry_only;
    } else {
        stage_start = Clock::now();
        PointCloud::Ptr target(new PointCloud(map_.local_map(prediction.translation())));
        result.times.local_map_ms = lap();
        result.target_points = target->size();
        if (target->size() < min_points) {
            result.status = ScanStatus::insufficient_target;
        } else {
            result.registration = registration_.align(source, target, prediction.matrix().cast<float>());
            result.times.registration_ms = lap();
            result.status = validate_registration(prediction, result, estimate);
            if (result.status == ScanStatus::accepted && map_from_odom &&
                outside_odometry_anchor(estimate, anchored_pose, config_.quality)) {
                result.status = ScanStatus::odometry_anchor_rejected;
                estimate = prediction;
            }
            result.times.validation_ms = lap();
        }
    }

    const bool accepted = result.status == ScanStatus::accepted;
    const bool odometry_only = result.status == ScanStatus::odometry_only;
    if (first_mapping_scan || (mode_ == MapMode::mapping && (accepted || odometry_only))) {
        stage_start = Clock::now();
        if (map_.should_add_keyframe(estimate) && map_.add_cloud(*source, estimate)) {
            result.map_updated = true;
            if (++pending_keyframes_ == config_.keyframes_per_downsample) {
                result.map_downsampled = map_.downsample();
                pending_keyframes_ = 0;
            }
        }
        result.times.map_update_ms = lap();
    }
    if (first_mapping_scan && !result.map_updated) {
        result.status = ScanStatus::map_initialization_failed;
        return finish();
    }

    // Also consume valid odometry on fallback frames so no delta is applied twice.
    pose_ = estimate;
    previous_odometry_ = *current_odometry;
    anchored_map_from_odom_ = map_from_odom;
    previous_stamp_ = stamp;
    result.final_pose = estimate;
    if (accepted || first_mapping_scan || odometry_only) {
        consecutive_failures_ = 0;
        tracking_ = accepted ? TrackingStatus::tracking :
                    first_mapping_scan ? TrackingStatus::initialized : TrackingStatus::odometry_only;
    } else if (config_.enable_registration) {
        if (consecutive_failures_ < std::numeric_limits<std::size_t>::max()) {
            ++consecutive_failures_;
        }
        tracking_ = consecutive_failures_ >= config_.quality.failures_until_lost
            ? TrackingStatus::lost : TrackingStatus::degraded;
    }
    stage_start = Clock::now();
    if (!source->empty()) {
        pcl::transformPointCloud(*source, result.aligned_cloud, estimate.matrix());
    }
    result.times.alignment_ms = lap();
    return finish();
}

ScanStatus ScanToMap::validate_registration(const Pose& prediction, ScanResult& result,
                                           Pose& estimate) const {
    const auto& registration = *result.registration;
    if (!registration.converged || registration.status != RegistrationStatus::success) {
        return ScanStatus::registration_failed;
    }
    if (!registration.source_to_target) {
        return ScanStatus::invalid_candidate;
    }
    const Eigen::Matrix4d matrix = registration.source_to_target->cast<double>();
    if (!is_rigid_transform(matrix)) {
        return ScanStatus::invalid_candidate;
    }
    const Pose candidate(matrix);
    const Pose correction = relative_motion(prediction, candidate);
    result.correction_translation_m = correction.translation().norm();
    const Eigen::Quaterniond rotation(correction.linear());
    result.correction_rotation_rad = Eigen::AngleAxisd(rotation.normalized()).angle();

    const auto& quality = config_.quality;
    if (!registration.fitness_m2 || !std::isfinite(*registration.fitness_m2) ||
        *registration.fitness_m2 < 0 || *registration.fitness_m2 > quality.max_fitness_m2) {
        return ScanStatus::fitness_rejected;
    }
    if (!registration.correspondence_fraction || !std::isfinite(*registration.correspondence_fraction) ||
        *registration.correspondence_fraction < quality.min_correspondence_fraction ||
        *registration.correspondence_fraction > 1) {
        return ScanStatus::overlap_rejected;
    }
    if (*result.correction_translation_m > quality.max_correction_translation_m ||
        *result.correction_rotation_rad > quality.max_correction_rotation_rad) {
        return ScanStatus::correction_rejected;
    }

    estimate = candidate;
    return ScanStatus::accepted;
}

bool ScanToMap::load_map(const std::filesystem::path& filename) {
    return mode_ == MapMode::localization && map_.load_pcd(filename);
}

bool ScanToMap::save_map(const std::filesystem::path& filename) const {
    return map_.save_pcd(filename);
}

const PointCloud& ScanToMap::map() const {
    return map_.cloud();
}

}  // namespace lidar_core
