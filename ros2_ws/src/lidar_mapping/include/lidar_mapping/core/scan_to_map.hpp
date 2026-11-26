#pragma once

#include "lidar_mapping/core/cloud_preprocessor.hpp"
#include "lidar_mapping/core/gicp_registration.hpp"
#include "lidar_mapping/core/map_manager.hpp"
#include "lidar_mapping/core/odometry_buffer.hpp"

namespace lidar_core {

struct RegistrationQuality {
    std::size_t min_points;
    double max_fitness_m2;
    double min_correspondence_fraction;
    double max_correction_translation_m;
    double max_correction_rotation_rad;
    std::size_t failures_until_lost;
    double max_odometry_drift_m = 0;  // Zero disables this mapping guard.
    double max_odometry_drift_rad = 0;
};

struct ScanToMapConfig {
    CloudFilterConfig cloud;
    GICPConfig gicp;
    MapConfig map;
    RegistrationQuality quality;
    std::size_t odometry_capacity;
    std::uint64_t max_odometry_gap_ns;
    std::size_t keyframes_per_downsample = 1;
    bool enable_registration = true;  // False selects the odometry-only mapping baseline.
};

enum class TrackingStatus { waiting, initialized, tracking, odometry_only, degraded, lost };
enum class ScanStatus {
    awaiting_initial_pose, awaiting_map, missing_odometry, invalid_odometry, stale_scan,
    invalid_prediction, insufficient_source, insufficient_target, map_initialization_failed, initialized, odometry_only, registration_failed,
    invalid_candidate, fitness_rejected, overlap_rejected, correction_rejected, accepted, odometry_anchor_rejected
};

struct StageTimes {
    double synchronization_ms = 0;
    double preprocessing_ms = 0;
    double prediction_ms = 0;
    double local_map_ms = 0;
    double registration_ms = 0;
    double validation_ms = 0;
    double map_update_ms = 0;
    double alignment_ms = 0;
    double total_ms = 0;
};

struct ScanResult {
    ScanStatus status = ScanStatus::awaiting_initial_pose;
    TrackingStatus tracking = TrackingStatus::waiting;
    std::optional<Pose> predicted_pose;
    std::optional<Pose> final_pose;
    PointCloud aligned_cloud;  // Filtered scan transformed by final_pose, including fallback.
    std::optional<CloudFilterDiagnostics> filter;
    std::size_t target_points = 0;
    std::optional<RegistrationResult> registration;
    std::optional<double> correction_translation_m;
    std::optional<double> correction_rotation_rad;
    std::size_t consecutive_failures = 0;
    bool map_updated = false;
    bool map_downsampled = false;
    StageTimes times;
};

class ScanToMap {
public:
    ScanToMap(ScanToMapConfig config, MapMode mode);

    // Start a new time sequence with an explicit pose in the current map frame.
    // Clears tracking and buffered odometry; keeps the map. A new map needs a new instance.
    bool start_session(const Pose& initial_map_from_robot);
    bool push_odometry(TimestampNs stamp, const Pose& odom_from_robot);

    // The first overload interpolates buffered odometry. The second uses a synchronized pose.
    ScanResult process(const PointCloud& scan, TimestampNs stamp);
    ScanResult process(const PointCloud& scan, TimestampNs stamp, const Pose& odom_from_robot);

    bool load_map(const std::filesystem::path& filename);  // Localization only, before loading.
    bool save_map(const std::filesystem::path& filename) const;
    const PointCloud& map() const;

private:
    ScanResult process_scan(const PointCloud& scan, TimestampNs stamp,
                            const Pose* synchronized_odometry);
    ScanStatus validate_registration(const Pose& prediction, ScanResult& result,
                                     Pose& estimate) const;

    ScanToMapConfig config_;
    MapMode mode_;
    OdometryBuffer odometry_;
    CloudPreprocessor preprocessor_;
    GICPRegistration registration_;
    MapManager map_;
    std::optional<Pose> pose_;
    std::optional<Pose> previous_odometry_;
    std::optional<Pose> anchored_map_from_odom_;
    std::optional<TimestampNs> previous_stamp_;
    TrackingStatus tracking_ = TrackingStatus::waiting;
    std::size_t consecutive_failures_ = 0;
    std::size_t pending_keyframes_ = 0;
};

}  // namespace lidar_core
