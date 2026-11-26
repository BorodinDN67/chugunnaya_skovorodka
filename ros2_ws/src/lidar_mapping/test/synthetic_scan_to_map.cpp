#include "lidar_mapping/core/scan_to_map.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace lidar_core;
namespace fs = std::filesystem;

Pose pose(double x, double y, double yaw) {
    Pose result = Pose::Identity();
    result.translation() = Eigen::Vector3d(x, y, 0.85);
    result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return result;
}

PointCloud room() {
    PointCloud cloud;
    for (int x = 0; x <= 40; ++x) {
        for (int y = 0; y <= 34; ++y) {
            cloud.emplace_back(0.15f * x, -2.55f + 0.15f * y, 0);
        }
    }
    for (int z = 1; z <= 17; ++z) {
        for (int y = 0; y <= 34; ++y) {
            cloud.emplace_back(6, -2.55f + 0.15f * y, 0.15f * z);
        }
        for (int x = 1; x < 40; ++x) {
            cloud.emplace_back(0.15f * x, -2.55f, 0.15f * z);
            cloud.emplace_back(0.15f * x, 2.55f, 0.15f * z);
        }
    }
    // Offset oblique panel and two faces of a low cabinet break symmetry.
    for (int i = 0; i <= 12; ++i) {
        for (int z = 1; z <= 13; ++z) {
            cloud.emplace_back(3.3f + 0.066f * i, -1.7f + 0.12f * i, 0.12f * z);
        }
    }
    for (int i = 0; i <= 9; ++i) {
        for (int j = 0; j <= 6; ++j) {
            cloud.emplace_back(2.6f, 0.75f + 0.12f * i, 0.12f * j);
            cloud.emplace_back(2.6f + 0.12f * j, 0.75f + 0.12f * i, 0.72f);
        }
    }
    return cloud;
}

PointCloud observe(const PointCloud& world, const Pose& map_from_robot, int frame) {
    const Pose robot_from_map = map_from_robot.inverse();
    PointCloud cloud;
    std::size_t index = 0;
    for (const auto& point : world) {
        Eigen::Vector3d local = robot_from_map * point.getVector3fMap().cast<double>();
        if (local.x() > 0.2 && local.norm() < 4.9 &&
            std::abs(local.y()) < 3.5 * local.x() &&
            std::abs(local.z()) < 1.2 * local.x()) {
            const double phase = static_cast<double>(index) * 1.7 + frame * 0.63;
            local += 0.002 * Eigen::Vector3d(std::sin(phase), std::cos(phase * 0.7),
                                             std::sin(phase * 1.2));
            cloud.emplace_back(static_cast<float>(local.x()), static_cast<float>(local.y()),
                               static_cast<float>(local.z()));
        }
        ++index;
    }
    return cloud;
}

Pose truth(int frame, bool localization) {
    if (localization) {
        return pose(0.65 + 0.065 * frame, 0.28 - 0.045 * frame, -0.045 + 0.007 * frame);
    }
    return pose(0.105 * frame, 0.15 * std::sin(0.22 * frame), 0.045 * std::sin(0.27 * frame));
}

Pose odometry(int frame, bool localization) {
    const Pose robot = truth(frame, localization);
    // Independent odometry origins with a systematic drift growing over time.
    const Pose origin = pose(localization ? 43 : -12, localization ? -21 : 7, 0.35);
    Pose biased = robot;
    biased.translation() += Eigen::Vector3d(0.029 * frame, -0.012 * frame, 0.004 * frame);
    biased.linear() = robot.linear() *
        Eigen::AngleAxisd(0.006 * frame, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return origin * biased;
}

struct Metrics {
    double translation_rmse_m = 0;
    double rotation_rmse_rad = 0;
    std::size_t accepted = 0;
    std::size_t lost = 0;
};

void require(bool ok, const std::string& message) {
    if (!ok) {
        throw std::runtime_error(message);
    }
}

void write_position(std::ostream& stream, const Pose& estimate) {
    stream << ',' << estimate.translation().x() << ',' << estimate.translation().y()
           << ',' << estimate.translation().z();
    const Eigen::Quaterniond rotation(estimate.linear());
    const auto q = rotation.normalized();
    stream << ',' << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w();
}

void write_optional(std::ostream& stream, const std::optional<double>& value) {
    stream << ',';
    if (value) {
        stream << *value;
    }
}

Metrics run(ScanToMap& pipeline, const PointCloud& world, const fs::path& csv, bool localization) {
    std::ofstream stream(csv);
    stream << std::setprecision(12)
           << "stamp_ns,truth_x,truth_y,truth_z,truth_qx,truth_qy,truth_qz,truth_qw"
              ",pred_x,pred_y,pred_z,pred_qx,pred_qy,pred_qz,pred_qw"
              ",estimate_x,estimate_y,estimate_z,estimate_qx,estimate_qy,estimate_qz,estimate_qw"
              ",translation_error_m,rotation_error_rad,scan_status,tracking_status"
              ",source_points,target_points,map_updated,fitness_m2,correspondence_fraction"
              ",correction_translation_m,correction_rotation_rad,sync_ms,preprocess_ms,predict_ms"
              ",local_map_ms,gicp_ms,validate_ms,map_update_ms,alignment_ms,total_ms\n";

    Metrics metrics;
    const int count = localization ? 9 : 13;
    for (int frame = 0; frame < count; ++frame) {
        const auto reference = truth(frame, localization);
        const TimestampNs stamp = static_cast<TimestampNs>(frame) * 100000000;
        require(pipeline.push_odometry(stamp, odometry(frame, localization)), "invalid synthetic odometry");
        const auto result = pipeline.process(observe(world, reference, frame), stamp);
        require(result.final_pose && result.predicted_pose && result.filter,
                "synthetic frame without an estimate");
        require(!localization || !result.map_updated, "localization updated the map");

        const Pose error = relative_motion(reference, *result.final_pose);
        const double distance = error.translation().norm();
        const Eigen::Quaterniond rotation(error.linear());
        const double angle = Eigen::AngleAxisd(rotation.normalized()).angle();
        metrics.translation_rmse_m += distance * distance;
        metrics.rotation_rmse_rad += angle * angle;
        metrics.accepted += result.status == ScanStatus::accepted;
        metrics.lost += result.tracking == TrackingStatus::lost;

        stream << stamp;
        write_position(stream, reference);
        write_position(stream, *result.predicted_pose);
        write_position(stream, *result.final_pose);
        stream << ',' << distance << ',' << angle << ',' << static_cast<int>(result.status)
               << ',' << static_cast<int>(result.tracking)
               << ',' << result.filter->output_points << ',' << result.target_points
               << ',' << result.map_updated;
        write_optional(stream, result.registration ? result.registration->fitness_m2 : std::nullopt);
        write_optional(stream, result.registration ? result.registration->correspondence_fraction : std::nullopt);
        write_optional(stream, result.correction_translation_m);
        write_optional(stream, result.correction_rotation_rad);
        const auto& t = result.times;
        stream << ',' << t.synchronization_ms << ',' << t.preprocessing_ms << ',' << t.prediction_ms
               << ',' << t.local_map_ms << ',' << t.registration_ms << ',' << t.validation_ms
               << ',' << t.map_update_ms << ',' << t.alignment_ms << ',' << t.total_ms << '\n';
    }
    stream.close();
    require(static_cast<bool>(stream), "cannot write " + csv.string());
    metrics.translation_rmse_m = std::sqrt(metrics.translation_rmse_m / count);
    metrics.rotation_rmse_rad = std::sqrt(metrics.rotation_rmse_rad / count);
    return metrics;
}

void write_summary(std::ostream& stream, const char* label, const Metrics& metrics) {
    stream << label << ',' << metrics.translation_rmse_m << ',' << metrics.rotation_rmse_rad
           << ',' << metrics.accepted << ',' << metrics.lost << '\n';
}

void exercise(const fs::path& directory) {
    fs::create_directories(directory);
    MapConfig map_config;
    map_config.directory = directory;
    ScanToMapConfig config{{0.2, 5.0, 0.05f}, {0.28, 0.17, 40, 16}, map_config,
                           {80, 0.12, 0.65, 0.22, 0.18, 4}, 32, 200000000, 2};
    const auto world = room();

    auto odom_config = config;
    odom_config.enable_registration = false;
    ScanToMap odom(odom_config, MapMode::mapping);
    require(odom.start_session(truth(0, false)), "odometry-only start failed");
    const auto odom_metrics = run(odom, world, directory / "odometry_only.csv", false);
    require(odom.save_map("odometry_only.pcd"), "cannot save odometry-only map");

    ScanToMap mapping(config, MapMode::mapping);
    require(mapping.start_session(truth(0, false)), "mapping start failed");
    const auto map_metrics = run(mapping, world, directory / "odometry_gicp.csv", false);
    require(mapping.save_map("odometry_gicp.pcd"), "cannot save GICP map");

    ScanToMap localization(config, MapMode::localization);
    require(localization.load_map("odometry_gicp.pcd"), "cannot load GICP map for localization");
    const PointCloud snapshot = localization.map();
    const auto first_scan = observe(world, truth(0, true), 0);
    require(localization.process(first_scan, 0, odometry(0, true)).status ==
                ScanStatus::awaiting_initial_pose, "localization should require an explicit pose");
    require(localization.start_session(pose(0.69, 0.26, -0.025)), "localization start failed");
    const auto loc_metrics = run(localization, world, directory / "localization.csv", true);
    const Pose restart = pose(0.75, 0.42, 0.02);
    require(localization.start_session(restart), "localization reset failed");
    const auto fresh = localization.process({}, 0, odometry(400, true));
    require(fresh.final_pose && fresh.final_pose->isApprox(restart) &&
                fresh.consecutive_failures == 1, "localization reset retained previous odometry");
    require(localization.map().size() == snapshot.size(), "localization changed map size");
    for (std::size_t i = 0; i < snapshot.size(); ++i) {
        require((snapshot[i].getVector3fMap().array() ==
                 localization.map()[i].getVector3fMap().array()).all(), "localization moved a map point");
    }

    std::ofstream summary(directory / "summary.csv");
    summary << "mode,translation_rmse_m,rotation_rmse_rad,accepted_registrations,lost_frames\n";
    write_summary(summary, "odometry_only", odom_metrics);
    write_summary(summary, "odometry_gicp", map_metrics);
    write_summary(summary, "localization", loc_metrics);
    summary.close();
    require(static_cast<bool>(summary), "cannot write summary.csv");
    write_summary(std::cout, "odometry_only", odom_metrics);
    write_summary(std::cout, "odometry_gicp", map_metrics);
    write_summary(std::cout, "localization", loc_metrics);

    require(odom_metrics.accepted == 0 && map_metrics.accepted >= 8 &&
                map_metrics.lost == 0 && map_metrics.translation_rmse_m < odom_metrics.translation_rmse_m,
            "GICP mapping did not improve the synthetic odometry baseline");
    require(loc_metrics.accepted >= 6 && loc_metrics.lost == 0 &&
                loc_metrics.translation_rmse_m < 0.18 && loc_metrics.rotation_rmse_rad < 0.12,
            "localization exceeded synthetic accuracy limits");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: lidar_synthetic <output-directory>\n";
        return 2;
    }
    try {
        exercise(argv[1]);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
