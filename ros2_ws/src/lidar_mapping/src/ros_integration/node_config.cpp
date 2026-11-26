#include "node_config.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace lidar_ros {
namespace {

template <typename T>
T parameter(rclcpp::Node& node, const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return node.declare_parameter<T>(name, value, descriptor);
}

int positive_int(rclcpp::Node& node, const std::string& name, int fallback) {
    const auto value = parameter<std::int64_t>(node, name, fallback);
    if (value < 1 || value > std::numeric_limits<int>::max()) {
        throw std::invalid_argument(name + " must be a positive integer");
    }
    return static_cast<int>(value);
}

Mode read_mode(const std::string& name) {
    if (name == "deskewed") { return Mode::deskewed; }
    if (name == "odometry") { return Mode::odometry; }
    if (name == "gicp_mapping") { return Mode::mapping; }
    if (name == "gicp_localization") { return Mode::localization; }
    throw std::invalid_argument("Unknown mode: " + name);
}

std::string output_name(Mode mode) {
    switch (mode) {
        case Mode::deskewed: return "map_baseline.pcd";
        case Mode::odometry: return "map_odometry.pcd";
        case Mode::mapping: return "map_gicp.pcd";
        case Mode::localization: return "";
    }
    return "";
}

}  // namespace

NodeConfig::NodeConfig(rclcpp::Node& node)
    : mode_name(parameter<std::string>(node, "mode", "gicp_mapping")), mode(read_mode(mode_name)),
      cloud_topic(mode == Mode::deskewed
                      ? parameter<std::string>(node, "topics.cloud_deskewed", "/utlidar/cloud_deskewed")
                      : parameter<std::string>(node, "topics.cloud_base", "/utlidar/cloud_base")),
      odom_topic(parameter<std::string>(node, "topics.odometry", "/utlidar/robot_odom")),
      map_frame(parameter<std::string>(node, "frames.map", "map")),
      odom_frame(parameter<std::string>(node, "frames.odom", "odom")),
      base_frame(parameter<std::string>(node, "frames.base", "base_link")),
      cloud_depth(positive_int(node, "qos.cloud_depth", 32)),
      odom_depth(positive_int(node, "qos.odometry_depth", 256)),
      pending_capacity(positive_int(node, "sync.pending_capacity", 32)),
      wait(positive_int(node, "sync.wait_ms", 250)),
      map_period(positive_int(node, "publish.map_period_ms", 1000)),
      path_capacity(positive_int(node, "publish.path_capacity", 3000)),
      publish_map_tf(parameter(node, "publish.map_to_odom_tf", true)),
      publish_odom_tf(parameter(node, "publish.odom_to_base_tf", true)),
      save_on_shutdown(parameter(node, "output.save_on_shutdown", true)),
      map_input(parameter<std::string>(node, "map.input_file", "map_gicp.pcd")),
      map_output(parameter(node, "map.output_file", output_name(mode))),
      metrics_file(parameter(node, "output.metrics_file", std::string("/ros2_ws/results/") + mode_name + ".csv")) {
    if (map_frame.empty() || odom_frame.empty() || base_frame.empty() ||
        map_frame == odom_frame || odom_frame == base_frame || base_frame == map_frame ||
        map_frame.front() == '/' || odom_frame.front() == '/' || base_frame.front() == '/') {
        throw std::invalid_argument("Frame IDs must be distinct and must not start with a slash");
    }
    if (mode != Mode::localization) {
        const std::filesystem::path file(map_output);
        if (file != file.filename() || file.extension() != ".pcd") {
            throw std::invalid_argument("map.output_file must be a plain .pcd filename");
        }
    }
    core.cloud = {parameter(node, "cloud.min_range_m", 0.3), parameter(node, "cloud.max_range_m", 40.0),
                  static_cast<float>(parameter(node, "cloud.voxel_size_m", 0.05))};
    core.gicp = {parameter(node, "gicp.max_correspondence_m", 0.5),
                 parameter(node, "gicp.overlap_distance_m", 0.20),
                 positive_int(node, "gicp.max_iterations", 30),
                 positive_int(node, "gicp.covariance_neighbors", 16)};
    core.map = {static_cast<float>(parameter(node, "map.voxel_size_m", 0.05)),
                parameter(node, "map.local_radius_m", 10.0), parameter(node, "map.keyframe_translation_m", 0.2),
                parameter(node, "map.keyframe_rotation_deg", 10.0),
                parameter<std::string>(node, "map.directory", "/ros2_ws/maps")};
    core.quality = {static_cast<std::size_t>(positive_int(node, "quality.min_points", 80)),
                    parameter(node, "quality.max_fitness_m2", 0.12), parameter(node, "quality.min_overlap", 0.50),
                    parameter(node, "quality.max_correction_m", 0.4), parameter(node, "quality.max_correction_rad", 0.30),
                    static_cast<std::size_t>(positive_int(node, "quality.failures_until_lost", 4)),
                    parameter(node, "quality.max_odometry_drift_m", 0.0),
                    parameter(node, "quality.max_odometry_drift_rad", 0.0)};
    core.odometry_capacity = positive_int(node, "sync.odometry_capacity", 2048);
    core.max_odometry_gap_ns = std::uint64_t{1000000} * positive_int(node, "sync.max_odometry_gap_ms", 50);
    core.keyframes_per_downsample = positive_int(node, "map.downsample_every", 4);
    core.enable_registration = mode == Mode::mapping || mode == Mode::localization;

    if (mode == Mode::localization) {
        // Use a typed empty default when no pose is supplied, then require explicit initialization.
        const auto pose = parameter<std::vector<double>>(node, "initial_pose_xyz_xyzw", std::vector<double>{});
        if (pose.size() != 7) {
            throw std::invalid_argument("Localization requires initial_pose_xyz_xyzw = [x,y,z,qx,qy,qz,qw]");
        }
        const Eigen::Vector3d position(pose[0], pose[1], pose[2]);
        const Eigen::Quaterniond orientation(pose[6], pose[3], pose[4], pose[5]);
        if (!position.allFinite() || !orientation.coeffs().allFinite() ||
            !std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
            throw std::invalid_argument("Invalid localization initial pose");
        }
        initial_pose.translation() = position;
        initial_pose.linear() = orientation.normalized().toRotationMatrix();
    }
}

}  // namespace lidar_ros
