#pragma once

#include "lidar_mapping/core/scan_to_map.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <string>

namespace lidar_ros {

enum class Mode { deskewed, odometry, mapping, localization };

struct NodeConfig {
    explicit NodeConfig(rclcpp::Node& node);

    std::string mode_name;
    Mode mode;
    std::string cloud_topic;
    std::string odom_topic;
    std::string map_frame;
    std::string odom_frame;
    std::string base_frame;
    std::size_t cloud_depth;
    std::size_t odom_depth;
    std::size_t pending_capacity;
    std::chrono::milliseconds wait;
    std::chrono::milliseconds map_period;
    std::size_t path_capacity;
    bool publish_map_tf;
    bool publish_odom_tf;
    bool save_on_shutdown;
    lidar_core::ScanToMapConfig core;
    std::string map_input;
    std::string map_output;
    std::filesystem::path metrics_file;
    lidar_core::Pose initial_pose = lidar_core::Pose::Identity();
};

}  // namespace lidar_ros
