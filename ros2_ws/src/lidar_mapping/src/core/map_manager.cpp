#include "lidar_mapping/core/map_manager.hpp"

#include <pcl/common/io.h>
#include <pcl/common/point_tests.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/exceptions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

namespace lidar_core {
namespace {

constexpr double radians_per_degree = 0.017453292519943295;

bool is_valid_cloud(const PointCloud& cloud) {
    return !cloud.empty() && std::all_of(cloud.begin(), cloud.end(), [](const Point& point) {
        return pcl::isFinite(point);
    });
}

bool is_pcd_filename(const std::filesystem::path& filename) {
    return filename == filename.filename() && filename.extension() == ".pcd";
}

bool has_xyz(const pcl::PCLPointCloud2& cloud) {
    for (const char* name : {"x", "y", "z"}) {
        const auto matches = [name](const pcl::PCLPointField& field) {
            return field.name == name && field.datatype == pcl::PCLPointField::FLOAT32 &&
                   field.count == 1;
        };
        if (std::none_of(cloud.fields.begin(), cloud.fields.end(), matches)) {
            return false;
        }
    }
    return true;
}

}  // namespace

MapManager::MapManager(MapConfig config, MapMode mode)
    : config_(std::move(config)), mode_(mode), map_(new PointCloud) {
    if (!std::isfinite(config_.voxel_size_m) || config_.voxel_size_m <= 0 ||
        !std::isfinite(1.0f / config_.voxel_size_m) ||
        !std::isfinite(config_.local_radius_m) || config_.local_radius_m <= 0 ||
        config_.local_radius_m > std::sqrt(std::numeric_limits<float>::max()) ||
        !std::isfinite(config_.keyframe_translation_m) || config_.keyframe_translation_m <= 0 ||
        !std::isfinite(config_.keyframe_rotation_deg) || config_.keyframe_rotation_deg <= 0 ||
        config_.keyframe_rotation_deg > 180 || config_.directory.empty()) {
        throw std::invalid_argument("Invalid map configuration");
    }
}

const PointCloud& MapManager::cloud() const {
    return *map_;
}

bool MapManager::should_add_keyframe(const Pose& map_from_robot) const {
    if (mode_ != MapMode::mapping || !is_rigid_transform(map_from_robot.matrix())) {
        return false;
    }
    if (!last_keyframe_) {
        return true;
    }

    const Pose delta = relative_motion(*last_keyframe_, map_from_robot);
    const Eigen::Quaterniond rotation(delta.linear());
    const double angle = Eigen::AngleAxisd(rotation.normalized()).angle();
    return delta.translation().norm() >= config_.keyframe_translation_m ||
           angle >= config_.keyframe_rotation_deg * radians_per_degree;
}

bool MapManager::add_cloud(const PointCloud& local_cloud, const Pose& map_from_robot) {
    if (mode_ != MapMode::mapping || !is_rigid_transform(map_from_robot.matrix()) ||
        !is_valid_cloud(local_cloud)) {
        return false;
    }

    PointCloud transformed;
    pcl::transformPointCloud(local_cloud, transformed, map_from_robot.matrix());
    constexpr std::size_t max_points = std::numeric_limits<std::uint32_t>::max();
    if (!is_valid_cloud(transformed) || transformed.size() > max_points ||
        map_->size() > max_points - transformed.size()) {
        return false;
    }

    map_->points.insert(map_->points.end(), transformed.begin(), transformed.end());
    map_->width = static_cast<std::uint32_t>(map_->size());
    map_->height = 1;
    last_keyframe_ = map_from_robot;
    return true;
}

bool MapManager::downsample() {
    if (mode_ != MapMode::mapping || map_->empty()) {
        return false;
    }
    pcl::VoxelGrid<Point> filter;
    filter.setInputCloud(map_);
    const float size = config_.voxel_size_m;
    filter.setLeafSize(size, size, size);
    PointCloud filtered;
    filter.filter(filtered);
    if (!is_valid_cloud(filtered)) {
        return false;
    }
    *map_ = std::move(filtered);
    return true;
}

PointCloud MapManager::local_map(const Eigen::Vector3d& center_in_map) const {
    PointCloud local;
    if (map_->empty() || !center_in_map.allFinite() ||
        center_in_map.cwiseAbs().maxCoeff() > std::numeric_limits<float>::max()) {
        return local;
    }

    const Eigen::Vector3f center = center_in_map.cast<float>();
    const Point query(center.x(), center.y(), center.z());
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(map_);
    pcl::Indices indices;
    std::vector<float> squared_distances;
    if (tree.radiusSearch(query, config_.local_radius_m, indices, squared_distances) > 0) {
        pcl::copyPointCloud(*map_, indices, local);
    }
    return local;
}

bool MapManager::save_pcd(const std::filesystem::path& filename) const {
    if (map_->empty() || !is_pcd_filename(filename)) {
        return false;
    }
    std::error_code error;
    std::filesystem::create_directories(config_.directory, error);
    if (error) {
        return false;
    }

    try {
        return pcl::io::savePCDFileBinary((config_.directory / filename).string(), *map_) == 0;
    } catch (const pcl::PCLException&) {
        return false;
    }
}

bool MapManager::load_pcd(const std::filesystem::path& filename) {
    if (!is_pcd_filename(filename) || (mode_ == MapMode::localization && !map_->empty())) {
        return false;
    }
    const auto path = config_.directory / filename;
    std::error_code error;
    if (!std::filesystem::is_regular_file(path, error)) {
        return false;
    }

    try {
        pcl::PCLPointCloud2 blob;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        if (pcl::io::loadPCDFile(path.string(), blob, origin, orientation) != 0 || !has_xyz(blob)) {
            return false;
        }
        PointCloud loaded;
        pcl::fromPCLPointCloud2(blob, loaded);
        if (!is_valid_cloud(loaded)) {
            return false;
        }
        loaded.is_dense = true;
        *map_ = std::move(loaded);
        last_keyframe_.reset();
        return true;
    } catch (const pcl::PCLException&) {
        return false;
    }
}

}  // namespace lidar_core
