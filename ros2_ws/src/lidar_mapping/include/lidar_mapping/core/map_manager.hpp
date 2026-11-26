#pragma once

#include "lidar_mapping/core/point_cloud.hpp"
#include "lidar_mapping/core/se3.hpp"

#include <filesystem>
#include <optional>

namespace lidar_core {

enum class MapMode { mapping, localization };

struct MapConfig {
    float voxel_size_m = 0.05f;
    double local_radius_m = 10.0;
    double keyframe_translation_m = 0.2;
    double keyframe_rotation_deg = 10.0;
    std::filesystem::path directory = "maps";
};

class MapManager {
public:
    explicit MapManager(MapConfig config = {}, MapMode mode = MapMode::mapping);
    MapManager(const MapManager&) = delete;
    MapManager& operator=(const MapManager&) = delete;

    const PointCloud& cloud() const;
    bool should_add_keyframe(const Pose& map_from_robot) const;

    // Inserts a nonempty scan with finite XYZ. The first insertion initializes the map.
    // A successful insertion records the pose of the latest accepted keyframe.
    bool add_cloud(const PointCloud& local_cloud, const Pose& map_from_robot);
    bool downsample();

    // Returns points in map coordinates within local_radius_m of center_in_map.
    PointCloud local_map(const Eigen::Vector3d& center_in_map) const;

    // Both operations take a plain .pcd filename within the configured directory.
    // Localization permits one successful load. Failed loads preserve the current map.
    bool save_pcd(const std::filesystem::path& filename) const;
    bool load_pcd(const std::filesystem::path& filename);

private:
    MapConfig config_;
    MapMode mode_;
    PointCloud::Ptr map_;
    std::optional<Pose> last_keyframe_;
};

}  // namespace lidar_core
