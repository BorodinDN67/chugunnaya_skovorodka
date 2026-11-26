#pragma once

#include "lidar_mapping/core/se3.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>

namespace lidar_core {

using TimestampNs = std::int64_t;

// Samples and queries use the same clock and frame convention (T_odom_base).
// The caller serializes access and clears the buffer on a clock or frame reset.
class OdometryBuffer {
public:
    // Capacity must be positive. A zero gap allows only exact matches.
    OdometryBuffer(std::size_t capacity, std::uint64_t max_interpolation_gap_ns);

    // Insertion order is unrestricted. Duplicate timestamps replace their pose.
    // After insertion the buffer retains at most capacity samples, newest by timestamp.
    void insert(TimestampNs stamp, const Pose& pose);

    // Exact matches are returned directly. Interpolation requires two samples
    // whose separation does not exceed max_interpolation_gap_ns.
    std::optional<Pose> pose_at(TimestampNs stamp) const;
    void clear();

private:
    std::size_t capacity_;
    std::uint64_t max_interpolation_gap_ns_;
    std::map<TimestampNs, Pose> samples_;
};

}  // namespace lidar_core
