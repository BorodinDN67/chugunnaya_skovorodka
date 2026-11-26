#include "lidar_mapping/core/odometry_buffer.hpp"

#include <iterator>
#include <stdexcept>

namespace lidar_core {
namespace {

std::uint64_t elapsed_ns(TimestampNs earlier, TimestampNs later) {
    // Unsigned arithmetic covers the full distance between two signed timestamps.
    return static_cast<std::uint64_t>(later) - static_cast<std::uint64_t>(earlier);
}

}  // namespace

OdometryBuffer::OdometryBuffer(std::size_t capacity, std::uint64_t max_interpolation_gap_ns)
    : capacity_(capacity), max_interpolation_gap_ns_(max_interpolation_gap_ns) {
    if (capacity_ == 0) {
        throw std::invalid_argument("Odometry buffer capacity must be positive");
    }
}

void OdometryBuffer::insert(TimestampNs stamp, const Pose& pose) {
    samples_.insert_or_assign(stamp, pose);
    if (samples_.size() > capacity_) {
        samples_.erase(samples_.begin());
    }
}

std::optional<Pose> OdometryBuffer::pose_at(TimestampNs stamp) const {
    const auto upper = samples_.lower_bound(stamp);
    if (upper != samples_.end() && upper->first == stamp) {
        return upper->second;
    }
    if (upper == samples_.begin() || upper == samples_.end()) {
        return std::nullopt;
    }

    const auto lower = std::prev(upper);
    const auto gap = elapsed_ns(lower->first, upper->first);
    if (gap > max_interpolation_gap_ns_) {
        return std::nullopt;
    }
    const double fraction = static_cast<double>(elapsed_ns(lower->first, stamp)) /
                            static_cast<double>(gap);
    return interpolate_pose(lower->second, upper->second, fraction);
}

void OdometryBuffer::clear() {
    samples_.clear();
}

}  // namespace lidar_core
