#include "node_config.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lidar_ros {
namespace {

using lidar_core::Pose;
using lidar_core::ScanResult;
using lidar_core::ScanStatus;
using lidar_core::TimestampNs;
using Cloud = sensor_msgs::msg::PointCloud2;
using Odometry = nav_msgs::msg::Odometry;
using Trigger = std_srvs::srv::Trigger;
using Stamp = builtin_interfaces::msg::Time;
using WallClock = std::chrono::steady_clock;
using Diagnostic = diagnostic_msgs::msg::DiagnosticStatus;
using Transform = geometry_msgs::msg::TransformStamped;

TimestampNs stamp_ns(const Stamp& stamp) {
    return static_cast<TimestampNs>(stamp.sec) * 1000000000 + stamp.nanosec;
}

bool valid_stamp(const Stamp& stamp) {
    return stamp.sec >= 0 && stamp.nanosec < 1000000000;
}

bool valid_xyz_layout(const Cloud& cloud) {
    if (cloud.is_bigendian || cloud.point_step < sizeof(float) ||
        cloud.row_step < std::uint64_t{cloud.width} * cloud.point_step ||
        cloud.data.size() < std::uint64_t{cloud.height} * cloud.row_step) {
        return false;
    }
    for (const char* axis : {"x", "y", "z"}) {
        const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(), [axis](const auto& value) {
            return value.name == axis;
        });
        if (field == cloud.fields.end() || field->count != 1 ||
            field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
            field->offset > cloud.point_step - sizeof(float)) {
            return false;
        }
    }
    return true;
}

std::optional<Pose> odometry_pose(const Odometry& msg) {
    const auto& position = msg.pose.pose.position;
    const auto& q = msg.pose.pose.orientation;
    const Eigen::Vector3d translation(position.x, position.y, position.z);
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    if (!translation.allFinite() || !orientation.coeffs().allFinite() ||
        !std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
        return std::nullopt;
    }
    Pose result = Pose::Identity();
    result.translation() = translation;
    result.linear() = orientation.normalized().toRotationMatrix();
    return result;
}

geometry_msgs::msg::Quaternion quaternion_msg(const Pose& pose) {
    const Eigen::Quaterniond q = Eigen::Quaterniond(pose.linear()).normalized();
    geometry_msgs::msg::Quaternion result;
    result.x = q.x();
    result.y = q.y();
    result.z = q.z();
    result.w = q.w();
    return result;
}

Transform transform_msg(const std::string& parent, const std::string& child, const Stamp& stamp, const Pose& pose) {
    Transform result;
    result.header.frame_id = parent;
    result.header.stamp = stamp;
    result.child_frame_id = child;
    result.transform.translation.x = pose.translation().x();
    result.transform.translation.y = pose.translation().y();
    result.transform.translation.z = pose.translation().z();
    result.transform.rotation = quaternion_msg(pose);
    return result;
}

const char* status_name(ScanStatus status) {
    switch (status) {
        case ScanStatus::awaiting_initial_pose: return "awaiting_initial_pose";
        case ScanStatus::awaiting_map: return "awaiting_map";
        case ScanStatus::missing_odometry: return "missing_odometry";
        case ScanStatus::invalid_odometry: return "invalid_odometry";
        case ScanStatus::stale_scan: return "stale_scan";
        case ScanStatus::invalid_prediction: return "invalid_prediction";
        case ScanStatus::insufficient_source: return "insufficient_source";
        case ScanStatus::insufficient_target: return "insufficient_target";
        case ScanStatus::map_initialization_failed: return "map_initialization_failed";
        case ScanStatus::initialized: return "initialized";
        case ScanStatus::odometry_only: return "odometry_only";
        case ScanStatus::registration_failed: return "registration_failed";
        case ScanStatus::invalid_candidate: return "invalid_candidate";
        case ScanStatus::fitness_rejected: return "fitness_rejected";
        case ScanStatus::overlap_rejected: return "overlap_rejected";
        case ScanStatus::correction_rejected: return "correction_rejected";
        case ScanStatus::accepted: return "accepted";
        case ScanStatus::odometry_anchor_rejected: return "odometry_anchor_rejected";
    }
    return "unknown";
}

double milliseconds(WallClock::duration duration) {
    return std::chrono::duration<double, std::milli>(duration).count();
}

}  // namespace

class MappingNode final : public rclcpp::Node {
public:
    MappingNode()
        : Node("lidar_mapping"), config_(*this),
          odometry_(config_.core.odometry_capacity, config_.core.max_odometry_gap_ns) {
        if (deskewed()) {
            // The cloud origin is the recording's odom origin. Reject the zero padding
            // without applying the robot-relative radial limits to these global points.
            deskewed_filter_.emplace(lidar_core::CloudFilterConfig{
                1e-6, std::numeric_limits<double>::max(), config_.core.cloud.voxel_size_m});
        }
        initialize_pipeline();
        open_metrics();
        aligned_pub_ = create_publisher<Cloud>("~/aligned_cloud", rclcpp::SensorDataQoS());
        map_pub_ = create_publisher<Cloud>("~/map", rclcpp::QoS(1).transient_local());
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1).transient_local());
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
        if (config_.publish_map_tf || config_.publish_odom_tf) {
            tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        cloud_sub_ = create_subscription<Cloud>(config_.cloud_topic, rclcpp::QoS(config_.cloud_depth).reliable(),
            [this](Cloud::ConstSharedPtr msg) { on_cloud(std::move(msg)); });
        odom_sub_ = create_subscription<Odometry>(config_.odom_topic, rclcpp::QoS(config_.odom_depth).reliable(),
            [this](Odometry::ConstSharedPtr msg) { on_odometry(*msg); });
        save_service_ = create_service<Trigger>("~/save_map",
            [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr reply) {
                if (reset_requested_.load()) {
                    reply->message = "Clock reset pending";
                    return;
                }
                reply->success = save_map(reply->message);
                flush_metrics();
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { on_timer(); });
        rcl_jump_threshold_t threshold{};
        threshold.min_backward.nanoseconds = -1;
        jump_ = get_clock()->create_jump_callback(nullptr,
            [this](const rcl_time_jump_t&) { reset_requested_.store(true); }, threshold);
        RCLCPP_INFO(get_logger(), "mode=%s cloud=%s odom=%s metrics=%s", config_.mode_name.c_str(),
                    config_.cloud_topic.c_str(), config_.odom_topic.c_str(), config_.metrics_file.c_str());
    }

    void finish() {
        for (const auto& pending : pending_) {
            ++dropped_;
            write_metrics(stamp_ns(pending.cloud->header.stamp), "shutdown_pending", elapsed(pending), nullptr);
        }
        flush_metrics();
        if (config_.save_on_shutdown && config_.mode != Mode::localization && !map().empty()) {
            std::string message;
            if (save_map(message)) {
                RCLCPP_INFO(get_logger(), "%s", message.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "%s", message.c_str());
            }
        }
        RCLCPP_INFO(get_logger(), "cloud_received=%zu odom_received=%zu processed=%zu accepted=%zu dropped=%zu map_points=%zu",
                    received_clouds_, received_odometry_, processed_, accepted_, dropped_, map().size());
    }

private:
    struct Pending {
        Cloud::ConstSharedPtr cloud;
        WallClock::time_point enqueued;
    };

    bool deskewed() const { return config_.mode == Mode::deskewed; }

    const lidar_core::PointCloud& map() const {
        return deskewed() ? deskewed_map_->cloud() : pipeline_->map();
    }

    void initialize_pipeline() {
        aligned_session_ = false;
        deskewed_steps_ = 0;
        if (deskewed()) {
            deskewed_map_ = std::make_unique<lidar_core::MapManager>(config_.core.map);
        } else if (config_.mode == Mode::localization) {
            if (!pipeline_) {
                pipeline_ = std::make_unique<lidar_core::ScanToMap>(config_.core, lidar_core::MapMode::localization);
                if (!pipeline_->load_map(config_.map_input)) {
                    throw std::runtime_error("Cannot load localization map: " + (config_.core.map.directory / config_.map_input).string());
                }
            }
            if (!pipeline_->start_session(config_.initial_pose)) {
                throw std::runtime_error("Cannot initialize localization pose");
            }
        } else {
            pipeline_ = std::make_unique<lidar_core::ScanToMap>(config_.core, lidar_core::MapMode::mapping);
        }
    }

    void reset_if_needed() {
        if (!reset_requested_.exchange(false)) { return; }
        for (const auto& pending : pending_) {
            drop(pending, "clock_reset_pending");
        }
        pending_.clear();
        odometry_.clear();
        last_scan_.reset();
        path_ = nav_msgs::msg::Path{};
        initialize_pipeline();
        map_stamp_ = Stamp{};
        last_map_publication_ = WallClock::time_point{};
        ++session_;
        last_status_ = "clock_reset";
        path_.header.frame_id = config_.map_frame;
        path_pub_->publish(path_);
        // Replace the transient-local map from the preceding playback session.
        map_dirty_ = true;
        publish_map_if_due();
        RCLCPP_WARN(get_logger(), "ROS time moved backwards, started session %zu", session_);
    }

    void open_metrics() {
        if (config_.metrics_file.empty()) { return; }
        if (config_.metrics_file.has_parent_path()) {
            std::filesystem::create_directories(config_.metrics_file.parent_path());
        }
        metrics_.open(config_.metrics_file, std::ios::out | std::ios::trunc);
        if (!metrics_) { throw std::runtime_error("Cannot open metrics file: " + config_.metrics_file.string()); }
        metrics_ << std::setprecision(12)
                 << "session,stamp_ns,status,wait_ms,input_points,source_points,target_points,map_points,"
                    "map_updated,fitness_m2,overlap,sync_ms,preprocess_ms,predict_ms,local_map_ms,gicp_ms,"
                    "validate_ms,map_update_ms,alignment_ms,total_ms,x,y,z,qx,qy,qz,qw\n";
    }

    void flush_metrics() {
        if (metrics_.is_open()) {
            metrics_.flush();
            if (!metrics_) {
                RCLCPP_ERROR(get_logger(), "Cannot write metrics file %s", config_.metrics_file.c_str());
                metrics_.close();
            }
        }
    }

    void write_metrics(TimestampNs stamp, const char* status, double wait, const ScanResult* result) {
        if (!metrics_) { return; }
        metrics_ << session_ << ',' << stamp << ',' << status << ',' << wait << ',';
        if (result && result->filter) {
            metrics_ << result->filter->input_points << ',' << result->filter->output_points;
        } else {
            metrics_ << ',';
        }
        metrics_ << ',';
        if (result) { metrics_ << result->target_points; }
        metrics_ << ',' << map().size() << ',' << (result && result->map_updated) << ',';
        if (result && result->registration && result->registration->fitness_m2) {
            metrics_ << *result->registration->fitness_m2;
        }
        metrics_ << ',';
        if (result && result->registration && result->registration->correspondence_fraction) {
            metrics_ << *result->registration->correspondence_fraction;
        }
        if (result) {
            const auto& time = result->times;
            metrics_ << ',' << time.synchronization_ms << ',' << time.preprocessing_ms << ',' << time.prediction_ms
                     << ',' << time.local_map_ms << ',' << time.registration_ms << ',' << time.validation_ms
                     << ',' << time.map_update_ms << ',' << time.alignment_ms << ',' << time.total_ms;
        } else {
            metrics_ << ",,,,,,,,,";
        }
        if (result && result->final_pose) {
            const auto& pose = *result->final_pose;
            const auto q = quaternion_msg(pose);
            metrics_ << ',' << pose.translation().x() << ',' << pose.translation().y() << ',' << pose.translation().z()
                     << ',' << q.x << ',' << q.y << ',' << q.z << ',' << q.w;
        } else {
            metrics_ << ",,,,,,,";
        }
        metrics_ << '\n';
    }

    static double elapsed(const Pending& pending) { return milliseconds(WallClock::now() - pending.enqueued); }

    void drop(const Pending& pending, const char* reason) {
        ++dropped_;
        last_status_ = reason;
        write_metrics(stamp_ns(pending.cloud->header.stamp), reason, elapsed(pending), nullptr);
    }

    void on_odometry(const Odometry& msg) {
        reset_if_needed();
        if (!valid_stamp(msg.header.stamp) || msg.header.frame_id != config_.odom_frame ||
            msg.child_frame_id != config_.base_frame) {
            ++invalid_odometry_;
            return;
        }
        const auto pose = odometry_pose(msg);
        if (!pose) {
            ++invalid_odometry_;
            return;
        }
        ++received_odometry_;
        odometry_.insert(stamp_ns(msg.header.stamp), *pose);
    }

    void on_cloud(Cloud::ConstSharedPtr msg) {
        reset_if_needed();
        ++received_clouds_;
        const auto stamp = stamp_ns(msg->header.stamp);
        Pending pending{std::move(msg), WallClock::now()};
        const auto& cloud = *pending.cloud;
        if (!valid_stamp(cloud.header.stamp) || cloud.header.frame_id != (deskewed() ? config_.odom_frame : config_.base_frame) ||
            !valid_xyz_layout(cloud)) {
            drop(pending, "invalid_cloud");
            return;
        }
        if (last_scan_ && stamp <= *last_scan_) {
            drop(pending, "stale_scan");
            return;
        }
        const auto position = std::lower_bound(pending_.begin(), pending_.end(), stamp,
            [](const Pending& item, TimestampNs stamp) { return stamp_ns(item.cloud->header.stamp) < stamp; });
        if (position != pending_.end() && stamp_ns(position->cloud->header.stamp) == stamp) {
            drop(pending, "duplicate_scan");
            return;
        }
        pending_.insert(position, std::move(pending));
        drain_pending();
        while (pending_.size() > config_.pending_capacity) {
            last_scan_ = stamp_ns(pending_.front().cloud->header.stamp);
            drop(pending_.front(), "queue_overflow");
            pending_.pop_front();
        }
    }

    void drain_pending() {
        while (!pending_.empty() && rclcpp::ok() && !reset_requested_.load()) {
            const auto& front = pending_.front();
            const auto stamp = stamp_ns(front.cloud->header.stamp);
            const auto odom = odometry_.pose_at(stamp);
            if (!odom && WallClock::now() - front.enqueued < config_.wait) { return; }
            auto pending = std::move(pending_.front());
            pending_.pop_front();
            last_scan_ = stamp;
            if (odom || deskewed()) {
                process(pending, odom);
            } else {
                ++missing_odometry_;
                drop(pending, "odometry_timeout");
            }
        }
    }

    ScanResult accumulate_deskewed(const lidar_core::PointCloud& input, const std::optional<Pose>& odom) {
        const auto start = WallClock::now();
        auto filtered = deskewed_filter_->process(input);
        const auto filtered_at = WallClock::now();
        ScanResult result;
        result.filter = filtered.diagnostics;
        result.final_pose = odom;
        result.map_updated = deskewed_map_->add_cloud(filtered.cloud, Pose::Identity());
        if (result.map_updated && ++deskewed_steps_ % config_.core.keyframes_per_downsample == 0) {
            result.map_downsampled = deskewed_map_->downsample();
        }
        result.status = result.map_updated ? ScanStatus::odometry_only : ScanStatus::insufficient_source;
        result.aligned_cloud = std::move(filtered.cloud);
        result.times.preprocessing_ms = milliseconds(filtered_at - start);
        result.times.map_update_ms = milliseconds(WallClock::now() - filtered_at);
        result.times.total_ms = milliseconds(WallClock::now() - start);
        return result;
    }

    void process(const Pending& pending, const std::optional<Pose>& odom) {
        const auto start = WallClock::now();
        const auto& stamp = pending.cloud->header.stamp;
        const double wait = elapsed(pending);
        lidar_core::PointCloud scan;
        pcl::fromROSMsg(*pending.cloud, scan);
        if (!deskewed() && config_.mode != Mode::localization && !aligned_session_) {
            if (!pipeline_->start_session(*odom)) {
                drop(pending, "invalid_initial_odometry");
                return;
            }
        }
        auto result = deskewed() ? accumulate_deskewed(scan, odom) : pipeline_->process(scan, stamp_ns(stamp), *odom);
        if (result.final_pose) { aligned_session_ = true; }
        ++processed_;
        accepted_ += result.status == ScanStatus::accepted;
        map_updates_ += result.map_updated;
        if (deskewed() && !odom) { ++missing_odometry_; }
        last_tracking_ = result.tracking;
        const char* status = status_name(result.status);
        if (deskewed() && result.map_updated) { status = odom ? "deskewed_accumulated" : "deskewed_no_odom"; }
        last_status_ = status;
        result.times.total_ms = milliseconds(WallClock::now() - start);
        write_metrics(stamp_ns(stamp), status, wait, &result);
        if (!result.aligned_cloud.empty()) {
            Cloud aligned;
            pcl::toROSMsg(result.aligned_cloud, aligned);
            aligned.header.frame_id = config_.map_frame;
            aligned.header.stamp = stamp;
            aligned_pub_->publish(std::move(aligned));
        }
        if (result.final_pose && odom) { publish_pose(*result.final_pose, *odom, stamp); }
        if (result.map_updated || (config_.mode == Mode::localization && !published_localization_map_)) {
            published_localization_map_ = config_.mode == Mode::localization;
            map_dirty_ = true;
            map_stamp_ = stamp;
        }
    }

    void publish_pose(const Pose& estimate, const Pose& odom, const Stamp& stamp) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = config_.map_frame;
        pose.header.stamp = stamp;
        pose.pose.position.x = estimate.translation().x();
        pose.pose.position.y = estimate.translation().y();
        pose.pose.position.z = estimate.translation().z();
        pose.pose.orientation = quaternion_msg(estimate);
        pose_pub_->publish(pose);
        path_.header = pose.header;
        if (path_.poses.size() >= config_.path_capacity) { path_.poses.erase(path_.poses.begin()); }
        path_.poses.push_back(pose);
        path_pub_->publish(path_);

        if (tf_) {
            std::vector<Transform> transforms;
            if (config_.publish_map_tf) {
                transforms.push_back(transform_msg(config_.map_frame, config_.odom_frame, stamp, estimate * odom.inverse()));
            }
            if (config_.publish_odom_tf) {
                transforms.push_back(transform_msg(config_.odom_frame, config_.base_frame, stamp, odom));
            }
            tf_->sendTransform(transforms);
        }
    }

    void publish_map_if_due() {
        const auto now = WallClock::now();
        if (!map_dirty_ || (last_map_publication_ != WallClock::time_point{} &&
                           now - last_map_publication_ < config_.map_period)) { return; }
        Cloud message;
        pcl::toROSMsg(map(), message);
        message.header.frame_id = config_.map_frame;
        message.header.stamp = map_stamp_;
        map_pub_->publish(std::move(message));
        last_map_publication_ = now;
        map_dirty_ = false;
    }

    void on_timer() {
        reset_if_needed();
        drain_pending();
        publish_map_if_due();
        const auto now = WallClock::now();
        if (now - last_heartbeat_ < std::chrono::seconds(1)) { return; }
        last_heartbeat_ = now;
        flush_metrics();
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = get_clock()->now();
        Diagnostic status;
        status.name = get_fully_qualified_name();
        status.hardware_id = "unitree_go2";
        status.level = processed_ == 0 || last_tracking_ == lidar_core::TrackingStatus::degraded ||
                       last_tracking_ == lidar_core::TrackingStatus::lost || !pending_.empty()
                       ? Diagnostic::WARN : Diagnostic::OK;
        status.message = pending_.empty() ? last_status_ : "waiting_for_odometry";
        const auto count = [&status](const char* name, std::size_t value) {
            diagnostic_msgs::msg::KeyValue item;
            item.key = name;
            item.value = std::to_string(value);
            status.values.push_back(std::move(item));
        };
        count("session", session_);
        count("cloud_received", received_clouds_);
        count("odom_received", received_odometry_);
        count("odom_invalid", invalid_odometry_);
        count("processed", processed_);
        count("accepted", accepted_);
        count("dropped", dropped_);
        count("odometry_missing", missing_odometry_);
        count("pending", pending_.size());
        count("map_updates", map_updates_);
        count("map_points", map().size());
        array.status.push_back(std::move(status));
        diagnostics_pub_->publish(std::move(array));
    }

    bool save_map(std::string& message) const {
        if (config_.mode == Mode::localization || map().empty()) {
            message = "No writable map in this mode or no map points yet";
            return false;
        }
        const std::string temporary = "." + config_.map_output + "." +
            std::to_string(WallClock::now().time_since_epoch().count()) + ".pcd";
        const bool saved = deskewed() ? deskewed_map_->save_pcd(temporary) : pipeline_->save_map(temporary);
        const auto directory = config_.core.map.directory;
        const auto destination = directory / config_.map_output;
        std::error_code error;
        if (saved) { std::filesystem::rename(directory / temporary, destination, error); }
        if (!saved || error) {
            std::error_code ignored;
            std::filesystem::remove(directory / temporary, ignored);
            message = "Failed to save " + destination.string() + (error ? ": " + error.message() : "");
            return false;
        }
        message = "Saved " + destination.string();
        return true;
    }

    const NodeConfig config_;
    lidar_core::OdometryBuffer odometry_;
    std::unique_ptr<lidar_core::ScanToMap> pipeline_;
    std::unique_ptr<lidar_core::MapManager> deskewed_map_;
    std::optional<lidar_core::CloudPreprocessor> deskewed_filter_;
    std::deque<Pending> pending_;
    std::optional<TimestampNs> last_scan_;
    bool aligned_session_ = false;
    bool published_localization_map_ = false;
    bool map_dirty_ = false;
    Stamp map_stamp_;
    WallClock::time_point last_map_publication_;
    WallClock::time_point last_heartbeat_;
    std::size_t deskewed_steps_ = 0;
    std::size_t session_ = 0;
    std::size_t received_clouds_ = 0;
    std::size_t received_odometry_ = 0;
    std::size_t invalid_odometry_ = 0;
    std::size_t processed_ = 0;
    std::size_t accepted_ = 0;
    std::size_t dropped_ = 0;
    std::size_t missing_odometry_ = 0;
    std::size_t map_updates_ = 0;
    std::string last_status_ = "waiting_for_input";
    lidar_core::TrackingStatus last_tracking_ = lidar_core::TrackingStatus::waiting;
    nav_msgs::msg::Path path_;
    std::ofstream metrics_;
    rclcpp::Publisher<Cloud>::SharedPtr aligned_pub_;
    rclcpp::Publisher<Cloud>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<Trigger>::SharedPtr save_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> reset_requested_{false};
    rclcpp::JumpHandler::SharedPtr jump_;
};

}  // namespace lidar_ros

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        const auto node = std::make_shared<lidar_ros::MappingNode>();
        rclcpp::spin(node);
        node->finish();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("lidar_mapping"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
