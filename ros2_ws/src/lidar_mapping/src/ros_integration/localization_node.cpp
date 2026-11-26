#include "lidar_mapping/core/scan_to_map.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

using namespace std::chrono_literals;
using namespace lidar_core;
using CloudMsg = sensor_msgs::msg::PointCloud2;
using OdomMsg = nav_msgs::msg::Odometry;
using Clock = std::chrono::steady_clock;

const char* scan_status(ScanStatus status) {
    switch (status) {
        case ScanStatus::accepted: return "accepted";
        case ScanStatus::missing_odometry: return "missing_odometry";
        case ScanStatus::stale_scan: return "stale_scan";
        case ScanStatus::insufficient_source: return "insufficient_source";
        case ScanStatus::insufficient_target: return "insufficient_target";
        case ScanStatus::registration_failed: return "registration_failed";
        case ScanStatus::invalid_candidate: return "invalid_candidate";
        case ScanStatus::fitness_rejected: return "fitness_rejected";
        case ScanStatus::overlap_rejected: return "overlap_rejected";
        case ScanStatus::correction_rejected: return "correction_rejected";
        default: return "invalid_state";
    }
}

std::string value(std::optional<double> measurement) {
    return measurement ? std::to_string(*measurement) : "";
}

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("localization"), odometry_(512, 100'000'000) {
        const auto map_parameter = declare_parameter<std::string>("map_path", "");
        if (map_parameter.empty()) {
            throw std::invalid_argument("Set map_path to the fixed mapping PCD");
        }
        const auto map_path = std::filesystem::absolute(map_parameter);
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        publish_map_to_odom_ = declare_parameter("publish_map_to_odom", false);
        publish_odom_to_base_ = declare_parameter("publish_odom_to_base", false);
        const auto initial = declare_parameter("initial_pose_xyz_rpy",
                                               std::vector<double>(6, 0.0));
        if (initial.size() != 6) {
            throw std::invalid_argument("initial_pose_xyz_rpy needs six values in meters/radians");
        }
        Pose seed = Pose::Identity();
        seed.translation() = Eigen::Vector3d(initial[0], initial[1], initial[2]);
        seed.linear() = (Eigen::AngleAxisd(initial[5], Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(initial[4], Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(initial[3], Eigen::Vector3d::UnitX())).toRotationMatrix();

        ScanToMapConfig config{};
        config.cloud = {declare_parameter("min_range_m", 0.3),
                        declare_parameter("max_range_m", 15.0),
                        static_cast<float>(declare_parameter("scan_voxel_m", 0.12))};
        config.gicp = {declare_parameter("max_correspondence_m", 0.5),
                       declare_parameter("overlap_distance_m", 0.3),
                       declare_parameter("max_iterations", 20), 12};
        config.map.directory = map_path.parent_path();
        config.map.local_radius_m = declare_parameter("local_map_radius_m", 10.0);
        config.quality = {30, declare_parameter("max_fitness_m2", 0.25),
                          declare_parameter("min_overlap", 0.4),
                          declare_parameter("max_correction_m", 0.5),
                          declare_parameter("max_correction_rad", 0.35), 10};
        config.odometry_capacity = 1;  // This adapter supplies interpolated poses explicitly.
        pipeline_ = std::make_unique<ScanToMap>(config, MapMode::localization);
        if (!pipeline_->start_session(seed) || !pipeline_->load_map(map_path.filename())) {
            throw std::runtime_error("Invalid initial pose or unreadable PCD: " + map_path.string());
        }

        const auto csv_path = declare_parameter<std::string>("csv_path", "");
        if (!csv_path.empty()) {
            csv_.open(csv_path, std::ios::out | std::ios::trunc);
            if (!csv_) {
                throw std::runtime_error("Cannot open CSV: " + csv_path);
            }
            csv_ << "stamp_ns,status,reason,fitness_m2,overlap,correction_m,correction_rad,"
                    "source_points,target_points,queue_ms,processing_ms,registration_ms,pipeline_ms,"
                    "x,y,z,qx,qy,qz,qw,pred_x,pred_y,pred_z\n";
            csv_ << std::setprecision(12);
        }

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", 1);
        map_pub_ = create_publisher<CloudMsg>("~/map", rclcpp::QoS(1).transient_local());
        aligned_pub_ = create_publisher<CloudMsg>("~/aligned_cloud", rclcpp::SensorDataQoS());
        diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/diagnostics", 10);
        if (publish_map_to_odom_ || publish_odom_to_base_) {
            tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        CloudMsg map;
        pcl::toROSMsg(pipeline_->map(), map);
        map.header.frame_id = map_frame_;
        // A zero stamp represents the static map and is valid before /clock starts.
        map_pub_->publish(map);

        cloud_sub_ = create_subscription<CloudMsg>("/utlidar/cloud_base",
            rclcpp::SensorDataQoS().keep_last(8), [this](CloudMsg::ConstSharedPtr msg) {
                if (msg->header.frame_id != base_frame_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Ignoring cloud frame '%s', expected '%s'",
                        msg->header.frame_id.c_str(), base_frame_.c_str());
                    return;
                }
                if (pending_.size() == 8) {
                    report_drop(pending_.front(), "queue_full");
                    pending_.pop_front();
                }
                pending_.push_back({std::move(msg), Clock::now()});
            });
        odom_sub_ = create_subscription<OdomMsg>("/utlidar/robot_odom",
            rclcpp::SensorDataQoS().keep_last(128), [this](OdomMsg::ConstSharedPtr msg) {
                if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Ignoring odometry frames '%s' -> '%s', expected '%s' -> '%s'",
                        msg->header.frame_id.c_str(), msg->child_frame_id.c_str(),
                        odom_frame_.c_str(), base_frame_.c_str());
                    return;
                }
                const auto& p = msg->pose.pose.position;
                const auto& q = msg->pose.pose.orientation;
                const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
                if (!rotation.coeffs().allFinite() || !std::isfinite(rotation.norm()) ||
                    rotation.norm() < 1e-9) {
                    return;
                }
                Pose odom = Pose::Identity();
                odom.translation() = Eigen::Vector3d(p.x, p.y, p.z);
                if (odom.translation().allFinite()) {
                    odom.linear() = rotation.normalized().toRotationMatrix();
                    odometry_.insert(rclcpp::Time(msg->header.stamp).nanoseconds(), odom);
                }
            });
        timer_ = create_wall_timer(2ms, [this] { process_pending(); });
        RCLCPP_INFO(get_logger(), "Loaded %zu map points from %s; map -> odom TF %s, odom -> base TF %s",
                    pipeline_->map().size(), map_path.string().c_str(),
                    publish_map_to_odom_ ? "enabled" : "disabled",
                    publish_odom_to_base_ ? "enabled" : "disabled");
    }

private:
    struct PendingScan {
        CloudMsg::ConstSharedPtr msg;
        Clock::time_point arrival;
    };

    double queue_ms(const PendingScan& scan) const {
        return std::chrono::duration<double, std::milli>(Clock::now() - scan.arrival).count();
    }

    void report_drop(const PendingScan& scan, const char* reason) {
        publish_diagnostics(scan.msg->header.stamp, "dropped", reason, {}, queue_ms(scan));
    }

    void process_pending() {
        if (pending_.empty()) {
            return;
        }
        auto& front = pending_.front();
        const auto process_start = Clock::now();
        const auto stamp = rclcpp::Time(front.msg->header.stamp).nanoseconds();
        const auto odom = odometry_.pose_at(stamp);
        if (!odom && Clock::now() - front.arrival < 250ms) {
            return;
        }
        auto scan = std::move(front);
        pending_.pop_front();
        if (!odom) {
            report_drop(scan, "missing_odometry");
            return;
        }

        const double wait_ms = queue_ms(scan);
        PointCloud cloud;
        pcl::fromROSMsg(*scan.msg, cloud);
        const auto result = pipeline_->process(cloud, stamp, *odom);
        const char* status = result.status == ScanStatus::accepted ? "accepted" :
                             result.final_pose ? "fallback" : "dropped";
        if (result.final_pose) {
            publish_estimate(scan.msg->header.stamp, *result.final_pose, *odom, result.aligned_cloud);
        }
        const auto pipeline_ms = std::chrono::duration<double, std::milli>(
            Clock::now() - process_start).count();
        publish_diagnostics(scan.msg->header.stamp, status, scan_status(result.status), result,
                            wait_ms, pipeline_ms);
    }

    void publish_estimate(const builtin_interfaces::msg::Time& stamp, const Pose& estimate,
                          const Pose& odom, const PointCloud& aligned) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = map_frame_;
        const Eigen::Quaterniond rotation(estimate.linear());
        pose.pose.position.x = estimate.translation().x();
        pose.pose.position.y = estimate.translation().y();
        pose.pose.position.z = estimate.translation().z();
        pose.pose.orientation.x = rotation.x();
        pose.pose.orientation.y = rotation.y();
        pose.pose.orientation.z = rotation.z();
        pose.pose.orientation.w = rotation.w();
        pose_pub_->publish(pose);
        path_.header = pose.header;
        path_.poses.push_back(pose);
        path_pub_->publish(path_);

        if (!aligned.empty()) {
            CloudMsg cloud;
            pcl::toROSMsg(aligned, cloud);
            cloud.header = pose.header;
            aligned_pub_->publish(cloud);
        }
        if (publish_map_to_odom_) {
            publish_transform(stamp, map_frame_, odom_frame_, estimate * odom.inverse());
        }
        if (publish_odom_to_base_) {
            publish_transform(stamp, odom_frame_, base_frame_, odom);
        }
    }

    void publish_transform(const builtin_interfaces::msg::Time& stamp, const std::string& parent,
                           const std::string& child, const Pose& pose) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = parent;
        transform.child_frame_id = child;
        transform.transform.translation.x = pose.translation().x();
        transform.transform.translation.y = pose.translation().y();
        transform.transform.translation.z = pose.translation().z();
        const Eigen::Quaterniond quaternion(pose.linear());
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        transform.transform.rotation.w = quaternion.w();
        tf_->sendTransform(transform);
    }

    void publish_diagnostics(const builtin_interfaces::msg::Time& stamp, const char* status,
                             const char* reason, const ScanResult& result, double wait_ms,
                             double pipeline_ms = 0) {
        using Level = diagnostic_msgs::msg::DiagnosticStatus;
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = stamp;
        message.header.frame_id = map_frame_;
        auto& diagnostic = message.status.emplace_back();
        diagnostic.name = "localization/gicp";
        diagnostic.hardware_id = "unitree_go2";
        diagnostic.level = result.status == ScanStatus::accepted ? Level::OK :
                           result.tracking == TrackingStatus::lost ? Level::ERROR : Level::WARN;
        diagnostic.message = status;
        const auto add = [&diagnostic](const std::string& key, const std::string& val) {
            auto& pair = diagnostic.values.emplace_back();
            pair.key = key;
            pair.value = val;
        };
        const auto fitness = result.registration ? result.registration->fitness_m2 : std::nullopt;
        const auto overlap = result.registration ? result.registration->correspondence_fraction
                                                 : std::nullopt;
        const auto source_points = result.filter ? result.filter->output_points : 0;
        add("reason", reason);
        add("fitness_m2", value(fitness));
        add("overlap", value(overlap));
        add("correction_m", value(result.correction_translation_m));
        add("correction_rad", value(result.correction_rotation_rad));
        add("source_points", std::to_string(source_points));
        add("target_points", std::to_string(result.target_points));
        add("queue_ms", std::to_string(wait_ms));
        add("processing_ms", std::to_string(result.times.total_ms));
        add("pipeline_ms", std::to_string(pipeline_ms));
        add("registration_ms", std::to_string(result.times.registration_ms));
        add("consecutive_failures", std::to_string(result.consecutive_failures));
        diagnostics_pub_->publish(message);

        if (csv_.is_open()) {
            csv_ << rclcpp::Time(stamp).nanoseconds() << ',' << status << ',' << reason << ','
                 << value(fitness) << ',' << value(overlap) << ','
                 << value(result.correction_translation_m) << ','
                 << value(result.correction_rotation_rad) << ',' << source_points << ','
                 << result.target_points << ',' << wait_ms << ',' << result.times.total_ms
                 << ',' << result.times.registration_ms << ',' << pipeline_ms;
            if (result.final_pose) {
                const auto& pose = *result.final_pose;
                const Eigen::Quaterniond q(pose.linear());
                csv_ << ',' << pose.translation().x() << ',' << pose.translation().y()
                     << ',' << pose.translation().z() << ',' << q.x() << ',' << q.y()
                     << ',' << q.z() << ',' << q.w();
            } else {
                csv_ << ",,,,,,,";
            }
            if (result.predicted_pose) {
                const auto translation = result.predicted_pose->translation();
                csv_ << ',' << translation.x() << ',' << translation.y() << ',' << translation.z();
            } else {
                csv_ << ",,,";
            }
            csv_ << '\n';
        }
    }

    std::string map_frame_, odom_frame_, base_frame_;
    bool publish_map_to_odom_ = false;
    bool publish_odom_to_base_ = false;
    OdometryBuffer odometry_;
    std::unique_ptr<ScanToMap> pipeline_;
    std::deque<PendingScan> pending_;
    std::ofstream csv_;
    nav_msgs::msg::Path path_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
    rclcpp::Subscription<CloudMsg>::SharedPtr cloud_sub_;
    rclcpp::Subscription<OdomMsg>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<CloudMsg>::SharedPtr map_pub_, aligned_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<LocalizationNode>());
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("localization"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
