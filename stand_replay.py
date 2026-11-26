"""Run the pinned KISS-ICP ROS node in the organizers' Humble container."""

import argparse
import csv
import json
import math
import resource
import signal
import subprocess
import time
from array import array
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
import yaml
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from run_kiss import CSV_FIELDS, FRAME, HERE, stamp_ns, xyz

TOPIC = "/utlidar/cloud_base"
INPUT = "/kiss/input"
ODOM = "/kiss/odometry"
PROFILE = "/kiss/profile"
PATH = "/kiss/results/path"
MAP = "/kiss/results/map"
PROFILE_FIELDS = (
    "decode_ms", "register_frame_ms", "node_callback_ms", "publish_to_pose_ms",
    "global_append_ms", "publish_lag_ms", "output_lag_ms",
)
PARAMETERS = {
    "base_frame": "base_link", "lidar_odom_frame": FRAME,
    "publish_odom_tf": False, "invert_odom_tf": False, "publish_debug_clouds": False,
    "use_sim_time": False, "max_range": 100.0, "min_range": 0.0,
    "voxel_size": 1.0, "max_points_per_voxel": 20, "initial_threshold": 2.0,
    "min_motion_th": 0.1, "max_num_iterations": 500, "convergence_criterion": 0.0001,
    "max_num_threads": 0,
}


def message_stamp(value):
    return Time(sec=value // 10**9, nanosec=value % 10**9)


def output_cloud(points, header):
    fields = [PointField(name=axis, offset=4*i, datatype=PointField.FLOAT32, count=1)
              for i, axis in enumerate("xyz")]
    return PointCloud2(header=header, height=1, width=len(points), fields=fields,
                       point_step=12, row_step=12*len(points), is_dense=True,
                       data=array("B", points.astype("<f4", copy=False).tobytes()))


def display_cloud(points, header):
    cloud = output_cloud(points, header)
    z_min, z_max = np.percentile(points[:, 2], [5, 97])
    height = np.clip((points[:, 2] - z_min) / max(float(z_max - z_min), 0.01), 0.0, 1.0)
    stops = (0.0, 0.22, 0.66, 1.0)
    red = np.interp(height, stops, (43, 63, 110, 236)).astype("<u4")
    green = np.interp(height, stops, (66, 137, 214, 250)).astype("<u4")
    blue = np.interp(height, stops, (91, 158, 222, 255)).astype("<u4")
    records = np.empty((len(points), 4), dtype="<f4")
    records[:, :3] = points
    records.view("<u4")[:, 3] = (red << 16) | (green << 8) | blue
    cloud.fields = [*cloud.fields, PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)]
    cloud.point_step = 16
    cloud.row_step = cloud.width * cloud.point_step
    cloud.data = array("B", records.tobytes())
    return cloud


def write_pcd(filename, points):
    with filename.open("wb") as stream:
        stream.write(("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                      f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n"
                      f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n").encode())
        points.tofile(stream)


def summary(values):
    p50, p95, p99 = np.percentile(values, [50, 95, 99])
    return {"mean": float(np.mean(values)), "median": float(p50), "p95": float(p95),
            "p99": float(p99), "max": float(max(values))}


def find_bag(path):
    if (path / "metadata.yaml").is_file():
        return path
    nested = list(path.glob("*/metadata.yaml"))
    if len(nested) == 1:
        return nested[0].parent
    raise ValueError(f"Expected exactly one MCAP bag under {path}")


def validate_scan(cloud, deskew):
    if (cloud.header.frame_id != "base_link" or cloud.is_bigendian
            or not cloud.width or cloud.height != 1
            or cloud.row_step != cloud.width * cloud.point_step
            or len(cloud.data) != cloud.row_step):
        raise ValueError("Expected a nonempty, little-endian unorganized base_link scan")
    required = set("xyz") | ({"time"} if deskew else set())
    offsets = {f.name: f.offset for f in cloud.fields
               if f.count == 1 and f.datatype == PointField.FLOAT32
               and f.offset + 4 <= cloud.point_step}
    if not required <= offsets.keys():
        raise ValueError(f"Missing FLOAT32 fields: {required - offsets.keys()}")
    if deskew:
        stamps = np.ndarray((cloud.width,), "<f4", cloud.data, offsets["time"], (cloud.point_step,))
        if not np.isfinite(stamps).all() or np.min(stamps) == np.max(stamps):
            raise ValueError("Deskew then requires finite, varying per-point timestamps")


def writer_for(path):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    for name, datatype in ((ODOM, "nav_msgs/msg/Odometry"), (PATH, "nav_msgs/msg/Path"),
                           (MAP, "sensor_msgs/msg/PointCloud2"),
                           (PROFILE, "geometry_msgs/msg/Vector3Stamped")):
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=datatype,
                                                    serialization_format="cdr"))
    return writer


def replay(node, args):
    bag = find_bag(args.bag)
    subprocess.run(["ros2", "bag", "info", str(bag)], check=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    expected_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if expected_types.get(TOPIC) != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"Expected PointCloud2 on {TOPIC}")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    args.output.mkdir(parents=True, exist_ok=True)
    writer = writer_for(args.output / "rosbag")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    input_pub = node.create_publisher(PointCloud2, INPUT, 10)
    map_pub = node.create_publisher(PointCloud2, MAP, qos)
    path_pub = node.create_publisher(RosPath, PATH, qos)
    trajectory = RosPath(header=Header(frame_id=FRAME))
    pending, timing, rows, clouds, profiles = {}, {}, [], [], {}
    seen = set()
    last_path_time = None

    def receive_pose(pose):
        nonlocal last_path_time
        arrival = time.monotonic()
        key = stamp_ns(pose.header)
        match = pending.pop(key, None)
        if match is None:
            return
        if pose.header.frame_id != FRAME or pose.child_frame_id != "base_link":
            raise RuntimeError("Unexpected odometry frames")
        scheduled, sent, recorded, cloud = match
        p, q = pose.pose.pose.position, pose.pose.pose.orientation
        row = (key, p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not np.isfinite(row[1:]).all() or not math.isclose(np.linalg.norm(row[4:]), 1.0, abs_tol=1e-6):
            raise ValueError(f"Invalid KISS-ICP pose at {key}")
        rows.append(row)
        t0 = time.monotonic()
        points = xyz(cloud)
        axis = np.array([q.x, q.y, q.z])
        cross = 2 * np.cross(axis, points)
        clouds.append((points + q.w * cross + np.cross(axis, cross)
                       + [p.x, p.y, p.z]).astype("<f4"))
        t1 = time.monotonic()
        timing[key] = (recorded, arrival, (arrival - sent)*1000, (t1 - t0)*1000,
                       (sent - scheduled)*1000, (t1 - scheduled)*1000)
        pose_msg = PoseStamped(header=pose.header, pose=pose.pose.pose)
        trajectory.header = pose.header
        trajectory.poses.append(pose_msg)
        writer.write(ODOM, serialize_message(pose), recorded)
        if last_path_time is None or recorded - last_path_time >= 1_000_000_000:
            path_pub.publish(trajectory)
            writer.write(PATH, serialize_message(trajectory), recorded)
            last_path_time = recorded

    def receive_profile(profile):
        key = stamp_ns(profile.header)
        profiles[key] = (profile.vector.x, profile.vector.y, profile.vector.z)
        if key in timing:
            writer.write(PROFILE, serialize_message(profile), timing[key][0])
        elif key in pending:
            writer.write(PROFILE, serialize_message(profile), pending[key][2])

    node.create_subscription(Odometry, ODOM, receive_pose, 100)
    node.create_subscription(Vector3Stamped, PROFILE, receive_profile, 100)
    parameters = {**PARAMETERS, "deskew": args.deskew, "voxel_size": args.voxel_size,
                  "max_range": args.max_range, "max_num_threads": args.threads,
                  "initial_threshold": args.initial_threshold}
    config = {"input": str(bag), "topic": TOPIC, "input_frame": "base_link", "rate": args.rate,
              "seconds": args.seconds, "global_voxel_m": args.global_voxel,
              "kiss_icp_commit": "85488b7b6a6e1566cff0f8af4b83a9ea77dca7a0",
              "ros_parameters": parameters}
    (args.output / "configuration.json").write_text(json.dumps(config, indent=2) + "\n")
    command = [str(Path(get_package_prefix("kiss_icp")) / "lib/kiss_icp/kiss_icp_node"),
               "--ros-args", "-r", f"pointcloud_topic:={INPUT}"]
    for key, value in parameters.items():
        command.extend(("-p", f"{key}:={str(value).lower()}"))
    process = subprocess.Popen(command)
    node_rss = None
    try:
        deadline = time.monotonic() + 20
        while not (input_pub.get_subscription_count() and node.count_publishers(ODOM)
                   and node.count_publishers(PROFILE)):
            if process.poll() is not None or time.monotonic() >= deadline:
                raise RuntimeError("KISS-ICP odometry or instrumentation did not connect")
            rclpy.spin_once(node, timeout_sec=0.05)
        first = last = None
        started = time.monotonic()
        while reader.has_next():
            _, data, record = reader.read_next()
            if first is None:
                first = record
            offset = (record - first) / 1e9
            if args.seconds is not None and offset >= args.seconds:
                break
            scan = deserialize_message(data, PointCloud2)
            validate_scan(scan, args.deskew)
            scheduled = started + offset / args.rate
            while (remaining := scheduled - time.monotonic()) > 0:
                rclpy.spin_once(node, timeout_sec=min(remaining, 0.05))
            if process.poll() is not None:
                raise RuntimeError("KISS-ICP exited during playback")
            key = stamp_ns(scan.header)
            if key in seen:
                raise RuntimeError(f"Duplicate scan stamp {key}")
            seen.add(key)
            pending[key] = (scheduled, time.monotonic(), record, scan)
            input_pub.publish(scan)
            last = record
            rclpy.spin_once(node, timeout_sec=0)
        deadline = time.monotonic() + 3
        while (pending or len(profiles) < len(rows)) and time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError("KISS-ICP exited during drain")
            rclpy.spin_once(node, timeout_sec=0.05)
        playback_time = time.monotonic() - started
        for line in Path(f"/proc/{process.pid}/status").read_text().splitlines():
            if line.startswith("VmHWM:"):
                node_rss = int(line.split()[1])
                break
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    if not rows or not clouds or not len(profiles):
        raise RuntimeError("Empty trajectory, map or profile")
    reduction_start = time.monotonic()
    points = np.concatenate(clouds)
    _, indices = np.unique(np.floor(points / args.global_voxel).astype(np.int64), axis=0,
                           return_index=True)
    points = points[indices]
    reduction_ms = (time.monotonic() - reduction_start)*1000
    write_pcd(args.output / "map_kiss.pcd", points)
    path_pub.publish(trajectory)
    map_msg = output_cloud(points, trajectory.header)
    map_pub.publish(map_msg)
    writer.write(PATH, serialize_message(trajectory), last)
    writer.write(MAP, serialize_message(map_msg), last)
    del writer
    with (args.output / "trajectory_kiss.csv").open("w", newline="") as stream:
        csv_writer = csv.writer(stream)
        csv_writer.writerow(CSV_FIELDS)
        csv_writer.writerows(rows)
    with (args.output / "processing_times.csv").open("w", newline="") as stream:
        csv_writer = csv.writer(stream)
        csv_writer.writerow(("stamp_ns", "record_ns", *PROFILE_FIELDS))
        for key, (recorded, _, *latencies) in timing.items():
            csv_writer.writerow((key, recorded, *profiles.get(key, ("",)*3), *latencies))
    values = np.array([(*profiles[key], *values[2:]) for key, values in timing.items() if key in profiles])
    elapsed = [values[1] for values in timing.values()]
    quartile = max(len(values)//4, 1)
    stats = {
        "input_scans": len(seen), "processed_scans": len(rows), "dropped_scans": len(pending),
        "profiled_scans": len(values), "map_points": len(points),
        "input_record_span_s": (last - first)/1e9, "playback_and_drain_s": playback_time,
        "whole_run_s": time.monotonic() - started,
        "output_rate_hz": (len(elapsed) - 1)/(elapsed[-1] - elapsed[0]) if len(elapsed) > 1 else None,
        "global_voxel_reduce_ms": reduction_ms, "node_peak_rss_kib": node_rss,
        "replay_peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "output_lag_first_quarter_p95_ms": float(np.percentile(values[:quartile, -1], 95)),
        "output_lag_last_quarter_p95_ms": float(np.percentile(values[-quartile:, -1], 95)),
        "node_over_71_ms": int(np.count_nonzero(values[:, 2] > 71)),
        "output_lag_over_71_ms": int(np.count_nonzero(values[:, -1] > 71)),
        **{field: summary(values[:, i]) for i, field in enumerate(PROFILE_FIELDS)},
    }
    report = json.dumps(stats, indent=2)
    (args.output / "metrics.json").write_text(report + "\n")
    print(report, flush=True)


def view(node, args):
    output = args.output
    with (output / args.map_file).open("rb") as stream:
        for line in stream:
            if line == b"DATA binary\n":
                break
        else:
            raise ValueError("Expected a binary XYZ PCD from replay")
        points = np.fromfile(stream, dtype="<f4").reshape(-1, 3)
    trajectory = RosPath(header=Header(frame_id=args.frame))
    with (output / args.trajectory_file).open(newline="") as stream:
        for row in csv.DictReader(stream):
            header = Header(frame_id=args.frame, stamp=message_stamp(int(row["stamp_ns"])))
            pose = PoseStamped(header=header)
            p, q = pose.pose.position, pose.pose.orientation
            p.x, p.y, p.z, q.x, q.y, q.z, q.w = (float(row[key]) for key in CSV_FIELDS[1:])
            trajectory.poses.append(pose)
            trajectory.header = header
    template = yaml.safe_load(args.config.read_text())
    manager = template["Visualization Manager"]
    manager["Global Options"]["Fixed Frame"] = args.frame
    manager["Displays"][0]["Name"] = args.map_label
    manager["Displays"][1]["Name"] = args.path_label
    lo, hi = np.percentile(points, [0.5, 99.5], axis=0)
    if args.focus_start:
        center = np.array([trajectory.poses[0].pose.position.x,
                           trajectory.poses[0].pose.position.y,
                           (lo[2] + hi[2]) / 2])
        distance = 10.0
    else:
        center = (lo + hi) / 2
        distance = float(np.linalg.norm((hi - lo)[:2]) * 1.18)
    orbit = manager["Views"]["Current"]
    orbit["Distance"] = distance
    orbit["Focal Point"] = {axis: float(coordinate) for axis, coordinate in zip("XYZ", center)}
    top = {**orbit, "Name": "Top", "Pitch": 1.56, "Yaw": 0.0,
           "Distance": distance * 1.12}
    manager["Views"]["Saved"] = [{"Name": "Perspective", **orbit}, top]
    if args.top:
        manager["Views"]["Current"] = top
    generated = output / ("rviz_top.rviz" if args.top else
                          "rviz_detail.rviz" if args.focus_start else "rviz_perspective.rviz")
    generated.write_text(yaml.safe_dump(template, sort_keys=False))
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(PointCloud2, MAP, qos)
    path_pub = node.create_publisher(RosPath, PATH, qos)
    anchor = TransformStamped(header=Header(frame_id=args.frame), child_frame_id="kiss_view_anchor")
    anchor.transform.rotation.w = 1.0
    tf = StaticTransformBroadcaster(node)
    tf.sendTransform(anchor)
    map_pub.publish(display_cloud(points, trajectory.header))
    path_pub.publish(trajectory)
    print(f"RViz: {len(points)} points, {len(trajectory.poses)} poses, frame {args.frame}, {generated}", flush=True)
    with subprocess.Popen(["rviz2", "-d", str(generated)], start_new_session=True) as rviz:
        try:
            while rviz.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.2)
        except KeyboardInterrupt:
            pass
        finally:
            if rviz.poll() is None:
                rviz.send_signal(signal.SIGINT)
                rviz.wait(timeout=10)
        if rviz.returncode not in (0, None):
            raise RuntimeError(f"RViz exit status: {rviz.returncode}")


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("--bag", type=Path, required=True)
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--seconds", type=float)
    run.add_argument("--global-voxel", type=float, default=0.10)
    run.add_argument("--voxel-size", type=float, default=1.0)
    run.add_argument("--max-range", type=float, default=100.0)
    run.add_argument("--rate", type=float, default=1.0)
    run.add_argument("--threads", type=int, default=0)
    run.add_argument("--initial-threshold", type=float, default=2.0)
    run.add_argument("--deskew", action=argparse.BooleanOptionalAction, default=True)
    viewer = commands.add_parser("view")
    viewer.add_argument("--output", type=Path, required=True)
    viewer.add_argument("--config", type=Path, default=HERE / "stand.rviz")
    viewer.add_argument("--map-file", default="map_kiss.pcd")
    viewer.add_argument("--trajectory-file", default="trajectory_kiss.csv")
    viewer.add_argument("--frame", default=FRAME)
    viewer.add_argument("--map-label", default="KISS-ICP map")
    viewer.add_argument("--path-label", default="KISS-ICP trajectory")
    viewer.add_argument("--top", action="store_true")
    viewer.add_argument("--focus-start", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("kiss_stand")
    try:
        if args.command == "run":
            for value in (args.seconds, args.global_voxel, args.voxel_size, args.max_range,
                          args.rate, args.initial_threshold):
                if value is not None and (not math.isfinite(value) or value <= 0):
                    parser.error("Duration, voxel, range and playback rate must be finite and positive")
            if args.threads < 0:
                parser.error("Thread count must be nonnegative")
            replay(node, args)
        else:
            view(node, args)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
