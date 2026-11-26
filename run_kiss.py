import argparse
import csv
import json
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
import rosbag2_py
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header

FRAME = "odom_lidar"
HERE = Path(__file__).resolve().parent
CSV_FIELDS = ("stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw")


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def xyz(cloud):
    offsets = {field.name: field.offset for field in cloud.fields
               if field.datatype == 7 and field.count == 1}
    dtype = ">f4" if cloud.is_bigendian else "<f4"
    shape = (cloud.height, cloud.width)
    strides = (cloud.row_step, cloud.point_step)
    points = np.stack([np.ndarray(shape, dtype, cloud.data, offsets[axis], strides)
                       for axis in "xyz"], axis=-1).reshape(-1, 3)
    return points[np.isfinite(points).all(axis=1)]


def save_results(args, scans, received, first_record, last_record, wall_time):
    rows, clouds, latencies, pose_times = [], [], [], []
    for elapsed, cloud, pose in received:
        p, q = pose.pose.pose.position, pose.pose.pose.orientation
        if pose.header.frame_id != FRAME or pose.child_frame_id != cloud.header.frame_id:
            raise RuntimeError("Odometry and point cloud frames do not match")
        rows.append((stamp_ns(pose.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        # Apply q * point * conjugate(q), then translate into odom_lidar.
        points = xyz(cloud)
        axis = np.array([q.x, q.y, q.z])
        cross = 2 * np.cross(axis, points)
        clouds.append((points + q.w * cross + np.cross(axis, cross)
                       + [p.x, p.y, p.z]).astype("<f4"))
        latencies.append(elapsed[1] - elapsed[0])
        pose_times.append(elapsed[1])
    points = np.concatenate(clouds) if clouds else np.empty((0, 3), dtype="<f4")
    if len(points):
        _, keep = np.unique(np.floor(points / 0.10).astype(np.int64), axis=0, return_index=True)
        points = points[keep]
    with (args.output / "trajectory.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        writer.writerows(rows)
    with (args.output / "map.pcd").open("wb") as stream:
        stream.write(("VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
                      f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n"
                      f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n").encode())
        points.tofile(stream)
    result = {
        "topic": args.topic, "deskew": args.deskew, "odometry_frame": FRAME,
        "input_scans": scans, "processed_scans": len(rows), "map_points": len(points),
        "record_span_s": (last_record - first_record) / 1e9 if scans else 0,
        "wall_playback_and_drain_s": wall_time,
        "pose_rate_hz": (len(rows) - 1) / (pose_times[-1] - pose_times[0])
        if len(rows) > 1 else None,
        "publish_to_odometry_ms_p50_p95": (np.percentile(latencies, [50, 95]) * 1000).tolist()
        if latencies else None,
    }
    report = json.dumps(result, indent=2)
    (args.output / "metrics.json").write_text(report + "\n")
    print(report, flush=True)
    if not rows or not len(points):
        raise RuntimeError("Empty trajectory or map")


def replay(args, node):
    subprocess.run(["ros2", "bag", "info", str(args.bag)], check=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if types.get(args.topic) != "sensor_msgs/msg/PointCloud2":
        raise ValueError(f"{args.topic} is not a PointCloud2 topic in this bag")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    publisher = node.create_publisher(PointCloud2, "/replay/points", 10)
    pending, received = {}, []

    def receive(pose):
        match = pending.pop(stamp_ns(pose.header), None)
        if match is not None:
            sent, cloud = match
            received.append(((sent, time.monotonic()), cloud, pose))

    node.create_subscription(Odometry, "/kiss/odometry", receive, 100)
    with subprocess.Popen([
        str(Path(get_package_prefix("kiss_icp")) / "lib/kiss_icp/kiss_icp_node"), "--ros-args",
        "-r", "pointcloud_topic:=/replay/points", "-p", f"deskew:={str(args.deskew).lower()}",
        "-p", "publish_debug_clouds:=false", "-p", "publish_odom_tf:=false",
        "-p", f"lidar_odom_frame:={FRAME}",
    ]) as process:
        try:
            deadline = time.monotonic() + 20
            while not (publisher.get_subscription_count() and node.count_publishers("/kiss/odometry")):
                if process.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("KISS-ICP did not connect")
                rclpy.spin_once(node, timeout_sec=0.05)
            first_record = last_record = scans = 0
            started = time.monotonic()
            while reader.has_next():
                _, data, recorded = reader.read_next()
                if scans == 0:
                    first_record = recorded
                offset = (recorded - first_record) / 1e9
                if args.seconds is not None and offset >= args.seconds:
                    break
                cloud = deserialize_message(data, PointCloud2)
                while (remaining := started + offset - time.monotonic()) > 0:
                    rclpy.spin_once(node, timeout_sec=min(remaining, 0.05))
                if process.poll() is not None:
                    raise RuntimeError("KISS-ICP exited during playback")
                key = stamp_ns(cloud.header)
                if key in pending:
                    raise RuntimeError(f"Duplicate pending scan stamp: {key}")
                pending[key] = (time.monotonic(), cloud)
                publisher.publish(cloud)
                last_record = recorded
                scans += 1
                rclpy.spin_once(node, timeout_sec=0)
            deadline = time.monotonic() + 3
            while pending and time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.05)
            wall_time = time.monotonic() - started
            if process.poll() is not None:
                raise RuntimeError("KISS-ICP exited before playback finished")
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
    save_results(args, scans, received, first_record, last_record, wall_time)


def view(output, node, frame, rviz_config, publish_only):
    with (output / "map.pcd").open("rb") as stream:
        for line in stream:
            if line == b"DATA binary\n":
                break
        else:
            raise ValueError("Expected a binary PCD generated by run_kiss.py")
        points = np.fromfile(stream, dtype="<f4").reshape(-1, 3)
    header = Header(frame_id=frame)
    trajectory = RosPath(header=header)
    with (output / "trajectory.csv").open(newline="") as stream:
        for row in csv.DictReader(stream):
            pose = PoseStamped(header=header)
            p, q = pose.pose.position, pose.pose.orientation
            p.x, p.y, p.z, q.x, q.y, q.z, q.w = (float(row[key]) for key in CSV_FIELDS[1:])
            trajectory.poses.append(pose)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    map_pub = node.create_publisher(PointCloud2, "/kiss/results/map", qos)
    path_pub = node.create_publisher(RosPath, "/kiss/results/path", qos)
    map_pub.publish(create_cloud_xyz32(header, points.tolist()))
    path_pub.publish(trajectory)
    if publish_only:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    else:
        subprocess.run(["rviz2", "-d", str(rviz_config)], check=True)


def main():
    parser = argparse.ArgumentParser()
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run")
    run.add_argument("--bag", type=Path, required=True)
    run.add_argument("--topic", default="/utlidar/cloud_base")
    run.add_argument("--output", type=Path, required=True)
    run.add_argument("--seconds", type=float)
    run.add_argument("--deskew", action=argparse.BooleanOptionalAction, default=True)
    viewer = commands.add_parser("view")
    viewer.add_argument("--output", type=Path, required=True)
    viewer.add_argument("--frame", default=FRAME)
    viewer.add_argument("--rviz-config", type=Path, default=HERE / "results.rviz")
    viewer.add_argument("--publish-only", action="store_true")
    args = parser.parse_args()
    if args.command == "run":
        if args.topic.strip("/") == "utlidar/cloud_deskewed":
            parser.error("/utlidar/cloud_deskewed is already in the global start frame")
        if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds <= 0):
            parser.error("--seconds must be finite and positive")
        args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node("kiss_hackathon")
    try:
        if args.command == "run":
            replay(args, node)
        else:
            view(args.output, node, args.frame, args.rviz_config, args.publish_only)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
