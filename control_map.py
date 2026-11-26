"""Accumulate the organizers' world-frame deskewed cloud for visual comparison."""

import argparse
import csv
from pathlib import Path

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

from stand_replay import find_bag, stamp_ns, write_pcd, xyz


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(find_bag(args.bag)), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=["/utlidar/cloud_deskewed", "/utlidar/robot_odom"]))
    scans, rows = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic.endswith("/cloud_deskewed"):
            msg = deserialize_message(data, PointCloud2)
            if msg.header.frame_id != "odom":
                raise ValueError("Expected the input cloud in odom")
            points = xyz(msg)
            scans.append(points[np.any(points != 0.0, axis=1)])
        else:
            msg = deserialize_message(data, Odometry)
            if msg.header.frame_id != "odom" or msg.child_frame_id != "base_link":
                raise ValueError("Expected odom -> base_link odometry")
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            rows.append((stamp_ns(msg.header), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    points = np.concatenate(scans)
    # Average points in each 5 cm voxel to retain the observed surface.
    _, ids, counts = np.unique(np.floor(points / 0.05).astype(np.int64), axis=0,
                               return_inverse=True, return_counts=True)
    cloud = np.column_stack([np.bincount(ids, weights=axis)/counts for axis in points.T])
    write_pcd(args.output / "map_control.pcd", cloud.astype("<f4"))
    rows.sort()
    start = rows[0][0]
    with (args.output / "trajectory_odom.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("stamp_ns", "x", "y", "z", "qx", "qy", "qz", "qw"))
        for row in rows:
            if row[0] >= start:
                writer.writerow(row)
                start = row[0] + 60_000_000
    print(f"{len(scans)} deskewed clouds, {len(points)} input points, {len(cloud)} occupied voxels")


if __name__ == "__main__":
    main()
