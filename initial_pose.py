"""Estimate an explicit localization seed by matching scans against a fixed KISS map."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import PointCloud2

from stand_replay import TOPIC, find_bag, stamp_ns, xyz


def first_scans(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(find_bag(bag)), storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    odom_topic = "/utlidar/robot_odom"
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC, odom_topic]))
    scans, odometry = [], []
    first = None
    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == odom_topic:
            msg = deserialize_message(serialized, Odometry)
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            odometry.append((stamp_ns(msg.header), (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))
        else:
            msg = deserialize_message(serialized, PointCloud2)
            if msg.header.frame_id != "base_link":
                raise ValueError("Expected base_link")
            stamp = stamp_ns(msg.header)
            if first is None:
                first = stamp
            elif stamp - first > 2_100_000_000:
                break
            elif stamp - first > 120_000_000 and (not scans or stamp - scans[-1][0] > 490_000_000):
                points = xyz(msg)
                distance = np.linalg.norm(points, axis=1)
                scans.append((stamp, points[(distance > 0.30) & (distance < 8.0)][::2]))
    odometry.sort()
    time = np.array([row[0] - first for row in odometry], dtype=float) / 1e9
    scan_time = np.array([stamp - first for stamp, _ in scans], dtype=float) / 1e9
    positions = np.array([row[1] for row in odometry])
    interpolated = np.column_stack([np.interp(scan_time, time, axis) for axis in positions.T])
    rotations = Slerp(time, Rotation.from_quat([row[2] for row in odometry]))(scan_time)
    inv = rotations[0].inv()
    aligned = [inv.apply(rot.apply(points) + offset - interpolated[0])
               for (_, points), offset, rot in zip(scans, interpolated, rotations)]
    return scans[0][0], np.concatenate(aligned)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("mapping", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    stamp, source = first_scans(args.bag)
    with (args.mapping / "map_kiss.pcd").open("rb") as stream:
        for line in stream:
            if line == b"DATA binary\n":
                break
        else:
            raise ValueError("Expected a binary XYZ PCD")
        target = cKDTree(np.fromfile(stream, dtype="<f4").reshape(-1, 3))
    with (args.mapping / "trajectory_kiss.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    sampled, centers = [], []
    for row in rows:
        p = np.array([float(row[axis]) for axis in "xyz"])
        q = Rotation.from_quat([float(row[name]) for name in ("qx", "qy", "qz", "qw")])
        if not centers or np.linalg.norm(p - np.asarray(centers), axis=1).min() >= 0.4:
            centers.append(p)
            sampled.append((p, q))

    candidates = []

    def score(p, q):
        distances, _ = target.query(q.apply(source) + p)
        return float(np.minimum(distances, 0.65).mean())

    for p, q in sampled:
        for angle in range(0, 360, 15):
            rotated = Rotation.from_euler("z", angle, degrees=True) * q
            candidates.append((score(p, rotated), p, rotated))
    ranked = sorted(candidates, key=lambda candidate: candidate[0])
    selected = []
    for candidate in ranked:
        _, p, q = candidate
        if all(np.linalg.norm(p - prior[1]) > 0.45 or
               (q * prior[2].inv()).magnitude() > np.deg2rad(30) for prior in selected):
            selected.append(candidate)
            if len(selected) == 5:
                break
    refined = []
    for _, p, q in selected:
        for x in (-0.20, 0.0, 0.20):
            for y in (-0.20, 0.0, 0.20):
                for yaw in (-7.5, 0.0, 7.5):
                    offset = p + [x, y, 0]
                    attitude = Rotation.from_euler("z", yaw, degrees=True) * q
                    refined.append((score(offset, attitude), offset, attitude))
    best = min(refined, key=lambda candidate: candidate[0])
    result = {"initial_pose_xyz_xyzw": [*best[1].tolist(), *best[2].as_quat().tolist()],
              "score_m": best[0], "source_points": len(source), "seed_scan_stamp_ns": stamp,
              "coarse_hypotheses": len(candidates), "coarse_distinct_scores_m": [x[0] for x in selected]}
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(",".join(str(value) for value in result["initial_pose_xyz_xyzw"]))


if __name__ == "__main__":
    main()
