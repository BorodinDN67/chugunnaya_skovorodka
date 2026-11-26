#!/usr/bin/env python3
"""Inspect recorded localization inputs without starting a ROS node."""
import argparse
import bisect
from collections import Counter
import csv
import json

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message

CLOUD = "/utlidar/cloud_base"
ODOM = "/utlidar/robot_odom"


def inspect_bag(uri, cloud_stamps):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    topics = {row.name: row.type for row in reader.get_all_topics_and_types()}
    types = {name: get_message(topics[name]) for name in (CLOUD, ODOM)}
    reader.set_filter(StorageFilter(topics=[CLOUD, ODOM]))
    frames = {name: Counter() for name in types}
    records = {name: [] for name in types}
    layouts = Counter()
    first_odom = None
    while reader.has_next():
        topic, raw, received = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        records[topic].append((stamp, received))
        if topic == ODOM:
            frames[topic][f"{msg.header.frame_id} -> {msg.child_frame_id}"] += 1
            if first_odom is None:
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                first_odom = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        else:
            frames[topic][msg.header.frame_id] += 1
            fields = ",".join(f"{f.name}@{f.offset}:{f.datatype}" for f in msg.fields)
            layouts[f"{msg.point_step} bytes, {fields}"] += 1

    odom_stamps = sorted(set(stamp for stamp, _ in records[ODOM]))
    gaps = []
    for stamp, _ in records[CLOUD]:
        i = bisect.bisect_left(odom_stamps, stamp)
        if i < len(odom_stamps) and odom_stamps[i] == stamp:
            gaps.append(0)
        elif 0 < i < len(odom_stamps):
            gaps.append(odom_stamps[i] - odom_stamps[i - 1])
    if cloud_stamps:
        with open(cloud_stamps, "w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(("stamp_ns", "storage_ns"))
            writer.writerows(records[CLOUD])
    print(json.dumps({
        "types": topics,
        "tf_topics": [name for name in topics if name in ("/tf", "/tf_static")],
        "frames": frames,
        "cloud_fields": layouts,
        "first_odometry_xyz_quaternion": first_odom,
        "timestamps": {
            name: {
                "count": len(rows),
                "first_header_and_storage_ns": rows[0],
                "last_header_and_storage_ns": rows[-1],
                "nonincreasing_headers": sum(b[0] <= a[0] for a, b in zip(rows, rows[1:])),
            } for name, rows in records.items()
        },
        "bracketed_scans": len(gaps),
        "max_bracket_gap_ms": max(gaps, default=0) / 1e6,
    }, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--cloud-stamps")
    args = parser.parse_args()
    inspect_bag(args.bag, args.cloud_stamps)
