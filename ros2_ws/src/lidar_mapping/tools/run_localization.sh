#!/usr/bin/env bash
# Run inside the ROS container with independent output paths.
set -eo pipefail

: "${ROS_DOMAIN_ID:?Set a dedicated ROS_DOMAIN_ID before running}"
export ROS_LOCALHOST_ONLY=1
bag="${1:?Usage: run_localization.sh BAG_DIR MAP_PCD OUTPUT_DIR [XYZ_RPY]}"
map="${2:?Missing map path}"
out="${3:?Missing output directory}"
seed="${4:-[0.0,0.0,0.0,0.0,0.0,0.0]}"
rate="${PLAY_RATE:-1.0}"
seconds="${PLAY_SECONDS:-10}"
record_tf="${RECORD_TF:-0}"
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$out"
exec 4>"$out/commands.log"
BASH_XTRACEFD=4
set -x
export ROS_LOG_DIR="$out/ros"
ros2 bag info "$bag" > "$out/bag-info.txt"
python3 "$package_dir/tools/inspect_localization_bag.py" "$bag" --cloud-stamps "$out/input-scans.csv" > "$out/input.json"

ros2 node list --no-daemon --spin-time 1 > "$out/nodes-before.txt"
if grep -q '[^[:space:]]' "$out/nodes-before.txt"; then
    echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID already has nodes" >&2
    exit 1
fi
tf_args=()
tf_topics=()
if [[ "$record_tf" == 1 ]]; then
    tf_args=(-p publish_map_to_odom:=true -p publish_odom_to_base:=true)
    tf_topics=(/tf)
fi
{
    printf 'ROS_DOMAIN_ID=%s\nROS_DISTRO=%s\nPLAY_RATE=%s\nPLAY_SECONDS=%s\nRECORD_TF=%s\n' \
        "$ROS_DOMAIN_ID" "${ROS_DISTRO:-}" "$rate" "$seconds" "$record_tf"
    printf 'RViz during measurement: no\nRecorder during measurement: yes\n'
    uname -a
    lscpu
    if ! pkg-config --modversion pcl_common 2>/dev/null; then
        grep -E '^#define PCL_(MAJOR|MINOR|REVISION)_VERSION ' /usr/include/pcl-*/pcl/pcl_config.h
    fi
    sha256sum "$map"
} > "$out/environment.txt"

node_pid=
recorder_pid=
player_pid=
stop() {
    local pid="$1"
    if [[ -n "$pid" ]]; then
        kill -INT "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}
cleanup() {
    stop "$player_pid"
    stop "$recorder_pid"
    stop "$node_pid"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Use the installed binary directly to keep signal delivery and CSV flush deterministic.
"$(ros2 pkg prefix lidar_mapping)/lib/lidar_mapping/lidar_localization" \
    --ros-args --params-file "$package_dir/config/localization.yaml" \
    -p "map_path:=$map" -p "csv_path:=$out/trajectory.csv" \
    -p "initial_pose_xyz_rpy:=$seed" "${tf_args[@]}" "${@:5}" > "$out/node.log" 2>&1 &
node_pid=$!
ros2 bag record -s mcap -o "$out/localization_bag" \
    /clock /localization/pose /localization/path /localization/map \
    /localization/aligned_cloud /localization/diagnostics "${tf_topics[@]}" \
    > "$out/record.log" 2>&1 &
recorder_pid=$!
sleep 2
kill -0 "$node_pid"
kill -0 "$recorder_pid"
timeout 10 ros2 param get /localization use_sim_time > "$out/use-sim-time.txt"

ros2 bag play "$bag" --start-paused --disable-keyboard-controls --clock 100 --rate "$rate" \
    --topics /utlidar/cloud_base /utlidar/robot_odom \
    > "$out/player.log" 2>&1 &
player_pid=$!
sleep 2
kill -0 "$player_pid"
for topic in /utlidar/cloud_base /utlidar/robot_odom /tf /tf_static; do
    ros2 topic info -v --no-daemon "$topic" >> "$out/live-topics.txt" 2>&1 || true
done
timeout 10 ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
if [[ "$seconds" == full ]]; then
    wait "$player_pid"
else
    sleep "$seconds"
    stop "$player_pid"
fi
player_pid=
# Drain measurements already received by the localizer before closing the recorder.
sleep 1
stop "$node_pid"
node_pid=
stop "$recorder_pid"
recorder_pid=
ros2 bag info "$out/localization_bag" > "$out/output-bag-info.txt"
python3 - "$out/trajectory.csv" "$out/localization_bag/metadata.yaml" "$out/input-scans.csv" "$seconds" "$record_tf" <<'PYTHON' | tee "$out/summary.json"
from collections import Counter
import csv
import json
import statistics
import sys
import yaml

with open(sys.argv[1], newline="") as stream:
    rows = list(csv.DictReader(stream))
with open(sys.argv[2]) as stream:
    topics = yaml.safe_load(stream)["rosbag2_bagfile_information"]["topics_with_message_count"]
with open(sys.argv[3], newline="") as stream:
    sent = {int(row["stamp_ns"]): int(row["storage_ns"]) for row in csv.DictReader(stream)}
processed = [row for row in rows if row["status"] != "dropped"]
summary = {"statuses": Counter(row["status"] for row in rows),
           "reasons": Counter(row["reason"] for row in rows),
           "output_messages": {row["topic_metadata"]["name"]: row["message_count"] for row in topics}}
summary["all_input_scans"] = len(sent)
summary["processed_scans"] = len(processed)
if rows:
    last = max(int(row["stamp_ns"]) for row in rows)
    span = sent if sys.argv[4] == "full" else {stamp: t for stamp, t in sent.items() if stamp <= last}
    summary["input_scans_in_measured_interval"] = len(span)
    summary["input_storage_interval_ns"] = [min(span.values()), max(span.values())]
    summary["input_scans_missing_in_node"] = len(span) - len({row["stamp_ns"] for row in rows})
if processed:
    accepted = summary["statuses"]["accepted"]
    summary["accepted_fraction_of_processed"] = accepted / len(processed)
    summary["fallback_fraction_of_processed"] = summary["statuses"]["fallback"] / len(processed)
for column in ("pipeline_ms", "processing_ms", "registration_ms", "queue_ms"):
    values = sorted(float(row[column]) for row in processed)
    if values:
        summary[column] = {"mean": statistics.mean(values),
                           "median": statistics.median(values),
                           "p95": values[(95 * len(values) + 99) // 100 - 1],
                           "p99": values[(99 * len(values) + 99) // 100 - 1],
                           "max": values[-1]}
if len(processed) > 1:
    summary["scan_stamp_span_s"] = (
        int(processed[-1]["stamp_ns"]) - int(processed[0]["stamp_ns"])) / 1e9
    summary["processed_scans_per_bag_second"] = (len(processed) - 1) / summary["scan_stamp_span_s"]
    n = max(1, len(processed) // 4)
    summary["queue_ms_first_quarter_median"] = statistics.median(float(row["queue_ms"]) for row in processed[:n])
    summary["queue_ms_last_quarter_median"] = statistics.median(float(row["queue_ms"]) for row in processed[-n:])
print(json.dumps(summary, indent=2))
required = ("/clock", "/localization/pose", "/localization/path", "/localization/map",
            "/localization/aligned_cloud", "/localization/diagnostics")
if sys.argv[5] == "1":
    required += ("/tf",)
sys.exit(0 if summary["statuses"]["accepted"] and
         all(summary["output_messages"].get(topic, 0) for topic in required) else 1)
PYTHON
