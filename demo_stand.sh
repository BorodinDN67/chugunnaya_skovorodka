#!/usr/bin/env bash
set -eo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
container=${KISS_CONTAINER:-r_hacatonius}
tools=/ros2_ws/results/kiss/tools
control=${KISS_CONTROL_OUT:-/ros2_ws/results/kiss/control}
mapping=/ros2_ws/results/kiss/mapping
command=${1:-view}
if [[ $# -gt 0 ]]; then
    shift
fi

case $command in
    prepare|view|kiss-view) ;;
    *) echo "Usage: $0 {prepare|view|kiss-view} [--top|--focus-start]" >&2; exit 2 ;;
esac

docker exec "$container" mkdir -p "$tools"
for file in control_map.py run_kiss.py stand_replay.py stand.rviz; do
    docker cp "$here/$file" "$container:$tools/$file"
done

if [[ $command != kiss-view ]]; then
    docker exec "$container" bash -c '
        set -e
        source /opt/ros/humble/setup.bash
        if [[ ! -s "$2/map_control.pcd" || ! -s "$2/trajectory_odom.csv" ]]; then
            python3 "$1/control_map.py" "$3" "$2"
        fi
    ' -- "$tools" "$control" "${KISS_MAPPING_BAG:-/ros2_ws/bag_for_mapping}"
fi
if [[ $command == prepare ]]; then
    exit
fi

viewer=(--output "$control" --map-file map_control.pcd --trajectory-file trajectory_odom.csv
        --frame odom --map-label "Control map" --path-label "Recorded odometry")
if [[ $command == kiss-view ]]; then
    viewer=(--output "$mapping")
fi
docker exec -e "ROS_DOMAIN_ID=${KISS_ROS_DOMAIN:-73}" -e ROS_LOCALHOST_ONLY=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 -e QT_X11_NO_MITSHM=1 "$container" bash -c '
        source /opt/ros/humble/setup.bash
        exec python3 "$@"
    ' -- "$tools/stand_replay.py" view "${viewer[@]}" "$@"
