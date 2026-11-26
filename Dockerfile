# ---------- Этап 1: сборка сообщений Unitree ----------
FROM ros:humble-ros-base AS builder

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential cmake python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rosidl-adapter \
    && rm -rf /var/lib/apt/lists/*

RUN git init /tmp/unitree_ros2 \
    && cd /tmp/unitree_ros2 && git fetch --depth 1 https://github.com/unitreerobotics/unitree_ros2 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88 && git checkout --detach FETCH_HEAD \
    && source /opt/ros/humble/setup.bash \
    && colcon build --install-base /opt/unitree_msgs --packages-select unitree_go unitree_api \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

ARG KISS_ICP_COMMIT=85488b7b6a6e1566cff0f8af4b83a9ea77dca7a0
RUN apt-get update && apt-get install -y --no-install-recommends \
    libeigen3-dev libtbb-dev ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /tmp/kiss_ws/src \
    && git init /tmp/kiss_ws/src/kiss-icp \
    && git -C /tmp/kiss_ws/src/kiss-icp fetch --depth 1 https://github.com/PRBonn/kiss-icp "${KISS_ICP_COMMIT}" \
    && git -C /tmp/kiss_ws/src/kiss-icp checkout --detach FETCH_HEAD \
    && test "$(git -C /tmp/kiss_ws/src/kiss-icp rev-parse HEAD)" = "${KISS_ICP_COMMIT}" \
    && cd /tmp/kiss_ws \
    && source /opt/ros/humble/setup.bash \
    && CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --install-base /opt/kiss_ws \
       --packages-select kiss_icp --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && install -Dm644 src/kiss-icp/LICENSE /opt/kiss_ws/LICENSE-KISS-ICP

# ---------- Этап 2: финальный образ ----------
FROM ros:humble-ros-base AS replay

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    COLCON_WS=/ros2_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    python3-numpy libtbb12 ros-humble-rclpy ros-humble-rosbag2-py ros-humble-sensor-msgs-py \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libxkbcommon-x11-0 libxkbcommon0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 \
    libxcb-randr0 libxcb-render-util0 \
    libxcb-xinerama0 libxcb-xkb1 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/unitree_msgs /opt/unitree_msgs
COPY --from=builder /tmp/unitree_ros2/LICENSE /opt/unitree_msgs/LICENSE

COPY --from=builder /opt/kiss_ws /opt/kiss_ws
COPY --from=builder /tmp/kiss_ws/build/kiss_icp/_deps/sophus-src/LICENSE.txt /opt/kiss_ws/LICENSE-Sophus
COPY --from=builder /tmp/kiss_ws/build/kiss_icp/_deps/tessil-src/LICENSE /opt/kiss_ws/LICENSE-robin-map
WORKDIR /opt/go2_playback
COPY play_viz.sh go2.rviz run_kiss.py results.rviz ./
RUN sed -i 's/\r$//' play_viz.sh && chmod +x play_viz.sh

WORKDIR /ros2_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source /opt/unitree_msgs/setup.bash" >> /root/.bashrc \
    && echo "source /opt/kiss_ws/local_setup.bash" >> /root/.bashrc \
    && echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", \
    "source /opt/ros/humble/setup.bash && source /opt/unitree_msgs/setup.bash \
     && source /opt/kiss_ws/local_setup.bash \
     && if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi \
     && exec \"$@\"", "--"]
CMD ["bash"]

# Optional development environment from the hackathon Dockerfile.
FROM replay AS full
RUN apt-get update && apt-get install -y --no-install-recommends \
    -o Dpkg::Options::=--force-unsafe-io \
    git build-essential cmake gdb \
    python3-pip python3-colcon-common-extensions python3-vcstool python3-rosdep \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-rosidl-default-generators \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-robot-localization \
    && rm -rf /var/lib/apt/lists/*

FROM replay AS portable
