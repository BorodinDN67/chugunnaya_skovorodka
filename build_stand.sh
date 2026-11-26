#!/usr/bin/env bash
set -eo pipefail

here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
src=/ros2_ws/src/kiss_vendor_integration
build=/ros2_ws/results/kiss/build
revision=85488b7b6a6e1566cff0f8af4b83a9ea77dca7a0
mkdir -p "$build"
if [[ ! -d "$src/.git" ]]; then
    mkdir "$src"
    git -C "$src" init -q
    git -C "$src" fetch --depth 1 https://github.com/PRBonn/kiss-icp "$revision"
    git -C "$src" checkout -q --detach FETCH_HEAD
fi
test "$(git -C "$src" rev-parse HEAD)" = "$revision"
touch "$src/COLCON_IGNORE"
if git -C "$src" apply --check "$here/kiss_profile.patch" 2>/dev/null; then
    git -C "$src" apply "$here/kiss_profile.patch"
else
    git -C "$src" apply --reverse --check "$here/kiss_profile.patch"
fi

source /opt/ros/humble/setup.bash
cd "$build"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log build --paths "$src/ros" --build-base build --install-base install \
    --packages-select kiss_icp --event-handlers console_direct+ \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee build.log
colcon --log-base test_log test --paths "$src/ros" --build-base build --install-base install \
    --packages-select kiss_icp 2>&1 | tee test.log
colcon test-result --test-result-base build --verbose 2>&1 | tee -a test.log
