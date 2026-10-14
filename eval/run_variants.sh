#!/usr/bin/env bash
# Builds the workspace inside the ros:humble container and runs each EKF variant over the Linkou bag,
# recording /ekf/odom to eval/results/run_<variant>.
#
# Usage (from the repo root):
#   docker run --rm -v "$PWD":/ws -v ~/Downloads/linkou:/data:ro ros:humble-ros-base-jammy \
#       bash /ws/eval/run_variants.sh A B C
set -e
source /opt/ros/humble/setup.bash
cd /ws

BUILD=/ws/eval/.docker_ws
colcon build --symlink-install --build-base $BUILD/build --install-base $BUILD/install \
    --cmake-args -DBUILD_TESTING=OFF > $BUILD.log 2>&1 || { tail -40 $BUILD.log; exit 1; }
source $BUILD/install/setup.bash

BAG=/data/linkou-2023-12-27-2-med
RATE=${RATE:-4}
mkdir -p /ws/eval/results

for V in "$@"; do
    OUT=/ws/eval/results/run_$V
    rm -rf "$OUT"
    ros2 launch kalman_filter_app launch_ekf_node.py variant:=$V > /ws/eval/results/node_$V.log 2>&1 &
    NODE=$!
    ros2 bag record -o "$OUT" /ekf/odom > /dev/null 2>&1 &
    REC=$!
    sleep 3
    ros2 bag play "$BAG" --rate $RATE --topics /wheel_odom /imu/data_raw > /ws/eval/results/play_$V.log 2>&1
    sleep 2
    kill -INT $REC; wait $REC || true
    kill -TERM $NODE; pkill -f "lib/kalman_filter_app/kalman_filter_app" || true
    timeout 10 tail --pid=$NODE -f /dev/null || kill -9 $NODE || true
    while pgrep -f "lib/kalman_filter_app/kalman_filter_app" > /dev/null; do sleep 0.5; done
    echo "variant $V: $(ros2 bag info "$OUT" | grep -o "Count: [0-9]*" | head -1) /ekf/odom messages"
    echo "variant $V done: $(grep -c . /ws/eval/results/node_$V.log) log lines"
done
chown -R "$(stat -c %u:%g /ws)" /ws/eval
