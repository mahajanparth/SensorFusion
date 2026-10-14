#!/usr/bin/env bash
# Shows EKF variants A/B/C live in RViz2 while the Linkou bag plays.
#
# Build the image once:
#   docker build -t sensorfusion-rviz -f eval/docker/Dockerfile.rviz eval/docker
# On your display (from the repo root):
#   xhost +local:docker
#   docker run --rm -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v "$PWD":/ws \
#       -v ~/Downloads/linkou:/data:ro sensorfusion-rviz bash /ws/eval/rviz_variants.sh gui
# Headless, saving screenshots + GIF to eval/results/rviz/:
#   docker run --rm -v "$PWD":/ws -v ~/Downloads/linkou:/data:ro sensorfusion-rviz \
#       bash /ws/eval/rviz_variants.sh record
set -e
MODE=${1:-gui}
RATE=${RATE:-4}
source /opt/ros/humble/setup.bash
cd /ws

BUILD=/ws/eval/.docker_ws
colcon build --symlink-install --build-base $BUILD/build --install-base $BUILD/install \
    --cmake-args -DBUILD_TESTING=OFF > $BUILD.log 2>&1 || { tail -40 $BUILD.log; exit 1; }
source $BUILD/install/setup.bash

OUT=/ws/eval/results/rviz
if [ "$MODE" = record ]; then
    export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
    Xvfb :99 -screen 0 1600x1000x24 > /dev/null 2>&1 &
    sleep 2
    rm -rf $OUT && mkdir -p $OUT/frames
fi

ros2 launch kalman_filter_app ekf_variants_rviz.launch.py \
    alignment_file:=/ws/eval/results/alignment.json > /tmp/viz_launch.log 2>&1 &
LAUNCH=$!
sleep 8

ros2 bag play /data/linkou-2023-12-27-2-med --rate $RATE --topics /wheel_odom /imu/data_raw /fix > /dev/null 2>&1 &
PLAY=$!

if [ "$MODE" = record ]; then
    i=0
    while kill -0 $PLAY 2> /dev/null; do
        import -window root $OUT/frames/$(printf %04d $i).png
        i=$((i + 1))
        sleep 1
    done
    sleep 2
    import -window root $OUT/ekf_variants_rviz.png
    convert -delay 12 -loop 0 -resize 50% $OUT/frames/*.png $OUT/ekf_variants_rviz.gif
    rm -rf $OUT/frames
    kill -TERM $LAUNCH; pkill -f "kalman_filter_app|rviz2|variant_path" || true
    chown -R "$(stat -c %u:%g /ws)" /ws/eval
    echo "saved $OUT/ekf_variants_rviz.png and .gif"
else
    wait $PLAY
    echo "Playback finished; close RViz to exit."
    wait $LAUNCH
fi
