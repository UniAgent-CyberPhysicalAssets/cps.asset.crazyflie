#!/usr/bin/env bash
set -e

xhost +local:"$USER"

docker run --rm \
  --device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
  --net=host --ipc=host --pid=host \
  --env="DISPLAY" \
  --env="XAUTHORITY=$XAUTHORITY" \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --volume="$XAUTHORITY:$XAUTHORITY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  cf-pyctrl \
  bash -lc "cfclient"