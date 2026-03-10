#!/usr/bin/env bash
set -e

# Example:
# ./cfpyctrl.sh \
#  --uri radio://0/80/2M/E7E7E7E7B1 \
#  --port 5000 \
#  --wsendpoint \
#  --wsport 8765

docker run --rm -it \
  --device-cgroup-rule='c 189:* rmw' \
  --net=host --ipc=host --pid=host \
  -e PYTHONUNBUFFERED=1 \
  --env="DISPLAY" \
  --volume="$XAUTHORITY:$XAUTHORITY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /run/udev:/run/udev:ro \
  -v /dev:/dev \
  cf-pyctrl \
  bash -ic "./cfpyctrl.sh $*"