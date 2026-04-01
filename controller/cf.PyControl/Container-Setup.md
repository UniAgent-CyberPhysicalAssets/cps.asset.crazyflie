# Container Workspace Setup: cf.PyControl

This document describes how to create and start a containerized environment with the following components installed:

- **cf.PyControl**: High-level Python controller for the Crazyflie
-  [**cfclient**](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/): The Crazyflie PC client

Usage instructions for the components can be found in the [README](README.md).

## Installation

### Preparation

This section explains the one-time setup you need to complete before building the image and getting started.

#### Allowing USB

- First make appropriate USB permissions under [Linux](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/) 

- Find out the major device number of the USB Crazyradio as described here: https://uniagent-cyberphysicalassets.github.io/cps.asset.crazyflie/12-getting-started
  - The **major device number** is in this example `189`
  - You have to add the respective rule when starting the Docker container (see example below): `--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev`
    

#### Enable GPU-accelerated Containers

- Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Building the Docker Image

```shell
$ docker build -t cf-pyctrl -f .devcontainer/Dockerfile .
```

## Usage

First, allow the Docker Container to Access to X Server (GUI):

```shell
$ xhost +local:"$USER"
```

(Revert the `xhost` settings to their original state afterward: `xhost -`)

### Running the Docker Container

**Standard**

```shell
# + USB devices (Crazyradio)

$ docker run --rm -it \
--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
--net=host --ipc=host --pid=host \
--env="DISPLAY" \
--env ROS_DOMAIN_ID=30 \
--env="XAUTHORITY=$XAUTHORITY" \
--volume="$XAUTHORITY:$XAUTHORITY" \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
cf-pyctrl
```

> **Note:** In `c 189:* rmw`, replace the `189` with the **major device number** you obtained in the [Allowing USB](#allowing-usb) section.

> **Note:** Add `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all` if you want to enable GPU acceleration with an Nvidia GPU inside the container. This requires the NVIDIA Container Toolkit to be installed on the host system.

**Sync Folders**

Enable "Dev-Mode":

```shell
--mount type=bind,source=$(pwd)/src,target=/home/user/dev_ws/libs/crazyflie-controller/src  \
```

Add additional example scripts for testing the drone:
```shell
--mount type=bind,source=$(pwd)/examples,target=/home/user/dev_ws/libs/crazyflie-controller/examples  \
```

**Direct Access to cf.PyControl**

```shell
$ docker run --rm -it \
  --device-cgroup-rule='c 189:* rmw' \
  -v /run/udev:/run/udev:ro \
  -v /dev:/dev \
  --net=host --ipc=host --pid=host \
  -e DISPLAY \
  -e XAUTHORITY=$XAUTHORITY \
  -e PYTHONUNBUFFERED=1 \
  -v $XAUTHORITY:$XAUTHORITY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  cf-pyctrl \
  bash -ic "./cfpyctrl.sh --uri radio://0/80/2M/E7E7E7E7B1 --port 5000 --wsendpoint --wsport 8765"
```



**cfclient**

Just start the cfclient tool:

```shell
$ docker run --rm \
  --device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
  --net=host --ipc=host --pid=host \
  --env="DISPLAY" \
  --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
  --env="XAUTHORITY=$XAUTHORITY" \
  --volume="$XAUTHORITY:$XAUTHORITY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  cf-pyctrl \
  bash -lc "cfclient"
```
