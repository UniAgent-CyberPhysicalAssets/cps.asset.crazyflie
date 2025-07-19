# Container Workspace Setup: cf.PyControl

This document describes how to create and start a containerized environment with the following components installed:

- **cf.PyControl**: High-level Python controller for the Crazyflie
-  [**cfclient**](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/): The Crazyflie PC client

Usage instructions for the components can be found in the [README](README.md).

## Installation

### Preparation

This section explains the one-time setup you need to complete before building the image and getting started.

#### Allowing USB

Make Crazyradio available in Docker container:
- First make appropriate USB permissions under [Linux](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/) 
- After, open the terminal and use the `lsusb` command to get a list of all USB devices connected to your system. Each line in the `lsusb` output represents a USB device with a Bus number and Device number.
- Then, you can select either way:
  - Use `udevadm` to get detailed information about the device, which includes the associated `/dev` entry.
  - Get device major number via `ls -la`
  - In both commands above, replace `001` with the Bus number and `009` with the Device number as shown in the output of `lsusb`.
- Example:
```shell
$ lsusb
[...]
Bus 001 Device 009: ID 1915:7777 Nordic Semiconductor ASA Bitcraze Crazyradio (PA) dongle
[...]

$ sudo udevadm info --query=all --name=/dev/bus/usb/001/009
[...]
U: usb
T: usb_device
D: c 189:262
N: bus/usb/001/009
[...]

$ ls -la /dev/bus/usb/001/009
crw-rw-r-- 1 root plugdev 189, 8 Aug  8 22:38 /dev/bus/usb/001/009
```

- Important is the **major device number**, which is in this example `189`
  
- Add the respective rule when starting the Docker container (see example below):
  - `--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev`
  - The *insecure*, ready-to-go option is: `--privileged -v /dev/bus/usb:/dev/bus/usb`

- See also: https://stackoverflow.com/questions/24225647/docker-a-way-to-give-access-to-a-host-usb-or-serial-device

#### Enable GPU-accelerated Containers

- Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

### Building the Docker Image

```shell
$ docker build -t cf-pyctrl -f .devcontainer/Dockerfile .
```

> **Note:** If you encounter a "permission denied" error, you may need to run `docker` with `sudo`.

## Usage

**First, allow the Docker Container to Access to X Server (GUI):**

```shell
xhost +local:$USER
```

Revert the `xhost` settings to their original state afterward: `xhost -`

### Running the Docker Container

**Standard**

```shell
# With USB devices
sudo docker run --rm -it \
--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
--net=host --ipc=host --pid=host \
--env="DISPLAY" \
--env="XAUTHORITY=$XAUTHORITY" \
--volume="$XAUTHORITY:$XAUTHORITY" \
--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
cf-pyctrl
```

> **Note:** In `c 189:* rmw`, replace the `189` with the **major device number** you obtained in the [Allowing USB](#allowing-usb) section.


**Development**

To sync folders add the following argument (example):

```shell
# Enable "Dev-Mode"
--mount type=bind,source=$(pwd)/src,target=/home/user/dev_ws/libs/crazyflie-controller/src  \
# Add additional example scripts for testing
--mount type=bind,source=$(pwd)/examplescripts,target=/home/user/dev_ws/libs/crazyflie-controller/examples  \
--mount type=bind,source=$(pwd)/lpstest,target=/home/user/dev_ws/libs/crazyflie-controller/examples  \
```

