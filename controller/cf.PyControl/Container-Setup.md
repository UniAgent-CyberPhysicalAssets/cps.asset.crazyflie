# Container Workspace Setup: cf.PyControl

This document describes how to create and use a containerized environment with the following components installed:

- cf.PyControl: high-level Python controller for the Crazyflie
-  cfclient (LPS GUI not working though)

## Installation

### Preparation

**Allowing USB:**

Make Crazyradio available in Docker container

- Use the `lsusb` command to get a list of all USB devices connected to your system. Each line in the `lsusb` output represents a USB device with a Bus number and Device number.

- Use `udevadm` to get detailed information about the device, which includes the associated `/dev` entry.
  Below replace `001` with the Bus number and `009` with the Device number from the `lsusb` output. Specifically, use the output of `DEVNAME`.
- Get device major number via `ls -la`using information from `DEVNAME`

```shell
lsusb
sudo udevadm info --query=all --name=/dev/bus/usb/001/009
ls -la /dev/bus/usb/001/009
# Output
crw-rw-r-- 1 root plugdev 189, 8 Aug  8 22:38 /dev/bus/usb/001/009
```

- The major device number is in this example "189"
  
- Add respective rules when starting the Docker container (see below)
  - the insecure option is: `--privileged -v /dev/bus/usb:/dev/bus/usb`

- See also: https://stackoverflow.com/questions/24225647/docker-a-way-to-give-access-to-a-host-usb-or-serial-device

### Building the Docker Image

```shell
docker build -t hlc-cf2 -f .devcontainer/Dockerfile .
```

## Usage

### Allow access to X Server

```shell
xhost +local:root
```

Revert the `xhost` settings to their original state afterward:

```shell
xhost -
```

### Running the Docker Container

```shell
# With USB devices
sudo docker run --rm -it \
--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
--net=host --ipc=host --pid=host \
--env="DISPLAY" \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
hlc-cf2
```

To sync folders add the following argument (example):

```shell
# dev-mode
--mount type=bind,source=$(pwd)/src,target=/home/user/dev_ws/libs/crazyflie-controller/src  \
# add additional example scripts for testing
--mount type=bind,source=$(pwd)/examplescripts,target=/home/user/dev_ws/libs/crazyflie-controller/examples  \
```

