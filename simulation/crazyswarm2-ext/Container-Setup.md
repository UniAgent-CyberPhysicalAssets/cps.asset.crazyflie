# Container Workspace Setup: crazyswarm2

This document describes the workspace installation procedure for the ROS2 package crazyswarm2

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
docker build -t crazyswarm2_simu -f .devcontainer/Dockerfile .
```

## Running the Docker Container

```shell
docker run --rm -it \
--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
--net=host --ipc=host --pid=host \
--env ROS_DOMAIN_ID=30 \
--env="DISPLAY" \
--env="XAUTHORITY=$XAUTHORITY" \
--volume="$XAUTHORITY:$XAUTHORITY" \
--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
crazyswarm2_simu
```

**Environment Variables:**

On the host machine and in the Docker container, check the following environment variables:

```shell
export ROS_DOMAIN_ID=30  # Ensure the same domain ID is used in both host and container 
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET # is the default, and for DDS based middleware it means it will discover any node reachable via multicast.
```

**Test communication between Host and Container:**

- Demo 1:
  - HOST: ros2 topic pub /example_topic std_msgs/String "data: 'Hello from host'"

  - CONTAINER: ros2 topic echo /example_topic

- Demo 2:
  - ros2 run demo_nodes_cpp listener

  - ros2 run demo_nodes_cpp talker


> [!IMPORTANT]
>
> If the environment variables are set and no communication is possible between the host and the container try to check that both the host and Docker container are using the same DDS implementation and that there are no configuration issues.
>
> For example, this will not work: The Docker container uses ROS2 Humble and the host uses ROS2 Jazzy.
