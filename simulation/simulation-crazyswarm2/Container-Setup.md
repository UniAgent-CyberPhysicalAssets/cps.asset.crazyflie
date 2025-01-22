# Container Workspace Setup: crazyswarm2

This document describes the workspace installation procedure for the ROS2 package crazyswarm2

## Preparation

**Get the cf.PyControl Python service:**

```shell
chmod +x ./prepare.sh
./prepare.sh
```

**Allow the Docker Container to Access X Server:**

```shell
xhost +local:root
```

Revert the `xhost` settings to their original state afterward:

```shell
xhost -
```

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

## Building the Docker Image

```shell
docker build -t crazyswarm2_simu -f .devcontainer/Dockerfile .
```

## Running the Docker Container

```shell
sudo docker run --rm -it \
--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
--net=host --ipc=host --pid=host \
--env ROS_DOMAIN_ID=30 \
--env DISPLAY \
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
