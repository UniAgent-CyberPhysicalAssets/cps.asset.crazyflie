# Container Workspace Setup: sim_cf2

This document describes the workspace installation procedure for the ROS2 package sim_cf2.

The following components are included:

- sim_cf2
- cf.PyControl
- crazyflie-firmware with SITL

> [!Note]
>
> This setup does not work for arm64 platforms since not for all ROS2 packages an arm64 binary is available. For example, "ros-humble-gazebo-no-physics" has no arm64 binary:
>
> - http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages
> - http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages 

## Usage

See [Readme](Readme.md)

## Installation

### Preparation

**Get the cf.PyControl controller service:**

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

### Build the Docker Image

```shell
docker build --network=host -t cf2_ros2_simu -f .devcontainer/Dockerfile .
```

### Running the Docker Container

This makes also the USB devices available in the container:

```shell
sudo docker run --rm -it \
--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
--env ROS_DOMAIN_ID=30 \
--net=host --ipc=host --pid=host \
--env DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
cf2_ros2_simu
```

Without sharing usb devices:

```shell
sudo docker run --rm -it \
--env ROS_DOMAIN_ID=30 \
--net=host --ipc=host --pid=host \
--env DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
cf2_ros2_simu
```

To expose endpoints while not using `--net=host`, simply add `-p`:

```shell
-p 5000:5000 -p 8765:8765 \
```

Copy files to the container

```shell
#containerName=
docker ps --filter "ancestor=cf2_ros2_simu" --format "{{.Names}}"
docker cp ./sim_cf2/launch/main.launch.xml containerName:/home/user/dev_ws/ros2/src/sim_cf2/launch
# Example:
docker cp ./sim_cf2/launch/crazyflies.yaml elated_hellman:/home/user/dev_ws/ros2/src/sim_cf2/launch
```



**Environment Variables:**

On the host machine and in the Docker container, check that the following environment variables are set:

```shell
export ROS_DOMAIN_ID=30  # Ensure the same domain ID is used in both host and container 
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET # is the default, and for DDS based middleware it means it will discover any node reachable via multicast.
```

**Test communication between Host and Docker:**

- HOST: `ros2 topic pub /example_topic std_msgs/String "data: 'Hello from host'"`

- CONTAINER: `ros2 topic echo /example_topic`

> [!IMPORTANT]
>
> If the environment variables are set and no communication is possible between the host and the container try to check that both the host and Docker container are using the same DDS implementation and that there are no configuration issues.
>
> For example, this will not work: The Docker container uses ROS2 Humble and the host uses ROS2 Jazzy.

