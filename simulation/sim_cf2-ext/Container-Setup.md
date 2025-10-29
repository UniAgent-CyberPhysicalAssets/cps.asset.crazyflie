# Container Workspace Setup: sim_cf2

This document describes the workspace installation procedure for the [ROS2 package sim_cf2](https://github.com/CrazyflieTHI/sim_cf2).

The following components are included:

- [sim_cf2](https://github.com/CrazyflieTHI/sim_cf2)
- [cf.PyControl](https://github.com/UniAgent-CyberPhysicalAssets/cps.asset.crazyflie/tree/main/controller/cf.PyControl)
- [crazyflie-firmware with SITL](https://github.com/CrazyflieTHI/crazyflie-firmware)

> [!Note]
>
> This setup does not work for arm64 platforms since not for all ROS2 packages an arm64 binary is available. For example, "ros-humble-gazebo-no-physics" has no arm64 binary:
>
> - http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages
> - http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-arm64/Packages

## Installation

### Preparation

This section explains the one-time setup you need to complete before building the image and getting started.

#### Get the cf.PyControl Controller Service

```shell
$ chmod +x ./prepare.sh
$ ./prepare.sh

```

#### Download and Upgrade cflib

```shell
$ chmod +x ./sync-upstream.sh
$ ./sync-upstream.sh
```


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
$ docker build --network=host -t cf2_ros2_sim -f .devcontainer/Dockerfile .
```

## Usage 

**First, allow the Docker Container to Access to X Server (GUI):**

```shell
$ xhost +local:$USER
```

Revert the `xhost` settings to their original state afterward: `xhost -`

### Running the Docker Container

Run the Docker container as follows.
This command also makes the USB devices available inside the container:

```shell
$ sudo docker run --rm -it \
--device-cgroup-rule='c 189:* rmw' -v /run/udev:/run/udev:ro -v /dev:/dev \
--env ROS_DOMAIN_ID=30 \
--net=host --ipc=host --pid=host \
--env="DISPLAY" \
--env="XAUTHORITY=$XAUTHORITY" \
--volume="$XAUTHORITY:$XAUTHORITY" \
--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
cf2_ros2_sim
```

**Configuration:**

- To remove GPU-acceleration, omit `--gpus`
- Without sharing usb devices, omit `--device-cgroup-rule`
- To expose endpoints while not using `--net=host`, simply add `-p`: `-p 5000:5000 -p 8765:8765 \`

**Copy files to the container:**

```shell
$ docker ps --filter "ancestor=cf2_ros2_sim:latest" --format "{{.Names}}"
[...]

# containerName=
$ docker cp ./sim_cf2/launch/main.launch.xml containerName:/home/user/dev_ws/ros2/src/sim_cf2/launch
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

