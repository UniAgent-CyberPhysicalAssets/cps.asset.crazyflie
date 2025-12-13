# Brief Usage Overview

## Prerequisites

### Docker

- https://docs.docker.com/compose/install/linux/
- https://dynamicswarms.github.io/ds-crazyflies/docker.html

### Prepare

```shell
$ cd ~/uniagent_ws/cps-asset/crazyflie2.x/simulation/ds-crazyflies-ext
$ ./prepare
```

### Build

```shell
$ cd simulation.ds-crazyflies/docker/
$ sudo docker compose build
```

## Usage

```shell
# Run this command once to enable GUI display:
$ xhost +local:$USER
```

####  1. Terminal

```shell
$ cd ~/uniagent_ws/cps-asset/crazyflie2.x/simulation/ds-crazyflies-ext/simulation.ds-crazyflies/docker/
$ sudo docker compose up
```

#### 2. Terminal

```shell
$ sudo docker exec -it ds-crazyflies-dev bash

$ webots /ds/crazywebotsworld/worlds/crazyflie.wbt --batch

$ nano /ds/crazywebotsworld/worlds/crazyflie.wbt
```

####  3. Terminal

```shell
sudo docker exec -it ds-crazyflies-dev bash

ros2 launch crazyflies framework.launch.py
ros2 launch crazyflies framework.launch.py backend:=webots
ros2 launch crazyflies framework.launch.py radio_channels:=[80] backend:=hardware
ros2 launch crazyflies framework.launch.py radio_channels:=[80] backend:=both
```

Check Crazyradio:

```shell
$ lsusb
$ ls -l /dev/bus/usb/003/010
$ lsusb -v -d 1915:7777
```

#### 4. Terminal (3 Options)

```shell
$ sudo docker exec -it ds-crazyflies-dev bash
```

**Webots**

```shell
$ ros2 service call /crazyflie_webots_gateway/add_crazyflie crazyflie_webots_gateway_interfaces/srv/WebotsCrazyflie "id: 0"
```

**Hardware**
```shell
$ ros2 service call /crazyflie_hardware_gateway/add_crazyflie \
crazyflie_hardware_gateway/srv/AddCrazyflie "{id: 0, channel: 80, initial_position: [0.0, 0.0, 0.0], type: 'default'}"
$ ros2 service call /crazyflie_hardware_gateway/add_crazyflie crazyflie_hardware_gateway_interfaces/srv/AddCrazyflie "{id: 0, channel: 80, initial_position: {x: 0.0, y: 0.0, z: 0.0}, type: 'default'}"
```

**GUI (Manual)**
```shell
$ rqt --force-discover
```


#### Commands

```shell
$ ros2 topic pub --once /cf0/takeoff crazyflie_interfaces/msg/Takeoff "{group_mask: 0, height: 0.5, yaw: 0.0, use_current_yaw: false, duration: {sec: 2, nanosec: 0}}"
```

```shell
$ ros2 topic pub --once /cf0/go_to crazyflie_interfaces/msg/GoTo "{group_mask: 0, relative: true, linear: false, goal: {x: 0.5, y: 0.5, z: 0.0}, yaw: 0.0, duration: {sec: 2, nanosec: 0}}"
$ ros2 topic pub --once /cf0/go_to crazyflie_interfaces/msg/GoTo "{group_mask: 0, relative: false, linear: false, goal: {x: 2.0, y: 1.0, z: 0.5}, yaw: 0.0, duration: {sec: 2, nanosec: 0}}"
$ ros2 topic pub --once /cf0/cmd_position crazyflie_interfaces/msg/Position "{x: 0.0, y: 1.0, z: 0.5, yaw: 1.0}"
```

```shell
ros2 topic pub --once /cf0/land crazyflie_interfaces/msg/Land "{group_mask: 0, height: 0.0, yaw: 0.0, use_current_yaw: false, duration: {sec: 2, nanosec: 0}}"
```

```shell
ros2 topic echo /cf_positions_poses --qos-reliability best_effort
```