# Crazyswarm2 Simulator Environment

## Installation

- Refer to the [container workspace setup](Container-Setup.md) documentation

## Usage

### Starting the crazyflie-server

This Python script launches the crazyflie-server, which is necessary for the communication between the Crazyflie and the simulator:

```shell
ros2 launch crazyflie start_crazyflie_server.launch.py backend:=cflib
```

> [!IMPORTANT]
>
> The Crazyradio dongle must be attached to the USB port.
>
> Refer to the [container workspace setup](Container-Setup.md) documentation for allowing USB.

Adjust drone parameters in the config files:

- `dev_ws/ros2/src/crazyswarm2/crazyflie/config/crazyflies.yaml`

> [!NOTE]
>
> When this server is started, cf.PyControl (a web-based high-level controller for Crazyflie drones) can be used with the crazyswarm2 simulator over its ROS2 interface.

**Example**

List available Crazyflie commands voa ROS2 Services:

```shell
ros2 service list
```

Send takeoff command: 

```shell
ros2 service call /cf231/takeoff crazyflie_interfaces/srv/Takeoff "{height: 0.2, duration: {sec: 2}}"
```

Send landing command:

```shell
ros2 service call /cf231/land crazyflie_interfaces/srv/Land "{height: 0.0, duration: {sec: 2}}"
```

### Simple Mapping Simulation

- Container A:
  - `ros2 launch crazyflie_ros2_multiranger_bringup simple_mapper_simulation.launch.py`
- Container B:
  - `ros2 run teleop_twist_keyboard teleop_twist_keyboard`

### Wall following simulation

- Container A:
  - `ros2 launch crazyflie_ros2_multiranger_bringup wall_follower_mapper_simulation.launch.py`
  - Stop Crazyflie: `ros2 service call /crazyflie/stop_wall_following std_srvs/srv/Trigger`



### Gazebo Velocity Control

- see [Gazebo Velocity Control](https://www.bitcraze.io/documentation/repository/crazyflie-simulation/main/user_guides/gazebo_velocity_control/)

```
gz sim -r worlds/crazyflie_world.sdf
```

- Find the teleop plugin in the Gazebo GUI
- Subscribe to `/crazyflie/gazebo/command/twist`