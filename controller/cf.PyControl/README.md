# cf.PyControl: A RESTful API to control the Crazyflie 2.X 

**tl;dr;** This Python package provides a web service to control a Crazyflie 2.X. The high-level actions of the CF drone are internally managed by a StateChart and can be triggered via web endpoints.

----

cf.PyControl is a web service written in Python for the Crazyflie 2.X used in combination with the FlowDeck and/or the Loco Position System (LPS).
The service implements the *state machine pattern* using the StateChart formalism to model and execute basic UAV operations such as TakeOff, Landing, Navigate, BatteryCheck, MotorCheck, and so forth.

The state machine, i.e., the drone actions, is accessible via a RESTful API (see [section](#controlling-the-state-machine)), which makes it easy to integrate cf.PyControl with other systems or user interfaces.

In this project, the state machine of the Crazyflie 2.X

[^1]: A state machine is a model that describes the behavior of the Crazyflie 2.X using the notion of states and transitions. Each transition is labeled with an action. Thus, each transition maps to an action, which represents and implements the specific low-level code of the cflib Python library for this specific state and transition of the drone.

 is modeled as described in [section](#drone-system-specification). Note that this is a custom design choice. It is general enough to be used in different use cases and applications.

----

**tldr (2);**

- cf.pycontrol-start.sh --uri radio://0/80/2M/E7E7E7E7E1 --port 5000
- cf.pycontrol-start.sh --uri radio://0/80/2M/E7E7E7E7E2 --port 5001
- cf.pycontrol-start.sh --uri radio://0/80/2M/E7E7E7E7E3 --port 5002
- curl -d {} http://127.0.0.1:5000/activate_idle && curl -d {} http://127.0.0.1:5001/activate_idle && curl -d {} http://127.0.0.1:5002/activate_idle
- curl -d {} http://127.0.0.1:5000/begin_takeoff
- curl -d {} http://127.0.0.1:5000/begin_landing

----

## Getting Started

> [!NOTE]
>
> The following example does not control the Crazyflie 2.x directly but serves as a quick demonstration of the intended usage.

Start the Crazyflie control service with the following argument:

```shell
cf.pycontrol-start.sh --debug --uri radio://0/80/2M/E7E7E7E7E1
```

The shell script is located in the `bin` folder.

> [!NOTE]
>
> You may need to change the Radio address. 
>
> Default usually is: radio://0/80/2M/E7E7E7E7E1
>
> Please refer to the standard setup as described at bitcraze.io.

Open the `webview.html` in the `webview` folder to see the state machine updating live.

Therefore, open another terminal and change the states:

```shell
curl http://127.0.0.1:5000/status
curl -d {} http://127.0.0.1:5000/activate_idle
curl http://127.0.0.1:5000/status
```

See the live update of the statemachine overview in the browser.

The following sections:

- Explain available UAV actions via the REST API
- Describes more advanced use cases
- How the service can be configured

### Overview

- If you request the specific URL, you will see that the live update is working (iff the transition can be actually fired).

- Go to "Basic Operations" and "Composed Operations" to learn all the details and effects when initiating drone actions such as takeoff, landing and navigation.

The RESTful HTTP interface of cf.PyControl offers the following actions:

### Basic Operations

#### Activate the Drone

This is usually the first command you need to run to allow the Crazyflie to fly.

The security lock is released that prevents accepting commands right after the drone was switched on.

```shell
curl -d {} http://127.0.0.1:5000/activate_idle
```

#### Drone Mission: NavigateToTarget (x,y,z)

```shell
curl -X post http://127.0.0.1:5000/navigate/x/y/z
```

Example:

```shell
curl -X post http://localhost:5000/navigate/1.0/1.0/0.5
```

Moves to (1,1,0.5) from the current position and keeps hovering afterward. 

Make sure you have enough space. 
If Flow deck is used, the initial position is roughly 0,0,0 when on the ground.
When LPS is used, the initial position depends on the origin of the LPS node system and where the drone is placed within the space.


The kalmanEstimate or stateEstimate can be used to get the position. 
The accuracy depends on the positioning system used. 
With LPS or Flow deck possible.

### Composed Operations

#### TakeOff - and - Landing

You can initiate the simple drone mission "TakeOff-Landing" as follows by composing HTTP POST requests in a row (order is important)

```shell
curl -d {} http://127.0.0.1:5000/activate_idle && \
curl -d {} http://127.0.0.1:5000/begin_takeoff && \
curl -d {} http://127.0.0.1:5000/begin_landing
```

#### NavigateToGoal - and - Landing

```shell
curl -d {} http://127.0.0.1:5000/activate_idle && \
curl -d {} http://127.0.0.1:5000/begin_takeoff && \
curl -d {} http://localhost:5000/navigate/0.2/0.2/0.6 && \
curl -d {} http://127.0.0.1:5000/begin_landing
# And back
curl -d {} http://127.0.0.1:5000/activate_idle && \
curl -d {} http://127.0.0.1:5000/begin_takeoff && \
curl -d {} http://localhost:5000/navigate/0.0/0.0/0.4 && \
curl -d {} http://127.0.0.1:5000/begin_landing
```

#### Navigate to multiple separated goals, each interrupted by a short hovering, and land

4 target positions: 

Hovering->Flying-Hovering->Flying->Hovering->Flying-Hovering->Flying->Hovering->Landed

```shell
curl -X post http://127.0.0.1:5000/activate_idle && \
curl -X post http://127.0.0.1:5000/begin_takeoff && \
curl -X post http://127.0.0.1:5000/navigate/0.3/0.0/0.4 && \
curl -X post http://127.0.0.1:5000/navigate/0.0/0.0/0.4 && \
curl -X post http://127.0.0.1:5000/navigate/0.0/0.3/0.4 && \
curl -X post http://127.0.0.1:5000/navigate/0.0/0.0/0.4 && \
curl -X post http://127.0.0.1:5000/begin_landing
```

Each target is represented as one Flying state

#### Navigate a path of multiple goals without pause, and land

```shell
curl -X post http://127.0.0.1:5000/activate_idle && \
curl -X post http://127.0.0.1:5000/begin_takeoff && \
curl -X post http://127.0.0.1:5000/navigate/append/0.3/0.3/0.6
curl -X post http://127.0.0.1:5000/navigate/append/0.2/0.2/0.6
curl -X post http://127.0.0.1:5000/navigate/append/0.1/0.1/0.6
curl -X post http://localhost:5000/navigate/0.0/0.0/0.4 && \
curl -X post http://127.0.0.1:5000/begin_landing
```

Hovering->Flying->Landed

All targets are represented as one Flying state.

First, appending a coordinate to the navigation queue of the drone. 
This will be finally considered when requesting the /navigate/x/y/z endpoint.
This is similar to posting the whole JSON coordinate data object as described above.
The first POST request of the navigate append endpoint is the first goal the drone will target. 
The last POST request to the /navigate/x/y/z endpoint is the final coordinate the drone will target.

And observe again with this consecutive approach. 
Since the drone goes to the hovering state after reaching a target, the drone "idles" a bit longer before going to the next goal:

```shell
curl -X post http://127.0.0.1:5000/activate_idle && \
curl -X post http://127.0.0.1:5000/begin_takeoff && \
curl -X post http://localhost:5000/navigate/0.3/0.3/0.6 && \
curl -X post http://localhost:5000/navigate/0.0/0.0/0.4 && \
curl -X post http://127.0.0.1:5000/begin_landing
```


### Controller Configuration

Change Port:

- use --port for web server (UAV operations)
- use --wsport for web socket server that publishes state information about the drone (e.g., position)

[//]: # (Change Name of drone &#40;important for multi-UAV scenarios&#41;: )

Mode: 

- `--sim` when using with sim_cf2
- `--debug` for more debug and verbose output in the terminal

### Error Handling

If the state machine cannot start, this can have several reasons:

- Your system environment is incomplete (e.g., missing packages) or not suited for this library (e.g., unsupported OS)
- The Crazyflie 2.x itself reports problems (perform a health check, battery/propeller test, check Crazyflie Radio USB)

What usually helps is the following:

- Try to disconnect crazyflie, Crazyradio first. Then reconnect the antenna first to the host computer, then connect CF via usb, switch it on, start the cfclient and try to connect, check battery and then disconnect. Switch off the Crazyflie and unplug the USB from it. 
- Place the Crazyflie somewhere with enough space, switch it on, then start the controller again.

## Drone System Specification


The version of the state machine employed in this Python package can be used for many use cases. 

**Visualisation**

| Figure                                                       | PlantUML                                                     |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| <img src="docs/cf_statemachine_model.svg" alt="cf-sm-model" style="zoom: 67%;" /> | @startuml<br/><br/><br/>state UAVSystem {<br/>    [*] --> OSGiLifecycle<br/>    state OSGiLifecycle {<br/>        [*] --> INSTALLED<br/>        INSTALLED --> RESOLVED : Dependencies resolved<br/>        RESOLVED --> STARTING : Start command issued<br/>        STARTING --> ACTIVE : Initialization complete<br/>        ACTIVE --> STOPPING : Stop command issued<br/>        STOPPING --> RESOLVED : Stopped successfully<br/>        RESOLVED --> UNINSTALLED : Uninstall command issued<br/>    }<br/><br/>    state UAVOperation {<br/>        Idle --> TakeOff : TakeOff command issued<br/>        TakeOff --> Flying : TakeOff complete<br/>        Flying --> Landing : Landing command issued<br/>        Landing --> Idle : Landing complete<br/>        Idle --> Shutdown : Shutdown command issued<br/>        Shutdown --> STOPPING : Begin stopping lifecycle<br/>    }<br/><br/>    ACTIVE --> Idle<br/>}<br/><br/>@enduml |



### Design Aspects

- UAV System (Parent State) contains two interrelated orthogonal regions
- They manage the software lifecycle and the physical state of the UAV

**Explanation of Transitions**

1. **OSGi Lifecycle**: This region manages the states related to the UAV's software bundle lifecycle.
   - Transitions between **INSTALLED**, **RESOLVED**, **STARTING**, **ACTIVE**, **STOPPING**, and **UNINSTALLED** follow the OSGi lifecycle.
2. **UAV Operation**: This region manages the physical state of the UAV.
   - **Idle** to **TakeOff**: Triggered by a takeoff command.
   - **TakeOff** to **Flying**: Occurs when the takeoff is complete.
   - **Flying** to **Landing**: Triggered by a landing command.
   - **Landing** to **Idle**: Occurs when the landing is complete.

### uav.OSGiLifecycle: Drone Software Bundle Life-cycle (Region)

##### States

The OSGi (Open Services Gateway initiative) lifecycle states for bundles typically include:

1. **INSTALLED**: The bundle has been successfully installed.
   1. Means: The UAV software bundle is installed but not yet resolved.; The service is generally available, cflib installed, is Java available, ROS Etc.
2. **RESOLVED**: All necessary dependencies have been resolved.
   1. Means: The UAV software dependencies are resolved, and it is ready to be started; the dependency requirements are checked, version compatability etc, and when required is updated.
3. **STARTING**: The bundle is in the process of starting.
   1. Means: The UAV is initializing; the required Decks are available; battery is checked; propellers etc.; Use-case specific pre-starting procedure; Logging; default noise measurement for static position (a simple heuristic is used to determine if the drone is flying based on acceleration)
4. **ACTIVE**: The bundle is running and active.
   1. Means: The UAV is fully operational; Logging (print last checkup values), Use-case specific pre-starting procedure;
   2.  transition initializes: prepares logging of values, prepares ROS2 messages, etc too
5. **STOPPING**: The bundle is in the process of stopping.
6. **UNINSTALLED**: The bundle has been uninstalled.

##### Transitions

- **INSTALLED** --> **RESOLVED**: Dependencies resolved
- **RESOLVED** --> **STARTING**: Start command issued
- **STARTING** --> **ACTIVE**: Initialization completes; webserver started, rest interface ready, logging ros setup
- **ACTIVE** --> **STOPPING**: Stop command issued
- **STOPPING** --> **RESOLVED**: Stopped successfully
- **RESOLVED** --> **UNINSTALLED**: Uninstall command issued

### uav.Operation: Drone-Specific Operation Life-cycle (Region)

##### States

In addition to these, we'll introduce UAV-specific states for operational control:

1. **IDLE**: The UAV is powered on, health checks are fine, battery is full, but motors not yet active. It is on the ground
2. **HOVERING**: The UAV is in the air and operational. the UAV is hovering at specific height // The UAV is in the process of taking off.
3. **FLYING**: The UAV is flying a manouvur or navigating to a goal, or follows a movement/swarm pattern.
4. **LANDED**: The UAV has landed after being in the process of landing.

- **Idle**: Start or End; Continuous states; loops here; Landing complete; or any other previous task was completed
- **HOVERING**: Command to take off issued; Continuous states; loops here
- **Flying**: Take off completeM Continuous states; loops here
- **Landing**: Command to land issued; Continuous states; loops here

[//]: # (##### Transitions)
