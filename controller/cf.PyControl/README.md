# cf.PyControl: A Terminal-based RESTful Controller for the Crazyflie 2.X

**cf.PyControl** is a Terminal-based **Web Service** written in **Python** for the **Crazyflie 2.X** used in combination with the **FlowDeck** and/or the **Loco Position System (LPS)**.

The high-level actions of the drone are internally managed by a [*State Machine*](Development.md#drone-software-controller-specification) and can be triggered via [*Web Endpoints*](#basic-operations).

**tl;dr;**

- cf.pycontrol-start.sh --uri radio://0/80/2M/E7E7E7E7E1 --port 5000
- cf.pycontrol-start.sh --uri radio://0/80/2M/E7E7E7E7E1 --port 5000 --wsendpoint --wsport 8765
- cf.pycontrol-start.sh --uri radio://0/80/2M/E7E7E7E7E2 --port 5001
- curl -d {} http://127.0.0.1:5000/activate_idle && curl -d {} http://127.0.0.1:5001/activate_idle
- curl -d {} http://127.0.0.1:5000/begin_takeoff
- curl -d {} http://127.0.0.1:5000/begin_landing

**Screenshot**

<img src="docs/cf-pycontrol-terminal-screenshot.jpg" alt="" style="zoom: 54%;" />

## Installation

> [!NOTE]
>
> (Default) Start quickly using the Docker image — see [Container-Setup.md](Container-Setup.md).
>
> (For Developers) For instructions on setting up the local workspace for cf.PyControl and getting started with development — see [Development.md](Development.md).

## Getting Started

> [!NOTE]
>
> This example doesn’t directly control the Crazyflie 2.x — it’s a quick demo to illustrate the intended usage.

Start the Crazyflie control service with the following argument:

```shell
$ cf.pycontrol-start.sh --debug --uri radio://0/80/2M/E7E7E7E7E1
```

The shell script is located in the `bin` folder.

> [!NOTE]
>
> You may need to update the radio URI.
>
> The default is usually: `radio://0/80/2M/E7E7E7E7E1`
>
> For details, see the standard setup guide at [bitcraze.io](https://www.bitcraze.io/).

Open the `webview.html` in the `webview` folder to see the state machine updating live.

Therefore, open another terminal and change the states (or, run another Docker container):

```shell
$ curl http://127.0.0.1:5000/status
$ curl -d {} http://127.0.0.1:5000/activate_idle
$ curl http://127.0.0.1:5000/status
```

Now, view the live state machine overview in your browser.

The following sections:
  - Explain available drone actions via the REST API
  - Describe advanced use cases
  - Show how to configure the service

## Basic Operations

Technically, this service implements the *state machine pattern* to model and execute basic drone operations such as **TakeOff**, **Landing**, **Navigate**, and so forth. 
The drone actions are accessible via a RESTful API, which makes it easy to integrate cf.PyControl with other systems or user interfaces. 
The state machine of the Crazyflie 2.X in cf.PyControl is specified as described in ["Drone Software Controller Specification"](Development.md#drone-software-controller-specification). 
Note that this is a custom design choice. 
It is general enough to be used in different use cases and applications.

**Before You Start**

It is recommended to check if everything is fine with the Crazyflie
- Connect the Crazyradio 2.0 to a USB port
- Switch on the Crazyflie 2.X drone
- Start the Crazyflie Client: `cfclient`


### Activate the Drone

This is usually the <u>first command</u> you need to run to allow the Crazyflie to fly.

The security lock is released that prevents accepting commands right after the drone was switched on.

```shell
curl -d {} http://127.0.0.1:5000/activate_idle
```

### Drone Mission: NavigateToTarget (x,y,z)

```shell
curl -d {} http://127.0.0.1:5000/navigate/x/y/z
```

Example:

```shell
curl -d {} http://localhost:5000/navigate/1.0/1.0/0.5
```

Moves to `(1, 1, 0.5)` from the current position and then hovers.

Ensure you have enough space.

- With the Flow deck, the initial position is approximately `(0, 0, 0)` on the ground, when the drone is started.
- With LPS, the initial position depends on the LPS node system’s origin and the drone’s placement.

> **Note:**
> You can use either the `kalmanEstimate` or `stateEstimate` to obtain the position.
> Accuracy depends on the positioning system in use—both LPS and Flow deck support this.
> This can be changed in `cf-ctrl-service.py` by changing the global variable `POSITION_ESTIMATE_FILTER`.

### Drone State Updates via WebSocket Endpoint

With [websocat](https://github.com/vi/websocat) installed on your system, you can use the following command to test the WebSocket endpoint of cf.PyControl:
```shell
$ websocat ws://localhost:8765
```

It provides you with live updates of the drone’s state (e.g., position).
Some output will be also visible in the terminal where cf.PyControl is running.

Therefore, the controller must be started with the `--wsendpoint --wsport 8765` flag.
The last argument is the port of the WebSocket endpoint, which can be changed.

### Drone State Updates via WebView


✅ Option 1: Use a Bind Mount (if you know the folder beforehand)

This is the cleanest way if you can plan ahead.

Step 1: Create a host directory for the shared data

mkdir -p /host/path/for/container-data

Step 2: Start the container with a bind mount pointing the container's folder to that host directory

docker run -v /host/path/for/container-data:/container/folder busybox

    This will mirror changes from /container/folder back to /host/path/for/container-data.

If your container is already running, this won’t help unless you restart it.

## Composed Operations

#### TakeOff - and - Landing

- You can initiate the simple drone mission "TakeOff-Landing" as follows by composing HTTP POST requests in a row (order is important)

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

- 4 target positions: 

- Hovering->Flying-Hovering->Flying->Hovering->Flying-Hovering->Flying->Hovering->Landed

```shell
curl -d {} http://127.0.0.1:5000/activate_idle && \
curl -d {} http://127.0.0.1:5000/begin_takeoff && \
curl -d {} http://127.0.0.1:5000/navigate/0.3/0.0/0.4 && \
curl -d {} http://127.0.0.1:5000/navigate/0.0/0.0/0.4 && \
curl -d {} http://127.0.0.1:5000/navigate/0.0/0.3/0.4 && \
curl -d {} http://127.0.0.1:5000/navigate/0.0/0.0/0.4 && \
curl -d {} http://127.0.0.1:5000/begin_landing
```

- Each target is represented as one Flying state

#### Navigate a path of multiple goals without pause, and land

```shell
curl -d {} http://127.0.0.1:5000/activate_idle && \
curl -d {} http://127.0.0.1:5000/begin_takeoff && \
curl -d {} http://127.0.0.1:5000/navigate/append/0.3/0.3/0.6
curl -d {} http://127.0.0.1:5000/navigate/append/0.2/0.2/0.6
curl -d {} http://127.0.0.1:5000/navigate/append/0.1/0.1/0.6
curl -d {} http://localhost:5000/navigate/0.0/0.0/0.4 && \
curl -d {} http://127.0.0.1:5000/begin_landing
```

**Hovering → Flying → Landed**

All targets are managed within a single Flying state.

Coordinates are appended to the drone’s navigation queue.
 These targets are processed when calling the `/navigate/x/y/z` endpoint, similar to posting the full JSON coordinate object described earlier.

- The first POST to the navigation endpoint sets the initial goal.
- Subsequent POSTs update the final target coordinate.

Using this consecutive approach, you’ll notice the drone hovers briefly upon reaching each target before proceeding to the next—effectively “idling” between waypoints.

```shell
curl -X post http://127.0.0.1:5000/activate_idle && \
curl -X post http://127.0.0.1:5000/begin_takeoff && \
curl -X post http://localhost:5000/navigate/0.3/0.3/0.6 && \
curl -X post http://localhost:5000/navigate/0.0/0.0/0.4 && \
curl -X post http://127.0.0.1:5000/begin_landing
```


## Controller Configuration

**Change Port:**

- use `--port` for web server (UAV operations)
- use `--wsport` for web socket server that publishes state information about the drone (e.g., position)

[//]: # (Change Name of drone &#40;important for multi-UAV scenarios&#41;: )

**Mode:** 

- `--sim` when using with sim_cf2
- `--debug` for more debug and verbose output in the terminal

## Error Handling

If the state machine cannot start, this can have several reasons:

- Your system environment is incomplete (e.g., missing packages) or not suited for this library (e.g., unsupported OS)
- The Crazyflie 2.x itself reports problems (perform a health check, battery/propeller test, check Crazyflie Radio USB)

What usually helps is the following:

- Try to disconnect crazyflie, Crazyradio first. Then reconnect the antenna first to the host computer, then connect CF via usb, switch it on, start the cfclient and try to connect, check battery and then disconnect. Switch off the Crazyflie and unplug the USB from it. 
- Place the Crazyflie somewhere with enough space, switch it on, then start the controller again.

## License

**cf.PyControl** is Open Source software released under the Apache 2.0 license.

## Acknowledgment

This tool is developed and maintained by the UniAgent Developers and contributors:
- Thanks to Mikhail Belov for providing the initial version of this script and the webview for the internal state machine.
- Thanks to Tianxiong Zhang for continuous testing.

---

Copyright © 2025 The UniAgent Developers and Contributors.