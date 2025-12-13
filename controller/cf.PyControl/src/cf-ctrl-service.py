#!venv/bin/python
import argparse
import asyncio
import json
import logging
import threading
import time

import cflib.crtp
import websockets
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper
from flask import Flask, jsonify, request
from rich.console import Console, Group
from rich.live import Live
from rich.table import Table
from rich.text import Text
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine
from werkzeug.routing import BaseConverter, ValidationError

from routes import drone_blueprint  # Import the blueprint

try:
    # Optional ROS import. rclpy may not be available on systems that do not use ds-crazyflie.
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
except Exception:
    rclpy = None

import cf_positioning
from cf_positioning import KalmanEstimatePositionStrategy, \
    StateEstimatePositionStrategy
from cf_positioning import RosPoseArrayPositionStrategy
from cf_drone_ops import HlCommanderCFOperationImpl, RosTopicCFOperationImpl
import cf_sm
from routes import FloatConverter


# ######################################################################################################################

def create_arg_parser():
    # Create the parser.
    parser = argparse.ArgumentParser(description='Crazyflie 2.x Drone Controller WebService')

    # Add the arguments.
    parser.add_argument('--host', type=str, default='0.0.0.0', help='The host of the web server.')
    parser.add_argument('--port', type=int, default=5000, help='Port of the web server.')
    parser.add_argument('--uri', type=str, help='The URI of the crazyflie 2.x drone.')
    parser.add_argument('--debug', action='store_true', help='Outputs many additional debug messages to the console.')
    parser.add_argument('--logging', action='store_true',
                        help='Add a logger that writes CF states to a file (e.g., position estimates).')
    parser.add_argument(
        '--ps',
        type=str,
        choices=['LPS', 'bcFlow2', 'LPS|bcFlow2'],
        default='bcFlow2',
        help='Specify the positioning system to use (LPS, bcFlow2, or LPS|bcFlow2).'
    )

    parser.add_argument('--sim', action='store_true',
                        help='Specify whether you want to run this controller with sim_cf2.')
    parser.add_argument(
        '--dscf',
        action='store_true',
        help='Use ds_crazyflie via ROS topics instead of cflib'
    )
    parser.add_argument(
        '--cf-prefix',
        default='/cf0',
        help="ROS namespace/prefix for the crazyflie (e.g. /cf0, /cf1). Used in --dscf mode."
    )

    ws_group = parser.add_argument_group('WebSocket settings')
    ws_group.add_argument('--wsendpoint', action='store_true',
                          help='Add a websocket that publishes CF state (e.g., position/accel/... estimates).')
    ws_group.add_argument('--wshost', type=str, default='0.0.0.0', help='The host of the websocket server.')
    ws_group.add_argument('--wsport', type=int, default=8765, help='Port of the websocket server.')
    ws_group.add_argument('--wsrate', type=int, default=100, help='Sending rate of the websocket server in ms.')
    return parser


# ######################################################################################################################

global drone
DEBUG = False

global ISRUNNING  # MainAppLoop
ISRUNNING = True

# Positioning Strategy
global positionEstimator
positionEstimator = None  # KalmanEstimatePositionStrategy(), StateEstimatePositionStrategy()
POSITION_ESTIMATE_FILTER = "kalman"  # "state" # "kalman"

websocketserver_started = threading.Event()  # websocket server signal
flask_started = threading.Event()  # webserver signal
deck_attached_event = threading.Event()  # flowDeck signal

# File logger for position updates
LOGGING = False
logger = None
loggingPeriod_in_ms = 100  # ms
logging.basicConfig(level=logging.ERROR)

# Terminal Output
console = Console()

# Shared state of the drone
logValues = {
    'roll': 0.0,
    'pitch': 0.0,
    'yaw': 0.0,
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'latency': 0.0,
    'battery': 0.0,
    'batteryLevel': 0.0,
    'batteryState': 0,  # 0 = Discharging, 1 = Charging
    'acc_x': 0.0,
    'acc_y': 0.0,
    'acc_z': 0.0,
}


# ######################################################################################################################

def power_log_callback(timestamp, data, logconf):
    global logValues
    logValues['battery'] = data['pm.vbat']
    logValues['batteryLevel'] = data['pm.batteryLevel']
    logValues['batteryState'] = data['pm.state']
    if logValues['battery'] < 3.4:
        print("⚠️ Battery low! Consider landing soon.")


def render_view(drone, positionEstimator):
    table = create_table_terminal()
    status_line = Text("", style="dim", markup=False)
    return Group(status_line, table)


def create_table_terminal():
    global logValues
    global positionEstimator

    table = Table(title="Crazyflie Telemetry (Live)")

    table.add_column("Key", justify="right")
    table.add_column("Value")

    battery_state = "Charging 🔌" if logValues['batteryState'] else "Discharging 🔋"
    table.add_row("Latency (ms)", f"{logValues['latency']:.1f}")
    table.add_row("Battery Voltage (V)", f"{logValues['battery']:.2f}")
    table.add_row("Battery Level (%)", f"{logValues['batteryLevel']:.2f}")
    table.add_row("Battery State", battery_state)

    table.add_row("Roll (°)", f"{logValues['roll']:.2f}")
    table.add_row("Pitch (°)", f"{logValues['pitch']:.2f}")
    table.add_row("Yaw (°)", f"{logValues['yaw']:.2f}")
    table.add_row("Acc X (m/s²)", f"{logValues['acc_x']:.2f}")
    table.add_row("Acc Y (m/s²)", f"{logValues['acc_y']:.2f}")
    table.add_row("Acc Z (m/s²)", f"{logValues['acc_z']:.2f}")
    table.add_row("X (m)", f"{positionEstimator.get_log_values()['x']:.2f}")
    table.add_row("Y (m)", f"{positionEstimator.get_log_values()['y']:.2f}")
    table.add_row("Z (m)", f"{positionEstimator.get_log_values()['z']:.2f}")

    return table


def LOG(msg: str):
    global logger, LOGGING
    if LOGGING and logger is not None:
        logger.log(msg)


def logCallback_isFlying(timestamp, data, logconf):
    global drone
    global positionEstimator

    logValues['roll'] = data['stabilizer.roll']
    logValues['pitch'] = data['stabilizer.pitch']
    logValues['yaw'] = data['stabilizer.yaw']
    logValues['acc_x'] = data['acc.x']
    logValues['acc_y'] = data['acc.y']
    logValues['acc_z'] = data['acc.z']

    isMovedByHand = (abs(data['acc.x']) > 0.04 or abs(data['acc.y']) > 0.04)

    # A simple heuristic to determine if the drone is moving based on acceleration
    if abs(data['acc.x']) > 0.1 or abs(data['acc.y']) > 0.1:  # or abs(data['acc.z']) > 0.1:
        drone.isFlying = True
    else:
        drone.isFlying = False

    LOG(f"[{drone.get_current_state()}]: {positionEstimator.position_estimate}")
    # if drone.get_current_state() == "flying" or (
    #         isMovedByHand and (drone.get_current_state() == "idle" or drone.get_current_state() == "active")):
    # console.print(f"\t--[S({drone.get_current_state()})]: Position Estimate={positionEstimator.position_estimate}",
    #              style="dim", markup=False)


# ######################################################################################################################

# Try to land the drone and stop MotionCommander
def cleanup():
    global drone
    try:
        if drone.uavOpStrategyImpl.mc is not None:
            drone.uavOpStrategyImpl.landing_simple()
            time.sleep(500)
            drone.uavOpStrategyImpl.mc.stop()
    except:
        console.print(f"[{drone.get_current_state()}] Quitting program: exception when stopping motor", markup=False)


# ######################################################################################################################

def param_deck_flow(_, value_str):
    value = int(value_str)
    if value:
        deck_attached_event.set()


# ######################################################################################################################

## Flask Webserver
def start_flask_app(app, host, port):
    flask_started.set()
    # DEV
    # app.run(host=host, port=port)
    # PROD
    from waitress import serve
    serve(app, host=host, port=port)


def before_request_callback():
    global drone
    drone.writeSMGraph()


# After each fully completed request: this is the final one: Here we send ACK that everything was OK. UAV OPERATION COMPLETED
def after_request_callback(response):
    # print("After request callback executed.")
    global drone
    drone.writeSMGraph()
    return response


def make_send_pos_data(rate_ms):
    async def send_pos_data(websocket):
        global positionEstimator
        while True:
            data = {
                "message": "crazyflie_position",
                "value": ("%s" % positionEstimator.position_estimate),
            }
            await websocket.send(json.dumps(data))
            await asyncio.sleep(rate_ms / 1000)

    return send_pos_data


def start_websocket_server(host, port, wsrate_ms):
    async def server():
        console.print(f"[{drone.get_current_state()}] Waiting until WebSocket Server has been fully started ...",
                      style="dim", markup=False)
        websocketserver_started.set()
        handler = make_send_pos_data(wsrate_ms)
        async with websockets.serve(handler, host, port):
            console.print(f"[{drone.get_current_state()}] WebSocket server started on ws://{host}:{port}", markup=False)
            await asyncio.Future()  # Keeps running the server

    # Launching the asyncio event loop inside the thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server())


# ######################################################################################################################
def resolve_uri(args):
    if args.uri:
        return uri_helper.uri_from_env(default=args.uri)

    for a in range(7):
        available = cflib.crtp.scan_interfaces(0xe7e7e7e700 + a)
        if available:
            return uri_helper.uri_from_env(default=available[0][0])

    raise RuntimeError("No Crazyflie interfaces found")


def start_flask_in_thread(args):
    global drone

    console.print(
        f"[{drone.get_current_state()}] Starting Flask Command WebServer now ...",
        style="dim",
        markup=False
    )

    app = Flask(__name__)
    app.register_blueprint(drone_blueprint)
    app.config['DRONE'] = drone

    # Register callbacks
    app.after_request(after_request_callback)
    app.before_request(before_request_callback)

    # Custom URL converters
    app.url_map.converters['float'] = FloatConverter

    flask_thread = threading.Thread(
        target=start_flask_app,
        args=(app, args.host, args.port),
        daemon=True
    )
    flask_thread.start()

    # Wait until Flask has fully started
    flask_started.wait()

    console.print(
        f"[{drone.get_current_state()}] Flask WebServer started at http://{args.host}:{args.port}",
        markup=False
    )


def setup_positioning_system(scf, args):
    if not args.sim:
        if args.ps in ["bcFlow2", "LPS|bcFlow2"]:
            scf.cf.param.add_update_callback(
                group='deck', name='bcFlow2', cb=param_deck_flow
            )
            if not deck_attached_event.wait(timeout=5):
                raise RuntimeError("Flow deck not detected")

        if args.ps in ["LPS", "LPS|bcFlow2"]:
            cf_positioning.reset_estimator0(scf, console)

    if args.sim:
        cf_positioning.reset_estimator0(scf, console)


def setup_stabilizer_logging(scf):
    logConfig_Acc = LogConfig(name='Stabilizer', period_in_ms=loggingPeriod_in_ms)
    logConfig_Acc.add_variable('acc.x', 'float')
    logConfig_Acc.add_variable('acc.y', 'float')
    logConfig_Acc.add_variable('acc.z', 'float')
    logConfig_Acc.add_variable('stabilizer.roll', 'float')
    logConfig_Acc.add_variable('stabilizer.pitch', 'float')
    logConfig_Acc.add_variable('stabilizer.yaw', 'float')
    logConfig_Acc.data_received_cb.add_callback(logCallback_isFlying)
    scf.cf.log.add_config(logConfig_Acc)
    logConfig_Acc.start()


def start_runtime_services(args):
    if STARTWSSERVER:
        server_thread = threading.Thread(
            target=start_websocket_server,
            args=(args.wshost, args.wsport, args.wsrate),
            daemon=True
        )
        server_thread.start()
        websocketserver_started.wait()


def main_loop():
    console.print("Drone ready to take commands.", markup=False)

    with Live(create_table_terminal(), refresh_per_second=10, screen=False) as live:
        try:
            while ISRUNNING:
                live.update(create_table_terminal())
                time.sleep(0.1)
        except KeyboardInterrupt:
            console.print("Shutdown requested", markup=False)


def run_dscf_mode(args):
    global drone, positionEstimator

    console.print("[DSCF] Starting ds_crazyflie mode", markup=False)

    if rclpy is None:
        raise RuntimeError("rclpy not available but --dscf was requested")

    console.print(
        f"[{drone.get_current_state()}] Initializing ROS 2 interface ...",
        style="dim",
        markup=False
    )

    rclpy.init()

    console.print(
        f"[{drone.get_current_state()}] ROS 2 initialized",
        markup=False
    )

    # Position from ROS
    cf_id = int(args.cf_prefix.replace('/cf', ''))

    console.print(
        f"[{drone.get_current_state()}] Using Crazyflie prefix {args.cf_prefix} (cf_id={cf_id})",
        markup=False
    )

    console.print(
        f"[{drone.get_current_state()}] Subscribing to /cf_positions_poses for position estimates",
        style="dim",
        markup=False
    )

    positionEstimator = RosPoseArrayPositionStrategy(
        logValues,
        cf_id=cf_id
    )

    console.print(
        f"[{drone.get_current_state()}] ROS-based position estimator started",
        markup=False
    )

    # ROS-based operations
    console.print(
        f"[{drone.get_current_state()}] Installing ROS topic-based drone operations",
        style="dim",
        markup=False
    )

    droneOpsImpl = RosTopicCFOperationImpl(cf_prefix=args.cf_prefix, debug=DEBUG, dronePosEst=positionEstimator)
    drone.set_uavOpsImpl(droneOpsImpl)

    console.print(
        f"[{drone.get_current_state()}] Drone operation implementation set (ROS topics)",
        markup=False
    )

    executor = MultiThreadedExecutor()
    executor.add_node(positionEstimator)
    executor.add_node(droneOpsImpl)
    threading.Thread(
        target=executor.spin,
        daemon=True
    ).start()

    drone.initialize()
    drone.writeSMGraph()

    console.print(
        f"[{drone.get_current_state()}] Drone initialized in DSCF mode",
        markup=False
    )

    start_runtime_services(args)

    console.print(
        f"[{drone.get_current_state()}] Runtime services started",
        markup=False
    )
    console.print(
        f"[{drone.get_current_state()}] The drone is ready to take commands.",
        markup=False
    )

    main_loop()


def run_cflib_mode(args):
    global drone, positionEstimator

    console.print("[CFLIB] Starting cflib mode", markup=False)

    console.print(
        f"[{drone.get_current_state()}]\tInitialize Crazyflie drivers ...",
        style="dim",
        markup=False
    )

    log_power = LogConfig(name='Power', period_in_ms=500)
    log_power.add_variable('pm.vbat', 'float')
    log_power.add_variable('pm.batteryLevel', 'float')
    log_power.add_variable('pm.state', 'uint8_t')  # 0 = Discharging, 1 = Charging

    # Driver init
    cflib.crtp.radiodriver.set_retries_before_disconnect(1500)
    cflib.crtp.radiodriver.set_retries(3)

    if args.sim:
        cflib.crtp.init_drivers(enable_sim_driver=True)
    else:
        cflib.crtp.init_drivers()

    console.print(
        f"[{drone.get_current_state()}] Crazyflie drivers initialized.",
        markup=False
    )
    console.print(
        f"[{drone.get_current_state()}] SIM_MODE = {args.sim}",
        markup=False
    )

    URI = resolve_uri(args)
    console.print(
        f"[{drone.get_current_state()}] URI of drone = {URI}",
        markup=False
    )

    console.print(
        f"[{drone.get_current_state()}] Connecting to drone now ...",
        style="dim",
        markup=False
    )

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        setup_positioning_system(scf, args)

        console.print(
            f"[{drone.get_current_state()}] Start logging interface to get state estimates",
            markup=False
        )

        # Battery logging
        scf.cf.log.add_config(log_power)
        log_power.data_received_cb.add_callback(power_log_callback)
        log_power.start()

        console.print(
            f"[{drone.get_current_state()}] Positioning System Activated: {args.ps}",
            markup=False
        )
        console.print(
            f"[{drone.get_current_state()}] Positioning Estimate Filter: {POSITION_ESTIMATE_FILTER}",
            markup=False
        )

        # Position estimator
        positionEstimator = (
            StateEstimatePositionStrategy(logValues)
            if POSITION_ESTIMATE_FILTER == "state"
            else KalmanEstimatePositionStrategy(logValues)
        )

        logConfig_Pos = LogConfig(name='Position', period_in_ms=loggingPeriod_in_ms)
        positionEstimator.add_variables(logConfig_Pos)
        logConfig_Pos.data_received_cb.add_callback(
            positionEstimator.estimatePositionLogCallback
        )
        scf.cf.log.add_config(logConfig_Pos)
        logConfig_Pos.start()

        console.print(
            f"[{drone.get_current_state()}] Waiting until first position estimate is received ...",
            style="dim",
            markup=False
        )

        if not positionEstimator.position_estimate_event.wait(timeout=5):
            console.print(
                f"[{drone.get_current_state()}] No position estimate received!",
                style="bold red",
                markup=False
            )
            raise RuntimeError("No position estimate received")

        console.print(
            f"[{drone.get_current_state()}] Got Position State Estimate: "
            f"{positionEstimator.position_estimate}",
            markup=False
        )

        # Stabilizer logs
        setup_stabilizer_logging(scf)

        droneOpsImpl = HlCommanderCFOperationImpl(scf=scf, debug=DEBUG)
        drone.set_uavOpsImpl(droneOpsImpl)

        console.print(
            f"[{drone.get_current_state()}] Drone operation implementation set",
            markup=False
        )

        drone.initialize()
        drone.writeSMGraph()

        start_runtime_services(args)

        console.print(
            f"[{drone.get_current_state()}] The drone is ready to take commands.",
            markup=False
        )
        console.print(
            f"[{drone.get_current_state()}] Check available commands here:",
            markup=False
        )
        console.print(
            f"[{drone.get_current_state()}] \thttp://{args.host}:{args.port}/routes",
            markup=False
        )

        main_loop()

        console.print(
            f"[{drone.get_current_state()}] Shutting down ...",
            markup=False
        )

        cleanup()
        log_power.stop()
        scf.cf.close_link()


if __name__ == '__main__':
    parser = create_arg_parser()
    args = parser.parse_args()

    DEBUG = args.debug
    LOGGING = args.logging
    STARTWSSERVER = args.wsendpoint

    if args.dscf and args.sim:
        raise RuntimeError("--dscf and --sim are mutually exclusive")

    # State machine
    drone = cf_sm.StateMachineDrone(drone_id="uav1", debug=DEBUG)
    drone.install()
    drone.start()

    # Flask always on
    start_flask_in_thread(args)

    if args.dscf:
        run_dscf_mode(args)
    else:
        run_cflib_mode(args)
