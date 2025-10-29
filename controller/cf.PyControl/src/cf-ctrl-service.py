#!venv/bin/python
import logging
import sys
import time
from typing import Optional
from threading import Event
import threading
import argparse
import asyncio
import websockets
import json

from rich.live import Live
from rich.table import Table
from rich.console import Console

from flask import Flask, jsonify, request
from routes import drone_blueprint  # Import the blueprint
from werkzeug.routing import BaseConverter, ValidationError
from statemachine import StateMachine, State, exceptions
from statemachine.contrib.diagram import DotGraphMachine

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils import uri_helper

import cf_positioning
from cf_positioning import Point3D, PositionEstimationStrategy, KalmanEstimatePositionStrategy, \
    StateEstimatePositionStrategy
from cf_drone_ops import CFOperationStrategy, HlCommanderCFOperationImpl, DebugLoggingCFOperationImpl
import cf_sm
from cf_logger import FileLogger
from routes import FloatConverter


# ######################################################################################################################

def create_arg_parser():
    # Create the parser.
    parser = argparse.ArgumentParser(description='Crazyflie 2.x RESTful Drone Controller Service')

    # Add the arguments.
    parser.add_argument('--host', type=str, default='0.0.0.0', help='The host of the web server.')
    parser.add_argument('--port', type=int, default=5000, help='Port of the web server.')
    parser.add_argument('--uri', type=str, help='The URI of the crazyflie 2.x drone.')
    parser.add_argument('--sim', action='store_false',
                        help='Specify whether you want to run this controller with sim_cf2.')
    parser.add_argument('--debug', action='store_true', help='Outputs many additional debug messages to the console.')
    parser.add_argument('--logging', action='store_true',
                        help='Add a logger that writes CF states to a file (e.g., position estimates).')
    parser.add_argument(
            '--ps',
            type=str,
            choices=['LPS', 'bcFlow2', 'LPS|bcFlow2'],
            default='LPS',
            help='Specify the positioning system to use (LPS, bcFlow2, or LPS|bcFlow2).'
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

# Shared state
log_values = {
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
    global log_values
    log_values['battery'] = data['pm.vbat']
    log_values['batteryLevel'] = data['pm.batteryLevel']
    log_values['batteryState'] = data['pm.state']
    if log_values['battery'] < 3.4:
        print("⚠️ Battery low! Consider landing soon.")


def create_table_terminal():
    global log_values
    global positionEstimator

    table = Table(title="Crazyflie Telemetry (Live)")

    table.add_column("Stabilizer", justify="right")
    table.add_column("Value")

    battery_state = "Charging 🔌" if log_values['batteryState'] else "Discharging 🔋"
    table.add_row("Latency (ms)", f"{log_values['latency']:.1f}")
    table.add_row("Battery Voltage (V)", f"{log_values['battery']:.2f}")
    table.add_row("Battery Level (%)", f"{log_values['batteryLevel']:.2f}")
    table.add_row("Battery State", battery_state)

    table.add_row("Roll (°)", f"{log_values['roll']:.2f}")
    table.add_row("Pitch (°)", f"{log_values['pitch']:.2f}")
    table.add_row("Yaw (°)", f"{log_values['yaw']:.2f}")
    table.add_row("Acc X (m/s²)", f"{log_values['acc_x']:.2f}")
    table.add_row("Acc Y (m/s²)", f"{log_values['acc_y']:.2f}")
    table.add_row("Acc Z (m/s²)", f"{log_values['acc_z']:.2f}")
    table.add_row("X (m)", f"{log_values['x']:.2f}")
    table.add_row("Y (m)", f"{log_values['y']:.2f}")
    table.add_row("Z (m)", f"{log_values['z']:.2f}")
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

    log_values['roll'] = data['stabilizer.roll']
    log_values['pitch'] = data['stabilizer.pitch']
    log_values['yaw'] = data['stabilizer.yaw']
    log_values['acc_x'] = data['acc.x']
    log_values['acc_y'] = data['acc.y']
    log_values['acc_z'] = data['acc.z']

    isMovedByHand = (abs(data['acc.x']) > 0.04 or abs(data['acc.y']) > 0.04)

    # A simple heuristic to determine if the drone is moving based on acceleration
    if abs(data['acc.x']) > 0.1 or abs(data['acc.y']) > 0.1:  # or abs(data['acc.z']) > 0.1:
        drone.isFlying = True
    else:
        drone.isFlying = False

    LOG(f"[{drone.get_current_state()}]: {positionEstimator.position_estimate}")
    if drone.get_current_state() == "flying" or (
            isMovedByHand and (drone.get_current_state() == "idle" or drone.get_current_state() == "active")):
        console.print(f"\t--[S({drone.get_current_state()})]: Position Estimate={positionEstimator.position_estimate}",
                      style="dim", markup=False)


# ######################################################################################################################

def cleanup():
    global drone
    try:
        if drone.uavOpStrategyImpl.mc is not None:
            drone.uavOpStrategyImpl.mc.stop()  # Stop MotionCommander
    except:
        console.print(f"--[{drone.get_current_state()}] Quitting program: exception when stopping motor", markup=False)


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
        websocketserver_started.set()
        handler = make_send_pos_data(wsrate_ms)
        async with websockets.serve(handler, host, port):
            console.print(f"--[{drone.get_current_state()}] WebSocket server started on ws://{host}:{port}", markup=False)
            await asyncio.Future()  # Keeps running the server
    # Launching the asyncio event loop inside the thread
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(server())

# ######################################################################################################################

if __name__ == '__main__':
    parser = create_arg_parser()
    args = parser.parse_args()
    SIM_MODE = args.sim
    DEBUG = args.debug
    LOGGING = args.logging
    STARTWSSERVER = args.wsendpoint
    POSITIONING_SYSTEM = args.ps
    # print("Arguments supplied: %s" % args)

    if LOGGING:
        logFile = f'./log_position-estimate-{POSITION_ESTIMATE_FILTER}.txt'
        logger = FileLogger(logFile)
        logger.close()
        console.print(f"Position estimate log file is {logFile}", markup=False)

    log_power = LogConfig(name='Power', period_in_ms=500)
    log_power.add_variable('pm.vbat', 'float')
    log_power.add_variable('pm.batteryLevel', 'float')
    log_power.add_variable('pm.state', 'uint8_t')  # 0 = Discharging, 1 = Charging

    # Instantiate the state machine
    uav_name = "uav1"
    drone = cf_sm.StateMachineDrone(drone_id=uav_name, debug=DEBUG)
    drone.writeSMGraph()
    console.print(f"--[{drone.get_current_state()}]\tInitialize Crazyflie drivers ...", style="dim", markup=False)
    # Initialize the low-level drivers
    cflib.crtp.radiodriver.set_retries_before_disconnect(1500)
    cflib.crtp.radiodriver.set_retries(3)
    if SIM_MODE:
        cflib.crtp.init_drivers(enable_sim_driver=True)
    else:
        cflib.crtp.init_drivers()
    console.print(f"--[{drone.get_current_state()}] Crazyflie drivers initialized. [OK]", markup=False)
    console.print(f"--[{drone.get_current_state()}] SIM_MODE = {SIM_MODE}", markup=False)
    URI = None
    if args.uri:
        cfURI = args.uri
        URI = uri_helper.uri_from_env(default=cfURI)
    else:
        console.print(f"--[{drone.get_current_state()}] No Crazyflie URI specified. Scanning interfaces now ...",
                      style="dim", markup=False)
        found = False
        for a in range(7):
            available = cflib.crtp.scan_interfaces(0xe7e7e7e700 + a)
            if (len(available) > 0):
                console.print(
                    f"--[{drone.get_current_state()}] Crazyflies found %s ... taking first one:" % len(available),
                    style="dim", markup=False)
                for i in available:
                    URI = uri_helper.uri_from_env(default=i[0])
                    console.print(i[0])
                    found = True
                    break
            if found: break
        if not found:
            console.print("🚫 No interfaces found ...")
            sys.exit(-1)
    console.print(f"--[{drone.get_current_state()}] URI of drone = {URI}", markup=False)
    # Installation
    console.print(f"--[{drone.get_current_state()}] Installing software packages now ... [OK]", markup=False)
    drone.install()
    drone.writeSMGraph()

    console.print(f"--[{drone.get_current_state()}] Resolving Dependencies now ... [OK]", markup=False)
    # TODO wrap in lambda function: tryRepeat(lambda: Function(Void) -> {}, maxFailCnt)
    failCnt = 0
    failCntMax = 3
    while drone.current_state != drone.starting:
        try:
            drone.writeSMGraph()
            drone.start()
        except:
            console.print(f"--[{drone.get_current_state()}] Try restart ...", markup=False)
            drone.writeSMGraph()
            time.sleep(1)
            if failCnt > failCntMax:
                console.print()
                console.print(f"🚫 --[{drone.get_current_state()}] FailCounter Max reached", style="bold red",
                              markup=False)
                sys.exit()
    drone.writeSMGraph()

    # Start Flask app in a separate thread
    console.print(f"--[{drone.get_current_state()}] Starting Flask Command WebServer now ...", style="dim",
                  markup=False)
    app = Flask(__name__)
    app.register_blueprint(drone_blueprint)  # Register the blueprint with the app
    app.config['DRONE'] = drone
    # Register the before request callback
    app.after_request(after_request_callback)
    app.before_request(
        before_request_callback)  # app.before_request(lambda: before_request_callback(callback_argument))
    # Register the custom converter
    app.url_map.converters['float'] = FloatConverter
    flask_thread = threading.Thread(target=start_flask_app, args=(app, args.host, args.port))
    flask_thread.daemon = True  # Allows the thread to exit when the main program does
    flask_thread.start()
    # Wait until Flask has fully started
    flask_started.wait()
    console.print(f"--[{drone.get_current_state()}] Flask WebServer started [OK]", markup=False)

    # "Main Loop"
    console.print(f"--[{drone.get_current_state()}] Connecting to drone now ...", style="dim", markup=False)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        if not SIM_MODE:
            if POSITIONING_SYSTEM in ["bcFlow2", "LPS|bcFlow2"]:
                scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=param_deck_flow)
                if not deck_attached_event.wait(timeout=5):
                    console.print(f'🚫 --[{drone.get_current_state()}] No flow deck detected!', style="bold red",
                                  markup=False)
                    sys.exit(1)
            if POSITIONING_SYSTEM in ["LPS", "LPS|bcFlow2"]:
                cf_positioning.reset_estimator(scf, console)
            console.print(f'--[{drone.get_current_state()}] Flow deck detected! [OK]', markup=False)

        if SIM_MODE:
            cf_positioning.reset_estimator(scf, console)

        # State Estimates
        # Create a logging configurations
        console.print("", end="\r")
        console.print(f"--[{drone.get_current_state()}] Start logging interface to get state estimates", markup=False)
        # Battery Logger
        scf.cf.log.add_config(log_power)
        log_power.data_received_cb.add_callback(power_log_callback)
        log_power.start()
        console.print(f"--[{drone.get_current_state()}] POSITIONING_SYSTEM: {POSITIONING_SYSTEM}", markup=False)
        console.print(f"--[{drone.get_current_state()}] POSITION ESTIMATE FILTER: {POSITION_ESTIMATE_FILTER}",
                      markup=False)
        logConfig_Pos = None
        positionEstimator = KalmanEstimatePositionStrategy(log_values)
        if POSITION_ESTIMATE_FILTER == "state":
            positionEstimator = StateEstimatePositionStrategy(log_values)
        logConfig_Pos = LogConfig(name='Position', period_in_ms=loggingPeriod_in_ms)
        positionEstimator.add_variables(logConfig_Pos)
        logConfig_Pos.data_received_cb.add_callback(positionEstimator.estimatePositionLogCallback)
        scf.cf.log.add_config(logConfig_Pos)
        logConfig_Pos.start()
        console.print(f"--[{drone.get_current_state()}] Waiting until first position estimate is received ...",
                      style="dim", markup=False)
        if not positionEstimator.position_estimate_event.wait(timeout=5):
            console.print(f'🚫 --[{drone.get_current_state()}] No position estimate received!', style="bold red",
                          markup=False)
            sys.exit(1)
        console.print(
            f"--[{drone.get_current_state()}] Got Position State Estimate: {positionEstimator.position_estimate}",
            markup=False)

        # Acceleration, Roll, Pitch, Yaw
        # KalmanStabilizer
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

        # Use mockmode or no-mockmode
        droneOpsImpl = HlCommanderCFOperationImpl(scf=scf,
                                                  debug=DEBUG)  # DebugLoggingCFOperationImpl(scf=scf, debug=DEBUG) #
        drone.set_uavOpsImpl(droneOpsImpl)
        console.print(f"--[{drone.get_current_state()}] Drone Operation Implementation set: {droneOpsImpl}",
                      markup=False)

        # TODO tryRepeat(lambda: )
        drone.initialize()
        drone.writeSMGraph()
        # console.print(f"--[{drone.get_current_state()}] Hardware checks completed.")

        if STARTWSSERVER:
            console.print(f"--[{drone.get_current_state()}] Waiting until WebSocket Server has been fully started ...",
                          style="dim", markup=False)
            server_thread = threading.Thread(target=start_websocket_server,
                                             args=(args.wshost, args.wsport, args.wsrate))
            server_thread.daemon = True  # Allows the thread to exit when the main program does
            server_thread.start()
            websocketserver_started.wait()
            console.print(f"--[{drone.get_current_state()}] WebSocket Server started [OK]", markup=False)

        # Main Loop
        console.print(f"--[{drone.get_current_state()}] The drone is ready to take commands.", markup=False)
        console.print(f"--[{drone.get_current_state()}] Check available commands here:", markup=False)
        console.print(f"--[{drone.get_current_state()}] http://{args.host}:{args.port}/routes", markup=False)

        # Rich live view
        with Live(create_table_terminal(), refresh_per_second=10, screen=False) as live:
            try:
                # Keep the main thread alive to keep the Crazyflie connection open
                while ISRUNNING:
                    live.update(create_table_terminal())
                    time.sleep(0.1)
            except KeyboardInterrupt:
                console.print(f"--[{drone.get_current_state()}] Keyboard interrupt detected. Shutting down ...",
                              markup=False)

        # Shutdown
        cleanup()
        scf.cf.close_link()
        logConfig_Acc.stop()
        logConfig_Pos.stop()
        log_power.stop()
