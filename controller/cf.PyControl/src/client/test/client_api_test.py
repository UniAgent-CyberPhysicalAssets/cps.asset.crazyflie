#!/usr/bin/env python3
"""
Entry point for testing the AutoStateMachine multi-drone client for cf.PyCtrl.

This script starts predefined client-side test sequences against one or more
running Crazyflie cf.PyCtrl REST services.
It is used as multi-drone coordination behavior.

The controller services must be started before running this script (for example with ds-crazyflie):
- ./cfpyctrl.sh --dscf --cf-prefix /cf0 --port 5000 --wsendpoint --wsport 8765
- ./cfpyctrl.sh --dscf --cf-prefix /cf1 --port 5001 --wsendpoint --wsport 8766

Author: Dominik Grzelak
"""

import threading
import sys
from pathlib import Path

CLIENT_DIR = Path(__file__).resolve().parents[1]

if str(CLIENT_DIR) not in sys.path:
    sys.path.insert(0, str(CLIENT_DIR))
import cf_pyctrl_client as asm

import logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def run_sequence(drone, sequence):
    if drone.execute_sequence(sequence):
        logger.info(f"Drone sequence executed successfully")
    else:
        logger.error(f"Drone sequence execution failed")

def control_multiple_drones():
    drone1 = asm.CfPyCtrlApiClient(base_url="http://127.0.0.1:5000")
    drone2 = asm.CfPyCtrlApiClient(base_url="http://127.0.0.1:5001")

    sequence1 = [
        {
            "action": "activate_idle",
            "wait_state": "idle",
            "timeout": 5.0
        },
        {
            "action": "begin_takeoff",
            "wait_state": "hovering",
            "timeout": 10.0,
            "wait_time": 2.0
        },
        {
            "action": "navigate",
            "x": 0.0,
            "y": 0.5,
            "z": 0.5,
            "wait_state": "hovering",
            "timeout": 60.0,
            "wait_time": 0.0
        },
        {
            "action": "fly_trajectory",
            "trajectory_file": "../../examples/trajectories/tra-1.json",
            "wait_state": "hovering",
            "timeout": 60.0,
            "wait_time": 1.0
        },
        {
            "action": "begin_landing",
            "wait_state": "idle",
            "timeout": 10.0
        }
    ]

    sequence2 = [
        {
            "action": "activate_idle",
            "wait_state": "idle",
            "timeout": 5.0
        },
        {
            "action": "begin_takeoff",
            "wait_state": "hovering",
            "timeout": 10.0,
            "wait_time": 0.0
        },
        {
            "action": "navigate",
            "x": 0.0,
            "y": -0.5,
            "z": 0.6,
            "wait_state": "hovering",
            "timeout": 60.0,
            "wait_time": 0.0
        },
        {
            "action": "navigate",
            "x": -0.2,
            "y": -0.2,
            "z": 0.6,
            "wait_state": "hovering",
            "timeout": 60.0,
            "wait_time": 0.0
        },
        {
            "action": "navigate",
            "x": 0,
            "y": 0,
            "z": 0.6,
            "wait_state": "hovering",
            "timeout": 60.0,
            "wait_time": 0.0
        },
        {
            "action": "navigate",
            "x": -0.2,
            "y": -0.2,
            "z": 0.6,
            "wait_state": "hovering",
            "timeout": 60.0,
            "wait_time": 0.0
        },
        {
            "action": "begin_landing",
            "wait_state": "idle",
            "timeout": 10.0
        }
    ]

    print("Create threads ...")
    thread1 = threading.Thread(target=run_sequence, args=(drone1, sequence1))
    thread2 = threading.Thread(target=run_sequence, args=(drone2, sequence2))


    print("Start threads ...")
    thread1.start()
    thread2.start()

    print("Wait for threads to finish ...")
    thread1.join()
    thread2.join()

if __name__ == "__main__":
    control_multiple_drones()