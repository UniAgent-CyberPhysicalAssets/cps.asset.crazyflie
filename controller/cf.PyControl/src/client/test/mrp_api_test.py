#!/usr/bin/env python3
"""
Two-drone multiranger push test client for cf.PyCtrl.

This client connects to two running cf.PyCtrl REST API servers, starts
MULTIRANGER_PUSH mode on both drones, waits for a fixed duration, and then
stops the mode again.

Expected running servers:
    http://127.0.0.1:5000
    http://127.0.0.1:5001

Author: Dominik Grzelak
"""

import logging
import threading
import time
import sys
from pathlib import Path

CLIENT_DIR = Path(__file__).resolve().parents[1]

if str(CLIENT_DIR) not in sys.path:
    sys.path.insert(0, str(CLIENT_DIR))
import cf_pyctrl_client as asm


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)


WAIT_TIME_SECONDS = 50.0

MULTIRANGER_MIN_DISTANCE = 0.3
MULTIRANGER_VELOCITY = 0.3
MULTIRANGER_LOOP_DELAY = 0.1

def multiranger_push_sequence(drone: asm.CfPyCtrlApiClient, name: str) -> bool:
    logger.info(f"[{name}] activating idle")

    if not drone.activate_idle():
        logger.error(f"[{name}] activate_idle failed")
        return False

    if not drone.wait_for_state("idle", timeout=5.0):
        logger.error(f"[{name}] did not reach idle")
        return False

    logger.info(f"[{name}] starting multiranger push")

    if not drone.begin_multiranger_push(
            min_distance=MULTIRANGER_MIN_DISTANCE,
            velocity=MULTIRANGER_VELOCITY,
            loop_delay=MULTIRANGER_LOOP_DELAY
    ):
        logger.error(f"[{name}] begin_multiranger_push failed")
        return False

    if not drone.wait_for_state("multiranger_push", timeout=5.0):
        logger.error(f"[{name}] did not reach multiranger_push")
        return False

    logger.info(f"[{name}] multiranger push active for {WAIT_TIME_SECONDS:.1f}s")
    time.sleep(WAIT_TIME_SECONDS)

    logger.info(f"[{name}] stopping multiranger push")

    if not drone.end_multiranger_push():
        logger.error(f"[{name}] end_multiranger_push failed")
        return False

    if not drone.wait_for_state("idle", timeout=10.0):
        logger.error(f"[{name}] did not return to idle")
        return False

    logger.info(f"[{name}] multiranger push test completed")
    return True

def run_on_drone(name: str, drone: asm.CfPyCtrlApiClient):
    try:
        success = multiranger_push_sequence(drone, name)

        if success:
            logger.info(f"[{name}] sequence executed successfully")
        else:
            logger.error(f"[{name}] sequence failed")

    except Exception as e:
        logger.error(f"[{name}] unexpected error: {e}")

def control_multiple_drones():
    drone1 = asm.CfPyCtrlApiClient(base_url="http://127.0.0.1:5000")
    drone2 = asm.CfPyCtrlApiClient(base_url="http://127.0.0.1:5001")

    thread1 = threading.Thread(
        target=run_on_drone,
        args=("drone1", drone1),
        daemon=False
    )

    thread2 = threading.Thread(
        target=run_on_drone,
        args=("drone2", drone2),
        daemon=False
    )

    logger.info("Starting multiranger push test threads")
    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

    logger.info("All drone tests finished")


if __name__ == "__main__":
    control_multiple_drones()