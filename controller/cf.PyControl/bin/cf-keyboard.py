#!/usr/bin/env python3
"""
Keyboard controller for the cf.PyCtrl REST client.

This script provides a keyboard-driven control interface for one Crazyflie
connected through a running cf.PyCtrl REST API server. It reads the latest
position estimate from the cf.PyCtrl WebSocket stream and sends absolute
navigation targets through the REST API.

The controller is intended for manual integration testing and lightweight
interactive operation, similar in spirit to the cfclient control interface.

Key mapping:
    W       -> move forward  (+x)
    S       -> move backward (-x)
    A       -> move left     (+y)
    D       -> move right    (-y)
    UP      -> move up       (+z)
    DOWN    -> move down     (-z)

    ENTER   -> activate idle
    SPACE   -> toggle takeoff / landing
    P       -> begin multiranger push mode
    O       -> end multiranger push mode

    Q / ESC -> quit

Motion keys use the latest WebSocket position as the current reference and
apply a predefined step size before sending the next target.

Author: Dominik Grzelak
"""
import threading
import logging
import argparse
import sys
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, Callable

from pynput import keyboard

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CLIENT_SRC = PROJECT_ROOT / "src" / "client"

if str(CLIENT_SRC) not in sys.path:
    sys.path.insert(0, str(CLIENT_SRC))

from cf_pyctrl_client import CfPyCtrlApiClient


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)


@dataclass
class KeyboardControlConfig:
    step_xy: float = 0.25
    step_z: float = 0.15

    min_z: float = 0.25
    max_z: float = 1.0

    multiranger_min_distance: float = 0.4
    multiranger_velocity: float = 0.2
    multiranger_loop_delay: float = 0.1

    base_url: str = "http://127.0.0.1:5000"
    websocket_host: str = "127.0.0.1"
    websocket_port: int = 8765

    websocket_timeout: float = 2.0


class KeyboardDroneController:
    def __init__(self, config: KeyboardControlConfig):
        self.config = config
        self.running = True

        self.command_lock = threading.Lock()
        self.command_active = False
        self.command_thread = None

        self.client = CfPyCtrlApiClient(
            base_url=config.base_url,
            wshost=config.websocket_host,
            wsport=config.websocket_port
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.5

    def read_position(self) -> Optional[Tuple[float, float, float]]:
        position = self.client.read_position_websocket(
            timeout=self.config.websocket_timeout
        )

        if position is None:
            logger.warning("No WebSocket position received; using last local target")
            return None

        self.current_x, self.current_y, self.current_z = position
        return position

    def print_status(self):
        logger.info(
            f"Current target: "
            f"x={self.current_x:.3f}, "
            f"y={self.current_y:.3f}, "
            f"z={self.current_z:.3f}"
        )

    def activate_idle_command(self):
        logger.info("[ENTER] activate idle")
        success = self.client.activate_idle()

        if success:
            self.client.wait_for_state("idle", timeout=5.0)
            self.read_position()
            self.print_status()

    def send_target(self, x: float, y: float, z: float) -> bool:
        z = max(self.config.min_z, min(self.config.max_z, z))

        logger.info(
            f"Sending target: "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

        success = self.client.navigate_to(x, y, z)

        if success:
            self.current_x = x
            self.current_y = y
            self.current_z = z
            self.print_status()
        else:
            logger.error("Failed to send keyboard navigation target")

        return success

    def toggle_takeoff_landing(self):
        current_state = self.client.get_current_state()
        logger.info(f"[SPACE] current state: {current_state}")

        if current_state == "idle":
            logger.info("[SPACE] begin takeoff")
            success = self.client.begin_takeoff()

            if success:
                self.client.wait_for_state("hovering", timeout=10.0)
                self.read_position()
                self.print_status()

            return

        if current_state == "hovering":
            logger.info("[SPACE] begin landing")
            success = self.client.begin_landing()

            if success:
                self.client.wait_for_state("idle", timeout=10.0)
                self.read_position()
                self.print_status()

            return

        logger.warning(
            f"[SPACE] ignored: takeoff/landing not allowed from state '{current_state}'"
        )

    def move_relative(self, dx: float, dy: float, dz: float):
        self.read_position()

        target_x = self.current_x + dx
        target_y = self.current_y + dy
        target_z = self.current_z + dz

        self.send_target(target_x, target_y, target_z)

    def move_forward(self):
        logger.info(f"[W] move forward +x by {self.config.step_xy} m")
        self.move_relative(dx=self.config.step_xy, dy=0.0, dz=0.0)

    def move_backward(self):
        logger.info(f"[S] move backward -x by {self.config.step_xy} m")
        self.move_relative(dx=-self.config.step_xy, dy=0.0, dz=0.0)

    def move_left(self):
        logger.info(f"[A] move left +y by {self.config.step_xy} m")
        self.move_relative(dx=0.0, dy=self.config.step_xy, dz=0.0)

    def move_right(self):
        logger.info(f"[D] move right -y by {self.config.step_xy} m")
        self.move_relative(dx=0.0, dy=-self.config.step_xy, dz=0.0)

    def move_up(self):
        logger.info(f"[UP] move up +z by {self.config.step_z} m")
        self.move_relative(dx=0.0, dy=0.0, dz=self.config.step_z)

    def move_down(self):
        logger.info(f"[DOWN] move down -z by {self.config.step_z} m")
        self.move_relative(dx=0.0, dy=0.0, dz=-self.config.step_z)

    def quit(self):
        logger.info("Quit keyboard controller")
        self.running = False
        return False

    def begin_multiranger_push_command(self):
        current_state = self.client.get_current_state()
        logger.info(f"[P] current state: {current_state}")

        if current_state not in ("idle"): # "hovering"
            logger.warning(
                f"[P] ignored: multiranger push can only start from "
                f"'idle', current state is '{current_state}'" # or 'hovering'
            )
            return

        logger.info("[P] begin multiranger push")

        success = self.client.begin_multiranger_push(
            min_distance=self.config.multiranger_min_distance,
            velocity=self.config.multiranger_velocity,
            loop_delay=self.config.multiranger_loop_delay
        )

        if success:
            self.client.wait_for_state("multiranger_push", timeout=5.0)


    def end_multiranger_push_command(self):
        current_state = self.client.get_current_state()
        logger.info(f"[O] current state: {current_state}")

        if current_state == "idle":
            logger.info("[O] multiranger push already ended; current state is idle")
            self.read_position()
            self.print_status()
            return

        if current_state != "multiranger_push":
            logger.warning(
                f"[O] ignored: end multiranger push can only run from "
                f"'multiranger_push' or be ignored from 'idle', "
                f"current state is '{current_state}'"
            )
            return

        logger.info("[O] end multiranger push")

        success = self.client.end_multiranger_push()

        if success:
            self.client.wait_for_state("idle", timeout=5.0)
            self.read_position()
            self.print_status()

    def on_press(self, key):
        try:
            char = key.char

            if char is None:
                return None

            char = char.lower()

            if char == "w":
                return self.run_guarded_command("W", self.move_forward)
            elif char == "s":
                return self.run_guarded_command("S", self.move_backward)
            elif char == "a":
                return self.run_guarded_command("A", self.move_left)
            elif char == "d":
                return self.run_guarded_command("D", self.move_right)
            elif char == " ":
                return self.run_guarded_command("SPACE", self.toggle_takeoff_landing)
            elif char == "p":
                return self.run_guarded_command("P", self.begin_multiranger_push_command)
            elif char == "o":
                return self.run_non_guarded_command("O", self.end_multiranger_push_command)
            elif char == "q":
                return self.quit()

        except AttributeError:
            if key == keyboard.Key.up:
                return self.run_guarded_command("UP", self.move_up)
            elif key == keyboard.Key.down:
                return self.run_guarded_command("DOWN", self.move_down)
            elif key == keyboard.Key.space:
                return self.run_guarded_command("SPACE", self.toggle_takeoff_landing)
            elif key == keyboard.Key.enter:
                return self.run_guarded_command("ENTER", self.activate_idle_command)
            elif key == keyboard.Key.esc:
                return self.quit()

        return None

    def run_guarded_command(self, command_name: str, command: Callable[[], object]):
        with self.command_lock:
            if self.command_thread is not None and self.command_thread.is_alive():
                logger.info(f"Ignored [{command_name}]: previous command still running")
                return None

            self.command_thread = threading.Thread(
                target=self._run_command_worker,
                args=(command_name, command),
                daemon=True
            )
            self.command_thread.start()

        return None

    def run_non_guarded_command(self, command_name: str, command: Callable[[], object]):
        thread = threading.Thread(
            target=self._run_command_worker,
            args=(command_name, command),
            daemon=True
        )
        thread.start()
        return None

    def _run_command_worker(self, command_name: str, command: Callable[[], object]):
        try:
            logger.info(f"Started [{command_name}]")
            command()
        except Exception as e:
            logger.error(f"Command [{command_name}] failed: {e}")
        finally:
            logger.info(f"Finished [{command_name}]")

    def run(self):
        logger.info("Keyboard controller started")
        logger.info(
            "Use W/S/A/D + UP/DOWN. ENTER activates idle. "
            "SPACE toggles takeoff/landing. "
            "P starts multiranger push. O ends multiranger push. "
            "Press Q or ESC to quit."
        )
        logger.info(f"REST API: ws={self.config.base_url}")
        logger.info(
            f"WebSocket: ws://{self.config.websocket_host}:"
            f"{self.config.websocket_port}"
        )

        self.read_position()
        self.print_status()

        with keyboard.Listener(on_press=self.on_press) as listener:
            listener.join()

def parse_args():
    parser = argparse.ArgumentParser(
        description="Keyboard controller for cf.PyCtrl REST client."
    )

    parser.add_argument(
        "--base-url",
        default="http://127.0.0.1:5000",
        help="Base URL of the cf.PyCtrl REST API."
    )

    parser.add_argument(
        "--websocket-host",
        "--wshost",
        default="127.0.0.1",
        help="WebSocket host for cf.PyCtrl position data."
    )

    parser.add_argument(
        "--websocket-port",
        "--wsport",
        type=int,
        default=8765,
        help="WebSocket port for cf.PyCtrl position data."
    )

    parser.add_argument(
        "--websocket-timeout",
        "--wstimeout",
        type=float,
        default=2.0,
        help="Timeout in seconds for reading a WebSocket position sample."
    )

    parser.add_argument(
        "--step-xy",
        type=float,
        default=0.25,
        help="Movement step for x/y commands in meters."
    )

    parser.add_argument(
        "--step-z",
        type=float,
        default=0.15,
        help="Movement step for z commands in meters."
    )

    parser.add_argument(
        "--min-z",
        type=float,
        default=0.25,
        help="Minimum allowed z target in meters."
    )

    parser.add_argument(
        "--max-z",
        type=float,
        default=1.00,
        help="Maximum allowed z target in meters."
    )

    parser.add_argument(
        "--multiranger-min-distance",
        type=float,
        default=0.4,
        help="Minimum obstacle distance in meters for multiranger push."
    )

    parser.add_argument(
        "--multiranger-velocity",
        type=float,
        default=0.2,
        help="Push velocity in m/s for multiranger push."
    )

    parser.add_argument(
        "--multiranger-loop-delay",
        type=float,
        default=0.1,
        help="Control-loop delay in seconds for multiranger push."
    )

    return parser.parse_args()

def main():
    args = parse_args()

    config = KeyboardControlConfig(
        step_xy=args.step_xy,
        step_z=args.step_z,
        min_z=args.min_z,
        max_z=args.max_z,
        base_url=args.base_url,
        websocket_host=args.websocket_host,
        websocket_port=args.websocket_port,
        websocket_timeout=args.websocket_timeout,
        multiranger_min_distance=args.multiranger_min_distance,
        multiranger_velocity=args.multiranger_velocity,
        multiranger_loop_delay=args.multiranger_loop_delay
    )

    logger.info("Keyboard configuration:")
    logger.info(f"  base_url          = {config.base_url}")
    logger.info(f"  websocket_host    = {config.websocket_host}")
    logger.info(f"  websocket_port    = {config.websocket_port}")
    logger.info(f"  websocket_timeout = {config.websocket_timeout}")
    logger.info(f"  step_xy           = {config.step_xy}")
    logger.info(f"  step_z            = {config.step_z}")
    logger.info(f"  min_z             = {config.min_z}")
    logger.info(f"  max_z             = {config.max_z}")
    logger.info(f"  multiranger_min_distance = {config.multiranger_min_distance}")
    logger.info(f"  multiranger_velocity     = {config.multiranger_velocity}")
    logger.info(f"  multiranger_loop_delay   = {config.multiranger_loop_delay}")


    controller = KeyboardDroneController(config)
    controller.run()


if __name__ == "__main__":
    main()